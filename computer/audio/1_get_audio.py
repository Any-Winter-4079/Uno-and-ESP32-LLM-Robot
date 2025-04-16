"""
ESP32 Audio Receiver Script
Receives and stores audio data from ESP32-WROVER via WebSockets

Features:
- Receives raw audio data streamed over WebSocket
- Converts to WAV format
- Saves to timestamped files for later review
- Works over mobile hotspot or local WiFi
"""

import os
import io
import asyncio
import datetime
import websockets
from pydub import AudioSegment

# Network configuration
USE_HOTSPOT = True                                                 # True for phone hotspot, False for home WiFi
WS_AUDIO_HOST = '172.20.10.4' if USE_HOTSPOT else "192.168.1.174"  # Server IP address
WS_AUDIO_PORT = 8888                                               # WebSocket server port

# Audio configuration
FRAME_RATE = 16000                                                 # Required for Whisper compatibility
CHANNELS = 1                                                       # Mono channel
BIT_DEPTH = 2                                                      # 16-bit audio
STORING_FOLDER = "recorded_audio/"                                 # Folder to save audio files
END_OF_AUDIO_SIGNAL = "END_OF_AUDIO"                               # Signal used to detect end of audio stream

async def audio_receiver(websocket, path):
    """
    Receives raw audio stream from ESP32-WROVER and saves it as a WAV file

    Args:
        websocket: WebSocket connection from ESP32
        path: WebSocket path (unused but required by handler)
    """
    audio_buffer = io.BytesIO()  # In-memory buffer for incoming audio data

    try:
        async for message in websocket:
            # If the ESP32 sends the end signal, finalize and save the audio
            if message == END_OF_AUDIO_SIGNAL:
                audio_buffer.seek(0)

                # Convert raw PCM data to an AudioSegment
                audio_segment = AudioSegment.from_raw(
                    audio_buffer,
                    sample_width=BIT_DEPTH,
                    frame_rate=FRAME_RATE,
                    channels=CHANNELS
                )

                # Export audio to WAV format
                wav_bytes = io.BytesIO()
                audio_segment.export(wav_bytes, format="wav")

                # Generate filename with timestamp
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{STORING_FOLDER}{timestamp}.wav"

                # Save the WAV data to file
                with open(filename, "wb") as wav_file:
                    wav_file.write(wav_bytes.getvalue())
                print(f"Audio saved to {filename}")

                # Reset buffer for the next recording
                audio_buffer.seek(0)
                audio_buffer.truncate(0)
            else:
                # Continuously write raw audio chunks to the buffer
                audio_buffer.write(message)

    except websockets.exceptions.ConnectionClosedError:
        print("WebSocket connection closed unexpectedly.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Free resources
        audio_buffer.close()

if __name__ == "__main__":
    # Ensure output folder exists
    os.makedirs(STORING_FOLDER, exist_ok=True)
    
    # Start WebSocket server and run forever
    start_server = websockets.serve(audio_receiver, WS_AUDIO_HOST, WS_AUDIO_PORT)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
