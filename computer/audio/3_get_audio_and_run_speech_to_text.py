"""
ESP32 Audio + Whisper STT
Receives audio from ESP32-WROVER, transcribes it using Whisper, and prints result.

Features:
- Streams raw audio over WebSocket
- Converts to WAV format
- Transcribes using OpenAI Whisper (Hugging Face)
- Filters out short/noisy audio via word count
"""

import io
import time
import asyncio
import websockets
from pydub import AudioSegment
from transformers.pipelines.audio_utils import ffmpeg_read
from transformers import WhisperProcessor, WhisperForConditionalGeneration

# Network configuration
USE_HOTSPOT = True                                                 # True for phone hotspot, False for home WiFi
WS_AUDIO_HOST = '172.20.10.4' if USE_HOTSPOT else "192.168.1.174"  # Server IP address
WS_AUDIO_PORT = 8888                                               # WebSocket port

# Audio configuration
FRAME_RATE = 16000                                                 # Required for Whisper
CHANNELS = 1                                                       # Mono audio
BIT_DEPTH = 2                                                      # 16-bit PCM
END_OF_AUDIO_SIGNAL = "END_OF_AUDIO"                               # Marker sent by ESP32

# STT configuration
MIN_WORDS_THRESHOLD = 2                                            # Ignore transcription if too short
MODEL = "whisper-tiny"                                             # Whisper model size

# Load Whisper model and processor
processor = WhisperProcessor.from_pretrained("openai/" + MODEL)
model = WhisperForConditionalGeneration.from_pretrained("openai/" + MODEL)

async def audio_receiver(websocket, path):
    """
    Receives raw audio via WebSocket, runs Whisper STT, and prints transcription.

    Args:
        websocket: WebSocket connection from ESP32
        path: WebSocket path (unused)
    """
    audio_buffer = io.BytesIO()

    try:
        async for message in websocket:
            if message == END_OF_AUDIO_SIGNAL:
                # Finalize current audio buffer
                audio_buffer.seek(0)

                # Convert raw audio to WAV format using pydub
                audio_segment = AudioSegment.from_raw(
                    audio_buffer,
                    sample_width=BIT_DEPTH,
                    frame_rate=FRAME_RATE,
                    channels=CHANNELS
                )
                wav_bytes = io.BytesIO()
                audio_segment.export(wav_bytes, format="wav")

                # Decode WAV into float array for Whisper
                inputs = ffmpeg_read(wav_bytes.getvalue(), sampling_rate=FRAME_RATE)

                # Start timing
                start_time = time.time()

                # Convert audio into model input format
                input_features = processor(inputs, sampling_rate=FRAME_RATE, return_tensors="pt").input_features

                # Generate text token IDs from audio
                predicted_ids = model.generate(input_features, max_new_tokens=256)

                # Decode token IDs to text
                transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)

                # End timing
                end_time = time.time()
                time_taken = end_time - start_time

                # Count words to help filter out noise
                num_words = len(transcription[0].split())

                print(f"Time taken for STT: {time_taken:.2f} seconds")
                print(f"Number of words in transcription: {num_words}")

                if num_words >= MIN_WORDS_THRESHOLD:
                    print("Transcription:", transcription)
                else:
                    print(f"Ignored: Too few words ({num_words})")

                # Clear buffer for next segment
                audio_buffer.seek(0)
                audio_buffer.truncate(0)
            else:
                # Accumulate audio stream into buffer
                audio_buffer.write(message)

    except websockets.exceptions.ConnectionClosedError:
        print("WebSocket connection closed unexpectedly.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        audio_buffer.close()

# Start WebSocket server
start_server = websockets.serve(audio_receiver, WS_AUDIO_HOST, WS_AUDIO_PORT)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
