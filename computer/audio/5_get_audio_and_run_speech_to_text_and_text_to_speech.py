"""
ESP32 STT + TTS Pipeline
Streams audio from ESP32-WROVER, transcribes with Whisper, and responds using Coqui TTS.

Features:
- Whisper STT using Hugging Face transformers
- Real-time audio streaming and decoding
- Voice cloning with Coqui TTS
- Sends generated audio response back to ESP32
- Filters noise by checking word count
- Detects repeated transcripts and triggers stop via HTTP
"""

import io
import time
import torch
import asyncio
import requests
import websockets
import numpy as np
from TTS.api import TTS
from scipy.io import wavfile
from collections import deque
from pydub import AudioSegment
from transformers.pipelines.audio_utils import ffmpeg_read
from transformers import WhisperProcessor, WhisperForConditionalGeneration

# Configuration
USE_HOTSPOT = True                                                 # True for mobile hotspot, False for home WiFi
WS_AUDIO_HOST = '172.20.10.4' if USE_HOTSPOT else "192.168.1.174"  # WebSocket server IP
WS_AUDIO_PORT = 8888                                               # WebSocket port
INCOMING_FRAME_RATE = 16000                                        # Whisper requires 16kHz
CHANNELS = 1                                                       # Mono channel
BIT_DEPTH = 2                                                      # 16-bit audio
END_OF_AUDIO_SIGNAL = "END_OF_AUDIO"                               # Special string sent by ESP32 to end a recording
MIN_WORDS_THRESHOLD = 3                                            # Minimum word count to consider valid speech
STT_MODEL = "whisper-tiny"                                         # Whisper model variant
MAX_SAME_TRANSCRIPTS = 4                                           # Repeated transcript threshold before sending stop
CHECK_INTERVAL = 1                                                 # How often to re-check audio buffer (in seconds)
ESP32_WROVER_IP = "172.20.10.12" if USE_HOTSPOT else "192.168.1.182"

# Language + TTS settings
STT_AND_TTS_LANGUAGE = "en"
TTS_RESPONSE_AUDIO_PATH = "response.wav"
TTS_MODEL_PATH = "tts_models/multilingual/multi-dataset/your_tts"
TTS_CLONING_VOICE_PATH = "cloning_voice/_tmp_gradio_44ac4bb097f23e412e30c27466081ecb559cfe85_output.wav"
TTS_DEVICE = "cpu"

# Load Whisper model and processor
processor = WhisperProcessor.from_pretrained("openai/" + STT_MODEL)
model = WhisperForConditionalGeneration.from_pretrained("openai/" + STT_MODEL)

# Store recent transcripts for duplication detection
recent_transcripts = deque(maxlen=MAX_SAME_TRANSCRIPTS)

async def process_audio(audio_buffer):
    """
    Converts audio bytes to transcription and token probabilities.
    """
    audio_buffer.seek(0)
    buffer_size = audio_buffer.getbuffer().nbytes

    if buffer_size == 0:
        return "", []

    # Decode audio and convert to float32 format
    audio_segment = AudioSegment.from_raw(audio_buffer, sample_width=BIT_DEPTH, frame_rate=INCOMING_FRAME_RATE, channels=CHANNELS)
    wav_bytes = io.BytesIO()
    audio_segment.export(wav_bytes, format="wav")
    inputs = ffmpeg_read(wav_bytes.getvalue(), sampling_rate=INCOMING_FRAME_RATE)

    # Generate transcription
    input_features = processor(inputs, sampling_rate=INCOMING_FRAME_RATE, language=STT_AND_TTS_LANGUAGE, return_tensors="pt").input_features
    with torch.no_grad():
        outputs = model.generate(
            input_features, 
            max_new_tokens=256, 
            return_dict_in_generate=True, 
            output_scores=True
        )

    # Decode predictions
    transcription = processor.batch_decode(outputs.sequences, skip_special_tokens=True)[0]

    # Extract token probabilities
    token_probs = []
    for step_scores in outputs.scores:
        step_probs = torch.nn.functional.softmax(step_scores[0], dim=-1)
        top_prob, top_token = step_probs.max(dim=-1)
        token = processor.decode([top_token.item()])
        token_probs.append((token, top_prob.item()))

    # Log probabilities
    for token, prob in token_probs:
        print(f"  {token}: {prob:.4f}")

    return transcription, token_probs

def create_audio(text):
    """
    Uses Coqui TTS to synthesize a WAV file from text input.
    """
    tts = TTS(TTS_MODEL_PATH).to(TTS_DEVICE)
    tts.tts_to_file(
        text=text,
        language=STT_AND_TTS_LANGUAGE,
        speaker_wav=TTS_CLONING_VOICE_PATH,
        file_path=TTS_RESPONSE_AUDIO_PATH
    )
    return TTS_RESPONSE_AUDIO_PATH

async def process_full_transcript(transcript, websocket):
    """
    Handles valid transcripts by generating and sending audio.
    """
    num_words = len(transcript.split())
    if num_words >= MIN_WORDS_THRESHOLD:
        print(f"Transcript meets minimum word threshold: {transcript}")
        audio_path = create_audio(transcript)
        await send_audio_to_esp32(websocket, audio_path)
    else:
        print(f"MIN_WORDS_THRESHOLD not met ({num_words} words)")

async def send_audio_to_esp32(websocket, audio_file_path):
    """
    Sends a WAV file back to the ESP32 in chunks over WebSocket.
    """
    try:
        sample_rate, audio_data = wavfile.read(audio_file_path)

        # Ensure format is 16-bit PCM
        if audio_data.dtype != np.int16:
            audio_data = (audio_data * 32767).astype(np.int16)

        audio_bytes = audio_data.tobytes()

        # Send in chunks
        chunk_size = 8 * 4096
        for i in range(0, len(audio_bytes), chunk_size):
            await websocket.send(audio_bytes[i:i + chunk_size])

        await websocket.send(END_OF_AUDIO_SIGNAL)
        print("Audio sent successfully over websockets")
    except Exception as e:
        print(f"Failed to send audio: {str(e)}")

def send_stop_recording(ip: str) -> dict:
    """
    Sends an HTTP request to the ESP32 to stop KY-037 sound-triggered recording.
    """
    esp32_stop_url = f"http://{ip}/stopRecording"
    print(f"Sending stop recording command to {esp32_stop_url}")
    data = {'stop': 'true'}
    headers = {'Content-Type': 'application/x-www-form-urlencoded'}
    
    try:
        response = requests.post(esp32_stop_url, data=data, headers=headers, timeout=5)
        return {"success": True, "message": response.text}
    except requests.RequestException as e:
        return {"success": False, "message": str(e)}

async def audio_receiver(websocket, path):
    """
    Handles audio stream from ESP32:
    - Buffers incoming data
    - Periodically transcribes using Whisper
    - Sends TTS reply if valid
    """
    audio_buffer = io.BytesIO()
    last_process_time = time.time()
    final_transcript = ""

    async def reset_buffer():
        nonlocal audio_buffer
        audio_buffer.close()
        audio_buffer = io.BytesIO()

    try:
        async for message in websocket:
            if message == END_OF_AUDIO_SIGNAL:
                final_transcript, _ = await process_audio(audio_buffer)
                print(f"Final transcript: {final_transcript}")
                await process_full_transcript(final_transcript, websocket)
                await reset_buffer()
                continue

            # Accumulate audio chunks
            audio_buffer.write(message)
            current_time = time.time()

            # Periodic check
            if current_time - last_process_time >= CHECK_INTERVAL:
                transcript, token_probs = await process_audio(audio_buffer)
                if transcript:
                    final_transcript = transcript
                    recent_transcripts.append(transcript)

                    print(f"Current transcript: {transcript}")
                    print("Token probabilities:")
                    for token, prob in token_probs:
                        print(f"  {token}: {prob:.4f}")

                # Detect repeated transcripts
                if len(recent_transcripts) == MAX_SAME_TRANSCRIPTS and len(set(recent_transcripts)) == 1:
                    print(f"{MAX_SAME_TRANSCRIPTS} consecutive identical transcripts detected. Sending stop signal.")
                    result = send_stop_recording(ESP32_WROVER_IP)
                    if result["success"]:
                        print("Stop recording command sent successfully")
                        await process_full_transcript(final_transcript, websocket)
                        await reset_buffer()
                    else:
                        print(f"Failed to send stop recording command: {result['message']}")

                last_process_time = current_time

    except websockets.exceptions.ConnectionClosedError:
        print("Websocket connection closed unexpectedly.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        audio_buffer.close()
        recent_transcripts.clear()

# Start WebSocket server
start_server = websockets.serve(audio_receiver, WS_AUDIO_HOST, WS_AUDIO_PORT)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
