"""
Speech-to-Text (STT) with Whisper

Runs OpenAI Whisper on a saved audio file to transcribe speech to text.

Features:
- Supports different Whisper model sizes (tiny to large-v3)
- Automatically detects audio file duration
- Benchmarks processing time
- Estimates whether input is voice or noise by word count
"""

import time
from transformers.pipelines.audio_utils import ffmpeg_read
from transformers import WhisperProcessor, WhisperForConditionalGeneration

# Configuration
SHORT_AND_QUIET = True                                    # True to use short_and_quiet.wav, False for long_and_loud.wav
MODEL = "whisper-large-v3-turbo"                          # Whisper model variant (e.g., tiny, small, medium, large)

# Performance notes for short_and_quiet.wav
# whisper-tiny       ~0.45s
# whisper-small      ~1.12s
# whisper-medium     ~2.83s
# whisper-large-v2   ~6.01s

# Performance notes for long_and_loud.wav
# whisper-tiny       ~0.66s
# whisper-small      ~2.39s
# whisper-medium     ~6.18s
# whisper-large-v2   ~11.26s

# Load Whisper processor and model from Hugging Face Hub
processor = WhisperProcessor.from_pretrained("openai/" + MODEL)
model = WhisperForConditionalGeneration.from_pretrained("openai/" + MODEL)

# Select audio path based on config
if SHORT_AND_QUIET:
    audio_path = "./voice_for_stt/short_and_quiet.wav"
else:
    audio_path = "./voice_for_stt/long_and_loud.wav"

# Set sample rate expected by the processor (usually 16000 Hz)
sampling_rate = processor.feature_extractor.sampling_rate

# Read audio file as bytes
with open(audio_path, "rb") as f:
    inputs = f.read()

# Decode bytes into array (using ffmpeg)
inputs = ffmpeg_read(inputs, sampling_rate=sampling_rate)

# Start STT timing
start_time = time.time()

# Prepare input features for Whisper
input_features = processor(inputs, sampling_rate=sampling_rate, return_tensors="pt").input_features

# Generate token predictions from audio input
predicted_ids = model.generate(input_features, max_new_tokens=256)

# Decode token IDs to text
transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)

# End STT timing
end_time = time.time()
time_taken = end_time - start_time

# Estimate word count (basic noise vs voice check)
num_words = len(transcription[0].split())

# Print results
print(f"Time taken for STT: {time_taken:.2f} seconds")
print(f"Number of words in transcription: {num_words}")
print("Transcription:")
print(transcription)
