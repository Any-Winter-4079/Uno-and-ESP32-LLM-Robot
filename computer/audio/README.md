# Notes on the `audio/` code:

## Overview

- These 5 scripts get audio from, and send audio to, the ESP32-WROVER via WebSocket, with speech-to-text (STT) and text-to-speech (TTS) inference using transformer's whisher(`WhisperForConditionalGeneration`) and coqui's `TTS`.

- `1_get_audio.py` – Streams raw audio from the ESP32 and saves it as a WAV file.

- `2_run_speech_to_text.py` – Runs Whisper STT on a saved WAV file.

- `3_get_audio_and_run_speech_to_text.py` – Combines ESP32 audio streaming with Whisper STT in real-time.

- `4_run_text_to_speech.py` – Converts a given text to speech using a TTS model.

- `5_get_audio_and_run_speech_to_text_and_text_to_speech.py` – Full pipeline for streaming audio, transcribing it, synthesizing a response, and sending audio back to the ESP32.

## Computer Setup

- Define in this line whether the robot and computer will share the phone hotspot (True) or the home WiFi (False) as the common network for communication (e.g., sending/receiving of audio):

```
USE_HOTSPOT = True
```

- Define in these lines the computer's IPs the phone hotspot or home WiFi will assign:

```
WS_AUDIO_HOST = '172.20.10.4' if USE_HOTSPOT else '192.168.1.174'
ESP32_WROVER_IP = '172.20.10.12' if USE_HOTSPOT else '192.168.1.182'
```

- Create `voice_for_stt/` and add a few test files, e.g., `short_and_quiet.wav` and `long_and_loud.wav`

- Create `cloning_voice/` and add one or a few voices to clone

## ESP32 Setup

- Make sure `esp32/wrover/production.ino` has been flashed to the ESP32-WROVER.

## Notes

- To see coqui's `TTS` models, run:

```
tts --list_models
```

- Whisper seems to require:

```
FRAME_RATE = 16000
```

- On Mac, enable MPS fallback:

```
export PYTORCH_ENABLE_MPS_FALLBACK=1
```

- And finally, make sure `ffmpeg` is installed and on the system path for `ffmpeg_read` to work.
