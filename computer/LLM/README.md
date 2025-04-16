##

Once:

- `arduino/production.ino`
- `esp32/cam/aithinker-production.ino`
- `esp32/cam/m5stackwide-production.ino`
- `esp32/wrover/production.ino`

have been flashed to the Arduino Uno, ESP32-CAMs and ESP32-WROVER.

Once all of the test sketches and scripts have been run and all instructions followed, having added:

- `depth_and_face_recognition/coco.names`
- `depth_and_face_recognition/yolov8n.pt`
- `face_recognition/production_database/`
- `LMM/.env` with:
  - OPENAI_API_KEY=yourKey
- `audio/cloning_voice/`

And having created -via running scripts-:

- `calibration/images/`
- `undistortion_and_rectification/stereo_maps/`

Clone llama.cpp into `computer/LLM` and `git checkout` the same commit if needed:

```
cd computer/LLM
git clone https://github.com/ggml-org/llama.cpp.git
cd llama.cpp
git checkout 213701b5
```

Place your model(s) under `llama.cpp/models`, e.g.,

```
llama.cpp/models/Phi-3-mini-4k-instruct-gguf/Phi-3-mini-4k-instruct-fp16.gguf
llama.cpp/models/Phi-3-mini-4k-instruct-gguf/Phi-3-mini-4k-instruct-q4.gguf
```

Test running the model(s):

```
make -j && ./llama-cli -m models/Phi-3-mini-4k-instruct-gguf/Phi-3-mini-4k-instruct-fp16.gguf -p "Building a website can be done in 10 simple steps:\nStep 1:" -n 400 -e
```

```
python CoT_Dec_PAL_tester_v1.py
python CoT_Dec_PAL_tester_v2.py
python CoT_Dec_PAL_tester_v3.py
```

```
cd Transformer.codes
npm i
npm run dev
```

Note `Transformer.codes` uses a cruip.com template, not (fully) shared.
