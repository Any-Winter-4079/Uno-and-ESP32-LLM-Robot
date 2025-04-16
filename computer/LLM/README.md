# Notes on the `LLM/` code:

## Computer Setup

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

- `undistortion_and_rectification/stereo_maps/`

Clone llama.cpp into `computer/LLM` and `git checkout` the same commit if needed:

```
cd computer/LLM
git clone https://github.com/ggml-org/llama.cpp.git
cd llama.cpp
git checkout 213701b5
```

Place your model(s) under `llama.cpp/models`, e.g.:

```
llama.cpp/models/Phi-3-mini-4k-instruct-gguf/Phi-3-mini-4k-instruct-fp16.gguf
llama.cpp/models/Phi-3-mini-4k-instruct-gguf/Phi-3-mini-4k-instruct-q4.gguf
```

And test running the model(s):

```
make -j && ./llama-cli -m models/Phi-3-mini-4k-instruct-gguf/Phi-3-mini-4k-instruct-fp16.gguf -p "Building a website can be done in 10 simple steps:\nStep 1:" -n 400 -e
```

## CoT/Dec/PAL/etc. Inference Tests

Then, to test inference methods, run:

```
python CoT_Dec_PAL_tester_v1.py
python CoT_Dec_PAL_tester_v2.py
python CoT_Dec_PAL_tester_v3.py
```

## Full Robot Execution

## Notes

- Note to deploy the LLM-controlled app, you can run:

```
cd Transformer.codes
npm i
npm run dev
```

- Keep in mind, to save time `Transformer.codes/` uses an adapted template from a lifelong [Cruip](https://cruip.com) subscription whose `public/`, `view/` code cannnot be openly shared. Feel free to add an open version if you're great at web design!
