/*
 * ESP32 Audio & Robot Controller
 * Captures audio from INMP441, streams it via WebSocket,
 * plays received audio through MAX98357A, and handles robot commands.
 * 
 * Features:
 * - Dual WiFi network support with static IPs
 * - Async web server for robot control via HTTP
 * - WebSocket audio streaming and playback
 * - Sound detection trigger using KY-037
 * - I2S audio input (microphone) and output (amplifier)
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include <driver/i2s.h>
#include <freertos/semphr.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoWebsockets.h>
#include <freertos/FreeRTOS.h>

// UART2
HardwareSerial mySerial(2);

// Buffers
#define bufferCnt 10
#define bufferLen 1024

// INMP441 buffer
int16_t sBuffer[bufferLen];

// MAX98357A amplifier buffer
#define AUDIO_BUFFER_SIZE 32768
uint8_t audioBuffer[AUDIO_BUFFER_SIZE];
size_t audioBufferIndex = 0;

// INMP441 microphone
#define INMP441_SD 26 // Serial Data
#define INMP441_WS 19 // Word Select
#define INMP441_SCK 18 // Serial Clock

#define INMP441_PORT I2S_NUM_0

const unsigned long RECORDING_DURATION_MS = 30000;

// KY-037 sound sensor
#define KY037_PIN 15
volatile bool soundDetected = false;

// MAX98357A amplifier
#define MAX98357A_BCLK 14 // Bit Clock
#define MAX98357A_LRC 12 // Left/Right Clock
#define MAX98357A_DIN 25  // Digital Input
#define MAX98357A_SD 21 // Shutdown

#define MAX98357A_PORT I2S_NUM_1

// Semaphore for thread-safe access to the audio buffer
SemaphoreHandle_t audioBufferSemaphore = NULL;

// Flag to indicate if audio playback is in progress
volatile bool isPlayingAudio = false;

// Primary network configuration (home WiFi)
const char* ssid1 = "";                                 // Home WiFi's SSID
const char* password1 = "";                             // Home WiFi's password
IPAddress staticIP1(192, 168, 1, 182);                  // Static IP for home network
IPAddress gateway1(192, 168, 1, 1);                     // Gateway for home network
IPAddress subnet1(255, 255, 255, 0);                    // Subnet mask
const char* websocket_server_host1 = "192.168.1.174";   // Computer IP

// Fallback network configuration (phone hotspot)
const char* ssid2 = "";                                 // Phone hotspot's SSID
const char* password2 = "";                             // Phone hotspot's password
IPAddress staticIP2(172, 20, 10, 12);                   // Static IP for hotspot
IPAddress gateway2(172, 20, 10, 1);                     // Gateway for hotspot
IPAddress subnet2(255, 255, 255, 0);                    // Subnet mask
const char* websocket_server_host2 = "172.20.10.4";     // Computer IP

// Websockets
char websocket_server_host[16]; // Computer IP
const uint16_t websocket_server_port = 8888; // Computer port

using namespace websockets;
WebsocketsClient client;
bool isWebSocketConnected;

// Outgoing and incoming audio signal
#define END_OF_AUDIO_SIGNAL "END_OF_AUDIO"

volatile bool stopRecording = false;

/*
 * Installs I2S driver for INMP441 microphone
 */
void inmp441_i2s_install() {
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // RX for input
        .sample_rate = 16000, // for Whisper
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Mono
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0,
        .dma_buf_count = bufferCnt,
        .dma_buf_len = bufferLen,
        .use_apll = false
    };

    i2s_driver_install(INMP441_PORT, &i2s_config, 0, NULL);
}

/*
 * Assigns pins for INMP441 microphone
 */
void inmp441_i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = INMP441_SCK,
        .ws_io_num = INMP441_WS,
        .data_out_num = -1,
        .data_in_num = INMP441_SD
    };

    i2s_set_pin(INMP441_PORT, &pin_config);
}

/*
 * Installs I2S driver for MAX98357A amplifier
 */
void max98357a_i2s_install() {
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX), // TX for output
        .sample_rate = 16000,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Stereo
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = bufferCnt,
        .dma_buf_len = bufferLen,
        .use_apll = false
    };

    i2s_driver_install(MAX98357A_PORT, &i2s_config, 0, NULL);
}

/*
 * Assigns pins for MAX98357A amplifier
 */
void max98357a_i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = MAX98357A_BCLK,
        .ws_io_num = MAX98357A_LRC,
        .data_out_num = MAX98357A_DIN,
        .data_in_num = I2S_PIN_NO_CHANGE // No input for this configuration
    };

    i2s_set_pin(MAX98357A_PORT, &pin_config);
}

// KY-037 sound sensor
volatile bool allowRecording = true;
void IRAM_ATTR handleSoundDetection() {
    if (allowRecording) {
        soundDetected = true;
    }
}

// AsyncWebServer to receive commands from computer
AsyncWebServer server(80);

/*
 * Attempts to connect to WiFi network
 * Returns SSID if successful, or nullptr otherwise
 */
const char* connectToWiFi(const char* ssid, const char* password, IPAddress staticIP, IPAddress gateway, IPAddress subnet) {
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.config(staticIP, gateway, subnet);
    WiFi.begin(ssid, password);

    for (int i = 0; i < 10; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Connected!");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());

            mySerial.print("SSID:");
            mySerial.println(ssid);
            return ssid;
        }
        delay(1000);
        Serial.print(".");
    }
    Serial.println("Connection failed.");
    return nullptr;
}

/*
 * Handles HTTP POST command requests
 * Forwards control instructions to Uno over UART
 */
void handleCommand(AsyncWebServerRequest *request) {
    if (request->method() == HTTP_POST) {
        String command = "";
        
        if (request->hasParam("leftMD", true)) {
            Serial.print("leftMD:" + request->getParam("leftMD", true)->value());
            command += "leftMD:" + request->getParam("leftMD", true)->value() + ",";
        }
        if (request->hasParam("rightMD", true)) {
            Serial.print("rightMD:" + request->getParam("rightMD", true)->value());
            command += "rightMD:" + request->getParam("rightMD", true)->value() + ",";
        }
        if (request->hasParam("motorsS", true)) {
            Serial.print("motorsS:" + request->getParam("motorsS", true)->value());
            command += "motorsS:" + request->getParam("motorsS", true)->value() + ",";
        }
        if (request->hasParam("angleVP", true)) {
            Serial.print("angleVP:" + request->getParam("angleVP", true)->value());
            command += "angleVP:" + request->getParam("angleVP", true)->value() + ",";
        }
        if (request->hasParam("angleHP", true)) {
            Serial.print("angleHP:" + request->getParam("angleHP", true)->value());
            command += "angleHP:" + request->getParam("angleHP", true)->value();
        }
        
        if (command != "") {
            mySerial.println(command);
            request->send(200, "text/plain", "Received command: " + command);
        } else {
            request->send(400, "text/plain", "Parameters are missing");
        }
    } else {
        request->send(405, "text/plain", "Method Not Allowed");
    }
}

/*
 * Handles HTTP request to stop audio recording
 */
void handleStopRecording(AsyncWebServerRequest *request) {
    if (request->method() == HTTP_POST) {
        if (request->hasParam("stop", true)) {
            String stopValue = request->getParam("stop", true)->value();
            if (stopValue == "true") {
                stopRecording = true;
                Serial.println("Received stop recording command via HTTP");
                request->send(200, "text/plain", "Stop recording command received");
            } else {
                request->send(400, "text/plain", "Invalid stop value");
            }
        } else {
            request->send(400, "text/plain", "Missing 'stop' parameter");
        }
    } else {
        request->send(405, "text/plain", "Method Not Allowed");
    }
}

/*
 * WebSocket event handler
 * Updates connection status and prints ping/pong events
 */
void onEventsCallback(WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("Connection Opened");
        isWebSocketConnected = true;
    } else if (event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("Connection Closed");
        isWebSocketConnected = false;
    } else if (event == WebsocketsEvent::GotPing) {
        Serial.println("Got a Ping!");
    } else if (event == WebsocketsEvent::GotPong) {
        Serial.println("Got a Pong!");
    }
}

/*
 * WebSocket message handler
 * Handles incoming audio or end-of-audio signals
 */
void onMessageCallback(WebsocketsMessage message) {
    if(message.isBinary()) {
        if(audioBufferIndex == 0) {
            startAudio();
        }
        processAudioChunk((const uint8_t*)message.c_str(), message.length());
    } else if(message.isText()) {
        Serial.print("Received text: ");
        Serial.println(message.data());
        if(message.data() == END_OF_AUDIO_SIGNAL) {
            // Wait for any remaining audio to finish playing
            while (audioBufferIndex > 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            stopAudio();  // Stop audio and power down amplifier
        }
    }
}

/*
 * Processes incoming audio chunk into buffer
 * Triggers playback if buffer is full
 */
void processAudioChunk(const uint8_t* data, size_t length) {
    if (xSemaphoreTake(audioBufferSemaphore, portMAX_DELAY) == pdTRUE) {
        if (audioBufferIndex + length > AUDIO_BUFFER_SIZE) {
            // Buffer overflow, play what we have and reset
            playAudio();
            audioBufferIndex = 0;
        }
        memcpy(audioBuffer + audioBufferIndex, data, length);
        audioBufferIndex += length;
        xSemaphoreGive(audioBufferSemaphore);
    }
}

/*
 * Plays buffered audio through MAX98357A
 * Applies volume reduction before playback
 */
void playAudio() {
    if (xSemaphoreTake(audioBufferSemaphore, portMAX_DELAY) == pdTRUE) {
        if (audioBufferIndex > 0 && !isPlayingAudio) {
            isPlayingAudio = true;
            digitalWrite(MAX98357A_SD, HIGH);  // Enable the amplifier

            // Lower the volume to 50%
            for (size_t i = 0; i < audioBufferIndex; i += 2) {
                int16_t sample = ((int16_t*)audioBuffer)[i/2];
                sample = sample / 2;  // Reduce volume to 50%
                ((int16_t*)audioBuffer)[i/2] = sample;
            }
            
            size_t bytesWritten = 0;
            esp_err_t result = i2s_write(MAX98357A_PORT, audioBuffer, audioBufferIndex, &bytesWritten, portMAX_DELAY);
            if (result != ESP_OK) {
                Serial.printf("Error writing to I2S: %d\n", result);
            } else {
                Serial.printf("Wrote %d bytes to I2S\n", bytesWritten);
            }
            
            audioBufferIndex = 0;
            isPlayingAudio = false;
        }
        xSemaphoreGive(audioBufferSemaphore);
    }
}

/*
 * Starts I2S audio stream and enables amplifier
 */
void startAudio() {
    digitalWrite(MAX98357A_SD, HIGH);  // Enable the amplifier
    i2s_start(MAX98357A_PORT);
}

/*
 * Stops I2S audio and sends silence to power down amp
 */
void stopAudio() {
    // Send a short period of silence
    const size_t silenceLength = 1024;
    uint8_t silence[silenceLength] = {0};
    size_t bytesWritten = 0;
    i2s_write(MAX98357A_PORT, silence, silenceLength, &bytesWritten, portMAX_DELAY);
    
    // Stop I2S driver
    i2s_stop(MAX98357A_PORT);
    
    // Disable the amplifier
    digitalWrite(MAX98357A_SD, LOW);
}

/*
 * Background task to handle continuous audio playback
 */
void audioPlaybackTask(void* parameter) {
    while (1) {
        playAudio();
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent tight looping
    }
}

/*
 * Connects to WebSocket server with retries
 */
void connectWSServer() {
    client.onEvent(onEventsCallback);
    client.onMessage(&onMessageCallback);
    int attempt = 0;
    const int max_attempts = 5;
    while (!client.connect(websocket_server_host, websocket_server_port, "/") && attempt < max_attempts) {
        delay(500);
        Serial.print(".");
        attempt++;
    }
    if (attempt < max_attempts) {
        Serial.println("Websocket Connected!");
    } else {
        Serial.println("Failed to connect to Websocket.");
    }
}

/*
 * Task that captures audio and streams it when triggered
 */
void microphoneTask(void* parameter) {
    size_t bytesIn = 0;
    while (1) {
        // Check and reconnect Wi-Fi if disconnected
        if (WiFi.status() != WL_CONNECTED) {
            connectToWiFi(ssid2, password2, staticIP2, gateway2, subnet2);
            if (WiFi.status() != WL_CONNECTED) {
                connectToWiFi(ssid1, password1, staticIP1, gateway1, subnet1);
            }
        }

        if (soundDetected) {
            Serial.println("Sound detected. Recording...");
            if (!isWebSocketConnected) {
                connectWSServer();
            }

            size_t bytesIn = 0;
            int lastElapsedSecond = -1;
            unsigned long startTime = millis();
            int recordingDurationS = RECORDING_DURATION_MS / 1000;
            
            while (millis() - startTime < RECORDING_DURATION_MS) {
                if (stopRecording) {
                    Serial.println("Stopping recording...");
                    break;
                }
                if (!isWebSocketConnected) {
                    connectWSServer();
                }
                int elapsedSeconds = (millis() - startTime) / 1000;
                
                if (elapsedSeconds != lastElapsedSecond) {
                    mySerial.print("Listening (");
                    mySerial.print(recordingDurationS - elapsedSeconds);
                    mySerial.println("s)...");
                    lastElapsedSecond = elapsedSeconds;
                }

                esp_err_t result = i2s_read(INMP441_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
                if (result == ESP_OK && isWebSocketConnected) {
                    client.sendBinary((const char*)sBuffer, bytesIn);
                }

                yield();
            }

            if (!stopRecording) {
                const char* endOfRecordingSignal = END_OF_AUDIO_SIGNAL;
                client.send(endOfRecordingSignal);
            }
            mySerial.println("Thinking...");
            stopRecording = false;
            soundDetected = false;
        }

        if (client.available()) {
            client.poll();
        }
    }
}

/*
 * Setup function - runs once at boot
 * Initializes WiFi, I2S, WebSocket, and async server
 */
void setup() {
    // Initialize serial communication for Uno at 9600 baud rate
    mySerial.begin(9600, SERIAL_8N1, 12, 13); // 12 (RX), 13 (TX)
    // and for logging at 115200 baud rate
    Serial.begin(115200);

    // Connect to Wi-Fi
    const char* connectedSSID = nullptr;
    while (!connectedSSID) {
        // Attempt to connect to the primary WiFi network
        connectedSSID = connectToWiFi(ssid2, password2, staticIP2, gateway2, subnet2);

        // If connection to the primary network fails, try the secondary network
        if (!connectedSSID) {
            connectedSSID = connectToWiFi(ssid1, password1, staticIP1, gateway1, subnet1);
        }
    }

    // Set websocket_server_host based on the connected network
    if (strcmp(connectedSSID, ssid2) == 0) {
        strcpy(websocket_server_host, websocket_server_host2);
    } else {
        strcpy(websocket_server_host, websocket_server_host1);
    }

    // Setup for KY-037 sound sensor
    pinMode(KY037_PIN, INPUT_PULLUP);

    // Setup for INMP441 microphone
    inmp441_i2s_install();
    inmp441_i2s_setpin();
    i2s_start(INMP441_PORT);

    // Create semaphore for audio buffer
    audioBufferSemaphore = xSemaphoreCreateMutex();

    // Initialize MAX98357A SD pin
    pinMode(MAX98357A_SD, OUTPUT);
    digitalWrite(MAX98357A_SD, LOW);

    // Setup for MAX98357A amplifier
    max98357a_i2s_install();
    max98357a_i2s_setpin();
    i2s_start(MAX98357A_PORT);

    // Attach interrupt handler for KY-037 sound sensor
    attachInterrupt(digitalPinToInterrupt(KY037_PIN), handleSoundDetection, RISING);

    // Start microphone task
    xTaskCreatePinnedToCore(microphoneTask, "microphoneTask", 10000, NULL, 1, NULL, 1);

    // Create audio playback task
    xTaskCreatePinnedToCore(audioPlaybackTask, "audioPlayback", 4096, NULL, 1, NULL, 0);

    // Start web server
    server.on("/command", handleCommand);
    server.on("/stopRecording", handleStopRecording);
    server.begin();
}

/*
 * Main loop - unused (all tasks are async)
 */
void loop() {
}