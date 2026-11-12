/** BLUEDOT ASSISTANT - ALTERNATING AUDIO/CAMERA
  Board: AI-Thinker ESP32-CAM (classic ESP32)
  Arduino-ESP32 core: 3.x

  Modes:
    CAMERA: OV2640 MJPEG stream at http://ESP32_IP:81/stream
    RECORD: camera HTTP server stops, Wi-Fi is disabled, GPIO13 ADC2 records
            LM358 analog audio, then Wi-Fi reconnects and PCM is sent to PC.
    PLAY:   Python sends 16 kHz mono PCM16 to TCP port 5001; GPIO14 PWM
            feeds PAM8403 through an RC low-pass filter.

  Python control TCP port: 5003
    Send: REC\n
  Python motor-command TCP port: 5004
    Send: F / B / L / R / S / V0..255 followed by newline.
    ESP32 forwards validated commands to UNO R4 over UART0 (GPIO1 TX, GPIO3 RX).

  ESP32 -> Python audio TCP port: 5002
    4-byte big-endian payload length, followed by signed PCM16 LE mono 8 kHz.
  Python -> ESP32 playback TCP port: 5001
    signed PCM16 LE mono 16 kHz, then close connection.

  IMPORTANT:
    - GPIO13 is ADC2. Wi-Fi is explicitly stopped during recording.
    - Camera is deinitialized during recording and restarted afterward.
    - This is alternating capture, NOT simultaneous camera and mic.
    - GPIO13 must receive a safe 0..3.3V signal, AC audio biased within range.
    - Do not connect a 5V LM358 output directly to GPIO13.
    - Camera uses GPIO32 as PWDN; GPIO32 is not used for microphone.
    - GPIO14 is used for PWM audio; do not use microSD.
*/

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"

// -------- USER SETTINGS --------
const char* WIFI_SSID = "Galaxy A14 5G";
const char* WIFI_PASSWORD = "shweta@07";
const char* PC_IP = "192.168.31.118";  // Set to your computer's LAN IPv4 address

// -------- NETWORK --------
static const uint16_t PLAY_PORT = 5001;
static const uint16_t AUDIO_UPLOAD_PORT = 5002;
static const uint16_t CONTROL_PORT = 5003;
static const uint16_t MOTOR_COMMAND_PORT = 5004;
static const uint32_t UNO_UART_BAUD = 9600;

// -------- AUDIO --------
static const int MIC_ADC_PIN = 13;       // ADC2; Wi-Fi OFF while sampling
static const int AUDIO_PWM_PIN = 14;     // PWM -> RC filter -> PAM8403 input
static const uint32_t MIC_RATE = 8000;   // 8 kHz mono capture
static const uint32_t PLAY_RATE = 16000; // 16 kHz mono playback
static const uint32_t RECORD_MS = 4000;  // 4 seconds per request
static const uint32_t PWM_CARRIER = 100000;
static const uint8_t PWM_BITS = 8;
static const uint8_t AUDIO_PWM_CHANNEL = 1;
static const size_t MIC_SAMPLES = (MIC_RATE * RECORD_MS) / 1000;

// -------- AI-Thinker camera pin map --------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

WiFiServer controlServer(CONTROL_PORT);
WiFiServer motorCommandServer(MOTOR_COMMAND_PORT);
WiFiServer playServer(PLAY_PORT);
httpd_handle_t cameraHttpd = nullptr;

static bool cameraStarted = false;
static int16_t micPcm[MIC_SAMPLES];

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;

    char header[96];
    int hlen = snprintf(header, sizeof(header),
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      (unsigned int)fb->len);

    res = httpd_resp_send_chunk(req, header, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    vTaskDelay(1);
  }
  return res;
}

static bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count = psramFound() ? 2 : 1;
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&c) != ESP_OK) {
    (void)0;
    return false;
  }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  cfg.stack_size = 8192;

  httpd_uri_t uri = {};
  uri.uri = "/stream";
  uri.method = HTTP_GET;
  uri.handler = streamHandler;

  if (httpd_start(&cameraHttpd, &cfg) != ESP_OK) {
    esp_camera_deinit();
    return false;
  }
  if (httpd_register_uri_handler(cameraHttpd, &uri) != ESP_OK) {
    httpd_stop(cameraHttpd);
    cameraHttpd = nullptr;
    esp_camera_deinit();
    return false;
  }
  cameraStarted = true;
  (void)0;
  return true;
}

static void stopCamera() {
  if (cameraHttpd) {
    httpd_stop(cameraHttpd);
    cameraHttpd = nullptr;
  }
  if (cameraStarted) {
    esp_camera_deinit();
    cameraStarted = false;
  }
}

static bool connectWiFi(uint32_t timeoutMs = 20000) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
  }
  if (WiFi.status() == WL_CONNECTED) {
    (void)0;
    return true;
  }
  (void)0;
  return false;
}

static void disconnectWiFiForAdc() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

static void recordMicrophone() {
  (void)0;

  // GPIO13 is ADC2; Wi-Fi must be fully disabled before analogRead.
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_ADC_PIN, ADC_11db);

  const uint32_t intervalUs = 1000000UL / MIC_RATE;
  int64_t nextUs = esp_timer_get_time();
  for (size_t i = 0; i < MIC_SAMPLES; ++i) {
    while (esp_timer_get_time() < nextUs) {}
    uint16_t raw = analogRead(MIC_ADC_PIN);
    micPcm[i] = (int16_t)(((int32_t)raw - 2048) * 16);
    nextUs += intervalUs;
  }
  (void)0;
}

static bool sendRecordedPcm() {
  WiFiClient client;
  (void)0;
  if (!client.connect(PC_IP, AUDIO_UPLOAD_PORT)) {
    (void)0;
    return false;
  }
  client.setNoDelay(true);

  uint32_t payloadLen = (uint32_t)MIC_SAMPLES * sizeof(int16_t);
  uint8_t lenHeader[4] = {
    (uint8_t)(payloadLen >> 24),
    (uint8_t)(payloadLen >> 16),
    (uint8_t)(payloadLen >> 8),
    (uint8_t)(payloadLen)
  };
  client.write(lenHeader, sizeof(lenHeader));

  const uint8_t* ptr = (const uint8_t*)micPcm;
  size_t remaining = payloadLen;
  while (remaining && client.connected()) {
    size_t chunk = remaining > 1024 ? 1024 : remaining;
    size_t written = client.write(ptr, chunk);
    if (written == 0) break;
    ptr += written;
    remaining -= written;
  }
  client.stop();
  (void)0;
  return remaining == 0;
}

static void handleRecordRequest() {
  (void)0;
  stopCamera();
  disconnectWiFiForAdc();

  recordMicrophone();

  if (!connectWiFi()) {
    ESP.restart();
  }
  sendRecordedPcm();

  startCamera();
  (void)0;
}

static void playPcm(WiFiClient &client) {
  (void)0;
  client.setNoDelay(true);
  uint8_t buf[512];
  bool pending = false;
  uint8_t low = 0;
  int64_t nextUs = esp_timer_get_time();

  while (client.connected() || client.available() > 0) {
    int n = client.read(buf, sizeof(buf));
    if (n <= 0) { delay(1); continue; }
    int i = 0;

    if (pending && n > 0) {
      int16_t s = (int16_t)((uint16_t)low | ((uint16_t)buf[0] << 8));
      uint8_t duty = (uint8_t)(((int32_t)s + 32768) >> 8);
      while (esp_timer_get_time() < nextUs) {}
      ledcWrite(AUDIO_PWM_CHANNEL, duty);
      nextUs += 1000000LL / PLAY_RATE;
      pending = false;
      i = 1;
    }

    for (; i + 1 < n; i += 2) {
      int16_t s = (int16_t)((uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8));
      uint8_t duty = (uint8_t)(((int32_t)s + 32768) >> 8);
      while (esp_timer_get_time() < nextUs) {}
      ledcWrite(AUDIO_PWM_CHANNEL, duty);
      nextUs += 1000000LL / PLAY_RATE;
    }
    if (i < n) { low = buf[i]; pending = true; }
  }
  ledcWrite(AUDIO_PWM_CHANNEL, 128);
  client.stop();
  (void)0;
}

static bool forwardMotorCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (!(cmd == "F" || cmd == "B" || cmd == "L" || cmd == "R" || cmd == "S" ||
        (cmd.length() >= 2 && cmd.length() <= 4 && cmd[0] == 'V'))) {
    return false;
  }
  if (cmd[0] == 'V') {
    for (size_t i = 1; i < cmd.length(); ++i) {
      if (!isDigit(cmd[i])) return false;
    }
    int speed = cmd.substring(1).toInt();
    if (speed < 0 || speed > 255) return false;
  }
  // UART0 is reassigned to the UNO link after boot. GPIO1=TX, GPIO3=RX.
  Serial.print(cmd);
  Serial.write('\n');
  Serial.flush();
  return true;
}

void setup() {
  // UART0 is the wired motor link to UNO R4: GPIO1 TX -> UNO D0 RX,
  // GPIO3 RX <- UNO D1 TX through a 3.3V/5V level shifter.
  // Do not use Serial Monitor debugging on this UART after startup.
  Serial.begin(UNO_UART_BAUD, SERIAL_8N1, 3, 1);
  delay(1000);

  ledcSetup(AUDIO_PWM_CHANNEL, PWM_CARRIER, PWM_BITS);
  ledcAttachPin(AUDIO_PWM_PIN, AUDIO_PWM_CHANNEL);
  ledcWrite(AUDIO_PWM_CHANNEL, 128);

  if (!connectWiFi()) {
    delay(5000);
    ESP.restart();
  }

  if (!startCamera()) {
  }

  controlServer.begin();
  motorCommandServer.begin();
  playServer.begin();
  controlServer.setNoDelay(true);
  playServer.setNoDelay(true);

  (void)0;
  (void)0;
  (void)0;
  (void)0;
  (void)0;
}

void loop() {
  WiFiClient control = controlServer.available();
  if (control) {
    String command = control.readStringUntil('\n');
    command.trim();
    control.println("OK");
    control.stop();

    if (command.equalsIgnoreCase("REC")) {
      handleRecordRequest();
    }
  }

  WiFiClient motorClient = motorCommandServer.available();
  if (motorClient) {
    motorClient.setTimeout(1000);
    String motorCmd = motorClient.readStringUntil('\n');
    bool accepted = forwardMotorCommand(motorCmd);
    motorClient.println(accepted ? "OK" : "ERR");
    motorClient.stop();
  }

  WiFiClient playback = playServer.available();
  if (playback) {
    playPcm(playback);
  }

  delay(2);
}
