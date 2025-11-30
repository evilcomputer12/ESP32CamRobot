#include "esp_camera.h" 
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems
#include "esp_http_server.h"

#define PART_BOUNDARY "123456789000000000000987654321"

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

// On AI Thinker, flash LED is usually GPIO 4
#define LED_GPIO_NUM 4

// Soft AP credentials
const char* ap_ssid     = "ESP32-Robot";
const char* ap_password = "12345678";

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// ================== HTML: video + drive joystick/arrow modes + cam arrows ==================
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32-CAM Robot</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <style>
    * {
      -webkit-touch-callout: none;
      -webkit-user-select: none;
      user-select: none;
    }

    html, body {
      margin: 0;
      padding: 0;
      height: 100%;
      width: 100%;
      overflow: hidden;
      font-family: Arial, sans-serif;
      background: #000;
      color: #fff;
    }

    #rotateWarning {
      position: fixed;
      top: 0; left: 0;
      width: 100%;
      height: 100%;
      display: none;
      align-items: center;
      justify-content: center;
      text-align: center;
      background: #000;
      color: #fff;
      padding: 20px;
      box-sizing: border-box;
      z-index: 20;
    }

    #app {
      position: relative;
      width: 100vw;
      height: 100vh;
      overflow: hidden;
      display: none;
    }

    /* Portrait: show warning only */
    @media (orientation: portrait) {
      #rotateWarning { display: flex; }
      #app { display: none; }
    }

    /* Landscape: show app */
    @media (orientation: landscape) {
      #rotateWarning { display: none; }
      #app { display: block; }
    }

    /* Fullscreen video background */
    #videoContainer {
      position: absolute;
      top: 0; left: 0;
      width: 100%;
      height: 100%;
      overflow: hidden;
      z-index: 1;
    }

    #videoContainer img {
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: block;
    }

    /* Overlay HUD layer */
    #hud {
      position: absolute;
      top: 0; left: 0;
      width: 100%;
      height: 100%;
      z-index: 5;
      pointer-events: none;
    }

    .joy-container {
      position: absolute;
      bottom: 15%;
      width: 40%;
      max-width: 260px;
      display: flex;
      flex-direction: column;
      align-items: center;
      pointer-events: auto;
    }

    #driveContainer {
      left: 5%;
    }

    #camContainer {
      right: 5%;
    }

    .title {
      font-size: 14px;
      margin-bottom: 4px;
      text-shadow: 0 0 4px #000;
    }

    /* Drive joystick */
    .joystick {
      width: 160px;
      height: 160px;
      border-radius: 50%;
      border: 2px solid rgba(255,255,255,0.6);
      position: relative;
      touch-action: none;
      background: rgba(0,0,0,0.35);
      backdrop-filter: blur(2px);
    }

    .stick {
      width: 50px;
      height: 50px;
      border-radius: 50%;
      border: 2px solid rgba(255,255,255,0.7);
      background: rgba(255,255,255,0.4);
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
    }

    .info {
      font-size: 11px;
      margin-top: 4px;
      color: #eee;
      text-shadow: 0 0 3px #000;
    }

    /* LED toggle button */
    #ledButton {
      position: absolute;
      top: 5%;
      left: 50%;
      transform: translateX(-50%);
      padding: 8px 16px;
      background: rgba(0,0,0,0.5);
      border-radius: 20px;
      border: 1px solid rgba(255,255,255,0.7);
      color: #fff;
      font-size: 14px;
      cursor: pointer;
      pointer-events: auto;
      text-shadow: 0 0 3px #000;
    }

    #ledButton.on {
      background: rgba(255,255,0,0.65);
      color: #000;
      border-color: rgba(255,255,255,0.9);
    }

    /* Mode toggle button for drive */
    #driveModeButton {
      margin-bottom: 6px;
      padding: 4px 10px;
      background: rgba(0,0,0,0.5);
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,0.7);
      color: #fff;
      font-size: 11px;
      cursor: pointer;
    }

    /* Camera & drive arrows grid */
    .cam-buttons, .drive-buttons {
      display: grid;
      grid-template-columns: 50px 50px 50px;
      grid-template-rows: 50px 50px 50px;
      gap: 6px;
      justify-content: center;
      align-items: center;
    }

    .cam-buttons button, .drive-buttons button {
      width: 50px;
      height: 50px;
      border-radius: 10px;
      border: 1px solid rgba(255,255,255,0.8);
      background: rgba(0,0,0,0.5);
      color: #fff;
      font-size: 20px;
      cursor: pointer;
      touch-action: manipulation;
    }

    .cam-buttons button:active,
    .drive-buttons button:active {
      background: rgba(255,255,255,0.7);
      color: #000;
    }

    #hint {
      position: absolute;
      bottom: 2%;
      left: 50%;
      transform: translateX(-50%);
      font-size: 11px;
      color: #eee;
      text-shadow: 0 0 3px #000;
      pointer-events: none;
    }

    .hidden {
      display: none;
    }
  </style>
</head>
<body>
  <div id="rotateWarning">
    <div>
      <h1>Please rotate your device</h1>
      <p>This control interface is designed for landscape mode.</p>
    </div>
  </div>

  <div id="app">
    <div id="videoContainer">
      <img src="" id="photo" alt="Camera stream">
    </div>

    <div id="hud">
      <div id="ledButton" onclick="toggleLED()">LED OFF</div>

      <!-- DRIVE: mode toggle + joystick OR arrows -->
      <div id="driveContainer" class="joy-container">
        <button id="driveModeButton" onclick="toggleDriveMode()">Mode: Joystick</button>

        <div class="title">Drive</div>

        <!-- Joystick wrapper -->
        <div id="driveJoystickWrapper">
          <div id="driveJoystick" class="joystick">
            <div id="driveStick" class="stick"></div>
          </div>
          <div class="info" id="driveInfo">X: 0 Y: 0 Z: 0</div>
        </div>

        <!-- Arrow buttons wrapper (hidden by default) -->
        <div id="driveButtonsWrapper" class="hidden">
          <div class="drive-buttons">
            <div></div>
            <button
              onpointerdown="driveArrowDown(event,'forward')"
              onpointerup="driveArrowUp(event)"
              onpointercancel="driveArrowUp(event)"
            >&#9650;</button>
            <div></div>

            <button
              onpointerdown="driveArrowDown(event,'left')"
              onpointerup="driveArrowUp(event)"
              onpointercancel="driveArrowUp(event)"
            >&#9664;</button>
            <div></div>
            <button
              onpointerdown="driveArrowDown(event,'right')"
              onpointerup="driveArrowUp(event)"
              onpointercancel="driveArrowUp(event)"
            >&#9654;</button>

            <div></div>
            <button
              onpointerdown="driveArrowDown(event,'back')"
              onpointerup="driveArrowUp(event)"
              onpointercancel="driveArrowUp(event)"
            >&#9660;</button>
            <div></div>
          </div>
          <div class="info" id="driveInfoArrows">Hold arrows to move</div>
        </div>
      </div>

      <!-- CAMERA: 4-way button pad -->
      <div id="camContainer" class="joy-container">
        <div class="title">Camera</div>
        <div class="cam-buttons">
          <div></div>
          <button onclick="camCommand('up')">&#9650;</button>
          <div></div>
          <button onclick="camCommand('left')">&#9664;</button>
          <div></div>
          <button onclick="camCommand('right')">&#9654;</button>
          <div></div>
          <button onclick="camCommand('down')">&#9660;</button>
          <div></div>
        </div>
        <div class="info" id="camInfo">Tap arrows to nudge</div>
      </div>

      <div id="hint">
        Mode: Joystick/Arrows, WASD = drive, Arrow keys = camera
      </div>
    </div>
  </div>

  <script>
    const driveJoy   = document.getElementById('driveJoystick');
    const driveStick = document.getElementById('driveStick');
    const driveInfo  = document.getElementById('driveInfo');
    const driveModeButton = document.getElementById('driveModeButton');
    const driveJoystickWrapper = document.getElementById('driveJoystickWrapper');
    const driveButtonsWrapper  = document.getElementById('driveButtonsWrapper');

    const camInfo = document.getElementById('camInfo');

    let driveMode = 'joystick'; // or 'arrows'

    const driveState = {
      active: false,
      x: 0,
      y: 0,
      z: 0
    };

    let ledOn = false;

    // ------- DRIVE SEND / STOP HELPERS -------
    function sendDrive(x, y, z) {
      fetch(`/drive?x=${x}&y=${y}&z=${z}`).catch(_ => {});
    }

    function stopDrive() {
      driveState.x = 0;
      driveState.y = 0;
      driveState.z = 0;
      driveInfo.innerText = 'X: 0 Y: 0 Z: 0';
      sendDrive(0, 0, 0);
    }

    function toggleDriveMode() {
      if (driveMode === 'joystick') {
        driveMode = 'arrows';
        driveModeButton.textContent = 'Mode: Arrows';
        driveJoystickWrapper.classList.add('hidden');
        driveButtonsWrapper.classList.remove('hidden');
        stopDrive();
      } else {
        driveMode = 'joystick';
        driveModeButton.textContent = 'Mode: Joystick';
        driveJoystickWrapper.classList.remove('hidden');
        driveButtonsWrapper.classList.add('hidden');
        stopDrive();
      }
    }

    function setupJoystick(joyElem, stickElem, state, infoElem) {
      let centerX = 0;
      let centerY = 0;
      let radius = 0;

      joyElem.addEventListener('pointerdown', e => {
        if (driveMode !== 'joystick') return;
        e.preventDefault();
        joyElem.setPointerCapture(e.pointerId);
        const rect = joyElem.getBoundingClientRect();
        centerX = rect.left + rect.width / 2;
        centerY = rect.top + rect.height / 2;
        radius = rect.width / 2;
        state.active = true;
        updateStick(e);
      });

      joyElem.addEventListener('pointermove', e => {
        if (!state.active || driveMode !== 'joystick') return;
        e.preventDefault();
        updateStick(e);
      });

      joyElem.addEventListener('pointerup', e => {
        if (driveMode !== 'joystick') return;
        e.preventDefault();
        state.active = false;
        stickElem.style.transform = 'translate(-50%, -50%)';
        stopDrive();
      });

      joyElem.addEventListener('pointercancel', e => {
        if (driveMode !== 'joystick') return;
        e.preventDefault();
        state.active = false;
        stickElem.style.transform = 'translate(-50%, -50%)';
        stopDrive();
      });

      function updateStick(e) {
        const dx = e.clientX - centerX;
        const dy = e.clientY - centerY;

        let dist = Math.sqrt(dx*dx + dy*dy);
        let clampedDx = dx;
        let clampedDy = dy;
        if (dist > radius) {
          const scale = radius / dist;
          clampedDx *= scale;
          clampedDy *= scale;
          dist = radius;
        }

        stickElem.style.transform =
          `translate(calc(-50% + ${clampedDx}px), calc(-50% + ${clampedDy}px))`;

        state.x = clampedDx / radius;   // -1..1
        state.y = -clampedDy / radius;  // -1..1
        const mag = dist / radius;      // 0..1
        state.z = mag;

        const xi = Math.round(state.x * 100);
        const yi = Math.round(state.y * 100);
        const zi = Math.round(state.z * 100);
        infoElem.innerText = `X: ${xi} Y: ${yi} Z: ${zi}`;
      }
    }

    setupJoystick(driveJoy, driveStick, driveState, driveInfo);

    // LED toggle
    function toggleLED() {
      ledOn = !ledOn;
      const btn = document.getElementById('ledButton');
      btn.textContent = ledOn ? 'LED ON' : 'LED OFF';
      if (ledOn) btn.classList.add('on');
      else btn.classList.remove('on');

      const state = ledOn ? 'on' : 'off';
      fetch(`/led?state=${state}`).catch(_ => {});
    }

    // Camera command (single nudge)
    function camCommand(dir) {
      let x = 0, y = 0;
      if (dir === 'up')    y = 100;
      if (dir === 'down')  y = -100;
      if (dir === 'left')  x = -100;
      if (dir === 'right') x = 100;

      camInfo.innerText = `Last: ${dir.toUpperCase()}`;
      fetch(`/cam?x=${x}&y=${y}`).catch(_ => {});
    }

    // DRIVE ARROWS HANDLERS
    function driveArrowDown(ev, dir) {
      if (driveMode !== 'arrows') return;
      ev.preventDefault();

      switch (dir) {
        case 'forward':
          driveState.x = 0;
          driveState.y = 1;
          driveState.z = 1;
          break;
        case 'back':
          driveState.x = 0;
          driveState.y = -1;
          driveState.z = 1;
          break;
        case 'left':
          driveState.x = -1;
          driveState.y = 0;
          driveState.z = 1;
          break;
        case 'right':
          driveState.x = 1;
          driveState.y = 0;
          driveState.z = 1;
          break;
      }
    }

    function driveArrowUp(ev) {
      if (driveMode !== 'arrows') return;
      ev.preventDefault();
      stopDrive();
    }

    // Keyboard control
    document.addEventListener('keydown', (e) => {
      if (e.repeat) return;

      switch (e.key.toLowerCase()) {
        case 'w':
          driveState.y = 1;
          driveState.z = 1;
          break;
        case 's':
          driveState.y = -1;
          driveState.z = 1;
          break;
        case 'a':
          driveState.x = -1;
          driveState.z = 1;
          break;
        case 'd':
          driveState.x = 1;
          driveState.z = 1;
          break;
      }

      switch (e.key) {
        case 'ArrowUp':
          camCommand('up');
          break;
        case 'ArrowDown':
          camCommand('down');
          break;
        case 'ArrowLeft':
          camCommand('left');
          break;
        case 'ArrowRight':
          camCommand('right');
          break;
      }

      if (
        ['w', 'a', 's', 'd'].includes(e.key.toLowerCase()) ||
        ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)
      ) {
        e.preventDefault();
      }
    });

    document.addEventListener('keyup', (e) => {
      switch (e.key.toLowerCase()) {
        case 'w':
        case 's':
          driveState.y = 0;
          driveState.z = 0;
          break;
        case 'a':
        case 'd':
          driveState.x = 0;
          driveState.z = 0;
          break;
      }
    });

    // send drive state every 100 ms
    setInterval(() => {
      const x = Math.round(driveState.x * 100);
      const y = Math.round(driveState.y * 100);
      const z = Math.round(driveState.z * 100);
      sendDrive(x, y, z);
    }, 100);

    window.onload = function() {
      const loc = window.location;
      const streamUrl = loc.protocol + '//' + loc.hostname + ':81/stream';
      document.getElementById("photo").src = streamUrl;
    };

    // ===== EXTRA SAFETY: STOP WHEN DIALOG / TAB CHANGE / CONTEXT MENU =====
    window.addEventListener('contextmenu', function(e) {
      e.preventDefault();
      stopDrive();
    });

    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        stopDrive();
      }
    });

    window.addEventListener('blur', () => {
      stopDrive();
    });

    window.addEventListener('beforeunload', () => {
      stopDrive();
    });

    window.addEventListener('pagehide', () => {
      stopDrive();
    });
  </script>
</body>
</html>
)rawliteral";

// ============== HTTP HANDLERS =========================

static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    
    if(fb->format != PIXFORMAT_JPEG){
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if(!jpeg_converted){
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
        if(_jpg_buf){
          free(_jpg_buf);
          _jpg_buf = NULL;
        }
        break;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }
    
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    
    if(res != ESP_OK){
      break;
    }
  }
  
  return res;
}

// /drive?x=..&y=..&z=..
static esp_err_t drive_handler(httpd_req_t *req){
  char buf[64];
  char value[16];

  int x = 0, y = 0, z = 0;

  size_t len = httpd_req_get_url_query_len(req) + 1;
  if (len > sizeof(buf)) len = sizeof(buf);
  if (len > 1) {
    if (httpd_req_get_url_query_str(req, buf, len) == ESP_OK) {
      if (httpd_query_key_value(buf, "x", value, sizeof(value)) == ESP_OK) {
        x = atoi(value);
      }
      if (httpd_query_key_value(buf, "y", value, sizeof(value)) == ESP_OK) {
        y = atoi(value);
      }
      if (httpd_query_key_value(buf, "z", value, sizeof(value)) == ESP_OK) {
        z = atoi(value);
      }
    }
  }

  if (x < -100) x = -100; if (x > 100) x = 100;
  if (y < -100) y = -100; if (y > 100) y = 100;
  if (z < 0) z = 0;       if (z > 100) z = 100;

  char msg[32];
  int n = snprintf(msg, sizeof(msg), "D %d %d %d\n", x, y, z);
  Serial.write((uint8_t*)msg, n);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// /cam?x=..&y=..
static esp_err_t cam_handler(httpd_req_t *req){
  char buf[64];
  char value[16];

  int x = 0, y = 0;

  size_t len = httpd_req_get_url_query_len(req) + 1;
  if (len > sizeof(buf)) len = sizeof(buf);
  if (len > 1) {
    if (httpd_req_get_url_query_str(req, buf, len) == ESP_OK) {
      if (httpd_query_key_value(buf, "x", value, sizeof(value)) == ESP_OK) {
        x = atoi(value);
      }
      if (httpd_query_key_value(buf, "y", value, sizeof(value)) == ESP_OK) {
        y = atoi(value);
      }
    }
  }

  if (x < -100) x = -100; if (x > 100) x = 100;
  if (y < -100) y = -100; if (y > 100) y = 100;

  char msg[24];
  int n = snprintf(msg, sizeof(msg), "C %d %d\n", x, y);
  Serial.write((uint8_t*)msg, n);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// /led?state=on or off
static esp_err_t led_handler(httpd_req_t *req){
  char buf[32];
  char value[8] = {0};

  size_t len = httpd_req_get_url_query_len(req) + 1;
  if (len > sizeof(buf)) len = sizeof(buf);

  if (len > 1) {
    if (httpd_req_get_url_query_str(req, buf, len) == ESP_OK) {
      if (httpd_query_key_value(buf, "state", value, sizeof(value)) == ESP_OK) {
        if (strcmp(value, "on") == 0) {
          digitalWrite(LED_GPIO_NUM, HIGH);
        } else {
          digitalWrite(LED_GPIO_NUM, LOW);
        }
      }
    }
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  const char resp[] = "OK";
  return httpd_resp_send(req, resp, strlen(resp));
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t drive_uri = {
    .uri       = "/drive",
    .method    = HTTP_GET,
    .handler   = drive_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t cam_uri = {
    .uri       = "/cam",
    .method    = HTTP_GET,
    .handler   = cam_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t led_uri = {
    .uri       = "/led",
    .method    = HTTP_GET,
    .handler   = led_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &drive_uri);
    httpd_register_uri_handler(camera_httpd, &cam_uri);
    httpd_register_uri_handler(camera_httpd, &led_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.begin(115200);   // to Nano
  Serial.setDebugOutput(false);

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(ap_ssid, ap_password);
  if (!apStarted) {
    Serial.println("SoftAP start failed!");
  } else {
    IPAddress ip = WiFi.softAPIP();
    Serial.println("");
    Serial.println("AP started");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("Connect and go to: http://");
    Serial.println(ip);
  }

  startCameraServer();
}

void loop() {
  // all handled by HTTP + Serial
}
