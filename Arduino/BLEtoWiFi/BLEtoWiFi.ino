/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

// CHANGE YOUR WIFI CREDENTIALS!
//char* WIFI_SSID = "NETGEAR05";
//char* WIFI_PASS = "wuxiaohua1011";

#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <uri/UriBraces.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <EEPROM.h> 
//#include "driver/adc.h" // possibly needed to disable WiFi ? I don't think so

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
//-------------------for EEPROM--------------------------------------
#define OFFSET 1 // Start SSID after identifier byte
#define MAXSSID 50 // MAX characters in SSID
#define MAXPASS 50 // MAX character in PASS
#define EEPROMSIZE (MAXSSID + MAXPASS + OFFSET) // Total EEPROM size
// -----------------for BLE-------------------------------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a9"
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue = 0;
char WIFI_SSID[MAXSSID] = "empty";
char WIFI_PASS[MAXPASS] = "empty";
static int j = 1; // tracking what step we are on in WiFi setup

//----------------------ESP32 CAM pins---------------------------
#define THROTTLE_PIN 14
#define STEERING_PIN 2
#define FLASH_LED_PIN 4
#define RED_LED_PIN 33
#define LED 4

//----------------------------For Camera---------------------------------

const uint8_t fps = 10;    //sets minimum delay between frames, HW limits of ESP32 allows about 12fps @ 800x600
static auto loRes = esp32cam::Resolution::find(320, 240);
//static auto loRes = esp32cam::Resolution::find(640, 480);
const char HANDSHAKE_START = '(';



WebServer server(80);
AsyncWebServer asyncws(81);
AsyncWebSocket ws("/ws");
//--------------------------Servo---------------------------------
Servo throttleServo;
Servo steeringServo;
volatile int32_t ws_throttle_read = 1500;
volatile int32_t ws_steering_read = 1500;


volatile bool isClientConnected = false;
bool isFlashLightOn = false;
bool isRedLEDOn = false;


//------------------------BLE Callbacks---------------------------

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
     deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    
    
    if(rxValue.length() > 0){
        int rxLength = rxValue.length();
        Serial.println("Start Receiving");
        Serial.print("Received Value: ");
        if(j == 1){
          j++;
          for (char i = 0; i < rxLength; i++) {
            if(i > MAXSSID){
              Serial.print("Your SSID is too long for my little memory :)");
              break;
            }
            EEPROM.write(i + OFFSET, rxValue[i]); // Store SSID in permanent memory
            WIFI_SSID[i] = rxValue[i];
            Serial.print(rxValue[i]);
          }
          for(char i = rxLength; i < MAXSSID; i++) {
            EEPROM.write(i + OFFSET, 0); // clear the rest of the EEPROM
            WIFI_SSID[i] = 0;
          }
        }
        else if(j == 2){
          j++;
          for (char i = 0; i < rxLength; i++) {
            if(i > MAXPASS){
              Serial.print("Your password is too long for my little memory :)");
              break;
            }
            WIFI_PASS[i] = rxValue[i];
            EEPROM.write(i + MAXSSID + OFFSET, rxValue[i]); // Store PASS in permanent memory
            Serial.print(rxValue[i]);
          }
          for(char i = rxLength; i < MAXPASS; i++) {
            EEPROM.write(i + MAXSSID + OFFSET, 0); // clear the rest of the EEPROM
            WIFI_PASS[i] = 0;
          }
        }
        Serial.println();
     }    
  }
};
//-----------------------------------START HERE-----------------------------------
void setup() {
  Serial.begin(115200);
  setupCamera(); // setup camera only once
  EEPROM.begin(EEPROMSIZE);
//  EEPROM.write(0,0); // Comment this out for normal functionality. Using this line disables loading from memory
  if(EEPROM.read(0) == 1){
    loadMemory();
    if(initializeSystem() == 0){
      j = 4;
    } else {
      j = 1;
      initializeBLE();
    }
  } else {
    initializeBLE();
  }
}
//---------------------------------------LOOP----------------------------------------
void loop() {

//----------Print current WiFi SSID and PASS----------------------------
  int i = 0;
  Serial.println("WiFi SSID");
  while(WIFI_SSID[i] != NULL && WIFI_SSID[i] < 256){
    Serial.print(WIFI_SSID[i]);
    i++;
  }
  Serial.println();
  Serial.println("WiFi Password");
  i = 0;
  while(WIFI_PASS[i] != NULL){
    Serial.print(WIFI_PASS[i]);
    i++;
  }
  Serial.println();
  delay(2000);

//------------Original while loop from esp_32_cam code---------------
  if(j == 3){
    BLEDevice::deinit(false); // deinitialized BLE so we can use WiFi and use BLE again later. 
    EEPROM.write(0, 1); // tells the system there are valid WiFi credentials on startup
    EEPROM.commit(); // Saves new values for SSID and PASS to permanent memory
    delay(1000);
    j++;
    if(initializeSystem()){ // if wifi doesn't setup return and reinitialize BLE
      j = 1;
      initializeBLE();
    }
  }
  while(j == 4){ // If everything went well with setup run this forever
    server.handleClient();
    ws.cleanupClients();

    writeToServo(); 
  }
  
}

void loadMemory() {
  for(char i = 0; i < MAXSSID; i++){
    if(EEPROM.read(i + OFFSET) != 0)
      WIFI_SSID[i] = EEPROM.read(i + OFFSET); 
  }
  for(char i = 0; i < MAXPASS; i++){
    if(EEPROM.read(i + MAXSSID + OFFSET) != 0)
      WIFI_PASS[i] = EEPROM.read(i + MAXSSID + OFFSET); 
  }
  
}

//---------------initialize BLE

void initializeBLE(){
  Serial.println("Starting BLE!");
  pinMode(LED, OUTPUT);
  //Create Device
  BLEDevice::init("ROAR ESP32 - Adam");
  Serial.println("Made it to 1");
  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  Serial.println("Made it to 2");
  pServer->setCallbacks(new MyServerCallbacks());
  //Create Service
  BLEService *pService = pServer->createService(SERVICE_UUID); 
  //Create BLE Characteristic
  Serial.println("Made it to 3");
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_TX,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  Serial.println("Made it to 4");
  pCharacteristic->addDescriptor(new BLE2902());

  Serial.println("Made it to 5");
  //-----Characteristic for the receiving end
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_RX,
                                          BLECharacteristic::PROPERTY_WRITE
                                         );
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  Serial.println("Made it to 6");
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Made it to 7");
// BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
//  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//  pAdvertising->addServiceUUID(SERVICE_UUID);
//  pAdvertising->setScanResponse(true);
//  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
//  pAdvertising->setMinPreferred(0x12);
//  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}
//---------------initialize system after BLE receives WiFi SSID and PASS

int initializeSystem(){
  if(setupWifi()){
    return 1;
  } // wifi failed to setup go back to begining
  setupRoutes();
  setupServo();
  initWebSocket();
  return 0;
}

//------------------esp_32_cam code


void writeToServo() {
   steeringServo.writeMicroseconds(ws_steering_read);    // tell servo to go to position 'steering'
   throttleServo.writeMicroseconds(ws_throttle_read);    // tell motor to drive with power = motorPower   
}

// setup functions

void initWebSocket() {
    asyncws.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", "Hello, world");
  });
  asyncws.begin();
  ws.onEvent(onEvent);
  asyncws.addHandler(&ws);
}

void setupServo() {

  ESP32PWM::timerCount[0]=4;
  ESP32PWM::timerCount[1]=4;
  throttleServo.setPeriodHertz(50);
  throttleServo.attach(THROTTLE_PIN, 1000, 2000);
  steeringServo.setPeriodHertz(50);    // standard 50 hz servo
  steeringServo.attach(STEERING_PIN, 1000, 2000); // attaches the servo on pin (whatever you assign)
}

void setupRoutes() {
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("  /cam-lo.jpg");
  Serial.println("  /cam.mjpeg");
  Serial.println("  /cmd/<THROTTLE>,<STEERING>");

  server.on("/cam-lo.jpg", handleJpgLo);
  server.on("/cam.mjpeg", handleMjpeg);

  server.on(UriBraces("/cmd/{}"), handleCmd);
  server.begin();
}

void disableWifi(){
  Serial.println("WiFi failed, try entering your credentials again!");
  WiFi.disconnect(true);
  //adc_power_off();
  WiFi.mode(WIFI_OFF);
}

int setupWifi() {
  int h = 0;
  int maxattempts = 20;
  WiFi.disconnect(true);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    blinkRedLED();
    Serial.print(".");
    h++;
    if(h > maxattempts){
      disableWifi();
      return 1;
    }
  }
  digitalWrite(RED_LED_PIN, LOW);
  Serial.println("Connected!");
  return 0;
}

void setupCamera() {
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(loRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(10);
    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }
}

// handle routes
void handleCmd() {
  String argument = server.pathArg(0);
  
  char buf[argument.length()] = "\0";
  argument.toCharArray(buf, argument.length());
  char *token = strtok(buf, ",");
  if (token != NULL) {
    unsigned int curr_throttle_read = atoi(token+1);
    if (curr_throttle_read >= 1000 and curr_throttle_read <= 2000) {
      ws_throttle_read = curr_throttle_read;
    } 
  }
  token = strtok(NULL, ",");
  if (token != NULL) {
    unsigned int curr_steering_read = atoi(token);
    if (curr_steering_read >= 1000 and curr_steering_read <= 2000) {
      ws_steering_read = curr_steering_read;
    }
  } 
  Serial.print(ws_throttle_read);
  Serial.print(" ");
  Serial.print(ws_steering_read);
  Serial.println();
  server.send(200, "text/plain","ack");
}

void handleJpgLo()
{
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  }
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

void handleMjpeg() {
  Serial.println("STREAM BEGIN");
  WiFiClient client = server.client();
  auto startTime = millis();
  int res = esp32cam::Camera.streamMjpeg(client);
  if (res <= 0) {
    Serial.printf("STREAM ERROR %d\n", res);
    return;
  }
  auto duration = millis() - startTime;
  Serial.printf("STREAM END %dfrm %0.2ffps\n", res, 1000.0 * res / duration);
}


// Utility functions
void blinkFlashlight() {
  if (isFlashLightOn) {
    ledcWrite(FLASH_LED_PIN, 0);
    isFlashLightOn = false;
  } else {
    ledcWrite(FLASH_LED_PIN, 100);
    isFlashLightOn = true;
  }
}

void blinkRedLED() {
  if (isRedLEDOn) {
    digitalWrite(RED_LED_PIN, HIGH);
    isRedLEDOn = false;
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    isRedLEDOn = true;
  }
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  char buf[len] = "\0";
  memcpy(buf, data, len);
  
  char *token = strtok(buf, ",");
    if (token[0] == HANDSHAKE_START) {
      if (token != NULL) {
        unsigned int curr_throttle_read = atoi(token + 1);
        if (curr_throttle_read >= 1000 and curr_throttle_read <= 2000) {
          ws_throttle_read = curr_throttle_read;
        } 
      }
      token = strtok(NULL, ",");
      if (token != NULL) {
        unsigned int curr_steering_read = atoi(token);
        if (curr_steering_read >= 1000 and curr_steering_read <= 2000) {
          ws_steering_read = curr_steering_read;
        }
      }
  } 
  Serial.print(ws_throttle_read);
  Serial.print(" ");
  Serial.print(ws_steering_read);
  Serial.println();
}
