#include <Arduino.h>
#include <SPI.h>
//Joystick setup
#include "Adafruit_seesaw.h"

Adafruit_seesaw ss;

#define BUTTON_RIGHT 6
#define BUTTON_DOWN  7
#define BUTTON_LEFT  9
#define BUTTON_UP    10
#define BUTTON_SEL   14
uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) | 
                (1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_SEL);
//ESP-NOW SETUP
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0x0C, 0xDC, 0x7E, 0xCC, 0x6B, 0xB8};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  uint16_t joyX;
  uint16_t joyY;
  bool rightPressed;
  bool downPressed;
  bool leftPressed;
  bool upPressed;
  bool selPressed;
} struct_message;

struct_message joyData;

esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success) {
      Serial.printf("Sent x:%d y:%d\n", joyData.joyX, joyData.joyY);
    } else {
      Serial.println("Failed");
    }

}

bool readJoystick();
void sendJoystick();

//Joystick tracking variables
int last_x = 0, last_y = 0;
bool rightPressed = false;
bool downPressed = false;
bool leftPressed = false;
bool upPressed = false;
bool selPressed = false;

void setup(void){
  Serial.begin(115200);
    //ESP_NOW Setup
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // ESP-NOW Setup Complete


  //Initialize Joystick
  ss.begin(0x49);
  ss.pinModeBulk(button_mask, INPUT_PULLUP);
  ss.setGPIOInterrupts(button_mask, 1);    


}
//minimum time between sending data
long sendDataDelay = 50; //Millis
long lastSendData = 0;

void loop(){
    //Send data when the joystick is ready
    if (readJoystick() && millis()-lastSendData > sendDataDelay){
        sendJoystick();
        
        lastSendData = millis();
    }
}

//Sends the joystick readings to the broadcast address
void sendJoystick(){
  // Set values to send
  joyData.joyX = last_x;
  joyData.joyY = last_y;
  joyData.rightPressed = rightPressed;
  joyData.downPressed = downPressed;
  joyData.leftPressed = leftPressed;
  joyData.upPressed = upPressed;
  joyData.selPressed = selPressed;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joyData, sizeof(joyData));
}

//Reads the current joystick values and updates the tracking variables
//Returns true if the values have changed
bool readJoystick(){
    //Assume no reading has occured
    bool newReading = false;
    //get newest reading from the joystick
    int x = ss.analogRead(2);
    int y = ss.analogRead(3);
    //If it moved, update the position of the joystick
    if ( (abs(x - last_x) > 3)  ||  (abs(y - last_y) > 3)) {
        newReading = true;
        last_x = x;
        last_y = y;
    }    
    uint32_t buttons = ss.digitalReadBulk(button_mask);
    rightPressed = !(buttons & (1 << BUTTON_RIGHT));
    downPressed = !(buttons & (1 << BUTTON_DOWN));
    leftPressed = !(buttons & (1 << BUTTON_LEFT));
    upPressed = !(buttons & (1 << BUTTON_UP));
    selPressed = ! (buttons & (1 << BUTTON_SEL));
    //If a button was pressed, show that there was a new reading
    if (rightPressed || downPressed || leftPressed || upPressed || selPressed){
        newReading = true;
    }
    return newReading;
}