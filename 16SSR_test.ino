/********************************************************************************************************
* This code allow a teensy micro controller to be remote controlled through an HC-06 bluetooth module and
* set the duty cycle of Solid State Relay (SSR) via pulse width modulation.
* The SSR can have its duty cycle set directly.
**********************************************************************************************************/
#include <vector>
#include <tuple>
#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include <SoftwareSerial.h>

#define USING_MICROS_RESOLUTION true
#include "Teensy_Slow_PWM.h"

#define HW_TIMER_INTERVAL_MS        0.01f
#define HW_TIMER_INTERVAL_FREQ      100000L

#define PIN_COUNT 16

// Used to indicate errors in function returns.
#define ERROR_TEMP -6666.0
#define EARLIEST_TIME 1577836800

// Indicates which pins of the Teensy are connected to the serial pins of the HC-06
const int blueRx = 0;
const int blueTx = 1;
SoftwareSerial hc(blueRx,blueTx);

const String teensyID = "Magis_SSR_Teensy";
String bluetoothBuffer = "";
String message = "";

int ssr_index = 0;
float ssr_df;

char print_buf[250]; // For printing errors easily

int pins[] = {
  3,
  2,
  5,
  4,
  7,
  6,
  9,
  8,
  11,
  10,
  14,
  12,
  18,
  15,
  22,
  19
};

int channels[PIN_COUNT];
float dutyCycles[PIN_COUNT];

float freq = 5.0f;

// Init Teensy timer TEENSY_TIMER_1
TeensyTimer ITimer(TEENSY_TIMER_1);
// Init Teensy_SLOW_PWM, each can service 16 different ISR-based PWM channels
Teensy_SLOW_PWM ISR_PWM;

void TimerHandler()
{
  ISR_PWM.run();
}

void setup() {

  hc.begin(9600);
  while(hc.available())
  {
    hc.read();
  }

  Serial.begin(9600);
  Serial.println("Testing Magis SSR");
  Serial.println(teensyID);

  for (int i = 0; i < PIN_COUNT; i++) {
    pinMode(pins[i], OUTPUT);
    dutyCycles[i] = 0.0f;
    channels[i] = ISR_PWM.setPWM(pins[i], freq, dutyCycles[i]);
  }

  if(ITimer.attachInterrupt(HW_TIMER_INTERVAL_FREQ, TimerHandler)) {
    Serial.println("Timer is good");
  } else {
    Serial.println("Timer is bad");
  }
}

bool heartbeat = true;

extern float tempmonGetTemp(void);

void sendHeartBeatPacket() {
  std::ostringstream buf;
  buf << '?' << heartbeat << '&';
  for (int i = 0; i < PIN_COUNT; i++) {
    buf << dutyCycles[i] << '&';
  }
  buf << tempmonGetTemp() << '\n';
  hc.write(buf.str().c_str());
  Serial.write(buf.str().c_str());
  heartbeat = !heartbeat;
}


void loop() {
  
  while(hc.available()) {
    bluetoothBuffer = hc.readString();
    Serial.println(bluetoothBuffer); 
      if(bluetoothBuffer.substring(0,3) == "SSR"){
        message=bluetoothBuffer.substring(bluetoothBuffer.lastIndexOf("R")+1,bluetoothBuffer.lastIndexOf("="));
        Serial.println(message);
        ssr_index = message.toInt();
        dutyCycles[ssr_index] = bluetoothBuffer.substring(bluetoothBuffer.lastIndexOf("=")+1).toFloat();
        ISR_PWM.modifyPWMChannel(channels[ssr_index], pins[ssr_index], freq, dutyCycles[ssr_index]);
      } else {
        message = "Incorrect settings received:" + bluetoothBuffer;
        Serial.println(message);
      }
//    parseBluetoothBuffer(bluetoothBuffer);
//    numberOfSecondsWithoutMessage = 0;
  }
//  sprintf(print_buf, "%d seconds without a message.", numberOfSecondsWithoutMessage);
//  Serial.println(print_buf);


  sendHeartBeatPacket();
  delay(1000);

}
