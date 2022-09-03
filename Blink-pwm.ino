/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.
 */

#include "Teensy_Slow_PWM.h"

#define HW_TIMER_INTERVAL_MS        0.01f
#define HW_TIMER_INTERVAL_FREQ      100000L

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int ssr0 = 3;
int ssr1 = 2;
int ssr2 = 5;
int ssr3 = 4;
int ssr4 = 7;
int ssr5 = 6;
int ssr6 = 9;
int ssr7 = 8;
int ssr8 = 11;
int ssr9 = 10;
int ssr10 = 14;
int ssr11 = 12;
int ssr12 = 18;
int ssr13 = 15;
int ssr14 = 22;
int ssr15 = 19;

// Init Teensy timer TEENSY_TIMER_1
TeensyTimer ITimer(TEENSY_TIMER_1);

// Init Teensy_SLOW_PWM, each can service 16 different ISR-based PWM channels
Teensy_SLOW_PWM ISR_PWM;

void TimerHandler()
{
  ISR_PWM.run();
}

int channelNum0;
int channelNum1;
int channelNum2;
int channelNum3;
int channelNum4;
int channelNum5;
int channelNum6;
int channelNum7;
int channelNum8;
int channelNum9;
int channelNum10;
int channelNum11;
int channelNum12;
int channelNum13;
int channelNum14;
int channelNum15;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);
  // initialize the digital pin as an output.
  pinMode(ssr0, OUTPUT);
  pinMode(ssr1, OUTPUT);
  pinMode(ssr2, OUTPUT);
  pinMode(ssr3, OUTPUT);
  pinMode(ssr4, OUTPUT);
  pinMode(ssr5, OUTPUT);
  pinMode(ssr6, OUTPUT);
  pinMode(ssr7, OUTPUT);
  pinMode(ssr8, OUTPUT);
  pinMode(ssr9, OUTPUT);
  pinMode(ssr10, OUTPUT);
  pinMode(ssr11, OUTPUT);
  pinMode(ssr12, OUTPUT);
  pinMode(ssr13, OUTPUT);
  pinMode(ssr14, OUTPUT);
  pinMode(ssr15, OUTPUT);

  float freq = 5.0f;
  float duty_cycle = 0.0;

  channelNum0 = ISR_PWM.setPWM(ssr0, freq, duty_cycle);
  channelNum1 = ISR_PWM.setPWM(ssr1, freq, duty_cycle);
  channelNum2 = ISR_PWM.setPWM(ssr2, freq, duty_cycle);
  channelNum3 = ISR_PWM.setPWM(ssr3, freq, duty_cycle);
  channelNum4 = ISR_PWM.setPWM(ssr4, freq, duty_cycle);
  channelNum5 = ISR_PWM.setPWM(ssr5, freq, duty_cycle);
  channelNum6 = ISR_PWM.setPWM(ssr6, freq, duty_cycle);
  channelNum7 = ISR_PWM.setPWM(ssr7, freq, duty_cycle);
  channelNum8 = ISR_PWM.setPWM(ssr8, freq, duty_cycle);
  channelNum9 = ISR_PWM.setPWM(ssr9, freq, duty_cycle);
  channelNum10 = ISR_PWM.setPWM(ssr10, freq, duty_cycle);
  channelNum11 = ISR_PWM.setPWM(ssr11, freq, duty_cycle);
  channelNum12 = ISR_PWM.setPWM(ssr12, freq, duty_cycle);
  channelNum13 = ISR_PWM.setPWM(ssr13, freq, duty_cycle);
  channelNum14 = ISR_PWM.setPWM(ssr14, freq, duty_cycle);
  channelNum15 = ISR_PWM.setPWM(ssr15, freq, duty_cycle);

  if(ITimer.attachInterrupt(HW_TIMER_INTERVAL_FREQ, TimerHandler)) {
    Serial.println("Timer is good");
    Serial.println(channelNum0);
    Serial.println(channelNum1);
    Serial.println(channelNum2);
    Serial.println(channelNum3);
    Serial.println(channelNum4);
    Serial.println(channelNum5);
    Serial.println(channelNum6);
    Serial.println(channelNum7);
    Serial.println(channelNum8);
    Serial.println(channelNum9);
    Serial.println(channelNum10);
    Serial.println(channelNum11);
    Serial.println(channelNum12);
    Serial.println(channelNum13);
    Serial.println(channelNum14);
    Serial.println(channelNum15);
  } else {
    Serial.println("Timer is bad");
  }
  

//  analogWriteFrequency(ssr0,5);
//  analogWriteFrequency(ssr1,5);
//  analogWriteFrequency(ssr2,5);
//  analogWriteFrequency(ssr3,5);
//  analogWriteFrequency(ssr4,5);
//  analogWriteFrequency(ssr5,5);
//  analogWriteFrequency(ssr6,5);
//  analogWriteFrequency(ssr7,5);
//  analogWriteFrequency(ssr8,5);
//  analogWriteFrequency(ssr9,5);
//  analogWriteFrequency(ssr10,5);
//  analogWriteFrequency(ssr11,5);
//  analogWriteFrequency(ssr12,5);
//  analogWriteFrequency(ssr13,5);
//  analogWriteFrequency(ssr14,5);
//  analogWriteFrequency(ssr15,5);

}

bool last_cycle_set = false;
int last_cycle;
// in microseconds
int cycle_time = 200000;
// 50% power
int on_time = cycle_time / 4;

bool duty_switch = false;
float freq = 5.0f;

// the loop routine runs over and over again forever:
void loop() {
//  if (!last_cycle_set) {
//    last_cycle_set = true;
//    last_cycle = micros();
//  }
//
//  int curr_time = micros();
//  int diff;
//  while (true) {
//    diff = curr_time - last_cycle;
//    if (diff >= cycle_time) {
//      last_cycle += cycle_time;
//    } else {
//      break;
//    }
//  }
//
//  if (diff <= on_time) {
//    digitalWrite(ssr0, 1);
//  } else {
//    digitalWrite(ssr0, 0);
//  }
  
//  analogWrite(ssr0, 127);  
//  analogWrite(ssr1, 127);
//  analogWrite(ssr2, 127);  
//  analogWrite(ssr3, 60);
//  analogWrite(ssr4, 127);  
//  analogWrite(ssr5, 60);
//  analogWrite(ssr6, 127);  
//  analogWrite(ssr7, 60);
//  analogWrite(ssr8, 127);  
//  analogWrite(ssr9, 60);
//  analogWrite(ssr10, 127);  
//  analogWrite(ssr11, 60);
//  analogWrite(ssr12, 127);  
//  analogWrite(ssr13, 60);
//  analogWrite(ssr14, 127);  
//  analogWrite(ssr15, 60);
//
  delay(2000);
  duty_switch = !duty_switch;
  if (duty_switch) {
    ISR_PWM.modifyPWMChannel(channelNum0, ssr0, freq, 10.0);
    ISR_PWM.modifyPWMChannel(channelNum1, ssr1, freq, 20.0);
    ISR_PWM.modifyPWMChannel(channelNum2, ssr2, freq, 30.0);
    ISR_PWM.modifyPWMChannel(channelNum3, ssr3, freq, 40.0);
    ISR_PWM.modifyPWMChannel(channelNum4, ssr4, freq, 50.0);
    ISR_PWM.modifyPWMChannel(channelNum5, ssr5, freq, 60.0);
    ISR_PWM.modifyPWMChannel(channelNum6, ssr6, freq, 70.0);
    ISR_PWM.modifyPWMChannel(channelNum7, ssr7, freq, 80.0);
    ISR_PWM.modifyPWMChannel(channelNum8, ssr8, freq, 90.0);
    ISR_PWM.modifyPWMChannel(channelNum9, ssr9, freq, 10.0);
    ISR_PWM.modifyPWMChannel(channelNum10, ssr10, freq, 20.0);
    ISR_PWM.modifyPWMChannel(channelNum11, ssr11, freq, 30.0);
    ISR_PWM.modifyPWMChannel(channelNum12, ssr12, freq, 40.0);
    ISR_PWM.modifyPWMChannel(channelNum13, ssr13, freq, 50.0);
    ISR_PWM.modifyPWMChannel(channelNum14, ssr14, freq, 60.0);
    ISR_PWM.modifyPWMChannel(channelNum15, ssr15, freq, 70.0);
  } else {
    ISR_PWM.modifyPWMChannel(channelNum0, ssr0, 5.0f, 90.0);
    ISR_PWM.modifyPWMChannel(channelNum1, ssr1, freq, 80.0);
    ISR_PWM.modifyPWMChannel(channelNum2, ssr2, freq, 70.0);
    ISR_PWM.modifyPWMChannel(channelNum3, ssr3, freq, 60.0);
    ISR_PWM.modifyPWMChannel(channelNum4, ssr4, freq, 55.0);
    ISR_PWM.modifyPWMChannel(channelNum5, ssr5, freq, 40.0);
    ISR_PWM.modifyPWMChannel(channelNum6, ssr6, freq, 30.0);
    ISR_PWM.modifyPWMChannel(channelNum7, ssr7, freq, 20.0);
    ISR_PWM.modifyPWMChannel(channelNum8, ssr8, freq, 10.0);
    ISR_PWM.modifyPWMChannel(channelNum9, ssr9, freq, 90.0);
    ISR_PWM.modifyPWMChannel(channelNum10, ssr10, freq, 80.0);
    ISR_PWM.modifyPWMChannel(channelNum11, ssr11, freq, 70.0);
    ISR_PWM.modifyPWMChannel(channelNum12, ssr12, freq, 60.0);
    ISR_PWM.modifyPWMChannel(channelNum13, ssr13, freq, 55.0);
    ISR_PWM.modifyPWMChannel(channelNum14, ssr14, freq, 40.0);
    ISR_PWM.modifyPWMChannel(channelNum15, ssr15, freq, 30.0);
  }
}
