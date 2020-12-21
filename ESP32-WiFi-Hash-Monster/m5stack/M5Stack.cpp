// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "M5Stack.h"

void M5Stack::begin(bool LCDEnable, bool SDEnable) {
    
    // UART 
    Serial.begin(115200);
    Serial.flush();
    Serial.print("M5Stack initializing...");

    // I2C
    pinMode(SCL, OUTPUT);
    digitalWrite(SDA, 1);

    // TONE
    Speaker.begin();

    // Setup the button with an internal pull-up
    pinMode(BUTTON_A_PIN, INPUT_PULLUP);
    pinMode(BUTTON_B_PIN, INPUT_PULLUP);
    pinMode(BUTTON_C_PIN, INPUT_PULLUP);
#ifdef ARDUINO_ODROID_ESP32
    pinMode(BUTTON_MENU_PIN, INPUT_PULLUP);
    pinMode(BUTTON_VOLUME_PIN, INPUT_PULLUP);
    pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
	pinMode(BUTTON_JOY_Y_PIN, INPUT_PULLDOWN);
	pinMode(BUTTON_JOY_X_PIN, INPUT_PULLDOWN);
#endif	
    // M5 LCD INIT
    if (LCDEnable) Lcd.begin();

    // TF Card
    if (SDEnable) SD.begin(TFCARD_CS_PIN, SPI, 40000000);
    
    // Set wakeup button
    setWakeupButton(BUTTON_A_PIN);

    // MPU9250
#ifdef MPU9250_INSDE
    Wire.begin();
#endif

    Serial.println("OK");
}

void M5Stack::update() {

    //Button update
    BtnA.read();
    BtnB.read();
    BtnC.read();
////#ifdef ARDUINO_ODROID_ESP32	
    BtnMenu.read();
    BtnVolume.read();
    BtnSelect.read();
    BtnStart.read();
	JOY_Y.readAxis();
	JOY_X.readAxis();
////#endif	

    //Speaker update
    Speaker.update();
#ifdef ARDUINO_ODROID_ESP32	
	battery.update();
#endif	
}

void M5Stack::setWakeupButton(uint8_t button) {
    _wakeupPin = button;
}

void M5Stack::powerOFF() {
    
    // power off the Lcd
    Lcd.setBrightness(0);
    Lcd.sleep();

    // ESP32 into deep sleep
    USE_SERIAL.printf("Enabling EXT0 wakeup on pins GPIO%d\n", _wakeupPin);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)_wakeupPin , LOW);
    
    while(digitalRead(_wakeupPin) == LOW) {
        delay(10);
    }
    USE_SERIAL.println("On deep sleep mode.");
    esp_deep_sleep_start();
    USE_SERIAL.println("On power OFF fail!");
}

M5Stack M5;
