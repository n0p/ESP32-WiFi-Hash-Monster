// Copyright (c) M5Stack. All rights reserved.

// Licensed under the MIT license. See LICENSE file in the project root for full license information.
/**
 * \par Copyright (C), 2016-2017, M5Stack
 * \class M5Stack
 * \brief   M5Stack library.
 * @file    M5Stack.h
 * @author  M5Stack
 * @version V0.1.5
 * @date    2017/03/27
 * @brief   Header for M5Stack.cpp module
 *
 * \par Description
 * This file is a drive for M5Stack core.
 *
 * \par Method List:
 *    
 *  System:
        M5.begin();
        M5.update();

    LCD:
        M5.lcd.setBrightness(uint8_t brightness);
        M5.Lcd.drawPixel(int16_t x, int16_t y, uint16_t color);
        M5.Lcd.drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
        M5.Lcd.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        M5.Lcd.fillScreen(uint16_t color);
        M5.Lcd.drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
        M5.Lcd.drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,uint16_t color);
        M5.Lcd.fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
        M5.Lcd.fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,int16_t delta, uint16_t color);
        M5.Lcd.drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
        M5.Lcd.fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
        M5.Lcd.drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        M5.Lcd.fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        M5.Lcd.drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
        M5.Lcd.drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h),
        M5.Lcd.drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
        M5.Lcd.setCursor(uint16_t x0, uint16_t y0);
        M5.Lcd.setTextColor(uint16_t color);
        M5.Lcd.setTextColor(uint16_t color, uint16_t backgroundcolor);
        M5.Lcd.setTextSize(uint8_t size);
        M5.Lcd.setTextWrap(boolean w);
        M5.Lcd.printf();
        M5.Lcd.print();
        M5.Lcd.println();
        M5.Lcd.drawCentreString(const char *string, int dX, int poY, int font);
        M5.Lcd.drawRightString(const char *string, int dX, int poY, int font);
        M5.Lcd.drawJpg(const uint8_t *jpg_data, size_t jpg_len, uint16_t x, uint16_t y);
        M5.Lcd.drawJpgFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);
        M5.Lcd.drawBmpFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);


    Button:
        M5.BtnA/B/C/Menu/Volume/Select/Start/Axis.read();
        M5.BtnA/B/C/Menu/Volume/Select/Start/Axis.isPressed();
        M5.BtnA/B/C/Menu/Volume/Select/Start.isReleased();
        M5.BtnA/B/C/Menu/Volume/Select/Start/Axis.wasPressed();
        M5.BtnA/B/C/Menu/Volume/Select/Start/.wasReleased();
        M5.BtnA/B/C/Menu/Volume/Select/Start/.pressedFor(uint32_t ms);
        M5.BtnA/B/C/Menu/Volume/Select/Start/.releasedFor(uint32_t ms);
        M5.BtnA/B/C/Menu/Volume/Select/Start/.lastChange();
		
keymap		M5stack		ODROID-GO
			BtnA		BtnA
			BtnB		BtnB		
			BtnC		BtnMenu

    Speaker:
        M5.Speaker.tone(uint32_t freq);
        M5.Speaker.tone(freq, time);
        M5.Speaker.beep();
        M5.Speaker.setBeep(uint16_t frequency, uint16_t duration);
        M5.Speaker.mute();

 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Zibin Zheng         2017/07/14        0.0.1          Rebuild the new.
 * Vladimir Yershov    2018/07/26						ODROID-GO support
 * </pre>
 *
 */

#ifndef _M5STACK_H_
#define _M5STACK_H_

#if defined(ESP32)

// #define MPU9250_INSDE
#include "gitTagVersion.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <SPI.h>
#include "FS.h"
#include "SD.h"

#include "M5Display.h"
#include "utility/Config.h"
#include "utility/Button.h"
#include "utility/Speaker.h"
#include "utility/M5Timer.h"
#ifdef ARDUINO_ODROID_ESP32
#include "utility/battery.h"
#endif

#ifdef MPU9250_INSDE
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"
#endif


class M5Stack {

 public:
    void begin(bool LCDEnable=true, bool SDEnable=true);
    void update();

    void setWakeupButton(uint8_t button);
    void powerOFF();

    // Button API
    #define DEBOUNCE_MS 5
////#ifdef ARDUINO_ODROID_ESP32	
    #define DEBOUNCE_MS_XY 5
	Button JOY_Y = Button(BUTTON_JOY_Y_PIN, true, DEBOUNCE_MS_XY);
	Button JOY_X = Button(BUTTON_JOY_X_PIN, true, DEBOUNCE_MS_XY);
	Button BtnMenu = Button(BUTTON_MENU_PIN, true, DEBOUNCE_MS);
	Button BtnVolume = Button(BUTTON_VOLUME_PIN, true, DEBOUNCE_MS);
	Button BtnSelect = Button(BUTTON_SELECT_PIN, true, DEBOUNCE_MS);
	Button BtnStart = Button(BUTTON_START_PIN, true, DEBOUNCE_MS);	
////#endif	
    Button BtnA = Button(BUTTON_A_PIN, true, DEBOUNCE_MS);
    Button BtnB = Button(BUTTON_B_PIN, true, DEBOUNCE_MS);
    Button BtnC = Button(BUTTON_C_PIN, true, DEBOUNCE_MS);

    // SPEAKER
    SPEAKER Speaker;
#ifdef ARDUINO_ODROID_ESP32	
	Battery battery;
#endif

    // LCD
    M5Display Lcd = M5Display();

    // UART
    // HardwareSerial Serial0 = HardwareSerial(0);
    // HardwareSerial Serial2 = HardwareSerial(2);

    // MPU9250
#ifdef MPU9250_INSDE
    MPU9250 IMU = MPU9250();
#endif

 private:
    uint8_t _wakeupPin;
};

extern M5Stack M5;
#define m5 M5
#define lcd Lcd

#else
#error “This library only supports boards with ESP32 processor.”
#endif

#endif

