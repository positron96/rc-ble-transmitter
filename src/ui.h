#ifndef UI_H_
#define UI_H_

#include <Arduino.h>
#include <TFT_eSPI.h>

constexpr int PIN_X = 30;
constexpr int PIN_Y = 31;
constexpr int PIN_C = 32;

extern TFT_eSPI tft;


void tick() {
    int x = analogRead(PIN_X);
    int y = analogRead(PIN_Y);
    bool down = digitalRead(PIN_C);

    constexpr int CX = 100, CY=100;

    tft.fillCircle(CX, CY, 50, TFT_GREEN);

    x = (x-512)*50/512;
    y = (y-512)*50/512;
    tft.drawLine(CX, CY, CX+x, CY+y, down ? TFT_ORANGE : TFT_BLUE);
}

#endif // UI_H_