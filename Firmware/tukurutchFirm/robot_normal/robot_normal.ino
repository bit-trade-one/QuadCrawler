#define mVersion "QuadCrawler1.0"


#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>
#include <util/delay.h>

#include <EEPROM.h>                     // for quadCrawlerRemocon
#include "quadCrawlerRemocon.h"

#include <Wire.h>                       // for Adafruit_PWMServoDriver
#include <Adafruit_PWMServoDriver.h>    // for quadCrawler
#include <Adafruit_NeoPixel.h>          // for quadCrawler
#include "quadCrawler.h"

void setup() {
  remoconRobo_init();
  quadCrawler_init();
  quadCrawler_colorWipe(COLOR_PURPLE);
  quadCrawler_beep(100);

  Serial.begin(115200);
  Serial.println("Normal: " mVersion);
}

static uint8_t lastkey = 0;
static uint8_t originAdj = 0;
static uint8_t lastSw4 = 1;

void loop() {
  #define A_DOWN_OFFSET  0x10
  remoconRobo_checkRemoteUpdated(0);
  struct remoconData rData = remoconRobo_getRemoteData();
  switch(rData.keys) {
  case BUTTON_A_XY:
    rData.keys = rData.xyKeys;
    break;
  case BUTTON_A_DOWN:
    rData.keys = rData.xyKeys + A_DOWN_OFFSET;
    break;
  }

  if(rData.keys != lastkey) {
    lastkey = rData.keys;
    switch(rData.keys) {
      case BUTTON_POWER:
      case BUTTON_C:
      case BUTTON_B:
      case BUTTON_CENTER:
        break;
      case BUTTON_MENU:
        quadCrawler_beep(3000);
        break;
      case BUTTON_0:
        quadCrawler_beep(50);
        for (int n = 0; n < 5; n++) {
          quadCrawler_rainbow(5);
        }
        break;

      case XY_UP:
      case BUTTON_UP:
        quadCrawler_colorWipe(COLOR_BLUE);
        quadCrawler_Walk(quadCrawler_fast, fw);
        break;
      case XY_LEFT:
      case BUTTON_LEFT:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, Left);
        break;
      case XY_RIGHT:
      case BUTTON_RIGHT:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, Rigt);
        break;
      case XY_DOWN:
      case BUTTON_DOWN:
        quadCrawler_colorWipe(COLOR_RED);
        quadCrawler_Walk(quadCrawler_fast, rw);
        break;
      case XY_UP_R:
      case XY_DOWN_L:
      case BUTTON_RETURN:
        quadCrawler_colorWipe(COLOR_GREEN);
        quadCrawler_Walk(quadCrawler_fast, cw);
        break;
      case XY_UP_L:
      case XY_DOWN_R:
      case BUTTON_TEST:
        quadCrawler_colorWipe(COLOR_GREEN);
        quadCrawler_Walk(quadCrawler_fast, ccw);
        break;

      case XY_UP_R + A_DOWN_OFFSET:
      case BUTTON_1:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, all_up);
        break;
      case BUTTON_A_CENTER:
      case BUTTON_2:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, all_dn);
        break;
      case XY_DOWN + A_DOWN_OFFSET:
      case BUTTON_3:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, t_dn);
        break;
      case XY_UP + A_DOWN_OFFSET:
      case BUTTON_4:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, h_dn);
        break;
      case XY_LEFT + A_DOWN_OFFSET:
      case BUTTON_5:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, l_dn);
        break;
      case XY_RIGHT + A_DOWN_OFFSET:
      case BUTTON_6:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, r_dn);
        break;

      case BUTTON_A_UP:
      case BUTTON_7:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, t_up_dn);
        break;
      case BUTTON_A_LEFT:
      case BUTTON_8:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, l_r_up);
        break;
      case BUTTON_A_RIGHT:
      case BUTTON_9:
        quadCrawler_colorWipe(COLOR_LIGHTBLUE);
        quadCrawler_Walk(quadCrawler_fast, all_up_dn);
        break;
      default:
        quadCrawler_colorWipe(COLOR_PURPLE);
        quadCrawler_Walk(quadCrawler_fast, stop);
        break;
    }
  }

  uint8_t sw4 = digitalRead(Sw4);
  if(lastSw4!=sw4 && sw4==0) {
    if(!originAdj) {
      quadCrawler_colorWipe(COLOR_RED);
      quadCrawler_setPose4(POSE_NEUTRAL, POSE_NEUTRAL, POSE_NEUTRAL, POSE_NEUTRAL, POSE_NEUTRAL, POSE_NEUTRAL, POSE_NEUTRAL, POSE_NEUTRAL);
    } else {
      quadCrawler_colorWipe(COLOR_PURPLE);
      quadCrawler_Walk(quadCrawler_fast, stop);
    }
    originAdj = !originAdj;
  } else if(originAdj && !quadCrawler_checkServoON()) {
    quadCrawler_colorWipe(COLOR_PURPLE);
    originAdj = 0;
  }
  lastSw4 = sw4;

  quadCrawler_servoLoop();
  if(rData.xyLevel >= 10) {
    quadCrawler_setSpeed(25000 / rData.xyLevel);
  }

  double sonner_val;
  sonner_val = quadCrawler_getSonner();
  //Serial.println(sonner_val);
  if (sonner_val < 8){
    quadCrawler_beep(sonner_val * 10);
    delay(sonner_val * 10);
  }
  else {
    delay(50);
  }
}
