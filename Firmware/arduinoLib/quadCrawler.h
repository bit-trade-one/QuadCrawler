#ifndef quadCrawler_h
#define quadCrawler_h

#include <stdint.h>

// ポート定義
enum {
  Sw1 = 3,
  Sw2 = 4,
  Sw3 = 5,
  Sw4 = 6,
};

//動作速度定義
enum {
  quadCrawler_sslow = 2000,
  quadCrawler_slow  = 1000,
  quadCrawler_typical = 500,
  quadCrawler_fast  = 200,
  quadCrawler_high  = 100,
};

//制御ステート定義
enum {
  stop = 0,

  // repeat
  fw,
  cw,
  ccw,
  rw,
  Rigt,
  Left,

  // normal
  all_up,
  all_dn,
  t_dn,
  h_dn,
  l_dn,
  r_dn,

  // repeat
  t_up_dn,
  l_r_up,
  all_up_dn,

  pose,
};

void quadCrawler_Walk(uint16_t speed, uint8_t com);
void quadCrawler_setSpeed(uint16_t speed);

enum {
  POSE_KEEP     = 0,
  POSE_NEUTRAL  = 1,
  POSE_UP       = 2,
  POSE_DOWN     = 3,
  POSE_DOWNMAX  = 4,

  POSE_REAR     = 2,
  POSE_FRONT    = 3,
};

void quadCrawler_setPose4(uint8_t rfk, uint8_t rfc, uint8_t rrk, uint8_t rrc, uint8_t lfk, uint8_t lfc, uint8_t lrk, uint8_t lrc);

enum {
  FRONT_R   = 0,
  REAR_R    = 1,
  FRONT_L   = 2,
  REAR_L    = 3,
};
void quadCrawler_setPose1(uint8_t index, uint8_t knee, uint8_t crach);

void quadCrawler_servoLoop(void);


void quadCrawler_init(void);

double quadCrawler_getSonner();
void quadCrawler_beep(int time);

void quadCrawler_colorWipe(uint8_t color);
enum {
  COLOR_OFF = 0,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_YELLOW,
  COLOR_PURPLE,
  COLOR_LIGHTBLUE,
};

void quadCrawler_rainbow(uint8_t wait);
uint8_t quadCrawler_checkServoON(void);

#endif  // quadCrawler_h
