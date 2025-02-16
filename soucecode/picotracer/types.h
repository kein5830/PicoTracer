#include <Arduino.h>

//ログ保存用構造体定義
  typedef struct {
    //ライン検知センサ
    uint16_t Curve_log;
    uint16_t LL_log;
    uint16_t L_log;
    uint16_t R_log;
    uint16_t RR_log;
    uint16_t Goal_log;
    //モーター出力
//    uint16_t R_motor_log;
//    uint16_t L_motor_log;
   }Log;

   Log data_log[18500];
