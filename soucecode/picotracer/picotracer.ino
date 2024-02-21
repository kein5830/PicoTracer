//タイマーライブラリ
#include "RPi_Pico_TimerInterrupt.h"
#include "hardware/pwm.h"
//メニュー選択スイッチピン up:前側　down:後ろ側
#define upswitch 17
#define downswitch 16

//AD変換ピン
#define SELPIN1 9    //前後どっちか確認する
#define SELPIN2 13   //前後どっちか確認する
#define DATAOUT 11   //MOSI
#define DATAIN 12    //MISO
#define SPICLOCK 10  //Clock
#define GOALSENSOR 26
#define ch0 0
#define ch1 1

//モータードライバピン
#define CLOCK_R 8
#define CWCCW_R 6
#define ENABLE_R 5

#define CLOCK_L 21
#define CWCCW_L 20
#define ENABLE_L 22

// //タイマー関連
// RPI_PICO_Timer ITimer0(0);
// // Select the timer you're using, from ITimer0(0)-ITimer3(3)
// // Init RPI_PICO_Timer
// RPI_PICO_Timer ITimer1(1);
//パルス生成用変数
unsigned int toggle0 = 0;
unsigned int toggle1 = 0;

int intervalL = 0;
int intervalR = 0;

bool pulseL = 0;
bool pulseR = 0;
//プッシュスイッチ入力回数格納
static int Mode = 0;

//センサー値格納変数
int sensorLL = 0;
int sensorL = 0;
int sensorR = 0;
int sensorRR = 0;
int sensorGoal = 0;
//ステップ数計測変数
unsigned int StepL = 0;
unsigned int StepR = 0;
float distanceL = 0.0;
float distanceR = 0.0;
float distance = 0.0;
//プロトタイプ宣言
int read_adc(int select, int channel);          //ADコンバータ
// bool TimerHandler0(struct repeating_timer *t);  //割り込む関数

// void meinrun();                                 //メイン走行関数
// void Ponly();                                   //テスト走行関数
// void accel();
// void accelrun();
float SPpulse(int SP);
// void accelrun2();
int pulseHz(int pulsefreq);  //パルス周波数⇨パルス幅変換

uint pwm_slice1 = pwm_gpio_to_slice_num(CLOCK_R);
uint pwm_slice2 = pwm_gpio_to_slice_num(CLOCK_L);

void setup() {
  //マイコン電源確認用LEDの点灯
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //メニュー選択スイッチ up:前側　down:後ろ側
  pinMode(upswitch, INPUT_PULLDOWN);
  pinMode(downswitch, INPUT_PULLDOWN);
  //AD変換ピン
  pinMode(SELPIN1, OUTPUT);
  pinMode(SELPIN2, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);

  digitalWrite(SELPIN1, HIGH);
  digitalWrite(SELPIN2, HIGH);
  digitalWrite(DATAOUT, LOW);
  digitalWrite(SPICLOCK, LOW);

  //モータードライバピン
  pinMode(CLOCK_L, OUTPUT);   //パルス出力ピン
  pinMode(CWCCW_L, OUTPUT);   //モータ回転方向出力ピン
  pinMode(ENABLE_L, OUTPUT);  //モーター電源ピン
  pinMode(CLOCK_R, OUTPUT);   //パルス出力ピン
  pinMode(CWCCW_R, OUTPUT);   //モータ回転方向出力ピン
  pinMode(ENABLE_R, OUTPUT);  //モーター電源ピン
  //回転方向制御
  digitalWrite(CWCCW_L, HIGH);
  digitalWrite(CWCCW_R, LOW);  //前進
  //モーター電源制御
  digitalWrite(ENABLE_L, LOW);
  digitalWrite(ENABLE_R, LOW);

  //シリアル通信用
  Serial.begin(115200);
  
  //PWM速度変化https://rikei-tawamure.com/entry/2021/02/08/213335
  gpio_set_function(CLOCK_R, GPIO_FUNC_PWM);
  gpio_set_function(CLOCK_L, GPIO_FUNC_PWM);

  

 //周期0.02s
  pwm_set_wrap(pwm_slice1, 1100);
  pwm_set_wrap(pwm_slice2, 1100);
  //duty　値を直接指定(固定値)　今回の用途では変更する必要なし
  pwm_set_chan_level(pwm_slice1, PWM_CHAN_A, 500);
  pwm_set_chan_level(pwm_slice2, PWM_CHAN_B, 500);
  //システムクロックを100分割　分周比１００
  pwm_set_clkdiv(pwm_slice1, 100.0);
  pwm_set_clkdiv(pwm_slice2, 100.0);




  // pwm_set_chan_level(slice_num, PWM_CHAN_A, 2315);
  // pwm_set_chan_level(slice_num, PWM_CHAN_A, 2315);
  //モード選択待機
  while (1) {
    static bool sw1 = 0;
    static bool sw2 = 0;
    static bool a = 0;
    sw1 = digitalRead(upswitch);
    sw2 = digitalRead(downswitch);

    if (sw1 == 1) {
      Mode++;
      digitalWrite(LED_BUILTIN, LOW);
      a = 0;
      delay(300);
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if (sw2 == 1) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    }
    if (a == 0) {
      Serial.print("ModeNumber:");
      Serial.println(Mode);
      a = 1;
    }
  }

  //  // Interval in unsigned long microseconds
  // if (ITimer0.attachInterruptInterval(5000L * 1000, TimerHandler0)){
  //   Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  // }else{
  //   Serial.println("Can't set ITimer. Select another freq. or timer");
  // }
  //  // Interval in unsigned long microseconds
  // if (ITimer1.attachInterruptInterval(5000L * 1000, TimerHandler1)){
  //   Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  // }else{
  //   Serial.println("Can't set ITimer. Select another freq. or timer");
  // }
}


void loop() {
  switch (Mode) {
    case 0:
      Scene0();
      break;
    case 1:
      Scene1();
      break;
    case 2:
      Scene2();
      break;
    case 3:
      Scene3();
      break;
    case 4:
      Scene4();
      break;
    default:
      Serial.println("error: no Number");
      break;
  }
}
