//C言語標準ライブラリ
#include "stdlib.h"

//タイマーライブラリ
#include "RPi_Pico_TimerInterrupt.h"
#include "hardware/pwm.h"

//SSD1306ディスプレイ関連ライブラリ
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // 別途「Adafruit BusIO」ライブラリ必要

// OLED設定
#define SCREEN_WIDTH 128  // OLED 幅指定
#define SCREEN_HEIGHT 64  // OLED 高さ指定（高さ32のものを使用する場合は32）
#define OLED_RESET -1     // リセット端子（未使用-1）

//メニュー選択スイッチピン up:前側　down:後ろ側
#define upswitch 15
#define downswitch 14

//AD変換ピン
#define SELPIN1 9    //前後どっちか確認する
#define SELPIN2 13   //前後どっちか確認する
#define DATAOUT 12   //mcp3002:DOUT---pico:GP12(SPI1 RX)
#define DATAIN 11    //mcp3002:DIN---pico:GP11(SPI1 TX)
#define SPICLOCK 10  //Clock
#define GOALSENSOR 28
#define Curve_Sensor 26
#define ch0 0
#define ch1 1
// 電圧測定
#define VOLT 27
//モータードライバピン
#define CLOCK_R 8
#define CWCCW_R 6
#define ENABLE_R 5

#define CLOCK_L 21
#define CWCCW_L 20
#define ENABLE_L 22

//ブザーピン
#define BUZZER 4

// I2Cに接続されたSSD1306用「display」の宣言
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//タイマー関連
RPI_PICO_Timer ITimer0(0);
// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer

// 変数宣言
//パルス生成用変数
unsigned int toggle0 = 0;
unsigned int toggle1 = 0;

int intervalL = 0;
int intervalR = 0;

bool pulseL = 0;
bool pulseR = 0;
//プッシュスイッチ入力回数格納
static int Mode = 0;
//実行スイッチ入力回数格納
static int Run = 0;

//センサー値格納変数
int sensorLL = 0;
int sensorL = 0;
int sensorR = 0;
int sensorRR = 0;
int sensorGoal = 0;
int Curve = 0;
//ステップ数計測変数
unsigned int Step = 0;
float distance = 0.0;
//プッシュスイッチカウント
static bool sw1 = 0;
static bool sw2 = 0;
//ループの中で１回しか実行させないための変数
// static bool a = false;
static unsigned int b = 0, c = 0;
//入力速度
static float inputL = 0;
static float inputR = 0;
//基準速度
static float SP = 0;
//PID制御
static float P = 0.0;
static float D = 0.0;
static float I = 0.0;
static int diff = 0;
static int bias = 0;
static int beforediff = 0;
static int sum = 0;
//Pゲイン
float pgain = 0.4;
//Dゲイン
float dgain = 0;
//Iゲイン
float igain = 0.0004;
//goalセンサーカウント
static int count = 0, cross = 0;
static bool tmp = 0, tmpc = 0;
// 電圧値監視
float voltage = 0.0;

//プロトタイプ宣言
int read_adc(int select, int channel);          //ADコンバータ
bool TimerHandler0(struct repeating_timer *t);  //割り込む関数

// void meinrun();                                 //メイン走行関数
// void Ponly();                                   //テスト走行関数
// void accel();
// void accelrun();
float SPpulse(int SP);
// void accelrun2();
int pulseHz(int pulsefreq);  //パルス周波数⇨パルス幅変換
//PWMスライス生成
uint pwm_slice1 = pwm_gpio_to_slice_num(CLOCK_R);
uint pwm_slice2 = pwm_gpio_to_slice_num(CLOCK_L);
// ディスプレイ表示
void Oled_run(float volt);

uint16_t LL_log[5000];
uint16_t RR_log[5000];
uint16_t SS_log[5000];
uint16_t TT_log[5000];
uint16_t GG_log[5000];


//ログ保存用構造体
//typedef struct {
//
//    
//}LOG_t
//
//LOG_t logdata;

void setup() {
  memset(LL_log, 0, sizeof(LL_log));
  memset(RR_log, 0, sizeof(RR_log));
  memset(SS_log, 0, sizeof(SS_log));
  memset(TT_log, 0, sizeof(TT_log));
  memset(GG_log, 0, sizeof(GG_log));

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

  //ブザー
  pinMode(BUZZER, OUTPUT);  

  //回転方向制御
  digitalWrite(CWCCW_L, HIGH);
  digitalWrite(CWCCW_R, LOW);  //前進
  //モーター電源制御
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);

  //シリアル通信用
  Serial.begin(115200);
  //raspi picoのanalogreadの精度を12bitに切り替え　↓以下を定義することで切り替え標準は10bit
//  analogReadResolution(12);
  //PWM速度変化https://rikei-tawamure.com/entry/2021/02/08/213335
  gpio_set_function(CLOCK_R, GPIO_FUNC_PWM);
  gpio_set_function(CLOCK_L, GPIO_FUNC_PWM);

 //周期0.02s
 //Slice1＝Reft Slice2=Left
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
  /*
  ディスプレイ設定　初期表示
  */
  //SSD1306本体初期化  
  Wire.setSDA(16);  // I2C0 SDA 端子番号設定
  Wire.setSCL(17);  // I2C0 SCL 端子番号設定
  Wire.begin();     // I2C通信開始設定(SDA,SDL)
  // OLED初期設定
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306:0 allocation failed"));
    for (;;); // エラーなら無限ループ
  }
  // OLED表示設定
  display.setTextColor(SSD1306_WHITE);  // 文字色
  
  // ディスプレイ表示
  Oled_run(0.0,0);
  delay(100);
  
  // ブザー鳴らす
  digitalWrite(BUZZER, HIGH);
  delay(500);
  digitalWrite(BUZZER, LOW);
  //モード選択待機
}


void loop() {
  // バッテリー電圧更新 12bit 4096通り
  voltage = ((analogRead(VOLT) * 3.3 / 1024)*6.1);
  
  Serial.print("raw :");
  Serial.print(analogRead(VOLT));
  Serial.print("Voltage :");
  Serial.println(voltage);
  Oled_run(voltage,Mode);
  
  //モード選択
  static bool a = 0;
  sw1 = digitalRead(upswitch);
  sw2 = digitalRead(downswitch);
  if (sw1 == 1) {
    Mode++;
    //Mode：９まで    
    if(Mode==10){
      Mode=0;
      }
    digitalWrite(LED_BUILTIN, LOW);
    a = 0;
    delay(300);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  
  if (a == 0) {
    Serial.print("ModeNumber:");
    Serial.println(Mode);
    a = 1;
  }

  if (sw2 == 1) {
    Run++;
    digitalWrite(LED_BUILTIN, LOW);
    a=1;
    delay(300);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  

  if (Run == 1) {
    if(a == 1){
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

    //モーター電源オン
    digitalWrite(ENABLE_L, LOW);
    digitalWrite(ENABLE_R, LOW);
    //実行時に一回だけLED点滅　値リセット
    a = 0;
    }
  
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
      case 5:
        Scene5();
        break;
      default:
        Serial.println("Undefined Number");
        break;
    }
  }
}
