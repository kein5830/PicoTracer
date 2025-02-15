//----------------------------------------------------------
//　使用ライブラリinclude
//----------------------------------------------------------
//C言語標準ライブラリ
#include "stdlib.h"

//タイマーライブラリ
#include "RPi_Pico_TimerInterrupt.h"
#include "hardware/pwm.h"

//SSD1306ディスプレイ関連ライブラリ
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // 別途「Adafruit BusIO」ライブラリ必要

//----------------------------------------------------------
//　マクロ定義
//----------------------------------------------------------
// OLED設定
#define SCREEN_WIDTH 128  // OLED 幅指定
#define SCREEN_HEIGHT 64  // OLED 高さ指定（高さ32のものを使用する場合は32）
#define OLED_RESET -1     // リセット端子（未使用-1）


//----------------------------------------------------------
//　マイコンピン割り当て定義
//----------------------------------------------------------
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

//3.3V電源検知
#define Power_Det 18


// I2Cに接続されたSSD1306用ライブラリの実態「display」の宣言
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//タイマー関連
RPI_PICO_Timer ITimer0(0);
// Select the timer you're using, from ITimer0(0)-ITimer3(3)
// Init RPI_PICO_Timer

//----------------------------------------------------------
//　グローバル変数定義
//----------------------------------------------------------
//パルス生成用変数
unsigned int toggle0 = 0;
unsigned int toggle1 = 0;

//pwm関連変数
int intervalL = 0;
int intervalR = 0;

bool pulseL = 0;
bool pulseR = 0;
//プッシュスイッチ入力回数格納
static uint8_t Scene = 0;
//実行スイッチ入力回数格納
static bool Run = 0;

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
//一度だけ呼び出す処理の記述
bool one = 0;
//モーター入力速度
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
//goalセンサカウント
static int count = 0, cross = 0;
static bool tmp = 0, tmpc = 0;
// 電圧値監視
float voltage = 0.0;

//経過時刻変数
unsigned long currentMillis = 0;

//以前の周期を記録する変数
static unsigned long volt_prevmillis = 0;
static unsigned long oled_prevmillis = 0;
static unsigned long button_prevmillis = 0;
static unsigned long run_prevmillis = 0;

//Scene4用スピード記録変数
static float Speed = 0.0;

//----------------------------------------------------------
//　関数プロトタイプ宣言
//----------------------------------------------------------
//ADコンバータ
int read_adc(int select, int channel);
//割り込む関数
bool TimerHandler0(struct repeating_timer *t);

float SPpulse(int SP);
// void accelrun2();
int pulseHz(int pulsefreq);  //パルス周波数⇨パルス幅変換
//PWMスライス生成
uint pwm_slice1 = pwm_gpio_to_slice_num(CLOCK_R);
uint pwm_slice2 = pwm_gpio_to_slice_num(CLOCK_L);
// ディスプレイ表示
void Oled_Update(float volt, int runscene, uint8_t runmode);
//リセット関数
void Reset();
//周波数、wrap値変換
uint16_t Hz_wrap(float pulsefreq);

//----------------------------------------------------------
// setup関数　　起動時に実行
//----------------------------------------------------------
void setup() {
 
  //3.3V電源検知
  pinMode(Power_Det, INPUT);
  //マイコン電源確認用LEDの点灯
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //メニュー選択スイッチ up:前側　down:後ろ側
  pinMode(upswitch, INPUT_PULLDOWN);
  pinMode(downswitch, INPUT_PULLDOWN);

  if(digitalRead(Power_Det)){
    //AD変換ピン
    pinMode(SELPIN1, OUTPUT);
    pinMode(SELPIN2, OUTPUT);
    pinMode(DATAOUT, OUTPUT);
    pinMode(DATAIN, INPUT);
    pinMode(SPICLOCK, OUTPUT);
    digitalWrite(SELPIN1, HIGH);//
    digitalWrite(SELPIN2, HIGH);//両方LOWにすると2.2V 0.9Vまで下がる
    digitalWrite(DATAOUT, LOW);
    digitalWrite(SPICLOCK, LOW);
  
    //ディスプレイ設定　初期表示
    //SSD1306本体初期化
    Wire.setSDA(16);  // I2C0 SDA 端子番号設定
    Wire.setSCL(17);  // I2C0 SCL 端子番号設定
    Wire.begin();     // I2C通信開始設定(SDA,SDL)
    // OLED初期設定
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println(F("SSD1306:0 allocation failed"));
    }else{
    // OLED表示設定 文字色
    display.setTextColor(SSD1306_WHITE); 
    // ディスプレイ表示
    Oled_Update(0.0, 0, 0);
    delay(100);
    }
  }
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
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);

  //シリアル通信用
  Serial.begin(115200);

  //PWM速度変化https://rikei-tawamure.com/entry/2021/02/08/213335
  gpio_set_function(CLOCK_R, GPIO_FUNC_PWM);
  gpio_set_function(CLOCK_L, GPIO_FUNC_PWM);

  //Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, 1100);
  pwm_set_wrap(pwm_slice2, 1100);
  //duty　値を直接指定(固定値)　今回の用途では変更する必要なし
  pwm_set_chan_level(pwm_slice1, PWM_CHAN_A, 500);
  pwm_set_chan_level(pwm_slice2, PWM_CHAN_B, 500);
  //システムクロックを100分割　分周比１００
  pwm_set_clkdiv(pwm_slice1, 100.0);
  pwm_set_clkdiv(pwm_slice2, 100.0);


  // ブザー鳴らす
  tone(BUZZER,1046,500);
  delay(500);
  //モード選択待機に移行
}


void loop() {

//----------------------------------------------------------
//----------------------------------------------------------
//　                     待機モード
//----------------------------------------------------------
//----------------------------------------------------------
  
  //----------------------------------------------------------
  //　ローカル変数定義
  //----------------------------------------------------------
  //プッシュスイッチチャタリング防止
  static bool temp1 = 0, temp2 = 0;

  //----------------------------------------------------------
  //　定周期処理
  //----------------------------------------------------------
  //待機モード時のみ実行
  if(Run == 0){
      //PWM停止　Resetで停止しているはずだが、どこかでONになってるみたいなので待機中は常に停止
      pwm_set_enabled(pwm_slice1, false);
      pwm_set_enabled(pwm_slice2, false);
    //バッテリー電圧更新 500ms
    if ((currentMillis = millis()) - volt_prevmillis >=  500) {
      voltage = ((analogRead(VOLT) * 3.3 / 1024) * 6.3);//値修正
      volt_prevmillis = currentMillis;
    }
  
    //ディスプレイ更新 100ms
    if ((currentMillis = millis()) - oled_prevmillis >=  100) {
      Oled_Update(voltage, Scene, Run);
      oled_prevmillis = currentMillis;
    }
  }
  //プッシュスイッチONOFF検知 10ms
  if ((currentMillis = millis()) - button_prevmillis >=  50) {
    sw1 = digitalRead(upswitch);
    sw2 = digitalRead(downswitch);
    button_prevmillis = currentMillis;
  }
  //----------------------------------------------------------
  //　イベント処理
  //----------------------------------------------------------
  //Sceneの遷移は待機モードの時のみ可能
  if(Run == 0){
    //Scene番号をボタンを押すたびに次ぎの番号へ遷移させていく処理
    if (sw1 == 1 && temp1 == 0) {
      Scene++;
      //Scene：0~9まで
      if (Scene == 10) {
        Scene = 0;
      }
      temp1 = 1;
    }
  }
  //チャタリング防止処理
  if (sw1 == 0 && temp1 == 1) {
    temp1 = 0;
  }
  if (sw2 == 0 && temp2 == 1) {
    temp2 = 0;
  }
  
  //実行、停止
  if (sw2 == 1 && temp2 == 0) {
    Run = !Run;
  //走行中にプッシュスイッチが押された場合、各変数をリセット
    if (Run == 0) {
      Reset();
    }
    temp2 = 1;
  }

//----------------------------------------------------------
//----------------------------------------------------------
//　                     実行モード
//----------------------------------------------------------
//----------------------------------------------------------

  if(Run ==1){
  //----------------------------------------------------------
  //　実行モードに遷移時に1回だけ実行する処理
  //----------------------------------------------------------
    if (one == 0) {
        //モーター電源オン
        digitalWrite(ENABLE_L, LOW);
        digitalWrite(ENABLE_R, LOW);

        //Running...表示させるために1回ディスプレイを更新
        Oled_Update(voltage, Scene, Run);
        one = 1; 
      }
  //----------------------------------------------------------
  //　実行関数 周期：10ms
  //----------------------------------------------------------
  if ((currentMillis = millis()) - run_prevmillis >=  10) {
        switch (Scene) {
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
          //未定義表示
          case 5:
            Oled_Update(voltage, Scene, 2);
            Scene5();
            break;
          //未定義表示
          case 6:
            Oled_Update(voltage, Scene, 2);
            break;
          //未定義表示
          case 7:
            Oled_Update(voltage, Scene, 2);
            break;
          //未定義表示
          case 8:
            Oled_Update(voltage, Scene, 2);
            break;
          //未定義表示
          case 9:
            Oled_Update(voltage, Scene, 2);
            break;
          //ありえないとは思うが一応他番号になった場合にエラー表示
          default:
            Oled_Update(voltage, Scene, 3);
            break;
        }
       run_prevmillis = currentMillis;
      }
   }
   
}
