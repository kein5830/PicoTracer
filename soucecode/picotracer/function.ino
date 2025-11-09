//構造体インクルード
#include "types.h"
//flash メモリ 2025/06/09時点で未使用
#include <hardware/flash.h>

//---------------------------------------------------------------------
//@fn　ADコンバータ(mcp3002)の値取得関数
//@brief 2つのADコンバータの値を順番に取得する
//@param (int channel) ADコンバータのch番号 ch0 or ch1
//@param (int select) ２つ搭載しているADコンバータの指定 SELPIN1 or SELPIN2
//@return AD変換値（ラインセンサー値）
//@details 接続しているラインセンサーの値を取得する
//---------------------------------------------------------------------
int read_adc(int channel, int select) {
  int adcvalue = 0;
  //byte commandbits = B11000000;  //B(スタート)(1:シングルエンドモード)
  byte commandbits = 0b11000000;  //B(スタート)(1:シングルエンドモード)

  //allow channel selection
  commandbits |= (channel << 5);

  digitalWrite(select, LOW);  //Select adc
  // setup bits to be written
  for (int i = 7; i >= 4; i--) {
    digitalWrite(DATAOUT, commandbits & 1 << i);
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }

  digitalWrite(SPICLOCK, HIGH);  //ignores 2 null bits
  digitalWrite(SPICLOCK, LOW);
  //  digitalWrite(SPICLOCK,HIGH);
  //  digitalWrite(SPICLOCK,LOW);

  //read bits from adc
  for (int i = 9; i >= 0; i--) {
    adcvalue += digitalRead(DATAIN) << i;
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }
  digitalWrite(select, HIGH);  //turn off device
  return adcvalue;
}

//---------------------------------------------------------------------
//@fn　タイマー割り込み関数
//@brief タイマーに設定した周期で割り込む関数
//@param (struct repeating_timer *t) 構造体？詳細不明
//@return おそらく割り込まれて時にtrueを返すだけだと思うが未調査
//@details 現在はゴール後に一定距離走行し、停止するための距離測定に利用
//---------------------------------------------------------------------

// バッテリー電圧更新＋OLEDディスプレイ更新処理
bool callback0(struct repeating_timer *t) {  //割り込む関数
  // バッテリー電圧更新
  voltage = ((analogRead(VOLT) * 3.3 / 1024) * 6.3);//値修正
  // OLEDディスプレイ更新
  Oled_Update(voltage, Scene, Run); 
  return true;
}

// プッシュスイッチのON、OFF検知処理
bool callback1(struct repeating_timer *t) {  //割り込む関数
  sw1 = digitalRead(upswitch);
  sw2 = digitalRead(downswitch);
  return true;
}

// 現在の走行距離更新処理
bool callback2(struct repeating_timer *t) {  //割り込む関数
  //uint = float なので小数点は切り捨て
  NowDistance = Get_Distance(Step_L,Step_R);
  return true;
}

// bool callback3(struct repeating_timer *t) {  //割り込む関数

//   return true;
// }
//---------------------------------------------------------------------
//@fn　周波数＿周期変換関数
//@brief ステッピングモータ速度用の周波数を周期に変換する関数
//@param 周波数（ステッピングモータの速度）
//@return 周期（タイマ関数への入力専用）
//@details// HIGH/LOW 1サイクルあたり
//?はif文と同じ　freq > 0なら1000000.0 / (freq * 2) 走じゃないなら 1000000を返す
//1000000.0 / (freq * 2) 　1000000.0はs -> us変換　freq * 2は パルス波のHIGHとLOWの２個分があるためその分の計算 　
//---------------------------------------------------------------------
unsigned long frequencyToInterval(float freq) {
  return (freq > 0) ? (1000000.0 / (freq * 2)) : 1000000;  
}

//---------------------------------------------------------------------
//@fn　周波数からwrap値に変換する関数　→　未使用
//@brief 周波数を入力するとwrap値が出力される
//@param (float pulsefreq):周波数(Hz)
//@return wrap値
//@details モーター速度をPWMで制御するためにwrap値というものが必要周波数の方がわかりやすいため
//　　　　　ユーザーからの入力は周波数とし、変換する関数で対応
//---------------------------------------------------------------------
uint16_t Hz_wrap(float pulsefreq){
  //F=sysclock/(wrap+1)*clkdiv
  uint32_t Wrap =(125000000/(pulsefreq*2*100))-1;
  if(Wrap > 65535){
    Wrap = 65535;
  }  
  return Wrap;
  //125000000:raspipicoのシステムクロック 100:システムクロックを100分割していること
}

//---------------------------------------------------------------------
//@fn　距離取得 (mm)
//@brief Step数から距離を計算する関数
//@param (uint64_t Step_L,R) ステップ数 
//@return float
//@details 実行メニューや、走行に仕様する変数をリセットする
//---------------------------------------------------------------------
float Get_Distance(uint64_t StepL,uint64_t StepR){
  float Temp = 0.0;
  //StepL/2:Stepは立ち上がり、立下がりを合わせた値なので、パルスの数はその半分
  //(StepL/2.0 + StepR/2.0)/2.0:LとRを合わせて平均値をとる
  Temp = (StepL/2.0 + StepR/2.0)/2.0;
  //総ステップ数×ステップ距離=総距離(mm) -20.0は現実の値に合わせるための調整値 step角：1.8°
  return (Temp * 0.785398163375)-20.0;
}

//---------------------------------------------------------------------
//@fn　リセット関数
//@brief 実行メニューや、走行に仕様する変数をリセットする
//@param なし
//@return void
//@details 実行メニューや、走行に仕様する変数をリセットする
//---------------------------------------------------------------------
void Reset(){
  //プッシュスイッチカウント
  Scene = 0;
  //走行距離
  distance=0,Step=0;
  //一回しか実行しないための変数（なぜかｂしか使ってない）
  b = 0, c = 0,one = 0;
  //入力速度
  PID_Result = 0;

  //PID制御
  P = 0.0;
  D = 0.0;
  I = 0.0;
  diff = 0;
  bias = 0;
  beforediff = 0;
  sum = 0;
  //ラインカウンタ
  count = 0, cross = 0,curve_count = 0;
  tmp = 0, tmpc = 0;
  //PWM停止
  // pwm_set_enabled(pwm_slice1, false);
  // pwm_set_enabled(pwm_slice2, false);
  delay(100);
  //モーター電源オフ
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);
  //Scene4スピード変数リセット
  Speed = 0.0;
  //タイマ停止
  TimerSW = 0;
  //距離計測停止
  StepSW == 0;
  // Serial.print("Reset");
  interval = 0;

  Step_L = 0;
  Step_R = 0;
  NowDistance = 0;
  //ディスプレイ表示、バッテリ電圧更新タイマー再開 
  ITimer0.attachInterrupt(10, callback0);
  // プッシュスイッチのON、OFF検知処理
  ITimer1.attachInterrupt(20, callback1);  //左モーター
}

//---------------------------------------------------------------------
//@fn　OLED表示更新関数
//@brief OLEDの表示内容を指定し、更新する
//@param (float volt) バッテリの電圧値
//@param (float runmode) 実行番号 
//@return void
//@details 実行メニューや、走行に仕様する変数をリセットする
//---------------------------------------------------------------------
void Oled_Update(float volt,int runscene,uint8_t runmode ){
  //点滅表示用処理
  static uint8_t count = 0;
  static bool ON = 0;
  count++;
//  Serial.print(millis());
//  Serial.print(" ");
//  Serial.println(count);
  if(count>=5){
    ON = !ON;
    count = 0;
    }
    
  display.clearDisplay();     // 表示クリア
  
  // タイトル表示
  display.setTextSize(2);     // 文字サイズ（1）
  display.setCursor(4, 0);    // 表示開始位置左上角（X,Y）
  display.println("PicoTracer");    // 表示内容

  //図形表示  
  display.drawLine(0, 20, 128, 20, WHITE);   // 線（始点終点指定）
  display.drawFastVLine(64, 22, 17, WHITE);  // 線（指定座標から垂線）
  display.drawFastHLine(0, 40, 128, WHITE);  // 線（指定座標から平行線）
  
  //電圧値
  display.setTextSize(2);
  display.setCursor(4, 22);
  display.println(volt);

  // No.
  display.setTextSize(2);
  display.setCursor(69, 22);
  display.println("No.");
  
  //走行モード
  
  display.setTextSize(2);
  display.setCursor(110, 22);
  display.println(runscene);

  //待機中表示
  if(ON == 1 && runmode == 0){
    display.setTextSize(2);
    display.setCursor(4, 44);
    display.println("Select No.");
  }else if(runmode == 1){
        //実行表示
        display.setTextSize(2);
        display.setCursor(4, 44);
        display.println("Running...");
    }else if(runmode == 2){
        //未定義表示
        display.setTextSize(2);
        display.setCursor(4, 44);
        display.println("No Action");
    }else if(runmode == 3){
        //エラー表示
        display.setTextSize(2);
        display.setCursor(4, 44);
        display.println("No. Error");
    }
  
  display.display();  // 表示実行
}

//---------------------------------------------------------------------
//@fn　Flashメモリへ書き込み処理（中）
//@brief 
//@param 
//@param 
//@return 
//@details 
//---------------------------------------------------------------------

/*W25Q16JVのBlock12のセクタ0の先頭アドレス = 0x2c0000
ブロック番号  開始アドレス  終了アドレス
//1走目
20  0x2C0000  0x2FFFFF
21  0x300000  0x33FFFF
22  0x340000  0x37FFFF
23  0x380000  0x3BFFFF
//2走目
24  0x3C0000  0x3FFFFF
25  0x400000  0x43FFFF
26  0x440000  0x47FFFF
27  0x480000  0x4BFFFF
//3走目
28  0x4C0000  0x4FFFFF
29  0x500000  0x53FFFF
30  0x540000  0x57FFFF
31  0x580000  0x5BFFFF
*/
//工事中
//const uint32_t FLASH_TARGET_OFFSET[3] = {0x2c0000,0x3C0000,0x4C0000};
//
//uint8_t Write_data_temp1=0,Write_data_temp2=0;
//uint8_t Write_data[FLASH_PAGE_SIZE];
//uint16_t i,count=0,temp=0; 
//
//bool Flash_Write(){
//      for(i=0;i<=log_count;i++){
//          //値を取得して、分割して代入
//          if((i+1)%2 != 0){
//            //奇数
//            Write_data_temp1=data_log[count].Curve_log >> 8;
//            Write_data[i]=Write_data_temp1;
//          }else{
//            //偶数
//            Write_data_temp2=data_log[count].Curve_log - (Write_data_temp1<<8);
//            Write_data[i]=Write_data_temp2;
//            count++;
//          }
//          //256Byteごとにフラッシュに書き込み処理を実行 
//          if((i+1)%256 == 0){        
//              // 割り込み無効にする
//              uint32_t ints = save_and_disable_interrupts();
//              // Flash消去。
//              //  消去単位はflash.hで定義されている FLASH_SECTOR_SIZE(4096Byte) の倍数とする
//              flash_range_erase(FLASH_TARGET_OFFSET[0], FLASH_SECTOR_SIZE);
//              // Flash書き込み。
//              //  書込単位はflash.hで定義されている FLASH_PAGE_SIZE(256Byte) の倍数とする
//              flash_range_program(FLASH_TARGET_OFFSET[0], write_data_temp, FLASH_PAGE_SIZE);
//              // 割り込みフラグを戻す
//              restore_interrupts(ints);
//          }
//          
//      }
//      //log_count分終了したのでフラッシュに書きこみ処理を実行
//      
//    return 1;
//  }