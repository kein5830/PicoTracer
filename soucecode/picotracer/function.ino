//構造体インクルード
#include "types.h"
//flash メモリ
#include <hardware/flash.h>
/*
---------------------------------------------------------------------
@fn　ADコンバータ(mcp3002)の値取得関数
@brief 2つのADコンバータの値を順番に取得する
@param (int channel) ADコンバータのch番号 ch0 or ch1
@param (int select) ２つ搭載しているADコンバータの指定 SELPIN1 or SELPIN2
@return AD変換値（ラインセンサー値）
@details 接続しているラインセンサーの値を取得する
---------------------------------------------------------------------
 */
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

/*
---------------------------------------------------------------------
@fn　タイマー割り込み関数
@brief タイマーに設定した周期で割り込む関数
@param (struct repeating_timer *t) 構造体？詳細不明
@return おそらく割り込まれて時にtrueを返すだけだと思うが未調査
@details 現在はゴール後に一定距離走行し、停止するための距離測定に利用
---------------------------------------------------------------------
 */
bool TimerHandler0(struct repeating_timer *t) {  //割り込む関数
  //距離測定用
    static int del = 0;
    del++;
    if (del >= 2) {
       Step++;
       distance = Step * 1.5;
       del = 0;
     }
  Serial.print(" distance:");
  Serial.println(distance);

  return true;
}


/*
---------------------------------------------------------------------
@fn　周波数＿周期変換関数
@brief ステッピングモータ速度用の周波数を周期に変換する関数
@param 周波数（ステッピングモータの速度）
@return 周期（タイマ関数への入力専用）
@details// HIGH/LOW 1サイクルあたり
  //?はif文と同じ　freq > 0なら1000000.0 / (freq * 2) 走じゃないなら 1000000を返す
  //1000000.0 / (freq * 2) 　1000000.0はs -> us変換　freq * 2は パルス波のHIGHとLOWの２個分があるためその分の計算 　
---------------------------------------------------------------------
 */
unsigned long frequencyToInterval(float freq) {
  return (freq > 0) ? (1000000.0 / (freq * 2)) : 1000000;  
}

/*
---------------------------------------------------------------------
@fn　周波数からwrap値に変換する関数
@brief 周波数を入力するとwrap値が出力される
@param (float pulsefreq):周波数(Hz)
@return wrap値
@details モーター速度をPWMで制御するためにwrap値というものが必要周波数の方がわかりやすいため
　　　　　ユーザーからの入力は周波数とし、変換する関数で対応
---------------------------------------------------------------------
 */
uint16_t Hz_wrap(float pulsefreq){
  //F=sysclock/(wrap+1)*clkdiv
  uint32_t Wrap =(125000000/(pulsefreq*2*100))-1;
  if(Wrap > 65535){
    Wrap = 65535;
  }  
  return Wrap;
  //125000000:raspipicoのシステムクロック 100:システムクロックを100分割していること
}

/*
---------------------------------------------------------------------
@fn　距離取得
@brief Step数から距離を計算する関数
@param (uint64_t Step_L,R) ステップ数 
@return float
@details 実行メニューや、走行に仕様する変数をリセットする
---------------------------------------------------------------------
 */
float Get_Distance(uint64_t StepL,uint64_t StepR){
  float Temp = 0.0;
  //StepL/2:Stepは立ち上がり、立下がりを合わせた値なので、パルスの数はその半分
  //(StepL/2.0 + StepR/2.0)/2.0:LとRを合わせて平均値をとる
  Temp = (StepL/2.0 + StepR/2.0)/2.0;
  //総ステップ数×ステップ距離=総距離(mm) -20.0は現実の値に合わせるための調整値
  return (Temp * 0.785398163375)-20.0;
}

/*
---------------------------------------------------------------------
@fn　リセット関数
@brief 実行メニューや、走行に仕様する変数をリセットする
@param なし
@return void
@details 実行メニューや、走行に仕様する変数をリセットする
---------------------------------------------------------------------
 */
void Reset(){
  //プッシュスイッチカウント
  Scene = 0;
  //走行距離
  distance=0,Step=0;
  //一回しか実行しないための変数（なぜかｂしか使ってない）
  b = 0, c = 0,one = 0;
  //入力速度
  inputL = 0;
  inputR = 0;
  //PID制御
  P = 0.0;
  D = 0.0;
  I = 0.0;
  diff = 0;
  bias = 0;
  beforediff = 0;
  sum = 0;
  //ラインカウンタ
  count = 0, cross = 0;
  tmp = 0, tmpc = 0;
  //PWM停止
  pwm_set_enabled(pwm_slice1, false);
  pwm_set_enabled(pwm_slice2, false);
  delay(100);
  //モーター電源オフ
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);
  //初回しか実行しない変数リセット
  one = 0;
  //Scene4スピード変数リセット
  Speed = 0.0;
  //タイマ停止
  TimerSW = 0;
  //距離計測停止
  StepSW == 0;
  Serial.print("Reset");
}

/*
---------------------------------------------------------------------
@fn　OLED表示更新関数
@brief OLEDの表示内容を指定し、更新する
@param (float volt) バッテリの電圧値
@param (float runmode) 実行番号 
@return void
@details 実行メニューや、走行に仕様する変数をリセットする
---------------------------------------------------------------------
 */
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

/*
---------------------------------------------------------------------
@fn　Flashメモリへ書き込み処理（中）
@brief 
@param 
@param 
@return 
@details 
---------------------------------------------------------------------
 */

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

/*
---------------------------------------------------------------------
@fn　走行関数Scene0
@brief ライントレース走行
@param なし
@return void
@details　PI制御　スタート時のゆっくり加速なし 最高速度450 定速
---------------------------------------------------------------------
 */
void Scene0() {
  //*******************************************************************
  //メイン走行スピード、PIDのゲイン値、その他ローカル変数定義
  //*******************************************************************
  

  if(c == 0){
    //基準速度
    SP = 400;
    //Pゲイン
    pgain = 0.35;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0;
    
    // 1回しか入らないようcを1にする
    c=1;
  }

  //*******************************************************************
  //センサー値格納　（202412月17日に配置更新）
  //*******************************************************************
  sensorCurve = analogRead(Curve_Sensor);
  sensorLL = read_adc(ch1, SELPIN1);//sensor ll
  sensorL = read_adc(ch0, SELPIN1)-10;//sensor l
  sensorR = read_adc(ch1, SELPIN2);//sensor r
  sensorRR = read_adc(ch0, SELPIN2)+20;//sensor rr
  sensorGoal = analogRead(GOALSENSOR);//

  //必要であれば
  Serial.print(curve_count, DEC);
  Serial.print(" ");
  Serial.print(count, DEC);
  Serial.print(" ");
  Serial.print(cross, DEC);
  Serial.print(" "); 
  Serial.print(sensorGoal, DEC);
  Serial.print(" ");
  Serial.print(sensorCurve, DEC);
  Serial.print(" ");
  Serial.print(tmpc, DEC);
  Serial.println(" ");

  //*******************************************************************
  //ラインカウンタ制御  
  //*******************************************************************
  //クロスラインを検知したら、クロスフラグをON
  if (tmpc == 0 && sensorL < 300 && sensorR < 300 && sensorLL < 300 && sensorRR < 300) {
    temp_distance = NowDistance;
    cross = 1;
    tmpc = 1;
    }
  if ( tmpc == 1 && sensorLL > 700 && sensorRR > 700) {
    tmpc = 0;
  }

  //ゴールセンサが検知したらtmpフラグをON
  if (tmp == 0 && sensorGoal < 300 ){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
    if(cross != 1){
      count++;
      //BUZZER入れたが音小さくて聞こえない
      tone(BUZZER,1178,100);
      StepSW = !StepSW;
    }
    tmp = 1;
  }
  //ゴールセンサが検知しなくなったらクロスフラグを確認し、ゴールマーカーを追加するか判断
  if (tmp == 1 && sensorGoal > 700) {
    tmp = 0;
  }

  // //カーブマーカー検知
  if (curve_temp == 0  && sensorCurve < 300){
      if(cross != 1){
      curve_count++;
      //BUZZER入れたが音小さくて聞こえない
      tone(BUZZER,1178,100);
    }
    curve_temp = 1;
  }
  if (sensorCurve > 700) {
    curve_temp = 0;
  }
  //クロス検知後から35mm進んだらおそらくゴール、カーブセンサは過ぎてるだろうからクロスフラグを戻す（誤検知する場合は調整する）
  if((NowDistance-temp_distance) >= 35){
    cross = 0;
  }

  // // スタートマーカー通過後に距離計測開始
  // if(count == 1){
  //   StepSW = 1;
  // }
  // LineDitection[0] = Get_Direction(Step_L,Step_R)

  //ゴール後少し進んで停止
  if (count == 2) { 
    if (b == 0) {
      //タイマースタート処理
      ITimer0.stopTimer();
      ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
      Step = 0;
    }
    b=1;
    
  }
  if (distance > 100) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      Reset();
      Run = 0;
      tone(BUZZER,1446,1000); 
    }

  Add_SensorL = sensorL + sensorLL;
  Add_SensorR = sensorR + sensorRR;

  //今回の差分
  diff = Add_SensorL - Add_SensorR - bias;  //biasは試走会で調整

  //-50~50なら差分なしとする
  if(diff<=650 && diff>=-650){
    diff=0;
  }
  //  Serial.print("diff:");
  //  Serial.print(diff);
  sum += diff;
  //P制御
  P = diff * pgain;
  //D制御
  D = (beforediff - diff) * dgain;
  //I制御
  I = sum * igain;
  //入力速度にPID制御値を代入
  //Dを-にしてPで発生するオーバーシュートを抑える形にすることも検討
  inputL = SP + (P + D + I);
  inputR = SP - (P + D + I);

  //前回の差分を保存
  beforediff = diff;

  //最大値補正
  if (inputL > (SP+100)) {
    inputL = SP+100;
  }
  if (inputR > (SP+100)) {
    inputR = (SP+100);
  }
  //最小値補正
  if (inputL < 0) {
    inputL = 0;
  }
  if (inputR < 0) {
    inputR = 0;
  }

  //クロス通過時は速度を固定する
  if (sensorLL < 400 && sensorRR < 400) {
    inputL = SP;
    inputR = SP;
  }

  //速度設定 Slice1＝Reft Slice2=Left
  // pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  // pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));
  //PWMスタート
  // pwm_set_enabled(pwm_slice1, true);
  // pwm_set_enabled(pwm_slice2, true);

  //距離計測開始
  // StepSW = 1; 
  //下記のようにすることで、モータを動作させる
  intervalR = frequencyToInterval(inputR);//R
  intervalL = frequencyToInterval(inputL);//L

  // Serial.print(" Step_L:");
  // Serial.print(Step_L);
  // Serial.print(" Step_R:");
  // Serial.print(Step_R);
  // Serial.print("Direction:");
  // Serial.println(Get_Direction(Step_L,Step_R));
  //  if (Get_Direction(Step_L,Step_R) > 300) {  //停止位置は試走会で調整

  //     Reset();
  //     Run = 0;
  //   }



  //ログ保存スタートボタンを押してから
  // if(count>=1 && count < 2){
    data_log[log_count].Curve_log = count;
    data_log[log_count].LL_log = sensorLL;
    data_log[log_count].L_log = sensorL;
    data_log[log_count].R_log = sensorR;
    data_log[log_count].RR_log = sensorRR;
    data_log[log_count].Goal_log = curve_count;
//    data_log[log_count].R_motor_log = inputR;
//    data_log[log_count].L_motor_log = inputL;
    log_count++;  
    
}

//基本走行
//P制御走行　カーブがきつくゲインをあげすぎるとがたがた　厳しそう
/*
---------------------------------------------------------------------
@fn　走行関数Scene1　＞＞＞旧方式でのモータ制御
@brief ライントレース走行
@param なし
@return void
@details　P制御のみ　スタート時のゆっくり加速あり+1　最高速度850
---------------------------------------------------------------------
 */
void Scene1() {
//  //ログ保存用構造体　宣言
//  static Log data_log[10000];
/*
限界感度法
P=0.6* 0.34=0.204
D=0.125*0.075=0.009375
I=0.5*0.075=0.0375

P=0.45*0.34=0.153
I=0.83*0.075=0.06225
*/
  if(c == 0){
    //基準速度
    SP = 0;
    //Pゲイン
    pgain = 0.38;
    //Dゲイン
    dgain = 0.000;
    //Iゲイン
    igain = 0.0;
    // 1回しか入らないようcを1にする
    c=1;
  }

//センサー配置修正12/17
  sensorCurve = analogRead(Curve_Sensor);
  sensorLL = read_adc(ch1, SELPIN1);//sensor ll
  sensorL = read_adc(ch0, SELPIN1)-10;//sensor l
  sensorR = read_adc(ch1, SELPIN2)-10;//sensor r
  sensorRR = read_adc(ch0, SELPIN2)+20;//sensor rr
  sensorGoal = analogRead(GOALSENSOR);

//  Serial.print(Curve, DEC);
//  Serial.print(" ");
//  Serial.print(sensorLL, DEC)
//  Serial.print(" ");
//  Serial.print(sensorL, DEC);
//  Serial.print(" "); 
//  Serial.print(sensorR, DEC);
//  Serial.print(" ");
//  Serial.print(sensorRR, DEC);
//  Serial.print(" ");
//  Serial.print(sensorGoal, DEC);
//  Serial.print(" ");


//ラインクロスカウンタ
  if (tmpc == 0 &&   sensorL < 300 && sensorR < 300 && sensorLL < 300 && sensorRR < 300) {  //速度によって調整
    cross++;
    //BUZZER入れたが音小さくて聞こえない
//    tone(BUZZER,1234,100);
    tmpc = 1;
  }
  if (sensorLL > 700 && sensorRR > 700) {
    tmpc = 0;
  }

  //ゴールセンサーカウンタ
  if (tmp == 0 && sensorGoal < 300 ){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
    count++;
    tmp = 1;
    count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
    cross = cross - cross;
    //BUZZER入れたが音小さくて聞こえない
//    tone(BUZZER,1178,100);
  }
  if (sensorGoal > 700) {
    tmp = 0;
  }
  // static uint8_t temp_count = 0; 
  // static bool ignore = 0;
  // // //カーブセンサ
  // if (Curve < 300 && ignore == 0){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
  //   // pgain = 3.0;
  //   // pgain = 0.17;
  //   // tmp = 1;
  //   // curve_count = curve_count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
  //   // cross = cross - cross;
  //   temp_count++;
  //   if (sensorGoal < 300) {
  //     ignore = 1;
  //   }    
  //   //BUZZER入れたが音小さくて聞こえない
  // //tone(BUZZER,1178,100);
  // }
  // //30ms以内に反対側のセンサが反応しないので、カーブマーカーと認識
  // if(temp_count >= 2){
  //   curve_count = ! curve_count;
  //   temp_count = 0;
  // }
  // //マーカーを抜けたら、カウント、ignoreをリセット
  // if(Curve > 700){
  //   temp_count = 0;
  //   ignore = 0;
  // }
  // //カーブ中なら、ゲインを高くする。
  // if(curve_count == 1){
  //   pgain = 0.3;
  // }else{
  //   pgain = 0.16;
  // }

  SP=SP + 5;
  if(SP>900.0){
    SP=900.0;
  }
  
  //ゴール後少し進んで停止
  if (count == 2) { 
    if (b == 0) {
      //タイマースタート処理
      ITimer0.stopTimer();
      ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
      Step = 0;
    }
    b=1;
    
  }
  if (distance > 100) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      Reset();
      tone(BUZZER,1446,1000); 
      
      Run = 0;
    }

  Add_SensorL = sensorL + sensorLL;
  Add_SensorR = sensorR + sensorRR;

  //今回の差分
  diff = Add_SensorL - Add_SensorR - bias;  //biasは試走会で調整
  
  //-50~50なら差分なしとする（）
  if(diff<=650 && diff>=-650){
    diff=0;
  }
  //  Serial.print("diff:");
  //  Serial.print(diff);
  sum += (diff*0.01);
  //P制御
  P = diff * pgain;
  //D制御
  D = (beforediff - diff)/0.01 * dgain;
  //I制御
  I = sum * igain;
  //SP350までは制御しない
  if(SP >= 350){
  //入力速度にPID制御値を代入
  inputL = SP + (P + D + I);
  inputR = SP - (P + D + I);
  }else{
  inputL = SP;
  inputR = SP;    
  }
  //前回の差分を保存
  beforediff = diff;

  //最大値補正
  if (inputL > (SP+200)) {
    inputL = SP+200;
  }
  if (inputR > (SP+200)) {
    inputR = (SP+200);
  }
  //最小値補正
  if (inputL < 0) {
    inputL = 0;
  }
  if (inputR < 0) {
    inputR = 0;
  }

  //クロス通過時は速度を固定する
  if (sensorLL < 500 && sensorRR < 500) {
    inputL = SP;
    inputR = SP;
  }

  //速度設定 Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

  //PWMスタート
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);

  // Serial.print(" P:");
  // Serial.print(P);
  // Serial.print(" D:");
  // Serial.print(D);
  // Serial.print(" I:");
  // Serial.print(I);
  // Serial.print(" inputL:");
  // Serial.print(inputL);
  // Serial.print(" inputR:");
  // Serial.print(inputR);
  // Serial.print(" count:");
  // Serial.print(count);
  // Serial.print(" cross:");
  // Serial.print(cross);
  // Serial.print(" tmp:");
  // Serial.print(distance);
  // Serial.print(" SP:");
  // Serial.println(SP);

  //ログ保存(スタートラインを越えてからゴールするまで)
  // if(count>=1 && count < 2){
    data_log[log_count].Curve_log = inputL;
    data_log[log_count].LL_log = sensorLL;
    data_log[log_count].L_log = sensorL;
    data_log[log_count].R_log = sensorR;
    data_log[log_count].RR_log = sensorRR;
    data_log[log_count].Goal_log = inputR;
//    data_log[log_count].R_motor_log = inputR;
//    data_log[log_count].L_motor_log = inputL;
    
    log_count++;  
    // }
  
}

/*
---------------------------------------------------------------------
@fn　走行関数Scene2 各Sceneの扱い検討中
@brief ライントレース走行
@param なし
@return void
@details　PI制御　スタート時のゆっくり加速あり　+0.25 最高速度700
---------------------------------------------------------------------
 */
// void Scene2() {
  
//  //  //ログ保存用構造体　宣言
// //  static Log data_log[10000];
// /*
// 限界感度法
// P=0.6* 0.34=0.204
// D=0.125*0.075=0.009375
// I=0.5*0.075=0.0375

// P=0.45*0.34=0.153
// I=0.83*0.075=0.06225
// */
//   if(c == 0){
//     //基準速度
//     SP = 0;
//     //Pゲイン
//     pgain = 0.4;
//     //Dゲイン
//     dgain = 0.000;
//     //Iゲイン
//     igain = 0.0;
//     // 1回しか入らないようcを1にする
//     c=1;
//   }

// //センサー配置修正12/17
//   Curve = analogRead(Curve_Sensor);
//   sensorLL = read_adc(ch1, SELPIN1);//sensor ll
//   sensorL = read_adc(ch0, SELPIN1)-10;//sensor l
//   sensorR = read_adc(ch1, SELPIN2)-10;//sensor r
//   sensorRR = read_adc(ch0, SELPIN2)+20;//sensor rr
//   sensorGoal = analogRead(GOALSENSOR);

// //  Serial.print(Curve, DEC);
// //  Serial.print(" ");
// //  Serial.print(sensorLL, DEC)
// //  Serial.print(" ");
// //  Serial.print(sensorL, DEC);
// //  Serial.print(" "); 
// //  Serial.print(sensorR, DEC);
// //  Serial.print(" ");
// //  Serial.print(sensorRR, DEC);
// //  Serial.print(" ");
// //  Serial.print(sensorGoal, DEC);
// //  Serial.print(" ");


// //ラインクロスカウンタ
//   if (tmpc == 0 &&   sensorL < 300 && sensorR < 300 && sensorLL < 300 && sensorRR < 300) {  //速度によって調整
//     cross++;
//     //BUZZER入れたが音小さくて聞こえない
// //    tone(BUZZER,1234,100);
//     tmpc = 1;
//   }
//   if (sensorLL > 700 && sensorRR > 700) {
//     tmpc = 0;
//   }

//   //ゴールセンサーカウンタ
//   if (tmp == 0 && sensorGoal < 300 ){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
//     count++;
//     tmp = 1;
//     count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
//     cross = cross - cross;
//     //BUZZER入れたが音小さくて聞こえない
// //    tone(BUZZER,1178,100);
//   }
//   if (sensorGoal > 700) {
//     tmp = 0;
//   }
//   // static uint8_t temp_count = 0; 
//   // static bool ignore = 0;
//   // // //カーブセンサ
//   // if (Curve < 300 && ignore == 0){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
//   //   // pgain = 3.0;
//   //   // pgain = 0.17;
//   //   // tmp = 1;
//   //   // curve_count = curve_count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
//   //   // cross = cross - cross;
//   //   temp_count++;
//   //   if (sensorGoal < 300) {
//   //     ignore = 1;
//   //   }    
//   //   //BUZZER入れたが音小さくて聞こえない
//   // //tone(BUZZER,1178,100);
//   // }
//   // //30ms以内に反対側のセンサが反応しないので、カーブマーカーと認識
//   // if(temp_count >= 2){
//   //   curve_count = ! curve_count;
//   //   temp_count = 0;
//   // }
//   // //マーカーを抜けたら、カウント、ignoreをリセット
//   // if(Curve > 700){
//   //   temp_count = 0;
//   //   ignore = 0;
//   // }
//   // //カーブ中なら、ゲインを高くする。
//   // if(curve_count == 1){
//   //   pgain = 0.3;
//   // }else{
//   //   pgain = 0.16;
//   // }

//   SP=SP + 5;
//   if(SP>1200.0){
//     SP=1200.0;
//   }
  
//   //ゴール後少し進んで停止
//   if (count == 2) { 
//     if (b == 0) {
//       //タイマースタート処理
//       ITimer0.stopTimer();
//       ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
//       Step = 0;
//     }
//     b=1;
    
//   }
//   if (distance > 100) {  //停止位置は試走会で調整
//       ITimer0.stopTimer();
//       Reset();
//       tone(BUZZER,1446,1000); 
      
//       Run = 0;
//     }

//   Add_SensorL = sensorL + sensorLL;
//   Add_SensorR = sensorR + sensorRR;

//   //今回の差分
//   diff = Add_SensorL - Add_SensorR - bias;  //biasは試走会で調整
  
//   //-50~50なら差分なしとする（）
//   if(diff<=625 && diff>=-625){
//     diff=0;
//   }

//   // if(diff>=900 || diff <= -900){
//   //   SP=900;
//   // }

//   //  Serial.print("diff:");
//   //  Serial.print(diff);
//   sum += (diff*0.01);
//   //P制御
//   P = diff * pgain;
//   //D制御
//   D = (beforediff - diff)/0.01 * dgain;
//   //I制御
//   I = sum * igain;
//   //SP350までは制御しない
//   if(SP >= 350){
//   //入力速度にPID制御値を代入
//   inputL = SP + (P + D + I);
//   inputR = SP - (P + D + I);
//   }else{
//   inputL = SP;
//   inputR = SP;    
//   }
//   //前回の差分を保存
//   beforediff = diff;

//   //最大値補正
//   if (inputL > (SP+200)) {
//     inputL = SP+200;
//   }
//   if (inputR > (SP+200)) {
//     inputR = (SP+200);
//   }
//   //最小値補正
//   if (inputL < 0) {
//     inputL = 0;
//   }
//   if (inputR < 0) {
//     inputR = 0;
//   }

//   //クロス通過時は速度を固定する
//   if (sensorLL < 500 && sensorRR < 500) {
//     inputL = SP;
//     inputR = SP;
//   }

//   //速度設定 Slice1＝Reft Slice2=Left
//   pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
//   pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

//   //PWMスタート
//   pwm_set_enabled(pwm_slice1, true);
//   pwm_set_enabled(pwm_slice2, true);

//   // Serial.print(" P:");
//   // Serial.print(P);
//   // Serial.print(" D:");
//   // Serial.print(D);
//   // Serial.print(" I:");
//   // Serial.print(I);
//   // Serial.print(" inputL:");
//   // Serial.print(inputL);
//   // Serial.print(" inputR:");
//   // Serial.print(inputR);
//   // Serial.print(" count:");
//   // Serial.print(count);
//   // Serial.print(" cross:");
//   // Serial.print(cross);
//   // Serial.print(" tmp:");
//   // Serial.print(distance);
//   // Serial.print(" SP:");
//   // Serial.println(SP);

//   //ログ保存(スタートラインを越えてからゴールするまで)
//   // if(count>=1 && count < 2){
//     data_log[log_count].Curve_log = inputL;
//     data_log[log_count].LL_log = sensorLL;
//     data_log[log_count].L_log = sensorL;
//     data_log[log_count].R_log = sensorR;
//     data_log[log_count].RR_log = sensorRR;
//     data_log[log_count].Goal_log = diff;
// //    data_log[log_count].R_motor_log = inputR;
// //    data_log[log_count].L_motor_log = inputL;
    
//     log_count++;  
//     // }
  
// }


//2023/11/04
//完走 accelrun　検証中
/*
---------------------------------------------------------------------
@fn　走行関数Scene3
@brief ライントレース走行
@param なし
@return void
@details　PI制御　スタート時のゆっくり加速あり+0.35 最高速度800
---------------------------------------------------------------------
 */
// void Scene3() {
  
//   if(c == 0){
//     //基準速度
//     SP = 0;
//     //Pゲイン
//     pgain = 0.32;
//     //Dゲイン
//     // dgain = 3;
//     //Iゲイン
//     igain = 0.0023;
//     // 1回しか入らないようcを1にする
//     c = 1;
//   }
         
//   Curve = analogRead(Curve_Sensor);
//   sensorLL = read_adc(ch1, SELPIN1);//sensor ll
//   sensorL = read_adc(ch0, SELPIN1);//sensor l
//   sensorR = read_adc(ch1, SELPIN2);//sensor r
//   sensorRR = read_adc(ch0, SELPIN2);//sensor rr
//   sensorGoal = analogRead(GOALSENSOR);

// //  Serial.print(Curve, DEC);
// //  Serial.print(" ");
// //  Serial.print(sensorLL, DEC)
// //  Serial.print(" ");
// //  Serial.print(sensorL, DEC);
// //  Serial.print(" "); 
// //  Serial.print(sensorR, DEC);
// //  Serial.print(" ");
// //  Serial.print(sensorRR, DEC);
// //  Serial.print(" ");
// //  Serial.print(sensorGoal, DEC);
// //  Serial.print(" ");

//   //ゴールセンサーカウンタ
//   if (tmp == 0 && sensorGoal < 300) {  //速度によって調整
//     count++;
//     tmp = 1;
//     count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
//     cross = cross - cross;
//   }
//   if (sensorGoal > 900) {
//     tmp = 0;
//   }

//   //ラインクロスカウンタ
//   if (tmpc == 0 && sensorLL < 300 && sensorL < 300 && sensorR < 300) {  //速度によって調整
//     cross++;
//     tmpc = 1;
//   }
//   if (sensorLL > 500) {
//     tmpc = 0;
//   }

//   //今回の差分
//   diff = sensorL - sensorR - bias;  //biasは試走会で調整

//   //  Serial.print("diff:");
//   //  Serial.print(diff);
//   sum += diff;
//   //P制御
//   P = diff * pgain;
//   //D制御
//   D = (beforediff - diff) * dgain;
//   //I制御
//   I = sum * igain;

// // 　初速が遅い時はゲインを弱くする 実現できず、、、
// // if(inputL < 400 || inputR < 400){
// //     //Pゲイン
// //     pgain = 0.3;
// //     //Dゲイン
// //     dgain = 0;
// //     //Iゲイン
// //     igain = 0.0004;
// //   }
//   //入力速度にPID制御値を代入
//   inputL = SP + (P + D + I);
//   inputR = SP - (P + D + I);

//   //前回の差分を保存
//   beforediff = diff;
//   //速度設定 Slice1＝Reft Slice2=Left
//   pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
//   pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

//   //PWMスタート
//   pwm_set_enabled(pwm_slice1, true);
//   pwm_set_enabled(pwm_slice2, true);

//   //最大値補正
//   if (inputL > (SP+100)) {
//     inputL = SP+100;
//   }
//   if (inputR > (SP+100)) {
//     inputR = (SP+100);
//   }
//   //最小値補正
//   if (inputL < 0) {
//     inputL = 0;
//   }
//   if (inputR < 0) {
//     inputR = 0;
//   }

//   //クロス通過時は速度を固定する
//   if (sensorLL < 300 && sensorRR < 300) {
//     inputL = SP;
//     inputR = SP;
//   }

//   SP=SP + 1;
//   if(SP>800.0){
//     SP=800.0;
//   }

//   //ゴール後少し進んで停止
//   if (count == 2) {  //いいいいいいいいいいいいいいいいいいいいいいいいいいいい一時的
//     if (b == 0) {
//       //タイマースタート処理
//       ITimer0.stopTimer();
//       ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
//       Step = 0;
//     }
//     b=1;
//     //  Serial.print(" tmp=");
//     //  Serial.println(tmp);
    
//   }
//   if (distance > 200) {  //停止位置は試走会で調整
//       ITimer0.stopTimer();
//       inputL=0.0;
//       inputR=0.0;
//       Reset();
//       Run = 0;
//     }

// //  Serial.print(" P:");
// //  Serial.print(P);
// //  Serial.print(" D:");
// //  Serial.print(D);
// //  Serial.print(" I:");
// //  Serial.print(I);
// //  Serial.print(" inputL:");
// //  Serial.print(inputL);
// //  Serial.print(" inputR:");
// //  Serial.print(inputR);
// //  Serial.print(" count:");
// //  Serial.print(count);
// //  Serial.print(" cross:");
// //  Serial.print(cross);
// //  Serial.print(" tmp:");
// //  Serial.print(distance);
// //  Serial.print(" SP:");
// //  Serial.println(SP);

// }

/*
---------------------------------------------------------------------
@fn　モーター動作確認用関数
@brief モーターが加速していく
@param なし
@return void
@details　ハードウェアを変更した時にモーターが動作するかを確認するようの関数
---------------------------------------------------------------------
 */
void Scene4() {
  //PWM
//  pwm_set_wrap(pwm_slice1, Hz_wrap(Speed));
//  pwm_set_wrap(pwm_slice2, Hz_wrap(Speed));

  Speed=Speed + 1;
  if(Speed>1500){
    Speed=1800;
  }
//  pwm_set_enabled(pwm_slice1, true);
//  pwm_set_enabled(pwm_slice2, true);
StepSW = 1;
    
  intervalR = frequencyToInterval(500);
  intervalL = frequencyToInterval(250);
 Serial.print(" Step_L:");
 Serial.print(Step_L);
 Serial.print(" Step_R:");
 Serial.print(Step_R);
 Serial.print("Direction:");
 Serial.println(Get_Distance(Step_L,Step_R));
   if (Get_Distance(Step_L,Step_R) > 300) {  //停止位置は試走会で調整

      Reset();
      Run = 0;
    }
}
//テスト
/*
---------------------------------------------------------------------
@fn　センサー値試験
@brief センサー値確認用処理
@param なし
@return void
@details　センサー値をSerialprintで出力する。
　　　　　　下側のSWを押すことで、出力を一時停止できる
---------------------------------------------------------------------
 */
void Scene5() {
  //モーター電源オフ
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);
  //センサー値出力オンオフ管理変数
  static bool sensor_out=true;

//実際には動かないが、Step上動作させるために定義
  intervalR = frequencyToInterval(10);//R
  intervalL = frequencyToInterval(10);//L
  //センサー値出力ONOFF切り替え
  if(sw1 == 1) {
    sensor_out= !sensor_out;
    delay(1000);
  }

  //クロスラインを検知したら、クロスフラグをON
  if (tmpc == 0 && sensorL < 300 && sensorR < 300 && sensorLL < 300 && sensorRR < 300) {
    temp_distance = NowDistance;
    cross = 1;
    tmpc = 1;
    }
  if ( tmpc == 1 && sensorLL > 700 && sensorRR > 700) {
    tmpc = 0;
  }

  //ゴールセンサが検知したらtmpフラグをON
  if (tmp == 0 && sensorGoal < 300 ){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
    if(cross != 1){
      count++;
      //BUZZER入れたが音小さくて聞こえない
      tone(BUZZER,1178,100);
      StepSW = !StepSW;
    }
    tmp = 1;
  }
  //ゴールセンサが検知しなくなったらクロスフラグを確認し、ゴールマーカーを追加するか判断
  if (tmp == 1 && sensorGoal > 700) {
    tmp = 0;
  }

  // // //カーブマーカー検知
  // if (curve_temp == 0  && sensorCurve < 300){
  //     if(cross != 1){
  //     curve_count++;
  //     //BUZZER入れたが音小さくて聞こえない
  //     tone(BUZZER,1178,100);
  //   }
  // }
  // if (sensorCurve > 700) {
  //   curve_temp = 0;
  // }
  //クロス検知後から35mm進んだらおそらくゴール、カーブセンサは過ぎてるだろうからクロスフラグを戻す
  if((NowDistance-temp_distance) >= 35){
    cross = 0;
  }

  

  if(c == 0){
    //基準速度
    SP = 0;
    //Pゲイン
    pgain = 0.34;
    //Dゲイン
    dgain = 0.001;
    //Iゲイン
    igain = 0.0;
    // 1回しか入らないようcを1にする
    c=1;
  }

  Add_SensorL = sensorL + sensorLL;
  Add_SensorR = sensorR + sensorRR;

  //今回の差分
  diff = Add_SensorL - Add_SensorR - bias;  //biasは試走会で調整

  P = diff * pgain;
  //D制御
  D = (beforediff - diff)/0.01 * dgain;
  
    //-50~50なら差分なしとする（）
  if(diff<=650 && diff>=-650){
    diff=0;
  }

  if(sensor_out==true){
    sensorCurve = analogRead(Curve_Sensor);
    Serial.print(sensorCurve, DEC);
    Serial.print(" ");
    sensorLL = read_adc(ch1, SELPIN2);//sensor ll
    Serial.print(sensorLL, DEC);
    Serial.print(" ");
    sensorL = read_adc(ch0, SELPIN2)-10;//sensor l
    Serial.print(sensorL, DEC);
    Serial.print(" ");
    sensorR = read_adc(ch1, SELPIN1)-10;//sensor r
    Serial.print(sensorR, DEC);
    Serial.print(" ");
    sensorRR = read_adc(ch0, SELPIN1)+15;//sensor rr
    Serial.print(sensorRR, DEC);
    Serial.print(" ");
    sensorGoal = analogRead(GOALSENSOR);
    Serial.print(sensorGoal, DEC);
    Serial.print("  count=");
    Serial.print(count, DEC);
    Serial.print(" cross=");
    Serial.print(cross, DEC);
    Serial.print(" tmpc=");
    Serial.print(tmpc, DEC);
    Serial.print(" tmp=");
    Serial.print(tmp, DEC);
    Serial.print(" NowDistance=");
    Serial.print(NowDistance, DEC);
    Serial.print(" tmp_distance=");
    Serial.print(temp_distance, DEC);
    Serial.print(" StepSW=");
    Serial.print(StepSW, DEC);
    Serial.println(" ");
  }
  
}
/*
---------------------------------------------------------------------
@fn　ログ出力　電源を消していない時のみ
@brief 
@param なし
@return void
@details
---------------------------------------------------------------------
*/
void Scene6(){

  static uint16_t cou = 0;

  Serial.print(data_log[cou].Curve_log);
  Serial.print("  ,  ");
  Serial.print(data_log[cou].LL_log);
  Serial.print("  ,  ");
  Serial.print(data_log[cou].L_log);
  Serial.print("  ,  ");
  Serial.print(data_log[cou].R_log);
  Serial.print(",  ");
  Serial.print(data_log[cou].RR_log);
  Serial.print("  ,  ");
  Serial.print(data_log[cou].Goal_log);
  Serial.print("  ,    ");
  Serial.println(cou);  
  cou++;
  if(cou > log_count){
      Reset();
      Run = 0;
      log_count = 0;
      cou = 0;
    }
}
