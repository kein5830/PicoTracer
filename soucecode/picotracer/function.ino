
/*
@fn　ADコンバータ(mcp3002)の値取得関数
@brief 2つのADコンバータの値を順番に取得する
@param (int channel) ADコンバータのch番号 ch0 or ch1
@param (int select) ２つ搭載しているADコンバータの指定 SELPIN1 or SELPIN2
@return AD変換値（ラインセンサー値）
@details 接続しているラインセンサーの値を取得する
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

//タイマー割り込み関数 ここまで

/*
@fn　タイマー割り込み関数
@brief タイマーに設定した周期で割り込む関数
@param (struct repeating_timer *t) 構造体？詳細不明
@return おそらく割り込まれて時にtrueを返すだけだと思うが未調査
@details 現在はゴール後に一定距離走行し、停止するための距離測定に利用
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
@fn　周波数からwrap値に変換する関数
@brief 周波数を入力するとwrap値が出力される
@param (float pulsefreq):周波数(Hz)
@return wrap値
@details モーター速度をPWMで制御するためにwrap値というものが必要周波数の方がわかりやすいため
　　　　　ユーザーからの入力は周波数とし、変換する関数で対応
 */
float Hz_wrap(float pulsefreq){
  //F=sysclock/(wrap+1)*clkdiv
  return (125000000/(pulsefreq*100))-1;
  //125000000:raspipicoのシステムクロック 100:システムクロックを100分割していること
}

/*
@fn　リセット関数
@brief 実行メニューや、走行に仕様する変数をリセットする
@param なし
@return void
@details 実行メニューや、走行に仕様する変数をリセットする
 */
void Reset(){
  //プッシュスイッチカウント
  Run = 0,Mode = 0;
  //走行距離
  distance=0,Step=0;
  //一回しか実行しないための変数（なぜかｂしか使ってない）
  b = 0, c = 0;
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
  Serial.print("Reset");
}

/*
@fn　OLED表示更新関数
@brief OLEDの表示内容を指定し、更新する
@param (float volt) バッテリの電圧値
@param (float runmode) 実行番号 
@return void
@details 実行メニューや、走行に仕様する変数をリセットする
 */
void Oled_run(float volt,int runmode){
  //表示
  display.clearDisplay();     // 表示クリア
  Serial.println("動作");
  // タイトル表示
  display.setTextSize(2);     // 文字サイズ（1）
  display.setCursor(4, 0);    // 表示開始位置左上角（X,Y）
  display.println("PicoTracer");    // 表示内容

  //図形表示  
  display.drawLine(0, 20, 128, 20, WHITE);   // 線（始点終点指定）
  display.drawFastVLine(64, 22, 17, WHITE);  // 線（指定座標から垂線）
  display.drawFastHLine(0, 40, 128, WHITE);  // 線（指定座標から平行線）
  
  //電圧値
  display.setTextSize(2);     // 文字サイズ（1）
  display.setCursor(4, 22);    // 表示開始位置左上角（X,Y）
  display.println(volt);    // 表示内容

  // No.
  display.setTextSize(2);     // 文字サイズ（1）
  display.setCursor(69, 22);    // 表示開始位置左上角（X,Y）
  display.println("No.");    // 表示内容
  
  //走行モード
  display.setTextSize(2);     // 文字サイズ（1）
  display.setCursor(110, 22);    // 表示開始位置左上角（X,Y）
  display.println(runmode);    // 表示内容

  display.display();  // 表示実行
}

/*
@fn　走行関数Scene1
@brief ライントレース走行
@param なし
@return void
@details　PI制御　スタート時のゆっくり加速なし 最高速度450 定速
 */
void Scene0() {
  while(1){

  if(c == 0){
    //基準速度
    SP = 450;
    //Pゲイン
    pgain = 0.35;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0005;
    // 1回しか入らないようcを1にする
    c=1;
  }


  Curve = analogRead(Curve_Sensor);
  Serial.print(Curve, DEC);
  Serial.print(" ");
  sensorLL = read_adc(ch1, SELPIN2);//sensor ll
  Serial.print(sensorLL, DEC);
  Serial.print(" ");
  sensorL = read_adc(ch0, SELPIN2);//sensor l
  Serial.print(sensorL, DEC);
  Serial.print(" ");
  sensorR = read_adc(ch1, SELPIN1);//sensor r
  Serial.print(sensorR, DEC);
  Serial.print(" ");
  sensorRR = read_adc(ch0, SELPIN1);//sensor rr
  Serial.print(sensorRR, DEC);
  Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  Serial.print(sensorGoal, DEC);
  Serial.print(" ");

  //ゴールセンサーカウンタ
  if (tmp == 0 && sensorGoal < 100 ){  //速度によって調整  && sensorRR > 500 && sensorLL > 500 && Curve > 800
    count++;
    tmp = 1;
    count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
    cross = cross - cross;
  }
  if (sensorGoal > 900) {
    tmp = 0;
  }

  //ラインクロスカウンタ
  if (tmpc == 0 &&   sensorL < 200 && sensorR < 200 && sensorLL < 200 && sensorRR < 200) {  //速度によって調整
    cross++;
    tmpc = 1;
  }
  if (sensorLL > 500 && sensorRR > 500) {
    tmpc = 0;
  }
  //ゴール後少し進んで停止
  if (count == 2) {  //いいいいいいいいいいいいいいいいいいいいいいいいいいいい一時的
    if (b == 0) {
      //タイマースタート処理
      ITimer0.stopTimer();
      ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
      Step = 0;
    }
    b=1;
    //  Serial.print(" tmp=");
    //  Serial.println(tmp);
    
  }
  if (distance > 100) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
      Reset();
      break;
    }

  //今回の差分
  diff = sensorL - sensorR - bias;  //biasは試走会で調整

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
  if (sensorLL < 300 && sensorRR < 300) {
    inputL = SP;
    inputR = SP;
  }

  //速度設定 Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

  //PWMスタート
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);


  Serial.print(" P:");
  Serial.print(P);
  Serial.print(" D:");
  Serial.print(D);
  Serial.print(" I:");
  Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR:");
  Serial.print(inputR);
  Serial.print(" count:");
  Serial.print(count);
  Serial.print(" cross:");
  Serial.print(cross);
  Serial.print(" tmp:");
  Serial.print(tmp);
  Serial.print(" tmpc:");
  Serial.println(tmpc);
  }
}

//基本走行
//P制御走行　カーブがきつくゲインをあげすぎるとがたがた　厳しそう
/*
@fn　走行関数Scene2
@brief ライントレース走行
@param なし
@return void
@details　P制御のみ　スタート時のゆっくり加速あり+0.35　最高速度850
 */
void Scene1() {

  while(1){
  
  if(c == 0){
    //基準速度
    SP = 0;
    //Pゲイン
    pgain = 0.32;
    //Dゲイン
    // dgain = 3;
    //Iゲイン
    igain = 0.0023;
    // 1回しか入らないようcを1にする
    c = 1;
  }
         
  Curve = analogRead(Curve_Sensor);
  Serial.print(Curve, DEC);
  Serial.print(" ");
  sensorLL = read_adc(ch1, SELPIN2);//sensor ll
  Serial.print(sensorLL, DEC);
  Serial.print(" ");
  sensorL = read_adc(ch0, SELPIN2);//sensor l
  Serial.print(sensorL, DEC);
  Serial.print(" ");
  sensorR = read_adc(ch1, SELPIN1);//sensor r
  Serial.print(sensorR, DEC);
  Serial.print(" ");
  sensorRR = read_adc(ch0, SELPIN1);//sensor rr
  Serial.print(sensorRR, DEC);
  Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  Serial.print(sensorGoal, DEC);
  Serial.print(" ");

  //ゴールセンサーカウンタ
  if (tmp == 0 && sensorGoal < 300) {  //速度によって調整
    count++;
    tmp = 1;
    count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
    cross = cross - cross;
  }
  if (sensorGoal > 900) {
    tmp = 0;
  }

  //ラインクロスカウンタ
  if (tmpc == 0 && sensorLL < 300 && sensorL < 300 && sensorR < 300) {  //速度によって調整
    cross++;
    tmpc = 1;
  }
  if (sensorLL > 500) {
    tmpc = 0;
  }

  //今回の差分
  diff = sensorL - sensorR - bias;  //biasは試走会で調整

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
  inputL = SP + (P + D + I);
  inputR = SP - (P + D + I);

  //前回の差分を保存
  beforediff = diff;
  //速度設定 Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

  //PWMスタート
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);

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
  if (sensorLL < 300 && sensorRR < 300) {
    inputL = SP;
    inputR = SP;
  }

  SP=SP+0.35;
  if(SP>850.0){
    SP=850.0;
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
  if (distance > 200) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      inputL=0.0;
      inputR=0.0;
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
      Reset();
      break;
    }

  Serial.print(" P:");
  Serial.print(P);
  Serial.print(" D:");
  Serial.print(D);
  Serial.print(" I:");
  Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR:");
  Serial.print(inputR);
  Serial.print(" count:");
  Serial.print(count);
  Serial.print(" cross:");
  Serial.print(cross);
  Serial.print(" tmp:");
  Serial.print(distance);
  Serial.print(" SP:");
  Serial.println(SP);
  }


}

/*
@fn　走行関数Scene2
@brief ライントレース走行
@param なし
@return void
@details　PI制御　スタート時のゆっくり加速あり　+0.25 最高速度700
 */
void Scene2() {
  while(1){
  
  if(c == 0){
    //基準速度
    SP = 0;
    //Pゲイン
    pgain = 0.3;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0015;
    // 1回しか入らないようcを1にする
    c = 1;
  }

  Curve = analogRead(Curve_Sensor);
  Serial.print(Curve, DEC);
  Serial.print(" ");
  sensorLL = read_adc(ch1, SELPIN2);//sensor ll
  Serial.print(sensorLL, DEC);
  Serial.print(" ");
  sensorL = read_adc(ch0, SELPIN2);//sensor l
  Serial.print(sensorL, DEC);
  Serial.print(" ");
  sensorR = read_adc(ch1, SELPIN1);//sensor r
  Serial.print(sensorR, DEC);
  Serial.print(" ");
  sensorRR = read_adc(ch0, SELPIN1);//sensor rr
  Serial.print(sensorRR, DEC);
  Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  Serial.print(sensorGoal, DEC);
  Serial.print(" ");

  //ゴールセンサーカウンタ
  if (tmp == 0 && sensorGoal < 300) {  //速度によって調整
    count++;
    tmp = 1;
    count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
    cross = cross - cross;
  }
  if (sensorGoal > 900) {
    tmp = 0;
  }

  //ラインクロスカウンタ
  if (tmpc == 0 && sensorLL < 300 && sensorL < 300 && sensorR < 300) {  //速度によって調整
    cross++;
    tmpc = 1;
  }
  if (sensorLL > 500) {
    tmpc = 0;
  }

  //今回の差分
  diff = sensorL - sensorR - bias;  //biasは試走会で調整

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
  inputL = SP + (P + D + I);
  inputR = SP - (P + D + I);

  //前回の差分を保存
  beforediff = diff;
  //速度設定 Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

  //PWMスタート
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);

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
  if (sensorLL < 300 && sensorRR < 300) {
    inputL = SP;
    inputR = SP;
  }

  SP=SP+0.25;
  if(SP>700.0){
    SP=700.0;
  }

  //ゴール後少し進んで停止
  if (count == 2) {  //いいいいいいいいいいいいいいいいいいいいいいいいいいいい一時的
    if (b == 0) {
      //タイマースタート処理
      ITimer0.stopTimer();
      ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
      Step = 0;
    }
    b=1;
    //  Serial.print(" tmp=");
    //  Serial.println(tmp);
    
  }
  if (distance > 200) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      inputL=0.0;
      inputR=0.0;
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
      Reset();
      break;
    }

  Serial.print(" P:");
  Serial.print(P);
  Serial.print(" D:");
  Serial.print(D);
  Serial.print(" I:");
  Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR:");
  Serial.print(inputR);
  Serial.print(" count:");
  Serial.print(count);
  Serial.print(" cross:");
  Serial.print(cross);
  Serial.print(" tmp:");
  Serial.print(distance);
  Serial.print(" SP:");
  Serial.println(SP);
  }
}


//2023/11/04
//完走 accelrun　検証中
/*
@fn　走行関数Scene3
@brief ライントレース走行
@param なし
@return void
@details　PI制御　スタート時のゆっくり加速あり+0.35 最高速度800
 */
void Scene3() {
  while(1){
  
  if(c == 0){
    //基準速度
    SP = 0;
    //Pゲイン
    pgain = 0.32;
    //Dゲイン
    // dgain = 3;
    //Iゲイン
    igain = 0.0023;
    // 1回しか入らないようcを1にする
    c = 1;
  }
         
  Curve = analogRead(Curve_Sensor);
  Serial.print(Curve, DEC);
  Serial.print(" ");
  sensorLL = read_adc(ch1, SELPIN2);//sensor ll
  Serial.print(sensorLL, DEC);
  Serial.print(" ");
  sensorL = read_adc(ch0, SELPIN2);//sensor l
  Serial.print(sensorL, DEC);
  Serial.print(" ");
  sensorR = read_adc(ch1, SELPIN1);//sensor r
  Serial.print(sensorR, DEC);
  Serial.print(" ");
  sensorRR = read_adc(ch0, SELPIN1);//sensor rr
  Serial.print(sensorRR, DEC);
  Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  Serial.print(sensorGoal, DEC);
  Serial.print(" ");

  //ゴールセンサーカウンタ
  if (tmp == 0 && sensorGoal < 300) {  //速度によって調整
    count++;
    tmp = 1;
    count = count - cross;  //クロスの分をカウントしないようにクロスの部分を通った時に引く
    cross = cross - cross;
  }
  if (sensorGoal > 900) {
    tmp = 0;
  }

  //ラインクロスカウンタ
  if (tmpc == 0 && sensorLL < 300 && sensorL < 300 && sensorR < 300) {  //速度によって調整
    cross++;
    tmpc = 1;
  }
  if (sensorLL > 500) {
    tmpc = 0;
  }

  //今回の差分
  diff = sensorL - sensorR - bias;  //biasは試走会で調整

  //  Serial.print("diff:");
  //  Serial.print(diff);
  sum += diff;
  //P制御
  P = diff * pgain;
  //D制御
  D = (beforediff - diff) * dgain;
  //I制御
  I = sum * igain;

// 　初速が遅い時はゲインを弱くする 実現できず、、、
// if(inputL < 400 || inputR < 400){
//     //Pゲイン
//     pgain = 0.3;
//     //Dゲイン
//     dgain = 0;
//     //Iゲイン
//     igain = 0.0004;
//   }
  //入力速度にPID制御値を代入
  inputL = SP + (P + D + I);
  inputR = SP - (P + D + I);

  //前回の差分を保存
  beforediff = diff;
  //速度設定 Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

  //PWMスタート
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);

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
  if (sensorLL < 300 && sensorRR < 300) {
    inputL = SP;
    inputR = SP;
  }

  SP=SP+0.35;
  if(SP>800.0){
    SP=800.0;
  }

  //ゴール後少し進んで停止
  if (count == 2) {  //いいいいいいいいいいいいいいいいいいいいいいいいいいいい一時的
    if (b == 0) {
      //タイマースタート処理
      ITimer0.stopTimer();
      ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
      Step = 0;
    }
    b=1;
    //  Serial.print(" tmp=");
    //  Serial.println(tmp);
    
  }
  if (distance > 200) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      inputL=0.0;
      inputR=0.0;
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
      Reset();
      break;
    }

  Serial.print(" P:");
  Serial.print(P);
  Serial.print(" D:");
  Serial.print(D);
  Serial.print(" I:");
  Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR:");
  Serial.print(inputR);
  Serial.print(" count:");
  Serial.print(count);
  Serial.print(" cross:");
  Serial.print(cross);
  Serial.print(" tmp:");
  Serial.print(distance);
  Serial.print(" SP:");
  Serial.println(SP);
  }
}

/*
@fn　モーター動作確認用関数
@brief モーターが加速していく
@param なし
@return void
@details　ハードウェアを変更した時にモーターが動作するかを確認するようの関数
 */
void Scene4() {
  while(1){

  //現在の速度
  static float Speed = 0.0;
 
  //PWM

  pwm_set_wrap(pwm_slice1, Hz_wrap(Speed));
  pwm_set_wrap(pwm_slice2, Hz_wrap(Speed));
  Speed=Speed+0.01;
  if(Speed>1500){
    Speed=1500;
  }
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);
  Serial.print(" Sw1:");
  Serial.print(sw1);
  Serial.print(" Sw2:");
  Serial.print(sw2);
  Serial.print(" Speed:");
  Serial.println(Speed);
  sw1 = digitalRead(upswitch);
  sw2 = digitalRead(downswitch);
  if (sw1 == 1) {
    Run = 0,Mode = 0,Speed=0.0;
    digitalWrite(ENABLE_L, HIGH);
    digitalWrite(ENABLE_R, HIGH);
    Reset();
    break;
  }
  }
}
//テスト
/*
@fn　試験用関数
@brief 新機能などの試験用
@param なし
@return void
@details　新機能などを軽く試すための場所11/21現在は未使用
 */
void Scene5() {
//  int step = 0;
//  int ct=0,mode = 0; 
  //int deg[45] = {49,48,}
  while(1){
  //旋回
//  digitalWrite(CWCCW_L, HIGH);
//  digitalWrite(CWCCW_R, HIGH); 
  //  モーターテストコード
//  output = !output;
//  delay(2);
//  Serial.print("count: ");
//  Serial.print(count);
// Serial.print(step);
// Serial.print(" ");
 //count二回で1.8° 400カウントでタイヤ１周
 //94.575mm(左右タイヤの中心感覚) * 3.14/8 =37.12mm (45°)
 //タイヤ円周＝48mm(直径)*3.14=150.72mm: 360°
 //150.72/200=0.7536mm : 1.8° count二回で0.7536mm動く.  37.12/0.7536=49.2569 *2 count=98.5 45°

 //前回の仕様の場合count:105 45° count:92 40° count:81 35° count:69 30° count:58 25° count:46 20° count:35 15° count:23 10° count:12 5° count:0 0° 1°あたり2.3count 
//sw検知
//sw1 = digitalRead(upswitch);
//sw2 = digitalRead(downswitch);
////sw2 loopからbreak
//if(sw1==1){
//  Reset();
//  break;
//}
////sw1を押すことでモード実行
//if (sw2 == 1){
//    mode = 1;
//    delay(300);
//  }
////stepが50を超えたらmodeを変更し、モーターを止める
//if(step >= 100){
//  mode = 2;
//}
////mode1 モーターを回す
//if(mode == 1){
//  output = !output;
//  count++;
//}
//if(count==2){
//  step++;
//  count = 0;
//}

//digitalWrite(CLOCK_L,output);
//digitalWrite(CLOCK_R,output);
//センサーテストコード
//  Curve = analogRead(Curve_Sensor);
//  Serial.print(count, DEC);
//  Serial.print(" ");
//  sensorLL = read_adc(ch1, SELPIN2);//sensor ll
//  Serial.print(sensorLL, DEC);
//  Serial.println(" ");
//  sensorL = read_adc(ch0, SELPIN2);//sensor l
//  Serial.print(sensorL, DEC);
//  Serial.print(" ");
//  sensorR = read_adc(ch1, SELPIN1);//sensor r
//  Serial.print(sensorR, DEC);
//  Serial.print(" ");
//  sensorRR = read_adc(ch0, SELPIN1);//sensor rr
//  Serial.print(sensorRR, DEC);
//  Serial.print(" ");
//  sensorGoal = analogRead(GOALSENSOR);
//  Serial.print(sensorGoal, DEC);
//  Serial.println(" ");
  // millis()
  }
  
}
