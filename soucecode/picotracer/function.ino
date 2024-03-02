//ADコンバータ内容取得関数
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

//タイマー割り込み関数


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

//パルス周波数⇨Wrap値に変換
float Hz_wrap(float pulsefreq){
  //F=sysclock/(wrap+1)*clkdiv
  return (125000000/(pulsefreq*100))-1;
  //125000000:raspipicoのシステムクロック 100:システムクロックを100分割していること
}

// //速度⇨パルス周波数
// float SPpulse(int SP) {  //SP[mm/s]
//   return SP / 1.5;       //1.5=１ステップあたりの距離
// }
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


//meinrun///////////////////////////////////////////////////////////////////////
//500：安定
//固定速度走行 PI制御
void Scene0() {
  while(1){

  if(c == 0){
    //基準速度
    SP = 500;
    //Pゲイン
    pgain = 0.4;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0004;
    // 1回しか入らないようcを1にする
    c=1;
  }


  sensorLL = read_adc(ch0, SELPIN1);
  // Serial.print(sensorLL, DEC);
  // Serial.print(" ");
  sensorL = read_adc(ch1, SELPIN1);
  // Serial.print(sensorL, DEC);
  // Serial.print(" ");
  sensorR = read_adc(ch0, SELPIN2);
  // Serial.print(sensorR, DEC);
  // Serial.print(" ");
  sensorRR = read_adc(ch1, SELPIN2);
  // Serial.print(sensorRR, DEC);
  // Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  // Serial.print(sensorGoal, DEC);
  // Serial.println(" ");

  //ゴールセンサーカウンタ
  if(tmp == 0 && sensorGoal < 300) {  //速度によって調整
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

//感度が強すぎるのかわからんがちょっとでもカーブするとすぐに抜ける、直線も振動しまくる
//P制御走行
void Scene1() {
  while(1){
  
  if(c == 0){
    //基準速度
     SP = 600;
    //Pゲイン
    pgain = 0.55;
    // 1回しか入らないようcを1にする
    c = 1;
 }
  sensorLL = read_adc(ch0, SELPIN1);
  Serial.print(sensorLL, DEC);
  Serial.print(" ");
  sensorL = read_adc(ch1, SELPIN1);
  Serial.print(sensorL, DEC);
  Serial.print(" ");
  sensorR = read_adc(ch0, SELPIN2);
  Serial.print(sensorR, DEC);
  Serial.print(" ");
  sensorRR = read_adc(ch1, SELPIN2);
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

  //ゴール後少し進んで停止
  if (count == 2) {  //一時的
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

  //P制御
  P = diff * pgain;

  //入力速度にPID制御値を代入
  inputL = SP + P;
  inputR = SP - P;

  //前回の差分を保存
  beforediff = diff;
  //速度設定 Slice1＝Reft Slice2=Left
  pwm_set_wrap(pwm_slice1, Hz_wrap(inputR));
  pwm_set_wrap(pwm_slice2, Hz_wrap(inputL));

 
  //PWMスタート
  pwm_set_enabled(pwm_slice1, true);
  pwm_set_enabled(pwm_slice2, true);


  //最大値補正
  if (inputL > SP) {
    inputL = SP;
  }
  if (inputR > SP) {
    inputR = SP;
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
  Serial.print(tmpc);
  Serial.print(" distance:");
  Serial.print(distance);
  Serial.print(" SP:");
  Serial.println(SP);
  }
}


//accel
void Scene2() {
  while(1){
  
  if(c == 0){
    //基準速度
    SP = 0;
    //Pゲイン
    pgain = 0.35;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0015;
    // 1回しか入らないようcを1にする
    c = 1;
  }

  sensorLL = read_adc(ch0, SELPIN1);
  // Serial.print(sensorLL, DEC);
  // Serial.print(" ");
  sensorL = read_adc(ch1, SELPIN1);
  // Serial.print(sensorL, DEC);
  // Serial.print(" ");
  sensorR = read_adc(ch0, SELPIN2);
  // Serial.print(sensorR, DEC);
  // Serial.print(" ");
  sensorRR = read_adc(ch1, SELPIN2);
  // Serial.print(sensorRR, DEC);
  // Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  // Serial.print(sensorGoal, DEC);
  // Serial.println(" ");

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
  if(SP>900.0){
    SP=900.0;
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
//完走 accelrun　工事中
void Scene3() {
  while(1){

  if(c == 0){
  //基準速度[m/s]
    SP = 250;  
    //Pゲイン
    pgain = 0.02;
    //Dゲイン
    dgain = 0.1;
    //Iゲイン
    igain = 0.0002;
    // 1回しか入らないようcを1にする
    c = 1;
  }

  //加速度[mm/s/s]
  float ac = 70;
  //1周期あたりにどのくらい加速するかの量
  float ac1 = ac / 1500;  //25000:メインループの周波数
  //現在の速度
  static float Speed = 0.0;


  //パルス周波数格納変数
  // //タイマースタート処理
  // if (a == false) {
  //   ITimer0.stopTimer();
  //   ITimer0.attachInterrupt(10000, TimerHandler0);  //左モーター
  //   a = true;
  // }

  Speed = Speed + ac1;

  //
  Serial.print("speed");
  Serial.println(inputL);
  //最大速度設定
  if (Speed > SP) {
    Speed = SP;
  }


  sensorLL = read_adc(ch0, SELPIN1);
  //     Serial.print(sensorLL,DEC);
  //     Serial.print(" ");
  sensorL = read_adc(ch1, SELPIN1);
  Serial.print(sensorL, DEC);
  Serial.print(" ");
  sensorR = read_adc(ch0, SELPIN2);
  Serial.print(sensorR, DEC);
  Serial.print(" ");
  sensorRR = read_adc(ch1, SELPIN2);
  //     Serial.print(sensorRR,DEC);
  //     Serial.print(" ");
  sensorGoal = analogRead(GOALSENSOR);
  //     Serial.print(sensorGoal,DEC);
  //     Serial.print(" ");

  //ゴールセンサーカウンタ
  if (tmp == 0 && sensorGoal < 100) {  //速度によって調整
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
  if (sensorLL > 600) {
    tmpc = 0;
  }

  //ゴール後少し進んで停止
  if (count == 2) {  //いいいいいいいいいいいいいいいいいいいいいいいいいいいい一時的
    if (b == 0) {
      Step = 0;
    }
    b++;
    //        Serial.print(" tmp=");
    //        Serial.println(tmp);
    if (distance > 230 && tmp == 0) {  //停止位置は試走会で調整
                                       //            Serial.print("Motor_Stop");

      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
      Reset();
      break;

    }
  }


  //今回の差分
  diff = sensorL - sensorR - bias;

  //    Serial.print("diff:");
  //    Serial.print(diff);
  sum += diff;
  //P制御
  P = diff * pgain;
  //D制御
  D = (beforediff - diff) * dgain;
  //I制御
  I = sum * igain;
  // inputL = SPpulse(Speed) + (P + D + I);
  // inputR = SPpulse(Speed) - (P + D + I);

  //前回の差分を保存
  beforediff = diff;



  //最大値補正
  if (inputL >= 146) {
    inputL = 146;
  }
  if (inputR >= 146) {
    inputR = 146;
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
    // inputL = SPpulse(Speed);
    // inputR = SPpulse(Speed);
  }


  // Serial.print(" SP:");
  // Serial.print(SPpulse(Speed));
  // Serial.print(" D:");
  // Serial.print(D);
  // Serial.print(" I:");
  // Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR");
  Serial.println(inputR);


  //パルス周期　パルス幅変換
  // intervalL = pulseHz(inputL);
  // intervalR = pulseHz(inputR);
  Serial.print("count=");
  Serial.print(count);
  //    Serial.print("cross:");
  //    Serial.print(cross);
  //    Serial.print(" StepL");
  //    Serial.println(StepL);
  }
}

//accelrun2　加速テスト
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
