//ADコンバータ内容取得関数
int read_adc(int channel, int select) {
  int adcvalue = 0;
  byte commandbits = B11000000;  //B(スタート)(1:シングルエンドモード)

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
  static int del = 0, der = 0;
  //タイマー割り込みはピンLED_BUILTINを切り替えます
  toggle0 = toggle0 + 1;
  toggle1 = toggle1 + 1;
  //toggleがintervalの値まで来たらpulseを上下してtoggleを0にする
  if (toggle0 >= intervalL) {
    pulseL = !pulseL;
    //距離計測用ステップカウンタ
    del++;
    if (del >= 2) {
      StepL++;
      distanceL = StepL * 1.5;
      del = 0;
    }
    //    Serial.print("StepL:");
    //    Serial.print(StepL);
    toggle0 = 0;
  }
  if (toggle1 >= intervalR) {
    pulseR = !pulseR;
    //距離計測用ステップカウンタ
    der++;
    if (der >= 2) {
      StepR++;
      distanceR = StepR * 1.5;
      der = 0;
    }
    //    Serial.print(" StepR:");
    //    Serial.print(StepR);
    toggle1 = 0;
  }
  distance = (distanceL + distanceR) / 2.0;
  //        Serial.print("pulseL=");
  //        Serial.print(toggle0);
  //        Serial.print("  pulseR=");
  //        Serial.println(toggle1);

  //パルスを入力
  digitalWrite(CLOCK_L, pulseL);
  digitalWrite(CLOCK_R, pulseR);

  return true;
}
//パルス周波数⇨パルス幅変換
int pulseHz(int pulsefreq) {
  return 10000 / (pulsefreq * 2);
  /*(pulsefreq*2)パルス周波数を単純な割り込み周波数に変更
    10000/  1秒間で10000カウントなので10000をどのくらいの数で分けるかの計算をする
    */
}
//速度⇨パルス周波数
float SPpulse(int SP) {  //SP[mm/s]
  return SP / 1.5;       //1.5=１ステップあたりの距離
}
//メイン走行関数///////////////////////////////////////////////////////////////////////


void meinrun() {
  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度
  static int SP = 100;
  //PID制御
  static float P = 0.0;
  static float D = 0.0;
  static float I = 0.0;
  static int diff = 0;
  static int bias = 0;
  static int beforediff = 0;
  static int sum = 0;
  //Pゲイン
  float pgain = 0.03;
  //Dゲイン
  float dgain = 0.1;
  //Iゲイン
  float igain = 0.0001;
  //goalセンサーカウント
  static int count = 0, cross = 0;
  static bool tmp = 0, tmpc = 0;

  //パルス周波数格納変数
  //タイマースタート処理
  if (a == false) {
    ITimer0.stopTimer();
    ITimer0.attachInterrupt(10000, TimerHandler0);  //左モーター
    a = true;
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
      StepL = 0;
      StepR = 0;
    }
    b++;
    //        Serial.print(" tmp=");
    //        Serial.println(tmp);
    if (distance > 230 && tmp == 0) {  //停止位置は試走会で調整
                                       //            Serial.print("Motor_Stop");
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
    }
  }


  //今回の差分
  diff = sensorL - sensorR - bias;  //biasは試走会で調整

  //    Serial.print("diff:");
  //    Serial.print(diff);
  sum += diff;
  //P制御
  P = diff * pgain;
  //D制御
  D = (beforediff - diff) * dgain;
  //I制御
  I = sum * igain;
  inputL = SP + (P + D + I);
  inputR = SP - (P + D + I);

  //前回の差分を保存
  beforediff = diff;




  //最大値補正
  if (inputL > 100) {
    inputL = 100.0;
  }
  if (inputR > 100) {
    inputR = 100.0;
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
  Serial.print(" inputR");
  Serial.println(inputR);

  //パルス周期　パルス幅変換
  intervalL = pulseHz(inputL);
  intervalR = pulseHz(inputR);
  //    Serial.print("count=");
  //    Serial.print(count);
  //    Serial.print("cross:");
  //    Serial.print(cross);
  //    Serial.print(" StepL");
  //    Serial.println(StepL);
}


void Ponly() {
  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度
  static int SP = 100;
  //PID制御
  static float P = 0.0;

  static int diff = 0;
  static int bias = 0;
  //Pゲイン
  float pgain = 0.01;

  //goalセンサーカウント
  static int count = 0, cross = 0;
  static bool tmp = 0, tmpc = 0;

  //パルス周波数格納変数
  //タイマースタート処理
  if (a == false) {
    ITimer0.stopTimer();
    ITimer0.attachInterrupt(10000, TimerHandler0);  //左モーター
    a = true;
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
  Serial.print(" tcou=");
  Serial.println(count);
  //ゴール後少し進んで停止
  if (count == 2) {  //いいいいいいいいいいいいいいいいいいいいいいいいいいいい一時的
    if (b == 0) {
      StepL = 0;
      StepR = 0;
    }
    b++;

    if (distance > 230 && tmp == 0) {  //停止位置は試走会で調整
                                       //            Serial.print("Motor_Stop");
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
    }
  }


  //今回の差分
  diff = sensorL - sensorR - bias;

  //    Serial.print("diff:");
  //    Serial.print(diff);

  //P制御
  P = diff * pgain;

  inputL = SP + P;
  inputR = SP - P;




  //最大値補正
  if (inputL > 100) {
    inputL = 100.0;
  }
  if (inputR > 100) {
    inputR = 100.0;
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

  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR");
  Serial.println(inputR);

  //パルス周期　パルス幅変換
  intervalL = pulseHz(inputL);
  intervalR = pulseHz(inputR);

  //    Serial.print("count=");
  //    Serial.print(count);
  //    Serial.print("cross:");
  //    Serial.print(cross);
  //    Serial.print(" StepL");
  //    Serial.println(StepL);
}

void accel() {
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //tmp
  static bool a = false;
  //goalセンサーカウント
  static int count = 0;
  //加速度[mm/s/s]
  float ac = 100;
  //1周期あたりにどのくらい加速するかの量
  float ac1 = ac / 15000;  //25000:メインループの周波数
  //現在の速度
  static float Speed = 0.0;
  //最大速度[m/s]
  float mspeed = 350;

  //static bool a=false;
  ////タイマースタート処理
  if (a == false) {
    ITimer0.stopTimer();
    ITimer0.attachInterrupt(10000, TimerHandler0);  //左モーター
    a = true;
  }
  //    Serial.print(ac1 );
  //Speedを１加速
  Speed = Speed + ac1;

  //
  Serial.print("speed");
  Serial.println(inputL);
  //最大速度設定
  if (Speed > mspeed) {
    Speed = mspeed;
  }
  //速度をパルス周波数に変換し入力
  inputL = SPpulse(Speed);
  inputR = SPpulse(Speed);

  intervalL = pulseHz(inputL);
  intervalR = pulseHz(inputR);
}
//2023/11/04
//完走
void accelrun() {

  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度[m/s]
  static int SP = 220;  //250
  //PID制御
  static float P = 0.0;
  static float D = 0.0;
  static float I = 0.0;
  static int diff = 0;
  static int bias = 9;
  static int beforediff = 0;
  static int sum = 0;
  //Pゲイン
  float pgain = 0.02;
  //Dゲイン
  float dgain = 0.1;
  //Iゲイン
  float igain = 0.0002;
  //goalセンサーカウント
  static int count = 0, cross = 0;
  static bool tmp = 0, tmpc = 0;


  //加速度[mm/s/s]
  float ac = 70;
  //1周期あたりにどのくらい加速するかの量
  float ac1 = ac / 1500;  //25000:メインループの周波数
  //現在の速度
  static float Speed = 0.0;


  //パルス周波数格納変数
  //タイマースタート処理
  if (a == false) {
    ITimer0.stopTimer();
    ITimer0.attachInterrupt(10000, TimerHandler0);  //左モーター
    a = true;
  }

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
      StepL = 0;
      StepR = 0;
    }
    b++;
    //        Serial.print(" tmp=");
    //        Serial.println(tmp);
    if (distance > 230 && tmp == 0) {  //停止位置は試走会で調整
                                       //            Serial.print("Motor_Stop");
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
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
  inputL = SPpulse(Speed) + (P + D + I);
  inputR = SPpulse(Speed) - (P + D + I);

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
    inputL = SPpulse(Speed);
    inputR = SPpulse(Speed);
  }


  Serial.print(" SP:");
  Serial.print(SPpulse(Speed));
  // Serial.print(" D:");
  // Serial.print(D);
  // Serial.print(" I:");
  // Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR");
  Serial.println(inputR);


  //パルス周期　パルス幅変換
  intervalL = pulseHz(inputL);
  intervalR = pulseHz(inputR);
     Serial.print("count=");
     Serial.print(count);
  //    Serial.print("cross:");
  //    Serial.print(cross);
  //    Serial.print(" StepL");
  //    Serial.println(StepL);
}

//加速テスト
void accelrun2() {

  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度[m/s]
  static int SP = 1500;  //250
  //PID制御
  static float P = 0.0;
  static float D = 0.0;
  static float I = 0.0;
  static int diff = 0;
  static int bias = 9;
  static int beforediff = 0;
  static int sum = 0;
  //Pゲイン
  float pgain = 0.03;
  //Dゲイン
  float dgain = 0.1;
  //Iゲイン
  float igain = 0.0001;
  //goalセンサーカウント
  static int count = 0, cross = 0;
  static bool tmp = 0, tmpc = 0;


  //加速度[mm/s/s]
  float ac = 25;
  //1周期あたりにどのくらい加速するかの量
  float ac1 = ac / 1500;  //25000:メインループの周波数
  //現在の速度
  static float Speed = 0.0;


  //パルス周波数格納変数
  //タイマースタート処理
  if (a == false) {
    ITimer0.stopTimer();
    ITimer0.attachInterrupt(10000, TimerHandler0);  //左モーター
    a = true;
  }

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
      StepL = 0;
      StepR = 0;
    }
    b++;
    //        Serial.print(" tmp=");
    //        Serial.println(tmp);
    if (distance > 230 && tmp == 0) {  //停止位置は試走会で調整
                                       //            Serial.print("Motor_Stop");
      digitalWrite(ENABLE_L, HIGH);
      digitalWrite(ENABLE_R, HIGH);
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
  inputL = SPpulse(Speed);// + (P + D + I)
  inputR = SPpulse(Speed);// - (P + D + I)

  //前回の差分を保存
  beforediff = diff;



  //最大値補正
  if (inputL >= SPpulse(Speed)) {
    inputL = SPpulse(Speed);
  }
  if (inputR >= SPpulse(Speed)) {
    inputR = SPpulse(Speed);
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
    inputL = SPpulse(Speed);
    inputR = SPpulse(Speed);
  }


  Serial.print(" SP:");
  Serial.print(SPpulse(Speed));
  // Serial.print(" D:");
  // Serial.print(D);
  // Serial.print(" I:");
  // Serial.print(I);
  Serial.print(" inputL:");
  Serial.print(inputL);
  Serial.print(" inputR");
  Serial.println(inputR);


  //パルス周期　パルス幅変換
  intervalL = pulseHz(inputL);
  intervalR = pulseHz(inputR);
  //    Serial.print("count=");
  //    Serial.print(count);
  //    Serial.print("cross:");
  //    Serial.print(cross);
  //    Serial.print(" StepL");
  //    Serial.println(StepL);
}
