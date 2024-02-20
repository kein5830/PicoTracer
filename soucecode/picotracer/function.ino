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
  // Serial.println("handler0");
  pulseL=!pulseL;
  //パルスを入力
  digitalWrite(CLOCK_L, pulseL);
  // digitalWrite(CLOCK_R, pulseR);

  return true;
}

bool TimerHandler1(struct repeating_timer *t) {  //割り込む関数
  // Serial.println("handler1");
  pulseR=!pulseR;
  //パルスを入力
  // digitalWrite(CLOCK_L, pulseL);
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




//meinrun///////////////////////////////////////////////////////////////////////
//500,550,600は速度違うが早くなるほど速度が上がりにくい→加速度が一定じゃない
//一応走るけど抜けることもある
void Scene0() {
  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度
  static int SP = 800;
  //PID制御
  static float P = 0.0;
  static float D = 0.0;
  static float I = 0.0;
  static int diff = 0;
  static int bias = 0;
  static int beforediff = 0;
  static int sum = 0;
  //Pゲイン
  float pgain = 0.5;
  //Dゲイン
  float dgain = 0;
  //Iゲイン
  float igain = 0.0005;
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

    //  Serial.print("diff:");
    //  Serial.print(diff);
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





//感度が強すぎるのかわからんがちょっとでもカーブするとすぐに抜ける、直線も振動しまくる
//P制御走行
void Scene1() {
  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度
  static int SP = 500;
  //PID制御
  static float P = 0.0;

  static int diff = 0;
  static int bias = 0;
  //Pゲイン
  float pgain = 1.0;

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
  // Serial.print(sensorL, DEC);
  // Serial.print(" ");
  sensorR = read_adc(ch0, SELPIN2);
  // Serial.print(sensorR, DEC);
  // Serial.print(" ");
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
  if (sensorLL > 500) {
    tmpc = 0;
  }
  Serial.print(" tcou=");
  Serial.print(count);
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
  if (sensorLL < SP && sensorRR < SP) {
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



//accel
void Scene2() {
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
//完走 accelrun
void Scene3() {

  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static float inputL = 0;
  static float inputR = 0;
  //基準速度[m/s]
  static int SP = 250;  //250
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

//accelrun2　加速テスト
void Scene4() {

  //ループの中で１回しか実行させないための変数
  static bool a = false;
  static unsigned int b = 0, c = 0;
  //入力速度
  static int inputL = 100.0;
  static int inputR = 100.0;
  //最高速度[m/s]
  static int SP = 1500;  //250

  //加速度[mm/s/s]
  float ac = 1;
  //1周期あたりにどのくらい加速するかの量
  float ac1 = 0.0001;//ac / 1500;  //25000:メインループの周波数
  //現在の速度
  static float Speed = 0.0;


  //パルス周波数格納変数
  //タイマースタート処理 １秒間に1万回
  if (a == false) {
    ITimer0.stopTimer();
    ITimer0.attachInterrupt(inputL, TimerHandler0);  //左モーター
    ITimer1.stopTimer();
    ITimer1.attachInterrupt(inputR, TimerHandler1);  //左モーター
    a = true;
  }
    
 
  Speed = Speed + ac1;
  if(inputL<2000){
    if(Speed>1.0){
      inputL = inputL+ac;
      inputR = inputR+ac;
      a=false;
      Speed=0.0;
    }
  }
  // Serial.print("speed");
  // Serial.println(inputL);
  
  // inputL = SPpulse(Speed);
  // inputR = SPpulse(Speed);
  
  inputL = inputL+ac1;
  inputR = inputR+ac1;
  
  //最大速度設定
  if (inputL > 2000 || inputR > 2000) {
    inputL = 2000;
    inputR = 2000;
  }

  // Serial.print(" Speed:");
  // Serial.print(Speed);
  // Serial.print(" inputL:");
  // Serial.print(inputL);
  // Serial.print(" inputR");
  // Serial.println(inputR);


  //パルス周期　パルス幅変換
  //interval:5000最遅、３最速
  intervalL = inputL;
  intervalR = inputR;
// delay(1);
}
