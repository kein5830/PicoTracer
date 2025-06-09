//構造体インクルード
#include "types.h"

//---------------------------------------------------------------------
//@fn　モーター動作確認用関数
//@brief モーターが加速していく
//@param なし
//@return void
//@details　ハードウェアを変更した時にモーターが動作するかを確認するようの関数
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
//@fn　センサー値試験
//@brief センサー値確認用処理
//@param なし
//@return void
//@details　センサー値をSerialprintで出力する。
//下側のSWを押すことで、出力を一時停止できる
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
//@fn　ログ出力　電源を消していない時のみ
//@brief 
//@param なし
//@return void
//@details
//---------------------------------------------------------------------
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