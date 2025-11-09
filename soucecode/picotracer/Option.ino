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
  static bool a = 0;
  if(interval <= 1000 && a == 0){
    interval_tar = 999;
  }
  if(interval == 999){
    a=1;
    interval_tar = 0;
  }
  // if(a == 1){
  // interval_tar = 0;
  // }
  Serial.print("StepL:");
  Serial.println(Step_L);
  // Serial.print("StepR:");
  // Serial.println(Step_R);
  
  // if(NowDistance > 1000){
  //   digitalWrite(ENABLE_L, HIGH);
  //   digitalWrite(ENABLE_R, HIGH);
  // }

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

// //実際には動かないが、Step上動作させるために定義
//   interval = frequencyToInterval(10);//R
//   intervalL = frequencyToInterval(10);//L
//   //センサー値出力ONOFF切り替え
//   if(sw1 == 1) {
//     sensor_out= !sensor_out;
//     delay(1000);
//   }

//   static uint16_t tmp_distance = 0;
//   static uint8_t index = 0;

  sensorCurve = analogRead(Curve_Sensor);
  sensorLL = read_adc(ch1, SELPIN1);
  sensorL = read_adc(ch0, SELPIN1);
  sensorR = read_adc(ch1, SELPIN2);
  sensorRR = read_adc(ch0, SELPIN2);
  sensorGoal = analogRead(GOALSENSOR);


  if(sensor_out==true){
    
    Serial.print(sensorCurve, DEC);
    Serial.print(" ");
    Serial.print(sensorLL, DEC);
    Serial.print(" ");
    Serial.print(sensorL, DEC);
    Serial.print(" ");
    Serial.print(sensorR, DEC);
    Serial.print(" ");
    Serial.print(sensorRR, DEC);
    Serial.print(" ");
    Serial.print(sensorGoal, DEC);
    Serial.print("  count=");
    Serial.print(count, DEC);
    Serial.print(" cross=");
    Serial.print(cross, DEC);
    Serial.print(" curve_count=");
    Serial.print(curve_count, DEC);
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
    //モーター電源オフ
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);
    for(int z=0;z<=71;z++){
      Serial.print("marker_distance：");
      Serial.println(marker_distance[z]);
    }

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

  Serial.print(cou);
  Serial.print(" markerdistance:");
  Serial.println(marker_distance[cou]);

  cou++;
  // if(cou > 100){cou = 100;}
  if(cou > log_count){
      Reset();
      Run = 0;
      log_count = 0;
      cou = 0;
    }
}