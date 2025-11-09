//構造体インクルード
#include "types.h"

//---------------------------------------------------------------------
//@fn　走行関数Scene2 各Sceneの扱い検討中
//@brief ライントレース走行
//@param なし
//@return void
//@details　PI制御　スタート時のゆっくり加速あり　+0.25 最高速度700
//---------------------------------------------------------------------
void Scene2() {
  //モーター電源オフ
  digitalWrite(ENABLE_L, HIGH);
  digitalWrite(ENABLE_R, HIGH);
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
  sensorCurve = analogRead(Curve_Sensor);
  sensorLL = read_adc(ch1, SELPIN1);
  sensorL = read_adc(ch0, SELPIN1);
  sensorR = read_adc(ch1, SELPIN2);
  sensorRR = read_adc(ch0, SELPIN2);
  sensorGoal = analogRead(GOALSENSOR);

// 正規化処理　正規化後の値 = （元の値 – 最小値）÷（最大値 – 最小値）
Curve = (float)(sensorCurve - sensorCurveMin)/(float)(sensorCurveMax - sensorCurveMin);
LL = (float)(sensorLL - sensorLLMin)/(float)(sensorLLMax - sensorLLMin);
L = (float)(sensorL - sensorLMin)/(float)(sensorLMax - sensorLMin);
R = (float)(sensorR - sensorRMin)/(float)(sensorRMax - sensorRMin);
RR = (float)(sensorRR - sensorRRMin)/(float)(sensorRRMax - sensorRRMin);
Goal = (float)(sensorGoal - sensorGoalMin)/(float)(sensorGoalMax - sensorGoalMin);
 Serial.print(Curve, DEC);
 Serial.print(" ");
 Serial.print(LL, DEC);
 Serial.print(" ");
 Serial.print(L, DEC);
 Serial.print(" "); 
 Serial.print(R, DEC);
 Serial.print(" ");
 Serial.print(RR, DEC);
 Serial.print(" ");
 Serial.print(Goal, DEC);
 Serial.println(" ");

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
  
}
