//構造体インクルード
#include "types.h"

//---------------------------------------------------------------------
//@fn　走行関数Scene3
//@brief ライントレース走行
//@param なし
//@return void
//@details　PI制御　スタート時のゆっくり加速あり+0.35 最高速度800
//---------------------------------------------------------------------
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