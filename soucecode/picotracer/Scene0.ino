//構造体インクルード
#include "types.h"

//---------------------------------------------------------------------
//@fn　走行関数Scene0
//@brief ライントレース走行
//@param なし
//@return void
//@details　PI制御　スタート時のゆっくり加速なし 最高速度300 定速
//---------------------------------------------------------------------
void Scene0() {
  //*******************************************************************
  //メイン走行スピード、PIDのゲイン値、その他ローカル変数定義
  //*******************************************************************
  //初期
  if(c == 0){
    //基準速度
    interval_tar = 350;
    //Pゲイン
    pgain = 0.3;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0;
    

    c=1;
  }
  static uint16_t tmp_distance = 0;

  //*******************************************************************
  //センサー値格納　（202412月17日に配置更新）
  //*******************************************************************
  sensorCurve = analogRead(Curve_Sensor);
  sensorLL = read_adc(ch1, SELPIN1);
  sensorL = read_adc(ch0, SELPIN1);
  sensorR = read_adc(ch1, SELPIN2);
  sensorRR = read_adc(ch0, SELPIN2);
  sensorGoal = analogRead(GOALSENSOR);

  // //必要であれば
  // Serial.print(sensorCurve, DEC);
  // Serial.print(" ");
  // Serial.print(sensorLL, DEC);
  // Serial.print(" ");
  // Serial.print(sensorL, DEC);
  // Serial.print(" "); 
  // Serial.print(sensorR, DEC);
  // Serial.print(" ");
  // Serial.print(sensorRR, DEC);
  // Serial.print(" ");
  // Serial.print(sensorGoal, DEC);
  // Serial.print(" ");
  // Serial.print(NowDistance, DEC);
  //   Serial.print(" ");
  // Serial.print(temp_distance, DEC);
  // Serial.println(" ");

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
      // StepSW = !StepSW;
    }
    tmp = 1;
  }
  //ゴールセンサが検知しなくなったらクロスフラグを確認し、ゴールマーカーを追加するか判断
  if (tmp == 1 && sensorGoal > 700) {
    tmp = 0;
  }

  //カーブマーカー検知
  if (curve_temp == 0  && sensorCurve < 300){
      //クロスではない場合
      if(cross != 1){
        curve_count++;      
        //BUZZER入れたが音小さくて聞こえない
        tone(BUZZER,1178,100);
        //スタートから1個目のマーカの時
        if(curve_count==1){
          tmp_distance = NowDistance;
          marker_distance[curve_count-1] = tmp_distance;
          //スタートから2個目以降のマーカー
        }else if(curve_count >=2){
        marker_distance[curve_count-1] = NowDistance - tmp_distance;
        tmp_distance = NowDistance;
        }
      curve_temp = 1;
      }
  }
  if (sensorCurve > 700) { 
    curve_temp = 0;
  }
  //クロス検知後から35mm進んだらおそらくゴール、カーブセンサは過ぎてるだろうからクロスフラグを戻す（誤検知する場合は調整する）
  if((NowDistance-temp_distance) >= 43){
    cross = 0;
  }

  //ゴール後処理
  if (count == 2) {
    if (b == 0) {
      //最後のマーカからゴールまでの距離を保存する
      marker_distance[curve_count] = NowDistance - tmp_distance;
      // //タイマースタート処理
      // ITimer0.stopTimer();
      // ITimer0.attachInterrupt(100, callback0);  //自宅コースはスタートエリアが狭いため一時的に無効化し即停止するようにしている
      interval_tar = 0;
      first_count = curve_count;
      Step = 0;
    }
    b=1;
    
  }
  if (count >= 2 && interval < 10) {  //停止位置は試走会で調整
      ITimer0.stopTimer();
      ITimer1.stopTimer();
      Reset();
      Run = 0;

      tone(BUZZER,1446,1000); 
    }
  //ライン制御
  // Add_SensorL = sensorL + sensorLL;
  // Add_SensorR = sensorR + sensorRR;

  //今回の差分
  diff = (sensorL + sensorLL) - (sensorR + sensorRR) - bias;  //biasは試走会で調整
  
  //-50~50なら差分なしとする
  if(diff<=400 && diff>=-400){
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
  PID_Result = P + D + I;
  // Serial.print("PID_Result：");
  // Serial.println(PID_Result);
  //前回の差分を保存き
  beforediff = diff;
  
  //クロス通過時は速度を固定する
  if (sensorLL < 400 && sensorRR < 400) {
    PID_Result = 0;
  }

  //ログ保存スタートボタンを押してから
    // data_log[log_count].Curve_log = sensorCurve;
    // data_log[log_count].LL_log = curve_count;
    // data_log[log_count].L_log = NowDistance;
    // data_log[log_count].R_log = cross;
    // data_log[log_count].RR_log = temp_distance;
    // data_log[log_count].Goal_log = count;
//    data_log[log_count].L_motor_log = PID_Result;
    // log_count++;  
    
}
