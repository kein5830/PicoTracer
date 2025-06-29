//構造体インクルード
#include "types.h"

//---------------------------------------------------------------------
//@fn　走行関数Scene0
//@brief ライントレース走行
//@param なし
//@return void
//@details　PI制御　スタート時のゆっくり加速なし 最高速度450 定速
//---------------------------------------------------------------------
void Scene0() {
  //*******************************************************************
  //メイン走行スピード、PIDのゲイン値、その他ローカル変数定義
  //*******************************************************************
  if(c == 0){
    //基準速度
    SP = 600;
    //Pゲイン
    pgain = 0.35;
    //Dゲイン
    dgain = 0;
    //Iゲイン
    igain = 0.0;
    
    // 1回しか入らないようcを1にする
    c=1;
  }
  static uint16_t tmp_distance = 0;
  static uint8_t index = 0;
  //*******************************************************************
  //センサー値格納　（202412月17日に配置更新）
  //*******************************************************************
  sensorCurve = analogRead(Curve_Sensor);
  sensorLL = read_adc(ch1, SELPIN1);//sensor ll
  sensorL = read_adc(ch0, SELPIN1)-10;//sensor l
  sensorR = read_adc(ch1, SELPIN2);//sensor r
  sensorRR = read_adc(ch0, SELPIN2)+20;//sensor rr
  sensorGoal = analogRead(GOALSENSOR);//

  // //必要であれば
  // Serial.print(curve_count, DEC);
  // Serial.print(" ");
  // Serial.print(temp_distance, DEC);
  // Serial.print(" ");
  // Serial.print(cross, DEC);
  // Serial.print(" "); 
  // Serial.print(NowDistance, DEC);
  // Serial.print(" ");
  // Serial.print(sensorCurve, DEC);
  // Serial.print(" ");
  // Serial.print(tmpc, DEC);
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
      StepSW = !StepSW;
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
    //最後のマーカからゴールまでの距離を保存する
    marker_distance[curve_count] = NowDistance - tmp_distance;
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
    data_log[log_count].Curve_log = cross;
    data_log[log_count].LL_log = sensorCurve;
    data_log[log_count].L_log = NowDistance;
    data_log[log_count].R_log = sensorR;
    data_log[log_count].RR_log = temp_distance;
    data_log[log_count].Goal_log = curve_count;
//    data_log[log_count].R_motor_log = inputR;
//    data_log[log_count].L_motor_log = inputL;
    log_count++;  
    
}
