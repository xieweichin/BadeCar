#pragma once
#include <Arduino.h>

/*
    ---------------------------------------------------------------------
    BadeCarCore.h
    ---------------------------------------------------------------------
    Maker-ESP32-Pro × MG310 Encoder Motor × 四驅 BadeCar 核心控制程式庫
    （最新版本：馬達編號統一為 M0 ~ M3）

    【硬體配置（面向前方）】

          前方 ↑
       M2(右前)     M3(左前)
       M0(右後)     M1(左後)
          後方 ↓

    定義：
      - RunSpeed(RPM > 0) → 車子前進（所有輪往前轉）
      - EncoderMotor 自動處理閉迴路控制 / PWM / 編碼器計數

    【編碼器】
      - MG310：馬達端 13PPR × full-quad × 減速比 ≈ 20.4
      - 換算後 CPR ≈ 1061（可調整）

    【Servo】
      使用 Maker-ESP32-Pro 的固定 GPIO:
        S0 → GPIO26
        S1 → GPIO25
        S2 → GPIO33
        S3 → GPIO32
      PWM：50Hz LEDC，16-bit 分辨率

    ---------------------------------------------------------------
*/

#define SERVOMIN  500     // Servo 0 度對應脈寬
#define SERVOMAX  2000    // Servo 180 度對應脈寬

// ------------------------------------------------------
// 板上按鈕（Reserved）
// ------------------------------------------------------
typedef enum {
  Button_A = 0,
  Button_B = 1
} ButtonT;

// ------------------------------------------------------
// 馬達編號（新版：0-based，全大寫 M0~M3）
// ------------------------------------------------------
typedef enum {
    M0 = 0,   // 右後
    M1 = 1,   // 左後
    M2 = 2,   // 右前
    M3 = 3    // 左前
} MotorNum;

// ------------------------------------------------------
// Servo 編號（Maker-ESP32-Pro 固定腳位）
// ------------------------------------------------------
typedef enum {
    S0 = 0,   // GPIO26
    S1 = 1,   // GPIO25
    S2 = 2,   // GPIO33
    S3 = 3    // GPIO32
} ServoNum;

class EncoderMotor;

// ------------------------------------------------------
// BadeCarCore 主類別
// ------------------------------------------------------
class BadeCarCore {
public:
    BadeCarCore();
    ~BadeCarCore();

    // -----------------------------
    // 0. 初始化
    // -----------------------------
    void init();

    // -----------------------------
    // 1. 按鈕
    // -----------------------------
    bool readButton(ButtonT b);
    bool readButtonEdge(ButtonT b, bool rising);

    // -----------------------------
    // 2. Servo 控制
    // -----------------------------
    void setServoPulseMs(ServoNum num, float ms);
    void setServoAngle(ServoNum num, int deg);

    // -----------------------------
    // 3. 馬達 PWM（開迴路）
    // -----------------------------
    void setMotorPwm(MotorNum id, int pwm);
    void setMotorsPwm(double m0, double m1, double m2, double m3);
    void stopAllMotors();

    // -----------------------------
    // 4. Encoder 回授
    // -----------------------------
    long  readMotorEnc(int n);
    int   readMotorDeg(int n);
    void  resetEnc();
    void  setMotorEnc(int n, int64_t v);
    void  setEncoderCPR(int cpr);

    // -----------------------------
    // 5. PID（透過 EncoderMotor 控制）
    // -----------------------------
    void initPID();
    void updatePID();        // EncoderMotor 已自動處理，保留空殼
    long computePID(int n);  // 回傳 duty（資訊用途）
    void setPIDTunings(double p, double i, double d);

    // -----------------------------
    // 6. 閉迴路速度控制（RPM）
    // -----------------------------
    void setMotorSpeed(double sp0, double sp1, double sp2, double sp3);

    // -----------------------------
    // 7. 位置控制（pulse）
    // -----------------------------
    void setMotorPosition(double p0, double p1, double p2, double p3);

    // -----------------------------
    // 8. 工具函式
    // -----------------------------
    double getMaxVelocity(const double motors[4]) const;
    double averageSpeed(const double motors[4], double ref) const;
    static double toPolar(double x, double y, int component);
    static double toCartesian(double r, double theta, int axis);
    void syncPidReference(bool keepOutput);

private:
    static const int TUBIT_B[2];
    static const int TB_MA[4];
    static const int TB_MB[4];
    static const int TB_EncA[4];
    static const int TB_EncB[4];
    static const int TB_ServoPin[4];

    bool lastButtonState[2] = {false,false};

    struct Impl;
    Impl* impl;
};
