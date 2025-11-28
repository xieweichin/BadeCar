#pragma once                   // 防止重複 include 本標頭檔（編譯器只會讀一次）

#include <Arduino.h>           // 使用 Arduino 基本類別與函式

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

// 伺服馬達 PWM 最小脈寬（500 微秒 ≒ 0 度）
#define SERVOMIN  500

// 伺服馬達 PWM 最大脈寬（2000 微秒 ≒ 180 度）
#define SERVOMAX  2000

// ------------------------------------------------------
// 板上按鈕（保留功能）
// ------------------------------------------------------
typedef enum {
  Button_A = 0,     // 第 0 號按鈕
  Button_B = 1      // 第 1 號按鈕
} ButtonT;

// ------------------------------------------------------
// 馬達編號（新版：0-based，全大寫 M0~M3）
// ------------------------------------------------------
typedef enum {
    M0 = 0,   // 馬達 0：右後輪
    M1 = 1,   // 馬達 1：左後輪
    M2 = 2,   // 馬達 2：右前輪
    M3 = 3    // 馬達 3：左前輪
} MotorNum;

// ------------------------------------------------------
// Servo 編號（Maker-ESP32-Pro 固定腳位）
// ------------------------------------------------------
typedef enum {
    S0 = 0,   // Servo 0 → GPIO26
    S1 = 1,   // Servo 1 → GPIO25
    S2 = 2,   // Servo 2 → GPIO33
    S3 = 3    // Servo 3 → GPIO32
} ServoNum;

// 前置宣告（避免交互依賴）
class EncoderMotor;

// ------------------------------------------------------
// BadeCarCore 主控制類別（對車子所有功能進行封裝）
// ------------------------------------------------------
class BadeCarCore {
public:
    BadeCarCore();       // 建構子（建立物件）
    ~BadeCarCore();      // 解構子（釋放動態記憶體）

    // -----------------------------
    // 0. 初始化
    // -----------------------------
    void init();         // 初始化馬達、Servo、編碼器等硬體

    // -----------------------------
    // 1. 按鈕
    // -----------------------------
    bool readButton(ButtonT b);                      // 讀取按鈕電平（LOW = pressed）
    bool readButtonEdge(ButtonT b, bool rising);     // 偵測按鈕觸發沿（上升沿/下降沿）

    // -----------------------------
    // 2. Servo 控制
    // -----------------------------
    void setServoPulseMs(ServoNum num, float ms);    // 設定伺服馬達脈寬（毫秒）
    void setServoAngle(ServoNum num, int deg);       // 設定伺服馬達角度（0-180 度）

    // -----------------------------
    // 3. 馬達 PWM（開迴路）— 無編碼器回授
    // -----------------------------
    void setMotorPwm(MotorNum id, int pwm);          // 單顆馬達 PWM（-255~255）
    void setMotorsPwm(double m0, double m1, double m2, double m3); // 四顆馬達一起 PWM
    void stopAllMotors();                            // 全部馬達停止

    // -----------------------------
    // 4. Encoder 回授
    // -----------------------------
    long  readMotorEnc(int n);                       // 讀取編碼器脈衝數
    int   readMotorDeg(int n);                       // 脈衝→角度（度）
    void  resetEnc();                                // 重設全部編碼器計數
    void  setMotorEnc(int n, int64_t v);             // 強制定義某輪編碼器值
    void  setEncoderCPR(int cpr);                    // 修改 CPR（換不同減速比時）

    // -----------------------------
    // 5. PID（透過 EncoderMotor 控制）
    // -----------------------------
    void initPID();                                  // 初始化 PID（使用預設參數）
    void updatePID();                                // 保留不用（EncoderMotor 已自動處理）
    long computePID(int n);                          // 讀取 PID 計算後的 PWM duty
    void setPIDTunings(double p, double i, double d);// 設定 PID 參數 P/I/D

    // -----------------------------
    // 6. 閉迴路速度控制（RPM）
    // -----------------------------
    void setMotorSpeed(double sp0, double sp1, double sp2, double sp3); // 四輪 RPM 控制

    // -----------------------------
    // 7. 位置控制（pulse）
    // -----------------------------
    void setMotorPosition(double p0, double p1, double p2, double p3); // 四輪位置控制（以 pulse 為單位）

    // -----------------------------
    // 8. 工具函式
    // -----------------------------
    double getMaxVelocity(const double motors[4]) const;     // 回傳四輪中速度絕對值最大者
    double averageSpeed(const double motors[4], double ref) const; // 四輪平均速度偏差
    static double toPolar(double x, double y, int component); // 2D 笛卡兒轉極座標
    static double toCartesian(double r, double theta, int axis);// 極座標→笛卡兒
    void syncPidReference(bool keepOutput);                   // 與 EncoderMotor 同步 PID 設定

private:
    // 靜態陣列（固定硬體腳位對應）
    static const int TUBIT_B[2];       // 按鈕
    static const int TB_MA[4];         // 馬達控制訊號 A
    static const int TB_MB[4];         // 馬達控制訊號 B
    static const int TB_EncA[4];       // 編碼器 A 相腳
    static const int TB_EncB[4];       // 編碼器 B 相腳
    static const int TB_ServoPin[4];   // Servo 腳

    bool lastButtonState[2] = {false,false}; // 按鈕前一刻狀態（用於偵測按鈕 edge）

    struct Impl;       // 內部實作細節（PImpl 設計）
    Impl* impl;        // 指向實作物件
};
