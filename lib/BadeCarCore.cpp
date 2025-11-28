#include "BadeCarCore.h"          // 對應的標頭檔，裡面宣告了 BadeCarCore 類別與相關 enum

#include <encoder_motor.h>        // 外部的 EncoderMotor 類別（負責編碼器 + 馬達 PWM + PID）
#include <encoder_motor_lib.h>    // EncoderMotor 的版本資訊（目前沒直接用到）

#include <math.h>                 // 使用 sqrt、atan2、cos、sin 這些數學函式

using em::EncoderMotor;           // 簡化名稱：之後直接寫 EncoderMotor 就好，不用 em::EncoderMotor


// =======================================================
// Maker-ESP32-Pro 硬體腳位設定（常數陣列）
// =======================================================

// 板載按鈕（目前預留，腳位填 0 → 表示沒有接）
const int BadeCarCore::TUBIT_B[2] = {0, 0};

// 馬達 MA / MB（四驅車，對應到 H 橋的兩個 PWM 腳）
// 順序：M0, M1, M2, M3
const int BadeCarCore::TB_MA[4] = {27, 4, 17, 15};  // 每顆馬達的「正轉腳」(positive pin)
const int BadeCarCore::TB_MB[4] = {13, 2, 12, 14};  // 每顆馬達的「反轉腳」(negative pin)

// 編碼器 A / B 相腳位
// 這裡對應到 MG310 的 A/B 訊號，配合 EncoderMotor 做正反轉判斷
const int BadeCarCore::TB_EncA[4] = {18, 5, 35, 34};  // A 相
const int BadeCarCore::TB_EncB[4] = {19, 23, 36, 39}; // B 相

// Servo 腳位（Maker-ESP32-Pro 上固定的 4 個 Servo 接腳）
const int BadeCarCore::TB_ServoPin[4] = {26, 25, 33, 32};


// =======================================================
// EncoderMotor 物件（四輪實體）
// 依照 BadeCarCore.h 裡定義的 M0~M3 = 右後、左後、右前、左前
// =======================================================

namespace {   // 匿名 namespace：這些變數只在本 .cpp 檔內可見

// 每轉脈衝數（馬達軸）Pulses Per Revolution
constexpr uint32_t kPPR       = 13;

// 減速比（原本大約 20.4，這裡簡化為 20）
// 真正總 CPR = PPR * 減速比 * 4 (Full-quad)，但 EncoderMotor 內部會處理
constexpr uint32_t kReduction = 20;

// EncoderMotor 裡使用的最大 PWM duty（10-bit：0~1023）
constexpr int16_t  kMaxDuty   = 1023;

// ----------------------------
// 建立 4 顆 EncoderMotor 實體
// ----------------------------
// 建構子參數：
//   (馬達正極腳, 馬達負極腳, 編碼器 A, 編碼器 B, PPR, 減速比, 相位關係)

// M0：右後輪
EncoderMotor em0(GPIO_NUM_27, GPIO_NUM_13,
                 GPIO_NUM_18, GPIO_NUM_19,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

// M1：左後輪
EncoderMotor em1(GPIO_NUM_4, GPIO_NUM_2,
                 GPIO_NUM_5, GPIO_NUM_23,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

// M2：右前輪
EncoderMotor em2(GPIO_NUM_17, GPIO_NUM_12,
                 GPIO_NUM_35, GPIO_NUM_36,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

// M3：左前輪
EncoderMotor em3(GPIO_NUM_15, GPIO_NUM_14,
                 GPIO_NUM_34, GPIO_NUM_39,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

} // namespace   // 匿名 namespace 結束


// =======================================================
// Impl - 內部實作細節（PImpl pattern）
// 實際的狀態都藏在 Impl，對外只看到 BadeCarCore 介面
// =======================================================
struct BadeCarCore::Impl {

    EncoderMotor* m[4];   // 指向四個 EncoderMotor 物件的指標陣列（M0~M3）

    int    encoderCPR;    // 每圈「總脈衝數」（包含減速比及 full-quad 等）
    double kp, ki, kd;    // PID 參數（速度控制用）

    double posTarget[4];  // 位置控制時，各輪的目標 pulse 計數

    // 建構子：初始化預設參數
    Impl() {
        // 把前面 namespace 中的 em0~em3 指標存進來
        m[0] = &em0;
        m[1] = &em1;
        m[2] = &em2;
        m[3] = &em3;

        // 預設 MG310 的 CPR ≈ 1061（可依實測修改）
        encoderCPR = 1061;

        // 預設 PID 參數（給 EncoderMotor 使用）
        kp = 1.2;
        ki = 0.25;
        kd = 0;

        // 位置目標一開始都設 0（例如當作原點）
        for (int i=0;i<4;i++)
            posTarget[i] = 0;
    }
};


// =======================================================
// 建構 / 解構
// =======================================================

BadeCarCore::BadeCarCore() {
    impl = new Impl();    // 建構時配置一個 Impl 物件（放在 heap）
}

BadeCarCore::~BadeCarCore() {
    delete impl;          // 解構時釋放 Impl（避免記憶體洩漏）
}


// =======================================================
// init() - 初始化馬達 + Servo
// =======================================================
void BadeCarCore::init() {

    // 1. 初始化 4 顆 EncoderMotor
    for (int i=0;i<4;i++)
        impl->m[i]->Init();   // 啟動該輪：設定 PWM、attach 中斷、開啟速度更新執行緒

    // 2. 初始化 4 個 Servo（使用 LEDC channel 10~13, 50Hz, 16bit）
    for (int i=0;i<4;i++){
        int ch = 10 + i;      // 每個 servo 一個獨立 channel（避免互相干擾）
        ledcSetup(ch, 50, 16);                 // 設定 PWM 頻率 50Hz、解析度 16-bit (0~65535)
        ledcAttachPin(TB_ServoPin[i], ch);     // 該 servo 腳接到對應 channel
    }
}


// =======================================================
// 按鈕（預留功能）
// =======================================================
bool BadeCarCore::readButton(ButtonT b){
    int idx = (int)b;                      // enum 轉為索引 0 或 1

    if (TUBIT_B[idx] == 0) return false;   // 若腳位設定為 0，表示沒接按鈕 → 直接回 false

    // 這裡假設按鈕為「按下＝LOW」（常見接法：INPUT_PULLUP + 按下接地）
    return (digitalRead(TUBIT_B[idx]) == LOW);
}

bool BadeCarCore::readButtonEdge(ButtonT b, bool rising){
    int idx = (int)b;               // 哪一顆按鈕
    bool now  = readButton(b);     // 現在狀態（true = pressed）
    bool last = lastButtonState[idx]; // 上一次紀錄的狀態
    lastButtonState[idx] = now;    // 更新狀態，用於下次判斷

    // 若 rising = true → 偵測「未按 → 按下」（上升沿）
    // 若 rising = false → 偵測「按下 → 放開」（下降沿）
    return rising ? (!last && now) : (last && !now);
}


// =======================================================
// Servo 控制
// =======================================================
void BadeCarCore::setServoPulseMs(ServoNum num, float ms){
    int idx = (int)num;        // Servo 編號 0~3
    int ch = 10 + idx;         // 對應 LEDC channel（與 init() 中一致）

    // 伺服馬達的控制週期固定為 20ms（50Hz）
    // duty_ratio = 高電平時間 / 20ms
    float duty_ratio = ms / 20.0f;

    // 保護：限制在 0.0 ~ 1.0 範圍（避免亂寫出界）
    duty_ratio = constrain(duty_ratio, 0.0f, 1.0f);

    // LEDC 解析度 16-bit → duty 0~65535
    uint32_t duty = duty_ratio * 65535.0f;

    // 寫入指定 channel 的 duty
    ledcWrite(ch, duty);
}

void BadeCarCore::setServoAngle(ServoNum num, int deg){
    // 限制角度範圍 0~180 度
    deg = constrain(deg, 0, 180);

    // 線性內插：0 度 → SERVOMIN (500us)，180 度 → SERVOMAX (2000us)
    float us = SERVOMIN + (SERVOMAX - SERVOMIN) * (deg / 180.0f);

    // 把「微秒」轉為「毫秒」後，呼叫 setServoPulseMs
    setServoPulseMs(num, us / 1000.0f);
}


// =======================================================
// 馬達 PWM（開迴路控制，無編碼器回授）
// =======================================================
void BadeCarCore::setMotorPwm(MotorNum id, int pwm){
    int i = (int)id;                     // motor index 0~3

    // 使用者傳入 -255 ~ +255（類似 Arduino analogWrite 的習慣）
    pwm = constrain(pwm, -255, 255);

    // map 到 EncoderMotor 使用的 -kMaxDuty ~ +kMaxDuty（這裡是 -1023 ~ 1023）
    int duty = map(pwm, -255, 255, -kMaxDuty, kMaxDuty);

    // 呼叫 EncoderMotor 的開迴路 PWM 函式
    impl->m[i]->RunPwmDuty((int16_t)duty);
}

void BadeCarCore::setMotorsPwm(double m0, double m1, double m2, double m3){
    // 四輪一次設定（方便使用）
    setMotorPwm(M0, (int)m0);
    setMotorPwm(M1, (int)m1);
    setMotorPwm(M2, (int)m2);
    setMotorPwm(M3, (int)m3);
}

void BadeCarCore::stopAllMotors(){
    // 叫 EncoderMotor 做 Stop（PWM 拉高兩腳，等同煞車）
    for (int i=0;i<4;i++)
        impl->m[i]->Stop();
}


// =======================================================
// Encoder（回授）
// =======================================================

// 回傳編碼器脈衝數（0-based：0~3 = M0~M3）
long BadeCarCore::readMotorEnc(int n){
    return impl->m[n]->EncoderPulseCount();   // 內部是 int64_t，這裡轉為 long 回傳
}

// 將 pulse 數轉成「角度（度）」
// 公式：deg = pulse * 360 / CPR
int BadeCarCore::readMotorDeg(int n){
    long p = readMotorEnc(n);                 // 目前這輪的 pulse 數
    return (int)((double)p * 360.0 / (double)impl->encoderCPR);
}

// 清除所有輪子的編碼器計數（例如歸零）
void BadeCarCore::resetEnc(){
    for (int i=0;i<4;i++)
        impl->m[i]->ResetEncoderPulseCount(); // 對每輪執行 reset
}

// 設定某一顆編碼器的 pulse（例如手動歸零到某個 offset）
void BadeCarCore::setMotorEnc(int n, int64_t v){
    impl->m[n]->SetEncoderPulseCount(v);      // 直接覆寫內部的計數值
}

// 修改 CPR（例如換不同減速比的 MG310，或是自己實測後修正）
void BadeCarCore::setEncoderCPR(int cpr){
    impl->encoderCPR = cpr;
}


// =======================================================
// PID（EncoderMotor 內建速度控制）
// =======================================================

// 使用預設的 kp, ki, kd 來設定四輪的 PID 參數
void BadeCarCore::initPID(){
    setPIDTunings(impl->kp, impl->ki, impl->kd);
}

// 手動設定 PID 參數
void BadeCarCore::setPIDTunings(double p, double i, double d){
    impl->kp = p;
    impl->ki = i;
    impl->kd = d;

    // 套用到四顆 EncoderMotor（它們各自有一組 speed PID）
    for (int i=0;i<4;i++)
        impl->m[i]->SetSpeedPid((float)p, (float)i, (float)d);
}

// EncoderMotor 自動更新 PID，不需要 BadeCarCore 幫忙計算，所以這裡保留空殼相容
void BadeCarCore::updatePID(){
    // no-op（故意什麼都不做，讓舊程式可以編譯）
}

// 回傳某一輪目前的 PWM duty（用來 debug 觀察 PID 調整結果）
long BadeCarCore::computePID(int n){
    return impl->m[n]->PwmDuty();
}


// =======================================================
// 閉迴路速度控制（RPM）
// =======================================================
//
// sp0~sp3 = 四輪目標轉速（RPM）
//   正值 → 該輪往前轉
//   負值 → 該輪往後轉
//
void BadeCarCore::setMotorSpeed(double sp0, double sp1, double sp2, double sp3){
    // 直接把 RPM 目標丟給 EncoderMotor，它裡面會用 PID + 編碼器自動調整 PWM
    impl->m[0]->RunSpeed((int16_t)sp0);
    impl->m[1]->RunSpeed((int16_t)sp1);
    impl->m[2]->RunSpeed((int16_t)sp2);
    impl->m[3]->RunSpeed((int16_t)sp3);
}


// =======================================================
// 位置控制（pulse 為單位）
// =======================================================
//
// 適用：
//   * 前進固定距離（每輪目標 pulse 一樣）
//   * 左右旋轉指定角度（左右輪 pulse 相反）
//   * 回原點（目標 = 0）
//
// 基本控制原理（簡單比例控制）：
//   err = target - current
//   rpm = err * Kp_pos
//   然後把 rpm 限制在 ±300 之內，避免暴衝
//
void BadeCarCore::setMotorPosition(double p0, double p1, double p2, double p3){
    // 1. 記錄四輪目標位置（pulse）
    impl->posTarget[0] = p0;
    impl->posTarget[1] = p1;
    impl->posTarget[2] = p2;
    impl->posTarget[3] = p3;

    // 位置控制的比例係數
    const double Kp_pos = 0.1;  // 越大 → 靠近時還是跑得比較快，高一點可能會 overshoot

    // 2. 對每一輪計算目前位置與目標位置之間的誤差
    for (int i=0;i<4;i++){
        double curr = impl->m[i]->EncoderPulseCount(); // 目前 pulse
        double err  = impl->posTarget[i] - curr;       // error = 目標 - 目前
        double rpm  = err * Kp_pos;                    // 簡單比例控制 → 轉換成目標 RPM

        // 3. 限速避免暴衝：把 rpm 限制在 [-300, 300]
        if (rpm > 300)  rpm = 300;
        if (rpm < -300) rpm = -300;

        // 4. 把目標 RPM 丟給 EncoderMotor 執行閉迴路速度控制
        impl->m[i]->RunSpeed((int16_t)rpm);
    }
}


// =======================================================
// 工具函式（運動學計算）
// =======================================================

// 四輪輸入中，取絕對值最大的速度（方便做 normalize 等運算）
double BadeCarCore::getMaxVelocity(const double motors[4]) const{
    double maxv = fabs(motors[0]);        // 先假設第 0 顆最大
    for (int i=1;i<4;i++)
        if (fabs(motors[i]) > maxv)      // 若後面有更大的絕對值，就更新
            maxv = fabs(motors[i]);
    return maxv;
}

// 算四輪相對於某個參考值 ref 的平均偏差
// 例如 ref 是預期速度，motors[i] 是實際速度
double BadeCarCore::averageSpeed(const double motors[4], double ref) const{
    return ((motors[0] - ref) +
            (motors[1] - ref) +
            (motors[2] - ref) +
            (motors[3] - ref)) / 4.0;
}

// (x,y) → (r,θ)
// component = 0 → 回傳 r（向量長度）
// component = 1 → 回傳 θ（弧度）
double BadeCarCore::toPolar(double x, double y, int component){
    double r = sqrt(x*x + y*y);   // 長度 sqrt(x^2 + y^2)
    double t = atan2(y, x);       // 角度（-π ~ +π），符合常見的座標定義
    return (component == 0) ? r : t;
}

// (r,θ) → 取出 x 或 y
// axis = 0 → 回傳 x
// axis = 1 → 回傳 y
double BadeCarCore::toCartesian(double r, double t, int axis){
    return (axis == 0) ? (r * cos(t)) : (r * sin(t));
}


// =======================================================
// syncPidReference（相容舊 TuBitCore）
// =======================================================
void BadeCarCore::syncPidReference(bool keepOutput){
    // EncoderMotor 內部自己管理 PID，這裡不需要做事
    // 保留這個函式只是為了讓舊版程式呼叫時不會編譯錯誤
}
