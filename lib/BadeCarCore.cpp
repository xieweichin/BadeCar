#include "BadeCarCore.h"

#include <encoder_motor.h>
#include <encoder_motor_lib.h>

#include <math.h>

using em::EncoderMotor;

// =======================================================
// Maker-ESP32-Pro 硬體腳位
// =======================================================

// 板載按鈕（預留功能）
const int BadeCarCore::TUBIT_B[2] = {0, 0};

// 馬達 MA / MB
const int BadeCarCore::TB_MA[4] = {27, 4, 17, 15};
const int BadeCarCore::TB_MB[4] = {13, 2, 12, 14};

// 編碼器 A / B
const int BadeCarCore::TB_EncA[4] = {18, 5, 35, 34};
const int BadeCarCore::TB_EncB[4] = {19, 23, 36, 39};

// Servo 腳位（固定）
const int BadeCarCore::TB_ServoPin[4] = {26, 25, 33, 32};


// =======================================================
// EncoderMotor 物件（四輪）
//（依照 M0~M3 = 右後、左後、右前、左前）
// =======================================================

namespace {

constexpr uint32_t kPPR       = 13;
constexpr uint32_t kReduction = 20;     // 20.4 簡化為 20
constexpr int16_t  kMaxDuty   = 1023;   // EncoderMotor PWM 最大 duty

// M0：右後
EncoderMotor em0(GPIO_NUM_27, GPIO_NUM_13,
                 GPIO_NUM_18, GPIO_NUM_19,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

// M1：左後
EncoderMotor em1(GPIO_NUM_4, GPIO_NUM_2,
                 GPIO_NUM_5, GPIO_NUM_23,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

// M2：右前
EncoderMotor em2(GPIO_NUM_17, GPIO_NUM_12,
                 GPIO_NUM_35, GPIO_NUM_36,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

// M3：左前
EncoderMotor em3(GPIO_NUM_15, GPIO_NUM_14,
                 GPIO_NUM_34, GPIO_NUM_39,
                 kPPR, kReduction, EncoderMotor::kAPhaseLeads);

} // namespace


// =======================================================
// Impl - 內部狀態
// =======================================================
struct BadeCarCore::Impl {

    EncoderMotor* m[4];

    int encoderCPR;   // 預設 1061（MG310）
    double kp, ki, kd;

    double posTarget[4];

    Impl() {
        m[0] = &em0;
        m[1] = &em1;
        m[2] = &em2;
        m[3] = &em3;

        encoderCPR = 1061;

        kp = 1.2;
        ki = 0.25;
        kd = 0;

        for (int i=0;i<4;i++)
            posTarget[i] = 0;
    }
};


// =======================================================
// 建構 / 解構
// =======================================================
BadeCarCore::BadeCarCore() {
    impl = new Impl();
}

BadeCarCore::~BadeCarCore() {
    delete impl;
}


// =======================================================
// init() - 初始化馬達 + Servo
// =======================================================
void BadeCarCore::init() {

    // 初始化 4 顆 EncoderMotor
    for (int i=0;i<4;i++)
        impl->m[i]->Init();

    // 初始化 Servo（用 LEDC channel 10~13, 50Hz, 16bit）
    for (int i=0;i<4;i++){
        int ch = 10 + i;
        ledcSetup(ch, 50, 16);
        ledcAttachPin(TB_ServoPin[i], ch);
    }
}


// =======================================================
// 按鈕
// =======================================================
bool BadeCarCore::readButton(ButtonT b){
    int idx = (int)b;
    if (TUBIT_B[idx] == 0) return false;
    return (digitalRead(TUBIT_B[idx]) == LOW);
}

bool BadeCarCore::readButtonEdge(ButtonT b, bool rising){
    int idx = (int)b;
    bool now  = readButton(b);
    bool last = lastButtonState[idx];
    lastButtonState[idx] = now;

    return rising ? (!last && now) : (last && !now);
}


// =======================================================
// Servo 控制
// =======================================================
void BadeCarCore::setServoPulseMs(ServoNum num, float ms){
    int idx = (int)num;
    int ch = 10 + idx;

    float duty_ratio = ms / 20.0f;
    duty_ratio = constrain(duty_ratio, 0.0f, 1.0f);

    uint32_t duty = duty_ratio * 65535.0f;
    ledcWrite(ch, duty);
}

void BadeCarCore::setServoAngle(ServoNum num, int deg){
    deg = constrain(deg, 0, 180);
    float us = SERVOMIN + (SERVOMAX - SERVOMIN) * (deg / 180.0f);
    setServoPulseMs(num, us / 1000.0f);
}


// =======================================================
// 馬達 PWM（開迴路）
// =======================================================
void BadeCarCore::setMotorPwm(MotorNum id, int pwm){
    int i = (int)id;
    pwm = constrain(pwm, -255, 255);
    int duty = map(pwm, -255, 255, -kMaxDuty, kMaxDuty);
    impl->m[i]->RunPwmDuty((int16_t)duty);
}

void BadeCarCore::setMotorsPwm(double m0, double m1, double m2, double m3){
    setMotorPwm(M0, (int)m0);
    setMotorPwm(M1, (int)m1);
    setMotorPwm(M2, (int)m2);
    setMotorPwm(M3, (int)m3);
}

void BadeCarCore::stopAllMotors(){
    for (int i=0;i<4;i++)
        impl->m[i]->Stop();
}

// =======================================================
// Encoder（回授）
// =======================================================

// 回傳編碼器脈衝數（0-based：M0~M3）
long BadeCarCore::readMotorEnc(int n){
    return impl->m[n]->EncoderPulseCount();
}

// pulse → 角度（度）
int BadeCarCore::readMotorDeg(int n){
    long p = readMotorEnc(n);
    return (int)((double)p * 360.0 / (double)impl->encoderCPR);
}

// 清除所有輪子的編碼器數值
void BadeCarCore::resetEnc(){
    for (int i=0;i<4;i++)
        impl->m[i]->ResetEncoderPulseCount();
}

// 設定某一顆編碼器的 pulse（例如手動歸零）
void BadeCarCore::setMotorEnc(int n, int64_t v){
    impl->m[n]->SetEncoderPulseCount(v);
}

// 修改 CPR（例如不同齒輪比的 MG310）
void BadeCarCore::setEncoderCPR(int cpr){
    impl->encoderCPR = cpr;
}


// =======================================================
// PID（EncoderMotor 內建速度控制）
// =======================================================
void BadeCarCore::initPID(){
    setPIDTunings(impl->kp, impl->ki, impl->kd);
}

void BadeCarCore::setPIDTunings(double p, double i, double d){
    impl->kp = p;
    impl->ki = i;
    impl->kd = d;

    // 套用到四顆輪子
    for (int i=0;i<4;i++)
        impl->m[i]->SetSpeedPid((float)p, (float)i, (float)d);
}

// EncoderMotor 自動更新 PID，此函式保留作相容用
void BadeCarCore::updatePID(){
    // no-op
}

// 回傳 PWM duty（給使用者 debug ）
long BadeCarCore::computePID(int n){
    return impl->m[n]->PwmDuty();
}


// =======================================================
// 閉迴路速度控制（RPM）
// =======================================================
//
// sp0~sp3 = 目標轉速（RPM）
//   正值 → 車子往前 / 該輪前轉
//   負值 → 該輪後退
//
void BadeCarCore::setMotorSpeed(double sp0, double sp1, double sp2, double sp3){
    impl->m[0]->RunSpeed((int16_t)sp0);
    impl->m[1]->RunSpeed((int16_t)sp1);
    impl->m[2]->RunSpeed((int16_t)sp2);
    impl->m[3]->RunSpeed((int16_t)sp3);
}


// =======================================================
// 位置控制（pulse）
// =======================================================
//
// 適用：
//   * 前進固定距離
//   * 左右旋轉指定角度（需自行換算 pulse）
//   * 回原點
//
// 基本原理：
//   err = target - current
//   rpm = err * Kp_pos
//   限幅到 ±300 RPM
//
void BadeCarCore::setMotorPosition(double p0, double p1, double p2, double p3){
    impl->posTarget[0] = p0;
    impl->posTarget[1] = p1;
    impl->posTarget[2] = p2;
    impl->posTarget[3] = p3;

    const double Kp_pos = 0.1;  // 比例控制係數

    for (int i=0;i<4;i++){
        double curr = impl->m[i]->EncoderPulseCount();
        double err  = impl->posTarget[i] - curr;
        double rpm  = err * Kp_pos;

        // 限速避免暴衝
        if (rpm > 300)  rpm = 300;
        if (rpm < -300) rpm = -300;

        impl->m[i]->RunSpeed((int16_t)rpm);
    }
}


// =======================================================
// 工具函式（運動學計算）
// =======================================================

// 四輪輸入中，取最大速度的絕對值
double BadeCarCore::getMaxVelocity(const double motors[4]) const{
    double maxv = fabs(motors[0]);
    for (int i=1;i<4;i++)
        if (fabs(motors[i]) > maxv)
            maxv = fabs(motors[i]);
    return maxv;
}

// 算四輪平均偏差
double BadeCarCore::averageSpeed(const double motors[4], double ref) const{
    return ((motors[0] - ref) +
            (motors[1] - ref) +
            (motors[2] - ref) +
            (motors[3] - ref)) / 4.0;
}

// (x,y) → (r,θ)
double BadeCarCore::toPolar(double x, double y, int component){
    double r = sqrt(x*x + y*y);
    double t = atan2(y, x);
    return (component == 0) ? r : t;
}

// (r,θ) → (x or y)
double BadeCarCore::toCartesian(double r, double t, int axis){
    return (axis == 0) ? (r * cos(t)) : (r * sin(t));
}


// =======================================================
// syncPidReference（相容舊 TuBitCore）
// =======================================================
void BadeCarCore::syncPidReference(bool keepOutput){
    // EncoderMotor 不需同步，保留空殼
}
