/**
 * @file encoder_motor.cpp
 */

#include "encoder_motor.h"     // 對應的標頭檔，宣告類別與函式
#include <Arduino.h>           // Arduino 基本函式，例如 pinMode / digitalRead 等
#include <driver/ledc.h>       // ESP32 LEDC（高速 PWM）控制
#include <esp_timer.h>         // 提供 esp_timer_get_time()（精準微秒計時）
#include <freertos/FreeRTOS.h> // Thread / RTOS
#include <freertos/task.h>
#include <atomic>              // 使用 atomic<bool> 等

namespace em {

// ==========================================================
//  建構子：設定腳位 + 計算 CPR
// ==========================================================
EncoderMotor::EncoderMotor(gpio_num_t motor_pos_pin,
                           gpio_num_t motor_neg_pin,
                           gpio_num_t encoder_a_pin,
                           gpio_num_t encoder_b_pin,
                           uint32_t ppr,
                           uint32_t gear_ratio,
                           EncoderPhase phase_type)
    : motor_pos_pin_(motor_pos_pin),   // PWM 正轉腳
      motor_neg_pin_(motor_neg_pin),   // PWM 反轉腳
      encoder_a_pin_(encoder_a_pin),   // encoder A 相
      encoder_b_pin_(encoder_b_pin),   // encoder B 相
      // 計算總 CPR：ppr × 減速比 × 4（full quadrature）
      pulses_per_rev_(ppr * gear_ratio * 4),
      phase_type_(phase_type) {

  // 尚未產生 thread、也未 attach 中斷，這些在 Init() 處理
}


// ==========================================================
//  解構子：確保 thread 正常關閉
// ==========================================================
EncoderMotor::~EncoderMotor() {
  thread_quit_ = true;       // 要 thread 結束（flag）
  vTaskDelay(20 / portTICK_PERIOD_MS);  // 稍等一下讓 thread 收尾
}


// ==========================================================
// Init(): 初始化馬達 PWM + Encoder 中斷 + 啟動背景 thread
// ==========================================================
void EncoderMotor::Init() {

  // --------------------------
  // 設定馬達 PWM 腳（使用 LEDC）
  // --------------------------
  pinMode(motor_pos_pin_, OUTPUT);
  pinMode(motor_neg_pin_, OUTPUT);

  // ESP32 LEDC 需要設定 channel 與頻率
  // duty 最大值由解析度決定（我們使用 10-bit: 0~1023）
  ledcSetup(motor_pos_pin_, 20000, 10);  // 使用 pin 當作 channel（ESP32 允許）
  ledcSetup(motor_neg_pin_, 20000, 10);  // 20kHz，沒有噪音，適合馬達

  ledcAttachPin(motor_pos_pin_, motor_pos_pin_);
  ledcAttachPin(motor_neg_pin_, motor_neg_pin_);

  Stop();  // 初始化先停止馬達（PWM 雙腳拉高，等同煞車）


  // --------------------------
  // 設定 Encoder A/B 相腳位（中斷輸入）
  // --------------------------
  pinMode(encoder_a_pin_, INPUT_PULLUP);
  pinMode(encoder_b_pin_, INPUT_PULLUP);

  // attach 中斷：A 相觸發 → 呼叫 EncoderIsrHandler
  // ESP32 要用 attachInterruptArg 才能傳遞 this 指標
  attachInterruptArg(encoder_a_pin_,
                     EncoderIsrHandler,   // ISR 函式（靜態）
                     this,                // 傳遞物件指標
                     CHANGE);             // A 相只要變化就觸發


  // --------------------------
  // 啟動速度計算 thread
  // --------------------------
  thread_quit_ = false;

  // 建立一個 FreeRTOS task（背景執行速度計算 + PID）
  xTaskCreate(
      [](void* arg){
          static_cast<EncoderMotor*>(arg)->SpeedThread();
      },
      "EM_SpeedThread",
      4096,         // Stack 大小
      this,         // 任務參數（傳遞 this）
      1,            // Priority
      nullptr);     // 回傳 Task handle（不需要）
}


// ==========================================================
// PWM：開迴路控制（RunPwmDuty）
// duty: -1023 ~ +1023
// ==========================================================
void EncoderMotor::RunPwmDuty(int16_t duty) {

  pwm_duty_ = duty;  // 儲存當前 duty（給外部查詢）

  duty = constrain(duty, -1023, 1023);    // 限制在範圍內

  if (duty > 0) {
    // --------------------
    // 正轉
    // --------------------
    ledcWrite(motor_pos_pin_, duty);   // 正腳輸出 PWM
    ledcWrite(motor_neg_pin_, 0);      // 反腳 0

  } else if (duty < 0) {
    // --------------------
    // 反轉：把 duty 改為正的
    // --------------------
    int16_t v = -duty;
    ledcWrite(motor_pos_pin_, 0);
    ledcWrite(motor_neg_pin_, v);

  } else {
    // --------------------
    // duty = 0 → 停止無煞車（coast）
    // 如果要煞車，應該呼叫 Stop()
    // --------------------
    ledcWrite(motor_pos_pin_, 0);
    ledcWrite(motor_neg_pin_, 0);
  }
}


// ==========================================================
// Stop(): 主動煞車
// PWM 雙腳輸出最大 → H-bridge 兩腳都為 HIGH
// = 電子煞車（短路煞車）
// ==========================================================
void EncoderMotor::Stop() {
  pwm_duty_ = 0;

  // 1023 = 10-bit 最大 duty
  ledcWrite(motor_pos_pin_, 1023);
  ledcWrite(motor_neg_pin_, 1023);
}


// ==========================================================
// Encoder 中斷（ISR）
// A 相變化時觸發，根據 B 相判斷正反轉
// ==========================================================
void IRAM_ATTR EncoderMotor::EncoderIsrHandler(void* arg) {

  EncoderMotor* self = static_cast<EncoderMotor*>(arg);

  // 讀取 A / B 相電平
  int a = digitalRead(self->encoder_a_pin_);
  int b = digitalRead(self->encoder_b_pin_);

  // --------------------------
  // Quadrature 方向判斷（核心）
  // --------------------------
  bool forward;

  if (self->phase_type_ == kAPhaseLeads) {
      // 若 A 領先 B → A 的變化方向決定正負
      forward = (a != b);
  } else {
      // 若 B 領先 A → 方向相反
      forward = (a == b);
  }

  // --------------------------
  // 更新 encoder 計數
  // --------------------------
  if (forward)
      self->encoder_pulse_count_++;
  else
      self->encoder_pulse_count_--;
}
// =====================================================================
// 傳回目前 PWM duty（外部 debug 用）
// =====================================================================
int16_t EncoderMotor::PwmDuty() const {
  return pwm_duty_;
}
// =====================================================================
// RunSpeed(rpm)
// 設定「目標 RPM」給速度 PID
// =====================================================================
void EncoderMotor::RunSpeed(int16_t rpm) {
  speed_target_ = rpm;   // 儲存目標速度，由 SpeedThread() 用
}
// =====================================================================
// SetSpeedPid(kp, ki, kd)
// 設定速度控制 PID 的 3 個係數
// SpeedThread() 裡會用到
// =====================================================================
void EncoderMotor::SetSpeedPid(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
// =====================================================================
// EncoderPulseCount()
// Read-only：外部讀取目前 encoder 計數
// =====================================================================
int64_t EncoderMotor::EncoderPulseCount() const {
  return encoder_pulse_count_;    // volatile → 允許 ISR隨時更新
}
// =====================================================================
// SetEncoderPulseCount()
// 手動設定編碼器脈衝：例：歸零用
// =====================================================================
void EncoderMotor::SetEncoderPulseCount(int64_t v) {
  encoder_pulse_count_ = v;
}
// =====================================================================
// ResetEncoderPulseCount()：快速歸零
// =====================================================================
void EncoderMotor::ResetEncoderPulseCount() {
  encoder_pulse_count_ = 0;
}
// =====================================================================
// SetVelocityUpdateCallback(cb)
// 可選：速度更新時呼叫 callback
// 例如要顯示目前速度、紀錄資料等
// =====================================================================
void EncoderMotor::SetVelocityUpdateCallback(std::function<void(float)> cb) {
  velocity_update_cb_ = cb;
}
// =====================================================================
// SpeedThread()
// 背景執行緒：負責速度演算、PID、PWM
// =====================================================================
void EncoderMotor::SpeedThread() {

  // 前一次的 timestamp（微秒）
  uint64_t last_time_us = esp_timer_get_time();

  last_pulse_count_ = encoder_pulse_count_;   // 初始化上一個 pulse

  while (!thread_quit_) {     // thread_quit_ = true → 結束執行緒

    // --------------------------------------------------
    // 每次 loop 停 10ms（100Hz 更新）
    // --------------------------------------------------
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 取得現在時間（微秒）
    uint64_t now_us = esp_timer_get_time();
    uint64_t dt_us  = now_us - last_time_us;   // 這一次更新間隔
    last_time_us = now_us;

    if (dt_us == 0)
      continue;  // 避免 0 除（理論上不會發生）

    // --------------------------------------------------
    // 計算脈衝變化量（Δpulse）
    // --------------------------------------------------
    int64_t now_pulse   = encoder_pulse_count_;
    int64_t delta_pulse = now_pulse - last_pulse_count_;
    last_pulse_count_   = now_pulse;

    // --------------------------------------------------
    // pulse → RPM
    // RPM = (Δpulse / CPR) / Δt * 60
    // --------------------------------------------------

    // Δt（秒）
    double dt_sec = dt_us / 1e6;

    // 每圈脈衝數 pulses_per_rev_ = PPR * 減速比 * 4
    double rev = (double)delta_pulse / (double)pulses_per_rev_;

    // 換成 RPM
    speed_ = (float)(rev / dt_sec * 60.0);

    // --------------------------------------------------
    // PID 控制（speed_target_ → speed_）
    // --------------------------------------------------
    float err = speed_target_ - speed_;     // 誤差 = 目標 - 目前

    // 積分前先更新積分項
    speed_err_int_ += err * dt_sec;

    // Anti-windup（避免積分爆掉）
    if (speed_err_int_ > 1000)  speed_err_int_ = 1000;
    if (speed_err_int_ < -1000) speed_err_int_ = -1000;

    // 微分項（用誤差差分）
    float d = (err - speed_err_prev_) / dt_sec;
    speed_err_prev_ = err;

    // PID output
    float output =
        kp_ * err +
        ki_ * speed_err_int_ +
        kd_ * d;

    // --------------------------------------------------
    // 限制 output 在 PWM duty 限制內（-1023 ~ 1023）
    // --------------------------------------------------
    if (output > 1023)  output = 1023;
    if (output < -1023) output = -1023;

    // --------------------------------------------------
    // 執行 PWM（RunPwmDuty）
    // --------------------------------------------------
    RunPwmDuty((int16_t)output);

    // --------------------------------------------------
    // 若有註冊 callback → 告知使用者目前速度
    // --------------------------------------------------
    if (velocity_update_cb_)
      velocity_update_cb_(speed_);
  }

  // thread_quit_ = true → 離開迴圈，速度歸零
  RunPwmDuty(0);
}
delta_pulse = now_pulse - last_pulse
rev = delta_pulse / CPR
RPM = rev / dt_sec * 60
err = target - speed
output = kp*err + ki*積分 + kd*微分
output ∈ [-1023, +1023]
RunPwmDuty(output)
motor.SetVelocityUpdateCallback([](float rpm){
    Serial.println(rpm);
});
}  // namespace em
using em::EncoderMotor;
