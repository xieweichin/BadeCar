#pragma once     // 防止重複 include

#ifndef _EM_MOTOR_H_   // include guard：避免重複定義
#define _EM_MOTOR_H_

/**
 * @file motor.h
 */

#include <stdint.h>          // 整數型別：uint8_t, int16_t
#include <driver/gpio.h>     // ESP32 GPIO 腳位相關
#include <Arduino.h>         // Arduino API（僅為相容性）
namespace em {

class Motor {
 public:

  // =====================================================
  // 1) 建構子（新版本 ESP-IDF 3.0 以上）
  //    只給腳位，使用 ledcAttachChannel()
  // =====================================================
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  Motor(const uint8_t positive_pin, const uint8_t negative_pin);
#endif
  // =====================================================
  // 2) 建構子（舊版 Arduino-ESP32）
  //    多帶兩個參數：PWM channel
  // =====================================================
  Motor(const uint8_t positive_pin,
        const uint8_t positive_pin_ledc_channel,
        const uint8_t negative_pin,
        const uint8_t negative_pin_ledc_channel);
  // =====================================================
  // 3) Init()
  // 設定 LEDC + 停止馬達
  // =====================================================
  void Init();
  // =====================================================
  // 4) RunPwmDuty(duty)
  // 開迴路 PWM（-1023 ~ +1023）
  // =====================================================
  void RunPwmDuty(const int16_t pwm_duty);
  // =====================================================
  // 5) 取得目前輸出的 PWM duty（debug 用）
  // =====================================================
  int16_t PwmDuty() const;
  // =====================================================
  // 6) Stop()
  // 兩腳輸出最大 duty → 主動煞車
  // =====================================================
  void Stop();
 private:
  static constexpr uint32_t kPwmFrequency = 20000;  // PWM 頻率 = 20kHz（安靜、不會叫）
  static constexpr uint8_t  kPwmResolution = 10;    // 10-bit = duty 0~1023
  static constexpr uint16_t kMaxPwmDuty = 1023;     // 最大 duty
  uint8_t positive_pin_;             // 正轉腳位
  uint8_t negative_pin_;             // 反轉腳位
  uint8_t positive_pin_ledc_channel_ = 0xFF;   // 若 = 0xFF → 新版 API 模式
  uint8_t negative_pin_ledc_channel_ = 0xFF;
  int16_t pwm_duty_ = 0;             // 目前 PWM duty 值
};
}  // namespace em

#endif  // _EM_MOTOR_H_
