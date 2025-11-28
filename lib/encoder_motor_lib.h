#pragma once      // 防止重複 include

#ifndef _EM_ENCODER_MOTOR_LIB_H_
#define _EM_ENCODER_MOTOR_LIB_H_
/**
 * @file encoder_motor_lib.h
 */

#include <WString.h>   // Arduino String 類別
namespace em {
namespace esp_encoder_motor_lib {
constexpr uint8_t kVersionMajor = 1;   // 主版本號
constexpr uint8_t kVersionMinor = 1;   // 次版本號
constexpr uint8_t kVersionPatch = 1;   // 修訂號（patch）
String Version() {
  return String(kVersionMajor) + '.' + kVersionMinor + '.' + kVersionPatch;
}
Serial.println(em::esp_encoder_motor_lib::Version());
}  // namespace esp_encoder_motor_lib
}  // namespace em
#endif  // _EM_ENCODER_MOTOR_LIB_H_
