#pragma once     // 防止重複 include

#include <stdint.h>    // 基本整數型別，如 uint32_t, int16_t
#include <driver/gpio.h>   // ESP32 GPIO 驅動，提供 GPIO_NUM_xxx
#include <functional>      // 提供 std::function，用於 callback

namespace em {

// ================================
// EncoderMotor 類別：封裝
//   ● PWM（雙腳 H-bridge）
//   ● Encoder（A/B 相）
//   ● 速度 PID
//   ● Thread 背景計算速度
// ================================
class EncoderMotor {
 public:

  // -------------------------------
  // A/B 相位關係（用來判斷正轉/反轉）
  // kAPhaseLeads：A 先變 → 正方向
  // kBPhaseLeads：B 先變 → 正方向
  // -------------------------------
  enum EncoderPhase {
    kAPhaseLeads = 0,
    kBPhaseLeads = 1,
  };

  // 建構子：給完整參數。包含：
  //   motor_pos_pin：馬達「正轉」PWM
  //   motor_neg_pin：馬達「反轉」PWM
  //   encoder_a_pin：A 相
  //   encoder_b_pin：B 相
  //   ppr：馬達輸入端的「每轉脈衝」(不是輪子端)
  //   gear_ratio：減速比（用來算總 CPR）
  //   phase_type：A/B 誰領先
  EncoderMotor(gpio_num_t motor_pos_pin,
               gpio_num_t motor_neg_pin,
               gpio_num_t encoder_a_pin,
               gpio_num_t encoder_b_pin,
               uint32_t ppr,
               uint32_t gear_ratio,
               EncoderPhase phase_type);

  ~EncoderMotor();   // 解構子，負責關掉 thread

  // -------------------------------
  // Init：初始化整個 motor
  //   ● 設定 PWM
  //   ● attach 中斷
  //   ● 啟動背景執行緒（算速度）
  // -------------------------------
  void Init();

  // -------------------------------
  // PWM 控制（開迴路）
  // duty ∈ [-1023, +1023]
  // -------------------------------
  void RunPwmDuty(int16_t duty);

  // 讀取「目前 PWM Duty」（debug 用）
  int16_t PwmDuty() const;

  // -------------------------------
  // Stop：馬達兩腳全部拉高
  // = 主動煞車 (brake)
  // -------------------------------
  void Stop();

  // -------------------------------
  // Speed Control（閉迴路）
  // 設定目標轉速（RPM）
  // -------------------------------
  void RunSpeed(int16_t rpm);

  // 設定 PID 參數
  void SetSpeedPid(float kp, float ki, float kd);

  // -------------------------------
  // Encoder：回授
  // -------------------------------

  // 讀取 encoder 目前 pulse（int64_t，避免溢位）
  int64_t EncoderPulseCount() const;

  // 設定脈衝（例如手動歸零）
  void SetEncoderPulseCount(int64_t v);

  // 歸零（把 pulse 設為 0）
  void ResetEncoderPulseCount();

  // -------------------------------
  // 設定速度更新 callback（選用）
  // host 提供一個函式，在每次更新速度時觸發
  // -------------------------------
  void SetVelocityUpdateCallback(std::function<void(float)> cb);

 private:
  // ===============================
  // 低階成員（硬體腳位設定）
  // ===============================

  gpio_num_t motor_pos_pin_;   // PWM 正轉腳
  gpio_num_t motor_neg_pin_;   // PWM 反轉腳

  gpio_num_t encoder_a_pin_;   // A 相
  gpio_num_t encoder_b_pin_;   // B 相

  // 計算總 CPR：ppr(13) × gear_ratio(20) × full-quad(4)
  uint32_t pulses_per_rev_;

  EncoderPhase phase_type_;    // A/B 相位關係

  // ===============================
  // Encoder 統計
  // ===============================

  volatile int64_t encoder_pulse_count_ = 0;
  // volatile → 避免編譯器優化掉，因為中斷會修改它

  // 上一次計算速度時記錄的 pulse（用來算 Δpulse）
  int64_t last_pulse_count_ = 0;

  // ===============================
  // PWM duty
  // ===============================
  volatile int16_t pwm_duty_ = 0;

  // ===============================
  // PID 相關變數
  // ===============================
  float kp_ = 1.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  float speed_ = 0.0f;          // 目前估算速度（RPM）
  float speed_target_ = 0.0f;   // 目標速度（RPM）
  float speed_err_int_ = 0.0f;  // 積分項
  float speed_err_prev_ = 0.0f; // 微分用

  // 速度更新 callback
  std::function<void(float)> velocity_update_cb_ = nullptr;

  // ===============================
  // Thread（背景執行緒）
  // 每隔固定時間：
  //   ● 計算速度
  //   ● 更新 PID
  //   ● 更新 PWM
  // ===============================

  bool thread_quit_ = false;   // 要求關閉 thread 的旗標

  // thread 物件（由 Init() 啟動）
  void SpeedThread();          // thread 主動 loop

  // 用於 FreeRTOS 中斷與 thread 同步
  static void IRAM_ATTR EncoderIsrHandler(void* arg);  // A/B 中斷 service routine

};

}  // namespace em
