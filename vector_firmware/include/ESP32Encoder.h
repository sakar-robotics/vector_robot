#ifndef ESP32_ENCODER_H_
#define ESP32_ENCODER_H_

#include <driver/gpio.h>
#include <driver/pcnt.h>

#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX   // Maximum number of encoders supported - 8
#define _INT16_MAX 32766                   // Maximum value of a 16-bit signed integer
#define _INT16_MIN -32766                  // Minimum value of a 16-bit signed integer
#define ISR_CORE_USE_DEFAULT (0xffffffff)  // Use default core for ISR service

/**
 * @brief Enumeration for encoder counting modes.
 */
enum class encType
{
  single, /**< Single edge counting */
  half,   /**< Half quadrature counting */
  full    /**< Full quadrature counting */
};

/**
 * @brief Enumeration for internal pull resistor configuration.
 */
enum class puType
{
  up,    // Enable internal pull-up resistor
  down,  // Enable internal pull-down resistor
  none   // Do not use internal pull resistors
};

class ESP32Encoder;

/**
 * @brief Encoder ISR callback function pointer type.
 *
 * @param ctx Pointer to context, expected to be an ESP32Encoder instance.
 */
typedef void (*enc_isr_cb_t)(void*);

/**
 * @brief Class for handling an ESP32 rotary encoder using the PCNT peripheral.
 */
class ESP32Encoder
{
public:
  /**
   * @brief Construct a new ESP32Encoder object.
   * 
   * @param always_interrupt Set to true to enable an interrupt on every encoder pulse.
   * @param enc_isr_cb Callback executed on every encoder ISR; receives a pointer to this instance.
   * @param enc_isr_cb_data Optional data passed to the ISR callback. Defaults to this instance if nullptr.
   */
  ESP32Encoder(bool always_interrupt = false, enc_isr_cb_t enc_isr_cb = nullptr,
               void* enc_isr_cb_data = nullptr);

  /**
   * @brief Destroy the ESP32Encoder object.
   */
  ~ESP32Encoder();

  /**
   * @brief Attach a half quadrature encoder.
   *
   * @param aPintNumber GPIO pin number for channel A.
   * @param bPinNumber GPIO pin number for channel B.
   */
  void attachHalfQuad(int aPintNumber, int bPinNumber);

  /**
   * @brief Attach a full quadrature encoder.
   *
   * @param aPintNumber GPIO pin number for channel A.
   * @param bPinNumber GPIO pin number for channel B.
   */
  void attachFullQuad(int aPintNumber, int bPinNumber);

  /**
   * @brief Attach a single edge encoder.
   *
   * @param aPintNumber GPIO pin number for channel A.
   * @param bPinNumber GPIO pin number for channel B.
   */
  void attachSingleEdge(int aPintNumber, int bPinNumber);

  int64_t getCount();
  int64_t clearCount();
  int64_t pauseCount();
  int64_t resumeCount();
  /**
   * @brief Detach the encoder and free associated resources.
   */
  void detach();
  bool isAttached()
  {
    return attached;
  }

  /**
   * @brief Set the encoder count.
   *
   * @param value The value to which the count should be set.
   */
  void setCount(int64_t value);

  /**
   * @brief Set the filter value for debouncing.
   *
   * @param value Filter value in the range [0, 1023]. A value of 0 disables the filter.
   */
  void setFilter(uint16_t value);

  // Array holding pointers to encoder instances.
  static ESP32Encoder* encoders[MAX_ESP32_ENCODERS];

  // Public member variables
  bool always_interrupt;       // Interrupt on every encoder pulse flag
  gpio_num_t aPinNumber;       // GPIO Number for channel A
  gpio_num_t bPinNumber;       // GPIO Number for channel B
  pcnt_unit_t unit;            // PCNT hardware unit used
  int countsMode = 2;          // Count mode (currently fixed to 2)
  volatile int64_t count = 0;  // Software-compensated encoder count
  pcnt_config_t r_enc_config;  // PCNT configuration structure

  // Internal pull resistor configuration
  static puType useInternalWeakPullResistors;

  // CPU core for ISR service; set to ISR_CORE_USE_DEFAULT to use the default core
  static uint32_t isrServiceCpuCore;

  // ISR callback function
  enc_isr_cb_t _enc_isr_cb;

  // Data passed to the ISR Callback
  void* _enc_isr_cb_data;

private:
  // Flag indicating whether the global ISR has been attached
  static bool attachedInterrupt;

  /**
   * @brief Internal attach function with specific encoder type.
   *
   * @param aPintNumber GPIO pin number for channel A.
   * @param bPinNumber GPIO pin number for channel B.
   * @param et Encoder type (single, half, or full).
   */
  void attach(int aPintNumber, int bPinNumber, encType et);

  /**
   * @brief Get the raw count value from the hardware counter.
   *
   * @return int64_t The raw counter value, adjusted for limit events.
   */
  int64_t getCountRaw();
  bool attached;   // True if the encoder is currently attached
  bool direction;  // Current counting direction (unused public API)
  bool working;    // Status flag indicating if the encoder is working
};

#endif  // ESP32_ENCODER_H_