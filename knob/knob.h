#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/i2c/i2c.h"

typedef struct {
  uint8_t type[3];
  uint8_t port;
  uint8_t pin;
  uint8_t enc_ch;
  uint8_t reg_iopwm;
  uint8_t bit_iopwm;
  uint8_t pwm_ch;
  uint8_t adc_ch;
  uint8_t pwm_module;
  uint8_t mode;
  uint8_t inv_output;
} pin_struct;

namespace esphome {
namespace knob_ns {

class MyKnobComponent : public light::LightOutput, public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
  public:
    void setup() override;
    void loop() override;
    void update() override;
    void dump_config() override;
    void update_knob_value(sensor::Sensor* knob) { this->knob_sensor = knob; }
    light::LightTraits get_traits() override;
    void write_state(light::LightState *state) override;
  protected:
    sensor::Sensor* knob_sensor{nullptr};
    light::LightState* light_state{nullptr};
    void write_i2c_byte(uint8_t reg, uint8_t data);
    uint8_t read_i2c_byte(uint8_t reg);
    void set_bits(uint8_t reg, uint8_t bits);
    void set_bit(uint8_t reg, uint8_t bit);
    uint8_t get_bit (uint8_t reg, uint8_t bit);
    void enable_adc();
    void clr_bits(uint8_t reg, uint8_t bits);
    void clr_bit(uint8_t reg, uint8_t bit);
    void change_bit(uint8_t reg, uint8_t bit, uint8_t state);
    void set_mode(uint8_t pin, uint8_t mode, uint8_t schmitt_trigger, uint8_t invert);
    uint8_t pwm_loading (uint8_t pwm_module);
    void pwm_load (uint8_t pwm_module, uint8_t wait_for_load);
    int read_rotary_encoder();
    void setup_rotary_encoder(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c);
    void output(uint8_t pin, uint16_t value, uint8_t load, uint8_t wait_for_load);
    void clear_rotary_encoder();
    void reset_knob();
    void set_pwm_period(uint16_t value);
    void set_pwm_control(uint8_t divider);
    void set_knob_hue(uint8_t vol);
    void set_knob_colour(float r, float g, float b);
};

}
}
