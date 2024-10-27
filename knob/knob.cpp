#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/gpio.h"

#include <time.h>

#include "knob_regs.h"
#include "knob.h"

namespace esphome {
namespace knob_ns {

static const char *TAG = "rgb_rotary_encoder";
uint8_t setup_complete = 0;

int cnt = 0;

// These values encode our desired pin function: IO, ADC, PWM
// alongside the GPIO MODE for that port and pin (section 8.1)
// the 5th bit additionally encodes the default output state
const uint8_t  PIN_MODE_IO = 0b00000;   // General IO mode, IE: not ADC or PWM
const uint8_t  PIN_MODE_QB = 0b00000;   // Output, Quasi-Bidirectional mode
const uint8_t  PIN_MODE_PP = 0b00001;   // Output, Push-Pull mode
const uint8_t  PIN_MODE_IN = 0b00010;   // Input-only (high-impedance)
const uint8_t  PIN_MODE_PU = 0b10000;   // Input (with pull-up)
const uint8_t  PIN_MODE_OD = 0b00011;   // Output, Open-Drain mode
const uint8_t  PIN_MODE_PWM = 0b00101;  // PWM, Output, Push-Pull mode
const uint8_t  PIN_MODE_ADC = 0b01010;  // ADC, Input-only (high-impedance)
const char *MODE_NAMES[3] = {"IO", "PWM", "ADC"};
const char *GPIO_NAMES[4] = {"QB", "PP", "IN", "OD"};
const char *STATE_NAMES[2] = {"LOW", "HIGH"};
const uint8_t  KNOB_A = 12;
const uint8_t  KNOB_B = 3;
const uint8_t  KNOB_C = 11;
const uint8_t  LED_RED = 1;
const uint8_t  LED_GREEN = 7;
const uint8_t  LED_BLUE = 2;

typedef struct {
  char colour[8];
  uint8_t values[3];
} colours_struct;
const colours_struct colours[8] = {
  {"RED", {255, 0, 0}},
  {"GREEN", {0, 255, 0}},
  {"YELLOW", {255, 255, 0}},
  {"BLUE", {0, 0, 255}},
  {"PURPLE", {255, 0, 255}},
  {"CYAN", {0, 255, 255}},
  {"WHITE", {255, 255, 255}},
  {"OFF", {0, 0, 0}}
};

pin_struct pins[15] = {{},                                                                                                                                                           // Pin |  ADC   |  PWM   |  ENC  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM},                .port = 1, .pin = 5, .enc_ch = 1, .reg_iopwm = 1, .bit_iopwm = 5, .pwm_ch = 5, .adc_ch = 0, .mode = 255},              // 1   |        | [CH 5] | CH 1  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM},                .port = 1, .pin = 0, .enc_ch = 2, .reg_iopwm = 0, .bit_iopwm = 2, .pwm_ch = 2, .adc_ch = 0, .mode = 255},              // 2   |        | [CH 2] | CH 2  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM},                .port = 1, .pin = 2, .enc_ch = 3, .reg_iopwm = 0, .bit_iopwm = 0, .pwm_ch = 0, .adc_ch = 0, .mode = 255},              // 3   |        | [CH 0] | CH 3  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM},                .port = 1, .pin = 4, .enc_ch = 4, .reg_iopwm = 1, .bit_iopwm = 1, .pwm_ch = 1, .adc_ch = 0, .mode = 255},              // 4   |        | [CH 1] | CH 4  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM},                .port = 0, .pin = 0, .enc_ch = 5, .reg_iopwm = 0, .bit_iopwm = 3, .pwm_ch = 3, .adc_ch = 0, .mode = 255},              // 5   |        | [CH 3] | CH 5  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM},                .port = 0, .pin = 1, .enc_ch = 6, .reg_iopwm = 0, .bit_iopwm = 4, .pwm_ch = 4, .adc_ch = 0, .mode = 255},              // 6   |        | [CH 4] | CH 6  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM, PIN_MODE_ADC},  .port = 1, .pin = 1, .enc_ch = 7, .reg_iopwm = 0, .bit_iopwm = 1, .pwm_ch = 1, .adc_ch = 7, .mode = 255},  // 7   | [CH 7] |  CH 1  | CH 7  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM, PIN_MODE_ADC},  .port = 0, .pin = 3, .enc_ch = 8, .reg_iopwm = 0, .bit_iopwm = 5, .pwm_ch = 5, .adc_ch = 6, .mode = 255},  // 8   | [CH 6] |  CH 5  | CH 8  |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM, PIN_MODE_ADC},  .port = 0, .pin = 4, .enc_ch = 9, .reg_iopwm = 1, .bit_iopwm = 3, .pwm_ch = 3, .adc_ch = 5, .mode = 255},  // 9   | [CH 5] |  CH 3  | CH 9  |
    {.type = {PIN_MODE_IO, PIN_MODE_ADC},                .port = 3, .pin = 0, .enc_ch = 10,.reg_iopwm = 0, .bit_iopwm = 0, .pwm_ch = 0, .adc_ch = 1, .mode = 255},  // 10  | [CH 1] |        | CH 10 |
    {.type = {PIN_MODE_IO, PIN_MODE_ADC},                .port = 0, .pin = 6, .enc_ch = 11,.reg_iopwm = 0, .bit_iopwm = 0, .pwm_ch = 0, .adc_ch = 3, .mode = 255},  // 11  | [CH 3] |        | CH 11 |
    {.type = {PIN_MODE_IO, PIN_MODE_PWM, PIN_MODE_ADC},  .port = 0, .pin = 5, .enc_ch = 12,.reg_iopwm = 1, .bit_iopwm = 2, .pwm_ch = 2, .adc_ch = 4, .mode = 255},  // 12  | [CH 4] |  CH 2  | CH 12 |
    {.type = {PIN_MODE_IO, PIN_MODE_ADC},                .port = 0, .pin = 7, .enc_ch = 13,.reg_iopwm = 0, .bit_iopwm = 0, .pwm_ch = 0, .adc_ch = 2, .mode = 255},  // 13  | [CH 2] |        | CH 13 |
    {.type = {PIN_MODE_IO, PIN_MODE_ADC},                .port = 1, .pin = 7, .enc_ch = 14,.reg_iopwm = 0, .bit_iopwm = 0, .pwm_ch = 0, .adc_ch = 0, .mode = 255},  // 14  | [CH 0] |        | CH 14 |
};

uint8_t regs_pwml[] = {REG_PWM0L, REG_PWM1L, REG_PWM2L, REG_PWM3L, REG_PWM4L, REG_PWM5L};
uint8_t regs_pwmh[] = {REG_PWM0H, REG_PWM1H, REG_PWM2H, REG_PWM3H, REG_PWM4H, REG_PWM5H};
uint8_t regs_m1[] = {REG_P0M1, REG_P1M1, 0, REG_P3M1};
uint8_t regs_m2[] = {REG_P0M2, REG_P1M2, 0, REG_P3M2};
uint8_t regs_p[] = {REG_P0, REG_P1, REG_P2, REG_P3};
uint8_t regs_ps[] = {REG_P0S, REG_P1S, REG_P2S, REG_P3S};
uint8_t regs_piocon[] = {REG_PIOCON0, REG_PIOCON1};
int encoder_last = 0;
int encoder_offset = 0;
uint8_t knob_value = 0;         // value read from rotary encoder

// check whether a number is in an array
bool array_contains(uint8_t arr[], uint8_t len, uint8_t num) {
  for (uint8_t i = 0; i < len; i++) {
    if (arr[i] == num) {
        return true;
    }
  }
  return false;
}

void delayms(int x) {
//    struct timespec ts;
//    ts.tv_sec = 0;
//    ts.tv_nsec = x * 1000000;
//    nanosleep(&ts, NULL);
  usleep(x * 1000);
}

bool isPowerOfTwo(uint8_t n) {
  return (n & (n - 1)) == 0;
}

int whichPowerOfTwo(uint8_t n) {
  uint8_t power = 0;
  while (n >>= 1) {
    ++power;
  }
  return power;
}

struct RGB {
    uint8_t r, g, b;
};

RGB hsvToRgb(float h) {
    float r, g, b;
    uint8_t i = uint8_t(h / 60) % 6;
    float f = h / 60 - i;
    float q = 1 - f;

    switch (i) {
        case 0: r = 1; g = f; b = 0; break;
        case 1: r = q; g = 1; b = 0; break;
        case 2: r = 0; g = 1; b = f; break;
        case 3: r = 0; g = q; b = 1; break;
        case 4: r = f; g = 0; b = 1; break;
        case 5: r = 1; g = 0; b = q; break;
    }

    return RGB{uint8_t(r * 255), uint8_t(g * 255), uint8_t(b * 255)};
}

void MyKnobComponent::write_i2c_byte(uint8_t reg, uint8_t data) {
  if (this->write_register(reg, &data, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write to register %2x", reg);
  }
}

uint8_t MyKnobComponent::read_i2c_byte(uint8_t reg) {
  uint8_t data;
  if (this->read_register(reg, &data, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read from register %2x", reg);
  }
  return data;
}

// Set the specified bits (using a mask) in a register.
void MyKnobComponent::set_bits(uint8_t reg, uint8_t bits) {
  //printf("set bits %02X to %02X\n", $bits, $reg);
  if (array_contains(BIT_ADDRESSED_REGS, BIT_ADDRESSED_REGS_SIZE, reg)) {
    uint8_t bit;
    for (bit = 0; bit < 8; bit++) {
      if (bits & (1 << bit) != 0) {
        this->write_i2c_byte(reg, bit | 0b1000);
      }
    }
  } else {
    uint8_t value = this->read_i2c_byte(reg);
    delayms(10);                            // delay for 10ms
    this->write_i2c_byte(reg, value | bits);
    //print("Check: "); read_i2c_byte($reg);
  }
}

// Set the specified bit (nth position from right) in a register.
void MyKnobComponent::set_bit(uint8_t reg, uint8_t bit) {
  //printf("set bit $bit in %02X\n", $reg);
  set_bits(reg, (1 << bit));
}

// Returns the specified bit (nth position from right) from a register."""
uint8_t MyKnobComponent::get_bit (uint8_t reg, uint8_t bit) {
  return this->read_i2c_byte(reg) & (1 << bit);
}

// Enable the analog to digital converter."""
void MyKnobComponent::enable_adc() {
  this->set_bit(REG_ADCCON1, 0);
};

// Clear the specified bits (using a mask) in a register.
void MyKnobComponent::clr_bits(uint8_t reg, uint8_t bits) {
  if (array_contains(BIT_ADDRESSED_REGS, BIT_ADDRESSED_REGS_SIZE, reg)) {
    //print("clear bits $bits from bit addressed $reg\n");
    uint8_t bit;
    for (bit = 0; bit < 8; bit++) {
      if (bits & (1 << bit)) {
        this->write_i2c_byte(reg, 0b0000 | bit);
      }
    }
  } else {
    //print("clear bits $bits from $reg\n");
    uint8_t value = read_i2c_byte(reg);
    delayms(10);                            // delay for 10ms
    this->write_i2c_byte(reg, value & ~bits);
    //print("Check: "); read_i2c_byte($reg);
  }
}

// Clear the specified bit (nth position from right) in a register.
void MyKnobComponent::clr_bit(uint8_t reg, uint8_t bit) {
  //printf("clear bit $bit in %02X\n", $reg);
  this->clr_bits(reg, (1 << bit));
}

// Toggle one register bit on/off
void MyKnobComponent::change_bit(uint8_t reg, uint8_t bit, uint8_t state) {
  if (state != 0) {
    this->set_bit(reg, bit);
  } else {
    this->clr_bit(reg, bit);
  }
}

// Set a pin output mode.
// Note mode is one of the supplied IN, OUT, PWM or ADC constants
void MyKnobComponent::set_mode(uint8_t pin, uint8_t mode, uint8_t schmitt_trigger = 0, uint8_t invert = 0) {
  pin_struct io_pin = pins[pin];
  //print_hash_params($io_pin);
  if (io_pin.mode == mode) {
    return;
  }

  uint8_t gpio_mode = mode & 0b11;
  uint8_t io_mode = (mode >> 2) & 0b11;
  uint8_t initial_state = mode >> 4;
  ESP_LOGI(TAG, "Setting pin %d to gpio mode %s and io mode %s", pin, GPIO_NAMES[gpio_mode], MODE_NAMES[io_mode]);

  if ((io_mode != PIN_MODE_IO) && (! array_contains(io_pin.type, 3, mode))) {
    ESP_LOGE(TAG, "Pin %d does not support %s", pin, MODE_NAMES[io_mode]);
    return;
  }

  pins[pin].mode = mode;
  //print_error("Setting pin $pin to mode $MODE_NAMES[$io_mode] $GPIO_NAMES[$gpio_mode], state: $STATE_NAMES[$initial_state]");

  if (mode == PIN_MODE_PWM) {
    this->set_bit(regs_piocon[io_pin.reg_iopwm], io_pin.bit_iopwm);
    if (io_pin.pwm_module == 0) {                   // Only module 0's outputs can be inverted
      this->change_bit(REG_PNP, io_pin.bit_iopwm, invert);
    }
    this->set_bit(REG_PWMCON0, 7);                           // Set PWMRUN bit
  } else {
    if (mode == PIN_MODE_ADC) {
      this->enable_adc();
    } else {
      if (array_contains(io_pin.type, 3, PIN_MODE_PWM)) {
        this->clr_bit(regs_piocon[io_pin.reg_iopwm], io_pin.bit_iopwm);
      }
    }
  }

  uint8_t pm1 = this->read_i2c_byte(regs_m1[io_pin.port]);
  uint8_t pm2 = this->read_i2c_byte(regs_m2[io_pin.port]);

  // Clear the pm1 and pm2 bits
  pm1 = pm1 & (255 - (1 << io_pin.pin));
  pm2 = pm2 & (255 - (1 << io_pin.pin));

  // Set the new pm1 and pm2 bits according to our gpio_mode
  pm1 = pm1 | ((gpio_mode >> 1) << io_pin.pin);
  pm2 = pm2 | ((gpio_mode & 0b1) << io_pin.pin);

  this->write_i2c_byte(regs_m1[io_pin.port], pm1);
  this->write_i2c_byte(regs_m2[io_pin.port], pm2);

  // Set up Schmitt trigger mode on inputs
  uint8_t tmp[2] = {PIN_MODE_PU, PIN_MODE_IN};
  if (array_contains(tmp, 2, mode)) {
    this->change_bit(regs_ps[io_pin.port], io_pin.pin, schmitt_trigger);
  }

  // If pin is a basic output, invert its initial state
  if ((mode == PIN_MODE_PP) and invert) {
    initial_state = ! initial_state;
    pins[pin].inv_output = 1;
  } else {
    pins[pin].inv_output = 0;
  }

  // 5th bit of mode encodes default output pin state
  this->write_i2c_byte(regs_p[io_pin.port], (initial_state << 3) | io_pin.pin);

}

uint8_t MyKnobComponent::pwm_loading (uint8_t pwm_module) {
  return this->get_bit(REG_PWMCON0, 6);
}

// Load new period and duty registers into buffer
void MyKnobComponent::pwm_load (uint8_t pwm_module, uint8_t wait_for_load) {
  //print_error("pwm load module $pwm_module");
  time_t t_start = time(nullptr);
  this->set_bit(REG_PWMCON0, 6);  // Set the "LOAD" bit of PWMCON0
  if (wait_for_load) {
    while (this->pwm_loading(pwm_module)) {
      delayms(10);                            // delay for 10ms, Wait for "LOAD" to complete
      if (difftime(time(nullptr), t_start) >= 1) {
        ESP_LOGE(TAG, "Timed out waiting for PWM load!");
        return;
      }
    }
  }
}

// Write an IO pin state or PWM duty cycle.
//    :param value: Either True/False for OUT, or a number between 0 and PWM period for PWM.
void MyKnobComponent::output(uint8_t pin, uint16_t value, uint8_t load = 1, uint8_t wait_for_load = 1) {
  pin_struct io_pin = pins[pin];
  //print_hash_params($io_pin);

  if (io_pin.mode == PIN_MODE_PWM) {
//    ESP_LOGI(TAG, "Outputting PWM %d to pin: %d", value, pin);
    uint8_t pwml = regs_pwml[io_pin.pwm_ch];
    uint8_t pwmh = regs_pwmh[io_pin.pwm_ch];
    this->write_i2c_byte(pwml, value & 0xFF);
    this->write_i2c_byte(pwmh, value >> 8);
    if (load != 0) {
      this->pwm_load(io_pin.pwm_module, wait_for_load);
    }
  } else {
    if (value == 0) {
//      ESP_LOGI(TAG, "Outputting LOW to pin: %d (or HIGH if inverted)", pin);
      this->change_bit(regs_p[io_pin.port], io_pin.pin, io_pin.inv_output);
    } else {
      if (value == 1) {
//        ESP_LOGI(TAG, "Outputting HIGH to pin: %d (or LOW if inverted)", pin);
        this->change_bit(regs_p[io_pin.port], io_pin.pin, ! io_pin.inv_output);
      }
    }
  }
}

// Clear the rotary encoder count value on a channel to 0."""
void MyKnobComponent::clear_rotary_encoder() {
  // Reset internal encoder count to zero
  this->write_i2c_byte(REG_ENC_1_COUNT, 0);
  encoder_last = 0;
  encoder_offset = 0;
}

void MyKnobComponent::setup_rotary_encoder(uint8_t pin_a, uint8_t pin_b, uint8_t pin_c) {
  pin_struct pin_ref= pins[pin_a];
  uint8_t enc_channel_a = pin_ref.enc_ch;
  pin_ref = pins[pin_b];
  uint8_t enc_channel_b = pin_ref.enc_ch;

  this->set_mode(pin_a, PIN_MODE_PU, 1);
  this->set_mode(pin_b, PIN_MODE_PU, 1);
  this->set_mode(pin_c, PIN_MODE_OD);
  this->output(pin_c, 0);
  this->write_i2c_byte(REG_ENC_1_CFG, enc_channel_a | (enc_channel_b << 4));
  this->change_bit(REG_ENC_EN, 1, 0);
  this->set_bit(REG_ENC_EN, 0);
  // Reset internal encoder count to zero
  this->clear_rotary_encoder();
}

// Read the step count from a rotary encoder."""
int MyKnobComponent::read_rotary_encoder() {
  int value = this->read_i2c_byte(REG_ENC_1_COUNT);

  if (value & 0b10000000) {
    value -= 256;
  }

  if ((encoder_last > 64) and (value < -64)) {
    encoder_offset += 256;
  }
  if ((encoder_last < -64) and (value > 64)) {
    encoder_offset -= 256;
  }

  encoder_last = value;

  int ret = encoder_offset + value;
  //#print("Read rotary encoder value = $ret\n");
  return ret;
}

void MyKnobComponent::reset_knob() {
  time_t t_start = time(nullptr);
  this->set_bits(REG_CTRL, MASK_CTRL_RESET);
  // Wait for a register to read its initialised value
  //print_error("reg = " . read_i2c_byte($REGS::REG_USER_FLASH));
  while (this->read_i2c_byte(REG_USER_FLASH) != 0x78) {
    delayms(10);
    if (difftime(time(nullptr), t_start) >= 1) {
      ESP_LOGE(TAG, "Timed out waiting for Reset!");
      //print_error("reg = " . this->read_i2c_byte($REGS::REG_USER_FLASH));
      return;
    }
  }
}

// Set the PWM period.
// The period is the point at which the PWM counter is reset to zero.
// The PWM clock runs at FSYS with a divider of 1/1.
// Also specifies the maximum value that can be set in the PWM duty cycle.
void MyKnobComponent::set_pwm_period(uint16_t value) {
  //print_error("set pwm period $value for $pwm_module");
  this->write_i2c_byte(REG_PWMPL, value & 0xFF);
  this->write_i2c_byte(REG_PWMPH, value >> 8);
  this->pwm_load(0, 1);
}

// Set PWM settings.
// PWM is driven by the 24MHz FSYS clock by default.
// :param divider: Clock divider, one of 1, 2, 4, 8, 16, 32, 64 or 128
void MyKnobComponent::set_pwm_control(uint8_t divider) {
  if (! isPowerOfTwo(divider)) {
    ESP_LOGE(TAG, "A clock divider of %d is not power of 2", divider);
    return;
  }
  uint8_t pwmdiv2 = whichPowerOfTwo(divider);
//  ESP_LOGI(TAG, "Power of 2: %d --> %d", divider, pwmdiv2);
  this->write_i2c_byte(REG_PWMCON1, pwmdiv2);
}

// Set the colour of the knob
void MyKnobComponent::set_knob_colour(float r, float g, float b) {
  //#print("set colour $r/$g/$b\n");
  this->output(LED_RED, uint16_t(r * 500));
  this->output(LED_GREEN, uint16_t(g * 500));
  this->output(LED_BLUE, uint16_t(b * 500));
}

// Set the colour of the knob
void MyKnobComponent::set_knob_hue(uint8_t vol) {
//  ESP_LOGI(TAG, "Setting knob hue to %d", vol);
  float h = 280 * vol / 100 + 80;
  RGB rgb = hsvToRgb(h);
//  ESP_LOGI(TAG, "new knob colours: %2x / %2x / %2x", rgb.r, rgb.b, rgb.g);
  this->output(LED_RED, rgb.r);
  this->output(LED_GREEN, rgb.g);
  this->output(LED_BLUE, rgb.b);
//  ESP_LOGI(TAG, "Done setting hue");
}

light::LightTraits MyKnobComponent::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::RGB});
  return traits;
}

void MyKnobComponent::write_state(light::LightState *state) {
  // This will be called by the light to get a new state to be written.
  this->light_state = state;
  float red, green, blue;
  // use any of the provided current_values methods
  this->light_state->current_values_as_rgb(&red, &green, &blue);

//  ESP_LOGI(TAG, "rgb %f %f %f", red, green, blue);
  this->set_knob_colour(red, green, blue);
}


// Never seems to be called so put the call to it in update()
void MyKnobComponent::setup() {
  setup_complete = 1;
//  ESP_LOGI(TAG, "Setting up RGB Rotary Encoder...");

  uint8_t data = 0;
  if (this->read_register(REG_USER_FLASH, &data, 1) !=
      i2c::ERROR_OK) {
    ESP_LOGE(TAG, "RGB Rotary Encoder Setup Failed");
    this->mark_failed();
    return;
  }
  reset_knob();
  // Set up the interrupt pin on the Pi, and enable the chip's output (???)
  this->set_bit(REG_INT, BIT_INT_OUT_EN);
  this->change_bit(REG_INT, BIT_INT_PIN_SWAP, 1);

  this->setup_rotary_encoder(KNOB_A, KNOB_B, KNOB_C);
  this->set_pwm_period(510);
  this->set_pwm_control(2);
  //print("set mode for red\n");
  this->set_mode(LED_RED, PIN_MODE_PWM, 0, 1);
  //print("set mode for green\n");
  this->set_mode(LED_GREEN, PIN_MODE_PWM, 0, 1);
  //print("set mode for blue\n");
  this->set_mode(LED_BLUE, PIN_MODE_PWM, 0, 1);
//  ESP_LOGI(TAG, "Done with most setup, now read knob and set hue");
  int knob = this->read_rotary_encoder();
//  ESP_LOGI(TAG, "RGB Rotary Encoder value: %x", knob);
  //setEncoderValue(0);
//  ESP_LOGI(TAG, "RGB Rotary Encoder data: %x (should be 78)", data);
  this->set_knob_hue(50);
}

void MyKnobComponent::update(){
  if (setup_complete == 0) {
    setup();
  }
}

void MyKnobComponent::dump_config(){

}

void MyKnobComponent::loop()  {
  // This will be called very often after setup time.
  // think of it as the loop() call in Arduino
  int knob_value = this->read_rotary_encoder();
  if ((this->knob_sensor != nullptr) && (!this->knob_sensor->has_state() || (this->knob_sensor->state != knob_value))) {
    ESP_LOGD(TAG, "Knob value: %d ", knob_value);
    this->knob_sensor->publish_state(knob_value);
  }
}

}
}

