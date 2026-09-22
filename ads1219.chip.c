#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
  pin_t pin_ain[4];
  uint8_t config_reg;
  uint8_t current_cmd;
  uint8_t rx_byte_count;
  uint8_t tx_byte_count;
  uint8_t tx_buf[3];
} chip_state_t;

static bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  return true; // 預設響應 I2C 位址 0x40
}

static uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  if (chip->current_cmd == 0x10) { // RDATA 指令
    if (chip->tx_byte_count < 3) {
      return chip->tx_buf[chip->tx_byte_count++];
    }
  } else if ((chip->current_cmd & 0xF0) == 0x20) { // RREG 讀取暫存器
    if (chip->tx_byte_count == 0) {
      chip->tx_byte_count++;
      return chip->config_reg;
    }
  }
  return 0x00;
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t*)user_data;

  if (chip->rx_byte_count == 0) {
    chip->current_cmd = data;
    chip->rx_byte_count = 1;

    if (data == 0x06) { // RESET
      chip->config_reg = 0x00;
    } else if (data == 0x08) { // START/SYNC (觸發 ADC 轉換)
      uint8_t mux = (chip->config_reg >> 5) & 0x07;
      float voltage = 0.0f;

      // 依據 Config 暫存器的 MUX 切換讀取對應 AIN 引腳電壓
      if (mux == 0x03) voltage = pin_adc_read(chip->pin_ain[0]);      // AIN0 - AVSS
      else if (mux == 0x04) voltage = pin_adc_read(chip->pin_ain[1]); // AIN1 - AVSS
      else if (mux == 0x05) voltage = pin_adc_read(chip->pin_ain[2]); // AIN2 - AVSS
      else if (mux == 0x06) voltage = pin_adc_read(chip->pin_ain[3]); // AIN3 - AVSS
      else voltage = pin_adc_read(chip->pin_ain[0]);                  // 預設 AIN0

      // ADS1219 24-bit 滿刻度 (2.048V VREF, Gain=1)
      int32_t code = (int32_t)((voltage / 2.048f) * 8388607.0f);
      if (code > 8388607) code = 8388607;
      if (code < -8388608) code = -8388608;

      uint32_t raw = (uint32_t)code & 0x00FFFFFF;
      chip->tx_buf[0] = (raw >> 16) & 0xFF; // MSB
      chip->tx_buf[1] = (raw >> 8) & 0xFF;
      chip->tx_buf[2] = raw & 0xFF;         // LSB
    } else if (data == 0x10) { // RDATA
      chip->tx_byte_count = 0;
    }
  } else {
    // 寫入 Config 暫存器 (WREG 0x40)
    if ((chip->current_cmd & 0xF0) == 0x40) {
      chip->config_reg = data;
    }
  }
  return true;
}

static void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  chip->rx_byte_count = 0;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->pin_ain[0] = pin_init("AIN0", ANALOG);
  chip->pin_ain[1] = pin_init("AIN1", ANALOG);
  chip->pin_ain[2] = pin_init("AIN2", ANALOG);
  chip->pin_ain[3] = pin_init("AIN3", ANALOG);

  chip->config_reg = 0x00;
  chip->rx_byte_count = 0;
  chip->tx_byte_count = 0;

  const i2c_config_t i2c_config = {
    .user_data = chip,
    .address = 0x40, // ADS1219 預設 I2C 地址
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
    .disconnect = on_i2c_disconnect,
  };
  i2c_init(&i2c_config);
}