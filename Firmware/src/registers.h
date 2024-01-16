// #include "Arduino.h"
// #include "SPI.h"

// uint8_t GCONF_REG_ADDR = 0x00;
// uint8_t GSTAT_REG_ADDR = 0x01;
// uint8_t IOIN_REG_ADDR = 0x04;
// uint8_t OTP_PROG_REG_ADDR = 0x04;
// uint8_t OTP_READ_REG_ADDR = 0x04;
// uint8_t FACTORY_CONF_REG_ADDR = 0x04;
// uint8_t SHORT_CONF_REG_ADDR = 0x04;
// uint8_t DRV_CONF_REG_ADDR = 0x04;

// #define SPI_MISO_PIN 18
// #define SPI_MOSI_PIN 8
// #define SPI_CLK_PIN 3

// #define SPI_MUX_A0 5
// #define SPI_MUX_A1 6
// #define SPI_MUX_A2 7

// void enable_spi_mux(uint8_t device){
//     digitalWrite(SPI_MUX_A0, device & 0x01);
//     digitalWrite(SPI_MUX_A1, device & 0x02);
//     digitalWrite(SPI_MUX_A2, device & 0x04);
// }

// void setup_spi(){
//     // pinMode(SPI_MISO_PIN, INPUT);
//     // pinMode(SPI_MOSI_PIN, OUTPUT);
//     // pinMode(SPI_CLK_PIN, OUTPUT);

//     // pinMode(SPI_MUX_A0, OUTPUT);
//     // pinMode(SPI_MUX_A1, OUTPUT);
//     // pinMode(SPI_MUX_A2, OUTPUT);

//     // enable_spi_mux(0);

//     // SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, -1);
//     // SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE3);
//     // SPI.beginTransaction(settings);

//     // SPI.transfer(GCONF_REG_ADDR & 0x7F);

//     // volatile uint32_t data = 0x00000000;
//     // data |= SPI.transfer(0x00) << 24;
//     // data |= SPI.transfer(0x00) << 16;
//     // data |= SPI.transfer(0x00) << 8;
//     // data |= SPI.transfer(0x00);
//     // SPI.endTransaction();

//     // enable_spi_mux(1);

//     // return;
// }

