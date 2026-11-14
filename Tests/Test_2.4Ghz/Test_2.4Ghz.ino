/*
 * SCANNER 2.4 GHz (RPD) - Testeaza fizic etajul de receptie al antenei nRF24L01+
 * Compatibil cu: Arduino Uno si Arduino Nano
 * Pini: CE=D7, CSN=D8, SCK=D13, MOSI=D11, MISO=D12 | Serial Monitor: 9600 baud
 */
#include <stdint.h>

#define REG8(a)   (*(volatile uint8_t  *)(a))
#define REG16(a)  (*(volatile uint16_t *)(a))
#define R_PORTB   REG8(0x25)
#define R_DDRB    REG8(0x24)
#define R_PORTD   REG8(0x2B)
#define R_DDRD    REG8(0x2A)
#define R_SPCR    REG8(0x4C)
#define R_SPSR    REG8(0x4D)
#define R_SPDR    REG8(0x4E)
#define R_UCSR0A  REG8(0xC0)
#define R_UCSR0B  REG8(0xC1)
#define R_UCSR0C  REG8(0xC2)
#define R_UBRR0   REG16(0xC4)
#define R_UDR0    REG8(0xC6)

#define P_NRF_CSN 0  // D8 (PB0)
#define P_SS_HW   2  // D10 (PB2 - tinut OUTPUT HIGH pentru SPI Master)
#define P_MOSI    3  // D11 (PB3)
#define P_MISO    4  // D12 (PB4)
#define P_SCK     5  // D13 (PB5)
#define P_NRF_CE  7  // D7 (PD7)

#define NRF_CSN_L() (R_PORTB &= ~(1 << P_NRF_CSN))
#define NRF_CSN_H() (R_PORTB |=  (1 << P_NRF_CSN))
#define NRF_CE_L()  (R_PORTD &= ~(1 << P_NRF_CE))
#define NRF_CE_H()  (R_PORTD |=  (1 << P_NRF_CE))

static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}
static void delay_ms(uint16_t n) { while (n--) delay_us(1000); }

static void uart_init() {
  R_UBRR0 = 103; // 9600 baud @ 16 MHz
  R_UCSR0B = (1 << 3);
  R_UCSR0C = (1 << 2) | (1 << 1);
}
static void uart_char(char c) { while (!(R_UCSR0A & (1 << 5))); R_UDR0 = c; }
static void uart_str(const char *s) { while (*s) uart_char(*s++); }
static void uart_hex(uint8_t v) {
  const char *hx = "0123456789ABCDEF";
  uart_char(hx[v >> 4]); uart_char(hx[v & 0x0F]);
}
static void uart_dec(uint8_t v) {
  if (v >= 10) uart_char('0' + (v / 10));
  uart_char('0' + (v % 10));
}

static void spi_init() {
  R_DDRB |= (1 << P_NRF_CSN) | (1 << P_SS_HW) | (1 << P_MOSI) | (1 << P_SCK);
  R_DDRB &= ~(1 << P_MISO);
  R_PORTB |= (1 << P_NRF_CSN) | (1 << P_SS_HW);
  R_SPCR = (1 << 6) | (1 << 4) | (1 << 0); // SPE, MSTR, fosc/16 (1 MHz)
}
static uint8_t spi_xfer(uint8_t b) {
  R_SPDR = b;
  while (!(R_SPSR & (1 << 7)));
  return R_SPDR;
}

static void nrf_wreg(uint8_t r, uint8_t v) {
  NRF_CSN_L(); spi_xfer(0x20 | r); spi_xfer(v); NRF_CSN_H();
}
static uint8_t nrf_rreg(uint8_t r) {
  NRF_CSN_L(); spi_xfer(r & 0x1F); uint8_t v = spi_xfer(0xFF); NRF_CSN_H(); return v;
}

void setup() {
  uart_init();
  spi_init();
  R_DDRD |= (1 << P_NRF_CE);
  NRF_CE_L(); NRF_CSN_H();
  delay_ms(100);

  nrf_wreg(0x00, 0x00); // Reset
  delay_ms(5);
  nrf_wreg(0x01, 0x00); // Dezactiveaza Auto-ACK
  nrf_wreg(0x06, 0x06); // 1 Mbps, sensibilitate maxima
  nrf_wreg(0x00, 0x0F); // PWR_UP=1, PRIM_RX=1
  delay_ms(5);

  uint8_t cfg = nrf_rreg(0x00);
  uart_str("\r\n=== SCANNER 2.4 GHz (TEST HARDWARE ANTENA) ===\r\n");
  uart_str("CONFIG=0x"); uart_hex(cfg);
  if (cfg == 0x0F) uart_str(" -> SPI OK. Apropie un telefon cu Wi-Fi/Bluetooth...\r\n\r\n");
  else uart_str(" -> EROARE SPI (verifica firele D8, D11, D12, D13)\r\n\r\n");
}

void loop() {
  uint8_t hits = 0;

  // Scaneaza primele 85 de canale (unde emit routerele Wi-Fi si telefoanele)
  for (uint8_t ch = 1; ch <= 85; ch++) {
    nrf_wreg(0x05, ch);
    NRF_CE_H();
    delay_us(220); // Timp necesar cipului sa masoare semnalul pe canalul curent
    if (nrf_rreg(0x09) & 0x01) hits++; // Registrul 0x09 = RPD (Received Power Detector)
    NRF_CE_L();
  }

  uart_str("Canale cu semnal detectat: ");
  uart_dec(hits);
  if (hits > 0) {
    uart_str("  --> [ANTENA RECEPTIONEAZA OK!]\r\n");
  } else {
    uart_str("  --> [0] (apropie telefonul cu hotspot/Wi-Fi sau verifica firul CE=D7)\r\n");
  }

  delay_ms(500);
}