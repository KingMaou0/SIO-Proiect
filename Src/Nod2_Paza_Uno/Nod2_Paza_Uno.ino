/*
 * NOD 2 (Arduino Uno) - Receptor Radio + Ecoul ACK + Alarma Buzzer
 * Pini: BUZ=D5, NRF_CE=D7, NRF_CSN=D8, MOSI=D11, MISO=D12, SCK=D13 | Baud: 9600
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

#define P_NRF_CSN 0   // PB0 = D8
#define P_SS_HW   2   // PB2 = D10 (OUTPUT HIGH pentru SPI Master)
#define P_MOSI    3   // PB3 = D11
#define P_MISO    4   // PB4 = D12
#define P_SCK     5   // PB5 = D13
#define P_BUZ     5   // PD5 = D5
#define P_NRF_CE  7   // PD7 = D7

static const uint8_t RF_ADDR[5] = {'0','0','0','0','1'};

#define NRF_CSN_L()  (R_PORTB &= ~(1 << P_NRF_CSN))
#define NRF_CSN_H()  (R_PORTB |=  (1 << P_NRF_CSN))
#define NRF_CE_L()   (R_PORTD &= ~(1 << P_NRF_CE))
#define NRF_CE_H()   (R_PORTD |=  (1 << P_NRF_CE))
#define BUZ_ON()     (R_PORTD |=  (1 << P_BUZ))
#define BUZ_OFF()    (R_PORTD &= ~(1 << P_BUZ))

/* ---------- Utilitare ---------- */
static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}
static void delay_ms(uint16_t n) { while (n--) delay_us(1000); }

static bool str_starts_with(const char *str, const char *prefix) {
  while (*prefix) {
    if (*str++ != *prefix++) return false;
  }
  return true;
}

static bool str_eq(const char *a, const char *b) {
  while (*a && *b) {
    if (*a++ != *b++) return false;
  }
  return (*a == *b);
}

/* ---------- UART0 (Serial 9600 baud) ---------- */
static void uart_init() {
  R_UBRR0 = 103;
  R_UCSR0B = (1 << 3);
  R_UCSR0C = (1 << 2) | (1 << 1);
}
static void uart_char(char c) { while (!(R_UCSR0A & (1 << 5))); R_UDR0 = c; }
static void uart_str(const char *s) { while (*s) uart_char(*s++); }
static void uart_ln(const char *s) { uart_str(s); uart_str("\r\n"); }
static void uart_dec(uint16_t v) {
  char t[5]; uint8_t n = 0;
  do { t[n++] = '0' + (v % 10); v /= 10; } while (v);
  while (n) uart_char(t[--n]);
}

/* ---------- SPI ---------- */
static void spi_init() {
  R_DDRB |= (1 << P_NRF_CSN) | (1 << P_SS_HW) | (1 << P_MOSI) | (1 << P_SCK);
  R_DDRB &= ~(1 << P_MISO);
  R_PORTB |= (1 << P_NRF_CSN) | (1 << P_SS_HW);
  R_SPCR = (1 << 6) | (1 << 4) | (1 << 0);
}
static uint8_t spi_xfer(uint8_t b) {
  R_SPDR = b;
  while (!(R_SPSR & (1 << 7)));
  return R_SPDR;
}

/* ---------- Buzzer ---------- */
static void beep(uint16_t ms) { BUZ_ON(); delay_ms(ms); BUZ_OFF(); }

static void sirena_intrus() {
  for (uint8_t i = 0; i < 10; i++) {
    BUZ_ON();  delay_ms(150);
    BUZ_OFF(); delay_ms(150);
  }
}

/* ---------- nRF24L01+ ---------- */
static void nrf_wreg(uint8_t reg, uint8_t val) {
  NRF_CSN_L(); spi_xfer(0x20 | reg); spi_xfer(val); NRF_CSN_H();
}
static void nrf_wreg_buf(uint8_t reg, const uint8_t *b, uint8_t n) {
  NRF_CSN_L(); spi_xfer(0x20 | reg);
  for (uint8_t i = 0; i < n; i++) spi_xfer(b[i]);
  NRF_CSN_H();
}
static uint8_t nrf_rreg(uint8_t reg) {
  NRF_CSN_L(); spi_xfer(reg & 0x1F); uint8_t v = spi_xfer(0xFF); NRF_CSN_H(); return v;
}
static uint8_t nrf_cmd(uint8_t c) {
  NRF_CSN_L(); uint8_t s = spi_xfer(c); NRF_CSN_H(); return s;
}

static void nrf_init_rx() {
  R_DDRD |= (1 << P_NRF_CE);
  NRF_CE_L(); NRF_CSN_H(); delay_ms(100);

  nrf_wreg(0x00, 0x00);
  delay_ms(5);
  nrf_wreg(0x01, 0x00); // Fara Auto-ACK hardware
  nrf_wreg(0x02, 0x01); // Pipe 0 activ
  nrf_wreg(0x03, 0x03); // Adresa 5 octeti
  nrf_wreg(0x04, 0x00); // Fara retrimiteri hardware
  nrf_wreg(0x05, 76);   // Canal 76
  nrf_wreg(0x06, 0x26); // 250 kbps + 0 dBm
  nrf_wreg_buf(0x10, RF_ADDR, 5); // TX_ADDR (pentru trimiterea ecoului ACK)
  nrf_wreg_buf(0x0A, RF_ADDR, 5); // RX_ADDR_P0
  nrf_wreg(0x11, 32);   // Payload 32 octeti

  nrf_cmd(0xE1);
  nrf_cmd(0xE2);
  nrf_wreg(0x07, 0x70);
  nrf_wreg(0x00, 0x0F); // RX mode
  delay_ms(5);

  NRF_CE_H(); // Ascultare activa
}

// Trimite inapoi catre Nano mesajul primit ca ACK software
static void nrf_send_ack_echo(const uint8_t *raw_payload) {
  // Lasa 3 ms pentru ca Nano sa comute pe receptie
  delay_ms(3);

  NRF_CE_L();
  nrf_wreg(0x00, 0x0E); // Comuta temporar pe TX
  delay_us(150);

  nrf_cmd(0xE1);
  nrf_wreg(0x07, 0x70);

  NRF_CSN_L();
  spi_xfer(0xA0);
  for (uint8_t i = 0; i < 32; i++) spi_xfer(raw_payload[i]);
  NRF_CSN_H();

  NRF_CE_H(); delay_us(20); NRF_CE_L();

  for (uint16_t t = 0; t < 5000; t++) {
    if (nrf_cmd(0xFF) & 0x20) break;
    delay_us(10);
  }
  nrf_wreg(0x07, 0x70);

  // Revine imediat pe receptie continua
  nrf_cmd(0xE2);
  nrf_wreg(0x00, 0x0F);
  NRF_CE_H();
}

static uint16_t rx_count = 0;
static bool connected_announced = false;

void setup() {
  R_DDRD |= (1 << P_BUZ);
  BUZ_OFF();
  uart_init();
  spi_init();

  delay_ms(400);
  uart_ln("\r\n=== NOD 2 (RECEPTOR UNO) ===");
  beep(120);

  nrf_init_rx();
  uint8_t cfg = nrf_rreg(0x00);
  if (cfg == 0x0F) {
    uart_ln("[OK] Receptor activ pe canal 76 (250 kbps). Astept Nod 1...");
  } else {
    uart_ln("[EROARE] Verifica firele SPI ale antenei!");
  }
}

void loop() {
  uint8_t st = nrf_cmd(0xFF);
  uint8_t fifo = nrf_rreg(0x17);

  if ((st & 0x40) || !(fifo & 0x01)) {
    uint8_t raw[32];
    char msg[33];

    NRF_CSN_L();
    spi_xfer(0x61);
    for (uint8_t i = 0; i < 32; i++) {
      raw[i] = spi_xfer(0xFF);
      msg[i] = (char)raw[i];
    }
    NRF_CSN_H();
    msg[32] = 0;
    nrf_wreg(0x07, 0x40);

    // 1. Trimite imediat pachetul inapoi ca ACK catre Nano
    nrf_send_ack_echo(raw);

    // 2. Proceseaza mesajul primit
    if (str_eq(msg, "Alex te bat cu LCD-ul paralel!")) {
      if (!connected_announced) {
        connected_announced = true;
        uart_ln("[CONECTAT] Handshake confirmat cu Nod 1!\r\n");
        beep(80);
      }
      return;
    }

    rx_count++;
    uart_str("RX #"); uart_dec(rx_count);
    uart_str(" <- "); uart_ln(msg);

    if (str_starts_with(msg, "ALERTA INTRUS")) {
      uart_ln("    [!] INTRUS DETECTAT -> Pornesc alarma (3 s)...");
      sirena_intrus();
    } else if (str_starts_with(msg, "ACCES PERMIS")) {
      uart_ln("    [OK] SAFE / ACCES PERMIS -> Bip confirmare.");
      beep(120);
    }
  }
}