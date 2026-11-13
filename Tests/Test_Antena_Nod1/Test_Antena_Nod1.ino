/*
 * TEST ANTENA 250kbps (FARA AUTO-ACK) - NOD 1 (Arduino Nano = TX)
 * Pini: CE=D7, CSN=D8, SCK=D13, MOSI=D11, MISO=D12 | Baud: 9600
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

static const uint8_t RF_ADDR[5] = {'0','0','0','0','1'};

static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}
static void delay_ms(uint16_t n) { while (n--) delay_us(1000); }

static void uart_init() {
  R_UBRR0 = 103; // 9600 baud @ 16MHz
  R_UCSR0B = (1 << 3);
  R_UCSR0C = (1 << 2) | (1 << 1);
}
static void uart_char(char c) { while (!(R_UCSR0A & (1 << 5))); R_UDR0 = c; }
static void uart_str(const char *s) { while (*s) uart_char(*s++); }
static void uart_ln(const char *s) { uart_str(s); uart_str("\r\n"); }
static void uart_hex(uint8_t v) {
  const char *hx = "0123456789ABCDEF";
  uart_char(hx[v >> 4]); uart_char(hx[v & 0x0F]);
}
static void uart_dec(uint16_t v) {
  char t[5]; uint8_t n = 0;
  do { t[n++] = '0' + (v % 10); v /= 10; } while (v);
  while (n) uart_char(t[--n]);
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

static void nrf_init_tx() {
  R_DDRD |= (1 << P_NRF_CE);
  NRF_CE_L(); NRF_CSN_H(); delay_ms(100);

  nrf_wreg(0x00, 0x00); // Reset temporar
  delay_ms(5);
  nrf_wreg(0x01, 0x00); // EN_AA: Auto-ACK DEZACTIVAT
  nrf_wreg(0x02, 0x01); // EN_RXADDR: Pipe 0 activ
  nrf_wreg(0x03, 0x03); // SETUP_AW: Adresa 5 octeti
  nrf_wreg(0x04, 0x00); // SETUP_RETR: Retrimiteri DEZACTIVATE
  nrf_wreg(0x05, 108);  // Canal 108 (2508 MHz - liber de Wi-Fi)
  nrf_wreg(0x06, 0x27); // 250 kbps + putere maxima +7 dBm / LNA gain
  nrf_wreg_buf(0x10, RF_ADDR, 5); // TX_ADDR: "00001"
  nrf_wreg_buf(0x0A, RF_ADDR, 5); // RX_ADDR_P0: "00001"
  nrf_wreg(0x11, 32);   // Payload 32 octeti

  nrf_cmd(0xE1); // FLUSH_TX
  nrf_cmd(0xE2); // FLUSH_RX
  nrf_wreg(0x07, 0x70); // Clear flags
  nrf_wreg(0x00, 0x0E); // PWR_UP=1, PRIM_RX=0 (TX), CRC16
  delay_ms(5);
}

static uint8_t last_status = 0;

static bool nrf_send_packet(const char *msg) {
  uint8_t buf[32];
  for (uint8_t i = 0; i < 32; i++) buf[i] = 0;
  for (uint8_t i = 0; i < 31 && msg[i]; i++) buf[i] = (uint8_t)msg[i];

  nrf_cmd(0xE1);        // Curata bufferul TX
  nrf_wreg(0x07, 0x70); // Curata flag-urile vechi

  NRF_CSN_L();
  spi_xfer(0xA0);       // W_TX_PAYLOAD
  for (uint8_t i = 0; i < 32; i++) spi_xfer(buf[i]);
  NRF_CSN_H();

  NRF_CE_H();
  delay_us(20);
  NRF_CE_L();

  uint8_t st = 0;
  for (uint16_t t = 0; t < 10000; t++) {
    st = nrf_cmd(0xFF);
    if (st & 0x30) break;
    delay_us(10);
  }
  last_status = st;
  nrf_wreg(0x07, 0x70);

  if (st & 0x20) return true;
  nrf_cmd(0xE1);
  return false;
}

static uint16_t cnt = 0;

void setup() {
  uart_init();
  spi_init();
  delay_ms(300);

  uart_ln("\r\n=== TEST ANTENA TX 250kbps (NANO) ===");
  nrf_init_tx();

  uint8_t cfg = nrf_rreg(0x00);
  uint8_t rf  = nrf_rreg(0x06);
  uart_str("CONFIG=0x"); uart_hex(cfg);
  uart_str(" RF_SETUP=0x"); uart_hex(rf);
  if (cfg == 0x0E && rf == 0x22) uart_ln(" -> SPI & 250kbps OK");
  else uart_ln(" -> EROARE SPI (verifica firele D8, D11, D12, D13)");
}

void loop() {
  cnt++;
  char msg[32] = "PING #";
  char t[5]; uint8_t n = 0; uint16_t v = cnt;
  do { t[n++] = '0' + (v % 10); v /= 10; } while (v);
  uint8_t k = 6;
  while (n) msg[k++] = t[--n];
  msg[k] = 0;

  bool ok = nrf_send_packet(msg);
  uart_str("Pachet: "); uart_str(msg);
  if (ok) {
    uart_ln("  --> [TRIMIS IN AER]");
  } else {
    uart_str("  --> [EROARE EMISIE] STATUS=0x"); uart_hex(last_status); uart_ln("");
  }

  delay_ms(1000);
}