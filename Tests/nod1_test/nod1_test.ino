/*
 * NOD 1 (Arduino Nano) - Terminal Acces RFID + Servo + LED-uri + Radio cu ACK Software
 * Pini: LED_VERDE=D2, SERVO=D3, LED_ROSU=D4, BUZ=D5, NRF_CE=D7, NRF_CSN=D8,
 *       RC_RST=D9, RC_SS=D10, MOSI=D11, MISO=D12, SCK=D13 | Baud: 9600
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
#define R_SREG    REG8(0x5F)
#define R_TIMSK0  REG8(0x6E)
#define R_TIMSK1  REG8(0x6F)
#define R_TCCR1A  REG8(0x80)
#define R_TCCR1B  REG8(0x81)
#define R_OCR1A   REG16(0x88)
#define R_OCR1B   REG16(0x8A)
#define R_UCSR0A  REG8(0xC0)
#define R_UCSR0B  REG8(0xC1)
#define R_UCSR0C  REG8(0xC2)
#define R_UBRR0   REG16(0xC4)
#define R_UDR0    REG8(0xC6)

#define P_NRF_CSN 0   // PB0 = D8
#define P_RC_RST  1   // PB1 = D9
#define P_RC_SS   2   // PB2 = D10
#define P_MOSI    3   // PB3 = D11
#define P_MISO    4   // PB4 = D12
#define P_SCK     5   // PB5 = D13

#define P_LED_V   2   // PD2 = D2 (LED Verde - SAFE)
#define P_SERVO   3   // PD3 = D3
#define P_LED_R   4   // PD4 = D4 (LED Rosu - INTRUS)
#define P_BUZ     5   // PD5 = D5
#define P_NRF_CE  7   // PD7 = D7

#define CLI() __asm__ __volatile__("cli" ::: "memory")
#define SEI() __asm__ __volatile__("sei" ::: "memory")

#define SERVO_CLOSED  1088
#define SERVO_OPEN    2944

static const uint8_t RF_ADDR[5] = {'0','0','0','0','1'};
// UID-ul cardului tau autorizat (76 4B 2E 03)
static const uint8_t VALID_UID[4] = {0x76, 0x4B, 0x2E, 0x03};

#define NRF_CSN_L()  (R_PORTB &= ~(1 << P_NRF_CSN))
#define NRF_CSN_H()  (R_PORTB |=  (1 << P_NRF_CSN))
#define RC_RST_L()   (R_PORTB &= ~(1 << P_RC_RST))
#define RC_RST_H()   (R_PORTB |=  (1 << P_RC_RST))
#define RC_SS_L()    (R_PORTB &= ~(1 << P_RC_SS))
#define RC_SS_H()    (R_PORTB |=  (1 << P_RC_SS))
#define NRF_CE_L()   (R_PORTD &= ~(1 << P_NRF_CE))
#define NRF_CE_H()   (R_PORTD |=  (1 << P_NRF_CE))
#define BUZ_ON()     (R_PORTD |=  (1 << P_BUZ))
#define BUZ_OFF()    (R_PORTD &= ~(1 << P_BUZ))
#define LED_V_ON()   (R_PORTD |=  (1 << P_LED_V))
#define LED_V_OFF()  (R_PORTD &= ~(1 << P_LED_V))
#define LED_R_ON()   (R_PORTD |=  (1 << P_LED_R))
#define LED_R_OFF()  (R_PORTD &= ~(1 << P_LED_R))

/* ---------- Utilitare ---------- */
static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}
static void delay_ms(uint16_t n) { while (n--) delay_us(1000); }
static void mem_zero(uint8_t *p, uint8_t n) { while (n--) *p++ = 0; }

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
static void uart_hex(uint8_t v) {
  const char *hx = "0123456789ABCDEF";
  uart_char(hx[v >> 4]); uart_char(hx[v & 0x0F]);
}
static void uart_ln(const char *s) { uart_str(s); uart_str("\r\n"); }
static void uart_dec(uint16_t v) {
  char t[5]; uint8_t n = 0;
  do { t[n++] = '0' + (v % 10); v /= 10; } while (v);
  while (n) uart_char(t[--n]);
}

/* ---------- SPI ---------- */
static void spi_init() {
  R_DDRB |= (1 << P_NRF_CSN) | (1 << P_RC_RST) | (1 << P_RC_SS) | (1 << P_MOSI) | (1 << P_SCK);
  R_DDRB &= ~(1 << P_MISO);
  R_PORTB |= (1 << P_NRF_CSN) | (1 << P_RC_SS);
  R_SPCR = (1 << 6) | (1 << 4) | (1 << 0); // SPE, MSTR, fosc/16 = 1 MHz
}
static uint8_t spi_xfer(uint8_t b) {
  R_SPDR = b;
  while (!(R_SPSR & (1 << 7)));
  return R_SPDR;
}

/* ---------- Servo (Timer1) ---------- */
static volatile uint16_t servo_ticks = SERVO_CLOSED;
extern "C" void __vector_11(void) __attribute__((signal, used, externally_visible));
extern "C" void __vector_12(void) __attribute__((signal, used, externally_visible));
extern "C" void __vector_11(void) { R_OCR1B = servo_ticks; R_PORTD |= (1 << P_SERVO); }
extern "C" void __vector_12(void) { R_PORTD &= ~(1 << P_SERVO); }

static void servo_init() {
  R_DDRD |= (1 << P_SERVO);
  R_TCCR1A = 0;
  R_TCCR1B = (1 << 3) | (1 << 1); // WGM12, CS11
  R_OCR1A = 39999;
  R_OCR1B = SERVO_CLOSED;
  R_TIMSK1 = (1 << 1) | (1 << 2); // OCIE1A, OCIE1B
}
static void servo_set(uint16_t t) {
  uint8_t s = R_SREG; CLI(); servo_ticks = t; R_SREG = s;
}
static void beep(uint16_t ms) { BUZ_ON(); delay_ms(ms); BUZ_OFF(); }

/* ---------- RC522 ---------- */
#define CommandReg     0x01
#define ComIrqReg      0x04
#define DivIrqReg      0x05
#define ErrorReg       0x06
#define FIFODataReg    0x09
#define FIFOLevelReg   0x0A
#define BitFramingReg  0x0D
#define CollReg        0x0E
#define ModeReg        0x11
#define TxControlReg   0x14
#define TxASKReg       0x15
#define CRCResultRegH  0x21
#define CRCResultRegL  0x22
#define TModeReg       0x2A
#define TPrescalerReg  0x2B
#define TReloadRegH    0x2C
#define TReloadRegL    0x2D
#define VersionReg     0x37

static void rc_write(uint8_t reg, uint8_t val) {
  RC_SS_L(); spi_xfer((reg << 1) & 0x7E); spi_xfer(val); RC_SS_H();
}
static uint8_t rc_read(uint8_t reg) {
  RC_SS_L(); spi_xfer(((reg << 1) & 0x7E) | 0x80);
  uint8_t v = spi_xfer(0); RC_SS_H(); return v;
}
static void rc_setbits(uint8_t r, uint8_t m) { rc_write(r, rc_read(r) | m); }
static void rc_clrbits(uint8_t r, uint8_t m) { rc_write(r, rc_read(r) & ~m); }

static void rc_init() {
  RC_RST_L(); delay_us(10); RC_RST_H(); delay_ms(50);
  rc_write(CommandReg, 0x0F); delay_ms(50);
  rc_write(TModeReg, 0x80); rc_write(TPrescalerReg, 0xA9);
  rc_write(TReloadRegH, 0x03); rc_write(TReloadRegL, 0xE8);
  rc_write(TxASKReg, 0x40); rc_write(ModeReg, 0x3D);
  rc_setbits(TxControlReg, 0x03);
}

static bool rc_transceive(const uint8_t *tx, uint8_t txLen, uint8_t *rx, uint8_t *rxLen, uint8_t txBits) {
  rc_write(CommandReg, 0x00); rc_write(ComIrqReg, 0x7F); rc_write(FIFOLevelReg, 0x80);
  for (uint8_t i = 0; i < txLen; i++) rc_write(FIFODataReg, tx[i]);
  rc_write(BitFramingReg, txBits); rc_write(CommandReg, 0x0C); rc_setbits(BitFramingReg, 0x80);
  uint16_t n = 3000;
  for (;;) {
    uint8_t irq = rc_read(ComIrqReg);
    if (irq & 0x30) break;
    if (irq & 0x01) return false;
    if (--n == 0) return false;
  }
  if (rc_read(ErrorReg) & 0x13) return false;
  if (rx && rxLen) {
    uint8_t len = rc_read(FIFOLevelReg);
    if (len > *rxLen) return false;
    *rxLen = len;
    for (uint8_t i = 0; i < len; i++) rx[i] = rc_read(FIFODataReg);
  }
  return true;
}

static void rc_crc(const uint8_t *d, uint8_t n, uint8_t *out) {
  rc_write(CommandReg, 0x00); rc_write(DivIrqReg, 0x04); rc_write(FIFOLevelReg, 0x80);
  for (uint8_t i = 0; i < n; i++) rc_write(FIFODataReg, d[i]);
  rc_write(CommandReg, 0x03);
  uint16_t t = 5000;
  while (!(rc_read(DivIrqReg) & 0x04) && --t);
  rc_write(CommandReg, 0x00);
  out[0] = rc_read(CRCResultRegL); out[1] = rc_read(CRCResultRegH);
}

static bool rc_read_uid(uint8_t *uid) {
  uint8_t rx[5], len;
  rc_clrbits(CollReg, 0x80);
  uint8_t reqa = 0x26; len = 2;
  if (!rc_transceive(&reqa, 1, rx, &len, 7) || len != 2) return false;
  uint8_t ac[2] = {0x93, 0x20}; len = 5;
  if (!rc_transceive(ac, 2, rx, &len, 0) || len != 5) return false;
  if ((rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) return false;
  uint8_t sel[9] = {0x93, 0x70, rx[0], rx[1], rx[2], rx[3], rx[4], 0, 0};
  rc_crc(sel, 7, sel + 7);
  uint8_t sak[3]; len = 3;
  rc_transceive(sel, 9, sak, &len, 0);
  for (uint8_t i = 0; i < 4; i++) uid[i] = rx[i];
  return true;
}

static void rc_halt() {
  uint8_t h[4] = {0x50, 0x00, 0, 0};
  rc_crc(h, 2, h + 2);
  rc_transceive(h, 4, 0, 0, 0);
  rc_write(CommandReg, 0x00);
}

static bool is_valid_card(const uint8_t *uid) {
  for (uint8_t i = 0; i < 4; i++) {
    if (uid[i] != VALID_UID[i]) return false;
  }
  return true;
}

/* ---------- nRF24L01+ (250 kbps + ACK Software prin Ecou) ---------- */
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

static void nrf_init() {
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
  nrf_wreg_buf(0x10, RF_ADDR, 5); // TX_ADDR
  nrf_wreg_buf(0x0A, RF_ADDR, 5); // RX_ADDR_P0
  nrf_wreg(0x11, 32);   // Payload 32 octeti

  nrf_cmd(0xE1);
  nrf_cmd(0xE2);
  nrf_wreg(0x07, 0x70);
  nrf_wreg(0x00, 0x0E); // TX mode
  delay_ms(5);
}

// Trimite un pachet si asteapta max ~60 ms ecoul identic de la Uno
static bool nrf_send_and_wait_ack(const char *msg) {
  uint8_t buf[32];
  mem_zero(buf, 32);
  for (uint8_t i = 0; i < 31 && msg[i]; i++) buf[i] = (uint8_t)msg[i];

  // 1. Comuta pe TX si emite pachetul
  NRF_CE_L();
  nrf_wreg(0x00, 0x0E); // PRIM_RX = 0 (TX)
  delay_us(150);
  nrf_cmd(0xE1);
  nrf_wreg(0x07, 0x70);

  NRF_CSN_L();
  spi_xfer(0xA0);
  for (uint8_t i = 0; i < 32; i++) spi_xfer(buf[i]);
  NRF_CSN_H();

  NRF_CE_H(); delay_us(20); NRF_CE_L();

  for (uint16_t t = 0; t < 5000; t++) {
    if (nrf_cmd(0xFF) & 0x20) break;
    delay_us(10);
  }
  nrf_wreg(0x07, 0x70);

  // 2. Comuta imediat pe RX pentru a prinde ACK-ul (ecoul) de la Uno
  nrf_cmd(0xE2);
  nrf_wreg(0x00, 0x0F); // PRIM_RX = 1 (RX)
  NRF_CE_H();

  bool ack_ok = false;
  for (uint16_t t = 0; t < 600; t++) { // ~60 ms fereastra de ascultare
    uint8_t st = nrf_cmd(0xFF);
    uint8_t fifo = nrf_rreg(0x17);
    if ((st & 0x40) || !(fifo & 0x01)) {
      char rx_msg[33];
      NRF_CSN_L();
      spi_xfer(0x61);
      for (uint8_t i = 0; i < 32; i++) rx_msg[i] = (char)spi_xfer(0xFF);
      NRF_CSN_H();
      rx_msg[32] = 0;
      nrf_wreg(0x07, 0x40);

      if (str_eq(rx_msg, msg)) {
        ack_ok = true;
        break;
      }
    }
    delay_us(100);
  }

  // Revine in standby TX
  NRF_CE_L();
  nrf_wreg(0x00, 0x0E);
  return ack_ok;
}

// Trimite in bucla pana primeste confirmarea de la Uno
static void nrf_send_reliable(const char *msg, bool silent) {
  uint16_t incercari = 0;
  while (!nrf_send_and_wait_ack(msg)) {
    incercari++;
    delay_ms(40);
  }
  if (!silent) {
    uart_str("    [ACK PRIMIT] Mesaj confirmat dupa ");
    uart_dec(incercari + 1);
    uart_ln(" incercari.");
  }
}

/* ===================== SETUP & LOOP ===================== */
void setup() {
  R_TIMSK0 = 0;
  R_DDRD |= (1 << P_BUZ) | (1 << P_LED_V) | (1 << P_LED_R);
  BUZ_OFF(); LED_V_OFF(); LED_R_OFF();

  uart_init();
  spi_init();
  servo_init();
  SEI();

  delay_ms(400);
  uart_ln("\r\n=== NOD 1: SISTEM CONTROL ACCES ===");

  // Test rapid componente locale
  LED_V_ON(); LED_R_ON();
  beep(100); delay_ms(100); beep(100);
  LED_V_OFF(); LED_R_OFF();

  rc_init();
  nrf_init();

  uint8_t cfg = nrf_rreg(0x00);
  if (cfg == 0x0E) {
    uart_ln("[OK] Hardware SPI & Radio gata.");
  } else {
    uart_ln("[EROARE] Verifica conexiunile SPI ale antenei!");
  }

  // Handshake initial silencios ("Alex e tare" nu se afiseaza in Serial Monitor)
  uart_ln("[...] Astept conexiunea cu receptorul (Nod 2)...");
  nrf_send_reliable("Alex te bat cu lcd-ul ala daca zici ca e I2C cand el e paralel", true);
  uart_ln("[CONECTAT] Legatura radio confirmata cu Nod 2!\r\n");

  // Semnal vizual scurt de conectare reusita
  LED_V_ON(); beep(80); delay_ms(150); LED_V_OFF();
}

void loop() {
  uint8_t uid[4];
  if (!rc_read_uid(uid)) {
    delay_ms(50);
    return;
  }

  bool safe = is_valid_card(uid);
  const char *prefix = safe ? "ACCES PERMIS: " : "ALERTA INTRUS: ";

  char msg[32];
  uint8_t k = 0;
  while (prefix[k]) { msg[k] = prefix[k]; k++; }

  const char *hx = "0123456789ABCDEF";
  uart_str("Card detectat UID: ");
  for (uint8_t i = 0; i < 4; i++) {
    uart_hex(uid[i]); if (i < 3) uart_char(' ');
    msg[k++] = hx[uid[i] >> 4];
    msg[k++] = hx[uid[i] & 0x0F];
  }
  msg[k] = 0;
  uart_ln("");

  if (safe) {
    // --- MOD SAFE ---
    uart_ln(" -> [SAFE] Acces permis! Deschid usa...");
    LED_R_OFF();
    LED_V_ON();
    beep(120);
    servo_set(SERVO_OPEN);

    // Trimite constant pana cand Uno confirma primirea mesajului
    nrf_send_reliable(msg, false);

    delay_ms(2500);
    servo_set(SERVO_CLOSED);
    LED_V_OFF();
  } else {
    // --- MOD INTRUS ---
    uart_ln(" -> [INTRUS] Card neautorizat! Trimit alerta...");
    LED_V_OFF();
    LED_R_ON();
    beep(100); delay_ms(80); beep(100); delay_ms(80); beep(100);

    // Trimite constant pana cand Uno confirma primirea alertei
    nrf_send_reliable(msg, false);

    delay_ms(1500);
    LED_R_OFF();
  }

  rc_halt();
  delay_ms(300);
}