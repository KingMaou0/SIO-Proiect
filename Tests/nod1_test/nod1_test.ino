/*
 * NOD 1 - TEST componente (fara LCD). Rezultatele ies pe Serial Monitor, 9600 baud.
 * Testeaza: buzzer, servo, RC522 (versiune + citire carduri), nRF24L01+ (registri + trimitere).
 * Fara biblioteci / fara headere avr-libc.
 * Salveaza-l intr-un folder numit "nod1_test".
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

#define B_SPE 6
#define B_MSTR 4
#define B_SPR0 0
#define B_SPIF 7
#define B_WGM12 3
#define B_CS11 1
#define B_OCIE1A 1
#define B_OCIE1B 2

#define P_NRF_CSN 0
#define P_RC_RST  1
#define P_RC_SS   2
#define P_MOSI    3
#define P_MISO    4
#define P_SCK     5
#define P_SERVO   3
#define P_BUZ     5
#define P_NRF_CE  7

#define CLI() __asm__ __volatile__("cli" ::: "memory")
#define SEI() __asm__ __volatile__("sei" ::: "memory")

#define SERVO_CLOSED  1088
#define SERVO_OPEN    2944
static const uint8_t RF_ADDR[5] = {'0','0','0','0','1'};

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

/* ---------- utilitare ---------- */
static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}
static void delay_ms(uint16_t n) { while (n--) delay_us(1000); }
static void mem_zero(uint8_t *p, uint8_t n) { while (n--) *p++ = 0; }

/* ---------- UART0 (Serial) ---------- */
static void uart_init() {
  R_UBRR0 = 103;                 // 9600 baud @ 16 MHz
  R_UCSR0B = (1 << 3);           // TXEN0
  R_UCSR0C = (1 << 2) | (1 << 1);// 8N1
}
static void uart_char(char c) { while (!(R_UCSR0A & (1 << 5))); R_UDR0 = c; }
static void uart_str(const char *s) { while (*s) uart_char(*s++); }
static void uart_hex(uint8_t v) {
  const char *hx = "0123456789ABCDEF";
  uart_char(hx[v >> 4]); uart_char(hx[v & 0x0F]);
}
static void uart_ln(const char *s) { uart_str(s); uart_str("\r\n"); }

/* ---------- SPI ---------- */
static void spi_init() {
  R_DDRB |= (1 << P_NRF_CSN) | (1 << P_RC_RST) | (1 << P_RC_SS) | (1 << P_MOSI) | (1 << P_SCK);
  R_DDRB &= ~(1 << P_MISO);
  R_PORTB |= (1 << P_NRF_CSN) | (1 << P_RC_SS);
  R_SPCR = (1 << B_SPE) | (1 << B_MSTR) | (1 << B_SPR0);
}
static uint8_t spi_xfer(uint8_t b) {
  R_SPDR = b;
  while (!(R_SPSR & (1 << B_SPIF)));
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
  R_TCCR1B = (1 << B_WGM12) | (1 << B_CS11);
  R_OCR1A = 39999;
  R_OCR1B = SERVO_CLOSED;
  R_TIMSK1 = (1 << B_OCIE1A) | (1 << B_OCIE1B);
}
static void servo_set(uint16_t t) {
  uint8_t s = R_SREG; CLI(); servo_ticks = t; R_SREG = s;
}
static void beep(uint16_t ms) { BUZ_ON(); delay_ms(ms); BUZ_OFF(); }

/* ---------- RC522 ---------- */
#define CommandReg 0x01
#define ComIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define ModeReg 0x11
#define TxControlReg 0x14
#define TxASKReg 0x15
#define CRCResultRegH 0x21
#define CRCResultRegL 0x22
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define VersionReg 0x37

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
static uint8_t dbg_stage = 0, dbg_a0 = 0, dbg_a1 = 0, dbg_err = 0, dbg_len = 0;
static bool rc_read_uid(uint8_t *uid) {
  uint8_t rx[5], len;
  dbg_stage = 1;
  rc_clrbits(CollReg, 0x80);
  uint8_t reqa = 0x26; len = 2;
  if (!rc_transceive(&reqa, 1, rx, &len, 7) || len != 2) return false;
  dbg_a0 = rx[0]; dbg_a1 = rx[1];
  dbg_stage = 2;
  uint8_t ac[2] = {0x93, 0x20}; len = 5;
  if (!rc_transceive(ac, 2, rx, &len, 0) || len != 5) { dbg_err = rc_read(ErrorReg); dbg_len = len; return false; }
  dbg_stage = 3;
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
static void nrf_init() {
  R_DDRD |= (1 << P_NRF_CE);
  NRF_CE_L(); NRF_CSN_H(); delay_ms(100);
  nrf_wreg(0x03, 0x03); nrf_wreg(0x04, 0x5F); nrf_wreg(0x05, 76);
  nrf_wreg(0x06, 0x06); nrf_wreg(0x01, 0x01); nrf_wreg(0x02, 0x01);
  nrf_wreg_buf(0x10, RF_ADDR, 5); nrf_wreg_buf(0x0A, RF_ADDR, 5);
  nrf_wreg(0x11, 32);
  nrf_cmd(0xE1); nrf_cmd(0xE2);
  nrf_wreg(0x07, 0x70);
  nrf_wreg(0x00, 0x0E);
  delay_ms(5);
}
// 0 = eroare, 1 = ACK primit
static bool nrf_send(const char *msg) {
  uint8_t buf[32]; mem_zero(buf, 32);
  for (uint8_t i = 0; i < 31 && msg[i]; i++) buf[i] = (uint8_t)msg[i];
  NRF_CSN_L(); spi_xfer(0xA0);
  for (uint8_t i = 0; i < 32; i++) spi_xfer(buf[i]);
  NRF_CSN_H();
  NRF_CE_H(); delay_us(15); NRF_CE_L();
  uint8_t st = 0;
  for (uint16_t t = 0; t < 5000; t++) {
    st = nrf_cmd(0xFF);
    if (st & 0x30) break;
    delay_us(10);
  }
  nrf_wreg(0x07, 0x30);
  if (st & 0x10) { nrf_cmd(0xE1); return false; }
  return (st & 0x20);
}

/* ===================== TESTE ===================== */
static void test_buzzer() {
  uart_ln("[1] BUZZER: 2 bipuri - le-ai auzit?");
  beep(150); delay_ms(150); beep(150);
}
static void test_servo() {
  uart_ln("[2] SERVO: 0 -> 90 -> 0 grade");
  servo_set(SERVO_CLOSED); delay_ms(800);
  servo_set(SERVO_OPEN);   delay_ms(1200);
  servo_set(SERVO_CLOSED); delay_ms(800);
  uart_ln("    daca s-a miscat = OK");
}
static void test_rc522() {
  rc_init();
  uint8_t v = rc_read(VersionReg);
  uart_str("[3] RC522 TxControl=0x"); uart_hex(rc_read(TxControlReg)); uart_ln(" (antena ON daca se termina in 3)");
  uart_str("    RC522 VersionReg = 0x"); uart_hex(v);
  if (v == 0x91 || v == 0x92 || v == 0x88 || v == 0xB2) uart_ln("  -> OK");
  else if (v == 0x00 || v == 0xFF) uart_ln("  -> EROARE (verifica 3.3V, SDA=D10, RST=D9, SCK/MOSI/MISO)");
  else uart_ln("  -> versiune necunoscuta (posibil clona, probabil functioneaza)");
}
static void test_nrf() {
  nrf_init();
  uint8_t cfg = nrf_rreg(0x00), ch = nrf_rreg(0x05), aw = nrf_rreg(0x03);
  uart_str("[4] nRF CONFIG=0x"); uart_hex(cfg);
  uart_str(" RF_CH=0x"); uart_hex(ch);
  uart_str(" SETUP_AW=0x"); uart_hex(aw);
  if (cfg == 0x0E && ch == 76 && aw == 0x03) uart_ln("  -> SPI OK");
  else uart_ln("  -> EROARE (verifica 3.3V + condensator, CE=D7, CSN=D8, SCK/MOSI/MISO)");
  uart_str("    trimit pachet test: ");
  if (nrf_send("TEST NOD1")) uart_ln("ACK primit (receptor activ) -> OK");
  else uart_ln("fara ACK (normal daca receptorul nu e pornit/pe acelasi canal)");
}

void setup() {
  R_TIMSK0 = 0;
  R_DDRD |= (1 << P_BUZ);
  BUZ_OFF();
  uart_init();
  spi_init();
  servo_init();
  SEI();

  delay_ms(500);
  uart_ln("\r\n=== TEST NOD 1 ===");
  test_buzzer();
  test_servo();
  test_rc522();
  test_nrf();
  uart_ln("\r\n[5] Apropie un card de RC522 (UID-ul apare mai jos + bip + trimis prin radio)");
}

void loop() {
  uint8_t uid[4];
  if (!rc_read_uid(uid)) {
    if (dbg_stage >= 2) {
      uart_str("    DIAG: card detectat, ATQA=");
      uart_hex(dbg_a0); uart_hex(dbg_a1);
      uart_str(dbg_stage == 2 ? " dar anticollision a esuat (Err=0x" : " dar BCC gresit (Err=0x");
      uart_hex(dbg_err); uart_ln(")");
      delay_ms(500);
    }
    delay_ms(50); return;
  }

  char msg[32] = "TEST UID: ";
  const char *hx = "0123456789ABCDEF";
  uart_str("    UID: ");
  uint8_t k = 10;
  for (uint8_t i = 0; i < 4; i++) {
    uart_hex(uid[i]); if (i < 3) uart_char(' ');
    msg[k++] = hx[uid[i] >> 4]; msg[k++] = hx[uid[i] & 0x0F];
  }
  msg[k] = 0;
  beep(80);
  uart_str("  | radio: ");
  uart_ln(nrf_send(msg) ? "ACK" : "fara ACK");

  rc_halt();
  delay_ms(300);
}
