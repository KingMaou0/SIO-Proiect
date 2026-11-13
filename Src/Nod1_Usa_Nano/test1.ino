/*
 * NOD 1 - Terminal de acces (Arduino Nano / ATmega328P @ 16 MHz)
 * FARA biblioteci si FARA headere avr-libc (nu se include avr/io.h,
 * avr/interrupt.h, util/delay.h, string.h).
 * Registrii sunt accesati direct prin adresele din datasheet ATmega328P.
 * Singurul header: <stdint.h> (tipuri standard, oferit de compilator).
 */
#include <stdint.h>

/* ===================== REGISTRI (adrese din datasheet) ===================== */
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
#define R_TWBR    REG8(0xB8)
#define R_TWSR    REG8(0xB9)
#define R_TWDR    REG8(0xBB)
#define R_TWCR    REG8(0xBC)

// pozitii de biti
#define B_SPE    6
#define B_MSTR   4
#define B_SPR0   0
#define B_SPIF   7
#define B_TWINT  7
#define B_TWSTA  5
#define B_TWSTO  4
#define B_TWEN   2
#define B_WGM12  3
#define B_CS11   1
#define B_OCIE1A 1
#define B_OCIE1B 2

// pini: PORTB: D8=0 (nRF CSN), D9=1 (RC522 RST), D10=2 (RC522 SS),
//       D11=3 MOSI, D12=4 MISO, D13=5 SCK
//       PORTD: D3=3 (servo), D5=5 (buzzer), D7=7 (nRF CE)
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

/* ===================== CONFIGURARE ===================== */
#define LCD_ADDR      0x27            // sau 0x3F
#define SERVO_CLOSED  1088            // 544 us  (0 grade)  [tick = 0.5 us]
#define SERVO_OPEN    2944            // 1472 us (90 grade)

static const uint8_t RF_ADDR[5] = {'0','0','0','0','1'};

// UID-uri autorizate (4 octeti) - inlocuieste cu cardurile tale
static const uint8_t AUTH_UIDS[][4] = {
  {0x8A, 0xB2, 0xC3, 0x4E},
};
#define N_AUTH (sizeof(AUTH_UIDS) / sizeof(AUTH_UIDS[0]))

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

/* ===================== UTILITARE (fara string.h / util/delay.h) ===================== */
// ~1 us la 16 MHz (12 NOP + ~4 cicluri overhead). Aproximativ, suficient pentru LCD/RF.
static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__(
      "nop\n nop\n nop\n nop\n nop\n nop\n"
      "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}
static void delay_ms(uint16_t n) { while (n--) delay_us(1000); }

static void mem_zero(uint8_t *p, uint8_t n) { while (n--) *p++ = 0; }
static bool uid_eq(const uint8_t *a, const uint8_t *b) {
  for (uint8_t i = 0; i < 4; i++) if (a[i] != b[i]) return false;
  return true;
}
static void str_cpy(char *d, const char *s) { while ((*d++ = *s++)); }
static void str_cat(char *d, const char *s) { while (*d) d++; str_cpy(d, s); }

/* ===================== SPI ===================== */
static void spi_init() {
  R_DDRB |= (1 << P_NRF_CSN) | (1 << P_RC_RST) | (1 << P_RC_SS) | (1 << P_MOSI) | (1 << P_SCK);
  R_DDRB &= ~(1 << P_MISO);
  R_PORTB |= (1 << P_NRF_CSN) | (1 << P_RC_SS);              // CS inactive
  R_SPCR = (1 << B_SPE) | (1 << B_MSTR) | (1 << B_SPR0);     // master, mod 0, fosc/16
}

static uint8_t spi_xfer(uint8_t b) {
  R_SPDR = b;
  while (!(R_SPSR & (1 << B_SPIF)));
  return R_SPDR;
}

/* ===================== TWI (I2C) ===================== */
static void twi_init() {
  R_TWSR = 0;
  R_TWBR = 72;                       // 100 kHz
  R_TWCR = (1 << B_TWEN);
}
static void twi_start() {
  R_TWCR = (1 << B_TWINT) | (1 << B_TWSTA) | (1 << B_TWEN);
  while (!(R_TWCR & (1 << B_TWINT)));
}
static void twi_write(uint8_t b) {
  R_TWDR = b;
  R_TWCR = (1 << B_TWINT) | (1 << B_TWEN);
  while (!(R_TWCR & (1 << B_TWINT)));
}
static void twi_stop() {
  R_TWCR = (1 << B_TWINT) | (1 << B_TWEN) | (1 << B_TWSTO);
  while (R_TWCR & (1 << B_TWSTO));
}

/* ===================== LCD1602 (PCF8574) ===================== */
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

static void lcd_send4(uint8_t nib, uint8_t rs) {
  uint8_t d = (uint8_t)(nib << 4) | LCD_BL | rs;
  twi_start();
  twi_write(LCD_ADDR << 1);
  twi_write(d | LCD_EN);
  twi_write(d);
  twi_stop();
  delay_us(50);
}
static void lcd_cmd(uint8_t c)  { lcd_send4(c >> 4, 0);      lcd_send4(c & 0x0F, 0); }
static void lcd_data(uint8_t c) { lcd_send4(c >> 4, LCD_RS); lcd_send4(c & 0x0F, LCD_RS); }

static void lcd_init() {
  delay_ms(50);
  lcd_send4(0x3, 0); delay_ms(5);
  lcd_send4(0x3, 0); delay_us(150);
  lcd_send4(0x3, 0);
  lcd_send4(0x2, 0);
  lcd_cmd(0x28);
  lcd_cmd(0x0C);
  lcd_cmd(0x06);
  lcd_cmd(0x01); delay_ms(2);
}

static void lcd_print(uint8_t row, const char *s) {
  lcd_cmd(0x80 | (row ? 0x40 : 0x00));
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) lcd_data(s[i]);
  for (; i < 16; i++) lcd_data(' ');
}

static void show_standby() {
  lcd_print(0, "  SISTEM ACCES");
  lcd_print(1, " Apropie cardul");
}

/* ===================== SERVO (Timer1, 50 Hz, PWM software pe D3) ===================== */
static volatile uint16_t servo_ticks = SERVO_CLOSED;

// Vectorii de intrerupere fara avr/interrupt.h:
// ATmega328P: TIMER1_COMPA = vector 12 (nume __vector_11), TIMER1_COMPB = vector 13 (__vector_12)
extern "C" void __vector_11(void) __attribute__((signal, used, externally_visible));
extern "C" void __vector_12(void) __attribute__((signal, used, externally_visible));

extern "C" void __vector_11(void) {      // inceput perioada (20 ms)
  R_OCR1B = servo_ticks;
  R_PORTD |= (1 << P_SERVO);
}
extern "C" void __vector_12(void) {      // sfarsit puls
  R_PORTD &= ~(1 << P_SERVO);
}

static void servo_init() {
  R_DDRD |= (1 << P_SERVO);
  R_TCCR1A = 0;
  R_TCCR1B = (1 << B_WGM12) | (1 << B_CS11);   // CTC, prescaler 8 -> 0.5 us/tick
  R_OCR1A  = 39999;                             // 20 ms
  R_OCR1B  = SERVO_CLOSED;
  R_TIMSK1 = (1 << B_OCIE1A) | (1 << B_OCIE1B);
}
static void servo_set(uint16_t ticks) {
  uint8_t sreg = R_SREG; CLI();
  servo_ticks = ticks;
  R_SREG = sreg;
}

/* ===================== BUZZER ===================== */
static void beep(uint16_t ms) { BUZ_ON(); delay_ms(ms); BUZ_OFF(); }
static void beep_error() {
  for (uint8_t i = 0; i < 3; i++) { beep(150); delay_ms(120); }
}

/* ===================== RC522 ===================== */
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

static void rc_write(uint8_t reg, uint8_t val) {
  RC_SS_L();
  spi_xfer((reg << 1) & 0x7E);
  spi_xfer(val);
  RC_SS_H();
}
static uint8_t rc_read(uint8_t reg) {
  RC_SS_L();
  spi_xfer(((reg << 1) & 0x7E) | 0x80);
  uint8_t v = spi_xfer(0);
  RC_SS_H();
  return v;
}
static void rc_setbits(uint8_t reg, uint8_t m) { rc_write(reg, rc_read(reg) | m); }
static void rc_clrbits(uint8_t reg, uint8_t m) { rc_write(reg, rc_read(reg) & ~m); }

static void rc_init() {
  RC_RST_L(); delay_us(10);
  RC_RST_H(); delay_ms(50);
  rc_write(CommandReg, 0x0F);          // soft reset
  delay_ms(50);
  rc_write(TModeReg, 0x80);
  rc_write(TPrescalerReg, 0xA9);
  rc_write(TReloadRegH, 0x03);
  rc_write(TReloadRegL, 0xE8);
  rc_write(TxASKReg, 0x40);
  rc_write(ModeReg, 0x3D);
  rc_setbits(TxControlReg, 0x03);      // antena ON
}

static bool rc_transceive(const uint8_t *tx, uint8_t txLen,
                          uint8_t *rx, uint8_t *rxLen, uint8_t txBits) {
  rc_write(CommandReg, 0x00);
  rc_write(ComIrqReg, 0x7F);
  rc_write(FIFOLevelReg, 0x80);
  for (uint8_t i = 0; i < txLen; i++) rc_write(FIFODataReg, tx[i]);
  rc_write(BitFramingReg, txBits);
  rc_write(CommandReg, 0x0C);          // Transceive
  rc_setbits(BitFramingReg, 0x80);     // StartSend

  uint16_t n = 3000;
  for (;;) {
    uint8_t irq = rc_read(ComIrqReg);
    if (irq & 0x30) break;
    if (irq & 0x01) return false;
    if (--n == 0)   return false;
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
  rc_write(CommandReg, 0x00);
  rc_write(DivIrqReg, 0x04);
  rc_write(FIFOLevelReg, 0x80);
  for (uint8_t i = 0; i < n; i++) rc_write(FIFODataReg, d[i]);
  rc_write(CommandReg, 0x03);          // CalcCRC
  uint16_t t = 5000;
  while (!(rc_read(DivIrqReg) & 0x04) && --t);
  rc_write(CommandReg, 0x00);
  out[0] = rc_read(CRCResultRegL);
  out[1] = rc_read(CRCResultRegH);
}

static bool rc_read_uid(uint8_t *uid) {
  uint8_t rx[5], len;

  rc_clrbits(CollReg, 0x80);
  uint8_t reqa = 0x26;
  len = 2;
  if (!rc_transceive(&reqa, 1, rx, &len, 7) || len != 2) return false;

  uint8_t ac[2] = {0x93, 0x20};
  len = 5;
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
  rc_transceive(h, 4, 0, 0, 0);        // timeout asteptat
  rc_write(CommandReg, 0x00);
}

/* ===================== nRF24L01+ (TX) ===================== */
#define NRF_CONFIG     0x00
#define NRF_EN_AA      0x01
#define NRF_EN_RXADDR  0x02
#define NRF_SETUP_AW   0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH      0x05
#define NRF_RF_SETUP   0x06
#define NRF_STATUS     0x07
#define NRF_RX_ADDR_P0 0x0A
#define NRF_TX_ADDR    0x10
#define NRF_RX_PW_P0   0x11

static void nrf_wreg(uint8_t reg, uint8_t val) {
  NRF_CSN_L(); spi_xfer(0x20 | reg); spi_xfer(val); NRF_CSN_H();
}
static void nrf_wreg_buf(uint8_t reg, const uint8_t *b, uint8_t n) {
  NRF_CSN_L(); spi_xfer(0x20 | reg);
  for (uint8_t i = 0; i < n; i++) spi_xfer(b[i]);
  NRF_CSN_H();
}
static uint8_t nrf_cmd(uint8_t c) {
  NRF_CSN_L(); uint8_t s = spi_xfer(c); NRF_CSN_H(); return s;
}

static void nrf_init() {
  R_DDRD |= (1 << P_NRF_CE);
  NRF_CE_L(); NRF_CSN_H();
  delay_ms(100);
  nrf_wreg(NRF_SETUP_AW, 0x03);
  nrf_wreg(NRF_SETUP_RETR, 0x5F);
  nrf_wreg(NRF_RF_CH, 76);
  nrf_wreg(NRF_RF_SETUP, 0x06);          // 1 Mbps, 0 dBm
  nrf_wreg(NRF_EN_AA, 0x01);
  nrf_wreg(NRF_EN_RXADDR, 0x01);
  nrf_wreg_buf(NRF_TX_ADDR, RF_ADDR, 5);
  nrf_wreg_buf(NRF_RX_ADDR_P0, RF_ADDR, 5);
  nrf_wreg(NRF_RX_PW_P0, 32);
  nrf_cmd(0xE1); nrf_cmd(0xE2);
  nrf_wreg(NRF_STATUS, 0x70);
  nrf_wreg(NRF_CONFIG, 0x0E);            // PWR_UP, CRC16, TX
  delay_ms(5);
}

static bool nrf_send(const char *msg) {
  uint8_t buf[32];
  mem_zero(buf, 32);
  for (uint8_t i = 0; i < 31 && msg[i]; i++) buf[i] = (uint8_t)msg[i];

  NRF_CSN_L();
  spi_xfer(0xA0);
  for (uint8_t i = 0; i < 32; i++) spi_xfer(buf[i]);
  NRF_CSN_H();

  NRF_CE_H(); delay_us(15); NRF_CE_L();

  uint8_t st = 0;
  for (uint16_t t = 0; t < 5000; t++) {
    st = nrf_cmd(0xFF);
    if (st & 0x30) break;
    delay_us(10);
  }
  nrf_wreg(NRF_STATUS, 0x30);
  if (st & 0x10) { nrf_cmd(0xE1); return false; }
  return (st & 0x20);
}

/* ===================== LOGICA ===================== */
static bool is_authorized(const uint8_t *uid) {
  for (uint8_t i = 0; i < N_AUTH; i++)
    if (uid_eq(uid, AUTH_UIDS[i])) return true;
  return false;
}

static void uid_to_str(const uint8_t *uid, char *out) {
  const char *hx = "0123456789ABCDEF";
  uint8_t k = 0;
  for (uint8_t i = 0; i < 4; i++) {
    out[k++] = hx[uid[i] >> 4];
    out[k++] = hx[uid[i] & 0x0F];
    if (i < 3) out[k++] = ' ';
  }
  out[k] = 0;
}

void setup() {
  R_TIMSK0 = 0;                // Timer0 nefolosit, servo fara jitter
  R_DDRD |= (1 << P_BUZ);
  BUZ_OFF();

  spi_init();
  twi_init();
  servo_init();
  SEI();

  lcd_init();
  nrf_init();
  rc_init();
  show_standby();
}

void loop() {
  uint8_t uid[4];
  if (!rc_read_uid(uid)) { delay_ms(50); return; }

  char s[12], msg[32];
  uid_to_str(uid, s);

  if (is_authorized(uid)) {
    lcd_print(0, "  ACCES PERMIS");
    lcd_print(1, "  Usa Deschisa");
    str_cpy(msg, "ACCES PERMIS: "); str_cat(msg, s);
    nrf_send(msg);
    beep(100);
    servo_set(SERVO_OPEN);
    delay_ms(3000);
    servo_set(SERVO_CLOSED);
    delay_ms(500);
  } else {
    lcd_print(0, " ACCES RESPINS!");
    lcd_print(1, "Card Neautorizat");
    str_cpy(msg, "ALERTA INTRUS: "); str_cat(msg, s);
    nrf_send(msg);
    beep_error();
    delay_ms(1000);
  }

  rc_halt();
  show_standby();
}
