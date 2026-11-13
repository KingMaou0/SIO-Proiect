/*
 * NOD 1 - Terminal de acces (Arduino Nano / ATmega328P @ 16 MHz)
 * FARA biblioteci: SPI, TWI (I2C), Timer1, GPIO - totul prin registri.
 * RC522 (SPI), nRF24L01+ (SPI, TX), LCD1602 + PCF8574 (TWI), servo SG90, buzzer.
 *
 * Nu se foloseste nicio functie Arduino (pinMode/delay/millis etc.),
 * doar setup()/loop() ca sa compileze in Arduino IDE. Pentru avr-gcc pur,
 * inlocuieste setup()/loop() cu main().
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

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

/* ===================== PINI ===================== */
// PORTB: D8=PB0 (nRF CSN), D9=PB1 (RC522 RST), D10=PB2 (RC522 SS),
//        D11=PB3 MOSI, D12=PB4 MISO, D13=PB5 SCK
// PORTD: D3=PD3 (servo), D5=PD5 (buzzer), D7=PD7 (nRF CE)
// PORTC: A4=PC4 SDA, A5=PC5 SCL
#define NRF_CSN_L()  (PORTB &= ~(1 << PB0))
#define NRF_CSN_H()  (PORTB |=  (1 << PB0))
#define RC_RST_L()   (PORTB &= ~(1 << PB1))
#define RC_RST_H()   (PORTB |=  (1 << PB1))
#define RC_SS_L()    (PORTB &= ~(1 << PB2))
#define RC_SS_H()    (PORTB |=  (1 << PB2))
#define NRF_CE_L()   (PORTD &= ~(1 << PD7))
#define NRF_CE_H()   (PORTD |=  (1 << PD7))
#define BUZ_ON()     (PORTD |=  (1 << PD5))
#define BUZ_OFF()    (PORTD &= ~(1 << PD5))

static void delay_ms(uint16_t n) { while (n--) _delay_ms(1); }

/* ===================== SPI ===================== */
static void spi_init() {
  // MOSI, SCK, SS(D10) ca iesiri; SS trebuie sa fie iesire ca sa ramana master
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB5);
  DDRB &= ~(1 << PB4);
  PORTB |= (1 << PB0) | (1 << PB2);                 // CS-uri inactive (high)
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);    // master, mod 0, fosc/16 = 1 MHz
}

static uint8_t spi_xfer(uint8_t b) {
  SPDR = b;
  while (!(SPSR & (1 << SPIF)));
  return SPDR;
}

/* ===================== TWI (I2C) ===================== */
static void twi_init() {
  TWSR = 0;
  TWBR = 72;                       // (16M/100k - 16) / 2 = 72 -> 100 kHz
  TWCR = (1 << TWEN);
}
static void twi_start() {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}
static void twi_write(uint8_t b) {
  TWDR = b;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}
static void twi_stop() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  while (TWCR & (1 << TWSTO));
}

/* ===================== LCD1602 (PCF8574) ===================== */
// P0=RS P1=RW P2=EN P3=backlight P4..P7=D4..D7
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
  _delay_us(50);
}
static void lcd_cmd(uint8_t c)  { lcd_send4(c >> 4, 0);      lcd_send4(c & 0x0F, 0); }
static void lcd_data(uint8_t c) { lcd_send4(c >> 4, LCD_RS); lcd_send4(c & 0x0F, LCD_RS); }

static void lcd_init() {
  delay_ms(50);
  lcd_send4(0x3, 0); delay_ms(5);
  lcd_send4(0x3, 0); _delay_us(150);
  lcd_send4(0x3, 0);
  lcd_send4(0x2, 0);               // mod 4 biti
  lcd_cmd(0x28);                   // 2 linii, 5x8
  lcd_cmd(0x0C);                   // display ON, cursor OFF
  lcd_cmd(0x06);                   // entry mode
  lcd_cmd(0x01); delay_ms(2);      // clear
}

static void lcd_print(uint8_t row, const char *s) {
  lcd_cmd(0x80 | (row ? 0x40 : 0x00));
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) lcd_data(s[i]);
  for (; i < 16; i++) lcd_data(' ');   // completeaza cu spatii
}

static void show_standby() {
  lcd_print(0, "  SISTEM ACCES");
  lcd_print(1, " Apropie cardul");
}

/* ===================== SERVO (Timer1, 50 Hz, PWM software pe D3) ===================== */
static volatile uint16_t servo_ticks = SERVO_CLOSED;

ISR(TIMER1_COMPA_vect) {           // inceput perioada (20 ms)
  OCR1B = servo_ticks;
  PORTD |= (1 << PD3);
}
ISR(TIMER1_COMPB_vect) {           // sfarsit puls
  PORTD &= ~(1 << PD3);
}

static void servo_init() {
  DDRD |= (1 << PD3);
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);   // CTC (TOP=OCR1A), prescaler 8 -> 0.5 us/tick
  OCR1A  = 39999;                         // 20 ms
  OCR1B  = SERVO_CLOSED;
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
}
static void servo_set(uint16_t ticks) {
  uint8_t sreg = SREG; cli();
  servo_ticks = ticks;
  SREG = sreg;
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
  RC_RST_L(); _delay_us(10);
  RC_RST_H(); delay_ms(50);
  rc_write(CommandReg, 0x0F);          // soft reset
  delay_ms(50);
  rc_write(TModeReg, 0x80);            // timer auto, ~25 ms timeout
  rc_write(TPrescalerReg, 0xA9);
  rc_write(TReloadRegH, 0x03);
  rc_write(TReloadRegL, 0xE8);
  rc_write(TxASKReg, 0x40);            // 100% ASK
  rc_write(ModeReg, 0x3D);             // CRC preset 0x6363
  rc_setbits(TxControlReg, 0x03);      // antena ON
}

// Transmite/primeste cu PICC. Returneaza true la succes.
static bool rc_transceive(const uint8_t *tx, uint8_t txLen,
                          uint8_t *rx, uint8_t *rxLen, uint8_t txBits) {
  rc_write(CommandReg, 0x00);
  rc_write(ComIrqReg, 0x7F);           // sterge flag-urile
  rc_write(FIFOLevelReg, 0x80);        // flush FIFO
  for (uint8_t i = 0; i < txLen; i++) rc_write(FIFODataReg, tx[i]);
  rc_write(BitFramingReg, txBits);
  rc_write(CommandReg, 0x0C);          // Transceive
  rc_setbits(BitFramingReg, 0x80);     // StartSend

  uint16_t n = 3000;
  for (;;) {
    uint8_t irq = rc_read(ComIrqReg);
    if (irq & 0x30) break;             // RxIRq | IdleIRq
    if (irq & 0x01) return false;      // TimerIRq = timeout
    if (--n == 0)   return false;
  }
  if (rc_read(ErrorReg) & 0x13) return false;   // buffer ovf / parity / protocol

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

// REQA -> anticollision CL1 -> SELECT. UID de 4 octeti.
static bool rc_read_uid(uint8_t *uid) {
  uint8_t rx[5], len;

  rc_clrbits(CollReg, 0x80);
  uint8_t reqa = 0x26;
  len = 2;
  if (!rc_transceive(&reqa, 1, rx, &len, 7) || len != 2) return false;

  uint8_t ac[2] = {0x93, 0x20};
  len = 5;
  if (!rc_transceive(ac, 2, rx, &len, 0) || len != 5) return false;
  if ((rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) return false;   // BCC

  uint8_t sel[9] = {0x93, 0x70, rx[0], rx[1], rx[2], rx[3], rx[4], 0, 0};
  rc_crc(sel, 7, sel + 7);
  uint8_t sak[3]; len = 3;
  rc_transceive(sel, 9, sak, &len, 0);

  for (uint8_t i = 0; i < 4; i++) uid[i] = rx[i];
  return true;
}

// HLTA: cardul nu mai raspunde la REQA pana e scos din camp si readus
static void rc_halt() {
  uint8_t h[4] = {0x50, 0x00, 0, 0};
  rc_crc(h, 2, h + 2);
  rc_transceive(h, 4, 0, 0, 0);        // timeout asteptat = succes
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
  DDRD |= (1 << PD7);
  NRF_CE_L(); NRF_CSN_H();
  delay_ms(100);
  nrf_wreg(NRF_SETUP_AW, 0x03);          // adresa 5 octeti
  nrf_wreg(NRF_SETUP_RETR, 0x5F);        // 1500 us, 15 retransmisii
  nrf_wreg(NRF_RF_CH, 76);
  nrf_wreg(NRF_RF_SETUP, 0x06);          // 1 Mbps, 0 dBm
  nrf_wreg(NRF_EN_AA, 0x01);             // auto-ack pe pipe 0
  nrf_wreg(NRF_EN_RXADDR, 0x01);
  nrf_wreg_buf(NRF_TX_ADDR, RF_ADDR, 5);
  nrf_wreg_buf(NRF_RX_ADDR_P0, RF_ADDR, 5);   // necesar pentru ACK
  nrf_wreg(NRF_RX_PW_P0, 32);
  nrf_cmd(0xE1); nrf_cmd(0xE2);          // flush TX/RX
  nrf_wreg(NRF_STATUS, 0x70);            // sterge flag-uri
  nrf_wreg(NRF_CONFIG, 0x0E);            // PWR_UP, CRC 16 biti, PRIM_RX=0 (TX)
  delay_ms(5);
}

static bool nrf_send(const char *msg) {
  uint8_t buf[32];
  memset(buf, 0, sizeof(buf));
  for (uint8_t i = 0; i < 31 && msg[i]; i++) buf[i] = (uint8_t)msg[i];

  NRF_CSN_L();
  spi_xfer(0xA0);                        // W_TX_PAYLOAD
  for (uint8_t i = 0; i < 32; i++) spi_xfer(buf[i]);
  NRF_CSN_H();

  NRF_CE_H(); _delay_us(15); NRF_CE_L();

  uint8_t st = 0;
  for (uint16_t t = 0; t < 5000; t++) {  // max ~50 ms
    st = nrf_cmd(0xFF);                  // NOP -> citeste STATUS
    if (st & 0x30) break;                // TX_DS | MAX_RT
    _delay_us(10);
  }
  nrf_wreg(NRF_STATUS, 0x30);
  if (st & 0x10) { nrf_cmd(0xE1); return false; }   // MAX_RT: flush TX
  return (st & 0x20);
}

/* ===================== LOGICA ===================== */
static bool is_authorized(const uint8_t *uid) {
  for (uint8_t i = 0; i < N_AUTH; i++)
    if (memcmp(uid, AUTH_UIDS[i], 4) == 0) return true;
  return false;
}

static void uid_to_str(const uint8_t *uid, char *out) {   // "8A B2 C3 4E"
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
  TIMSK0 = 0;                // opreste intreruperea Timer0 (nefolosit), servo fara jitter
  DDRD |= (1 << PD5);        // buzzer
  BUZ_OFF();

  spi_init();
  twi_init();
  servo_init();
  sei();

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
    strcpy(msg, "ACCES PERMIS: "); strcat(msg, s);
    nrf_send(msg);
    beep(100);
    servo_set(SERVO_OPEN);
    delay_ms(3000);
    servo_set(SERVO_CLOSED);
    delay_ms(500);                       // timp sa ajunga servo la 0
  } else {
    lcd_print(0, " ACCES RESPINS!");
    lcd_print(1, "Card Neautorizat");
    strcpy(msg, "ALERTA INTRUS: "); strcat(msg, s);
    nrf_send(msg);
    beep_error();
    delay_ms(1000);
  }

  rc_halt();                             // nu mai citeste cardul pana e scos
  show_standby();
}
