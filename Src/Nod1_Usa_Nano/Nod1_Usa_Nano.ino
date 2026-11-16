/*
 * NOD 1 (Arduino Nano - ATmega328P) - Terminal Acces RFID + LCD1602 + Servo + Radio cu ACK Software
 * Conform Datasheet Atmel-7810 (ATmega328P) - Registi nativi AVR si constante simbolice
 *
 * Pinout Hardware:
 *   Port D: LED_VERDE=PD2 (D2), SERVO=PD3 (D3), LED_ROSU=PD4 (D4), BUZ=PD5 (D5), NRF_CE=PD7 (D7)
 *   Port B: NRF_CSN=PB0 (D8), RC_RST=PB1 (D9), RC_SS=PB2 (D10), MOSI=PB3 (D11), MISO=PB4 (D12), SCK=PB5 (D13)
 *   Port C: LCD_D4=PC0 (A0), LCD_D5=PC1 (A1), LCD_D6=PC2 (A2), LCD_D7=PC3 (A3), LCD_RS=PC4 (A4), LCD_E=PC5 (A5)
 */

#include <stdint.h>
#include <avr/io.h>        // Definitiile registrilor ATmega328P (PORTB, DDRB, SPCR, TCCR1B, UDR0 etc.)
#include <avr/interrupt.h> // Macro-uri ISR(), cli(), sei() (Atmel-7810 Cap. 11)

/* ==================== CONFIGURARE CEAS & COMUNICATIE UART0 ==================== */
#define F_CPU_HZ            16000000UL  // Frecventa cristalului extern pe Arduino Nano (16 MHz)
#define UART_BAUD_RATE      9600UL      // Rata de transfer seriala (9600 bps)
#define UART_UBRR_VAL       ((F_CPU_HZ / (16UL * UART_BAUD_RATE)) - 1) // 103 pentru 9600 bps @ 16 MHz (Tabel 19-1)

/* ==================== MAPARE PINI ATMEGA328P (PORTB, PORTC, PORTD) ==================== */
/* Port B (Digital D8..D13) */
#define PIN_NRF_CSN         PB0         // D8  - Chip Select Not pentru nRF24L01+
#define PIN_RC_RST          PB1         // D9  - Reset hardware pentru MFRC522
#define PIN_RC_SS           PB2         // D10 - Slave Select (SDA) pentru MFRC522 (si pin hardware SS)
#define PIN_SPI_MOSI        PB3         // D11 - Master Out Slave In (SPI)
#define PIN_SPI_MISO        PB4         // D12 - Master In Slave Out (SPI)
#define PIN_SPI_SCK         PB5         // D13 - Serial Clock (SPI)

/* Port C (Analog A0..A5 folositi ca I/O digital pentru LCD1602 4-bit) */
#define PIN_LCD_D4          PC0         // A0  - Linia de date D4 a LCD1602
#define PIN_LCD_D5          PC1         // A1  - Linia de date D5 a LCD1602
#define PIN_LCD_D6          PC2         // A2  - Linia de date D6 a LCD1602
#define PIN_LCD_D7          PC3         // A3  - Linia de date D7 a LCD1602
#define PIN_LCD_RS          PC4         // A4  - Register Select LCD (0=Comanda, 1=Date)
#define PIN_LCD_E           PC5         // A5  - Enable LCD (front descendent valideaza datele)
#define LCD_PORTC_MASK      0x3F        // Masca pentru primii 6 biti ai Portului C (PC0..PC5)
#define LCD_DATA_MASK       0x0F        // Masca pentru cei 4 biti de date ai LCD-ului (PC0..PC3)

/* Port D (Digital D2..D7) */
#define PIN_LED_VERDE       PD2         // D2  - LED Verde (indica acces permis / SAFE)
#define PIN_SERVO_PWM       PD3         // D3  - Semnal PWM software generat de Timer1 pentru SG90
#define PIN_LED_ROSU        PD4         // D4  - LED Rosu (indica alerta / INTRUS)
#define PIN_BUZZER          PD5         // D5  - Buzzer activ pentru feedback acustic
#define PIN_NRF_CE          PD7         // D7  - Chip Enable pentru activarea emisiei/receptiei nRF24L01+

/* ==================== MACRO-URI CONTROL PINI (MANIPULARE DIRECTA PORT) ==================== */
#define NRF_CSN_LOW()       (PORTB &= ~(1 << PIN_NRF_CSN))
#define NRF_CSN_HIGH()      (PORTB |=  (1 << PIN_NRF_CSN))
#define RC_RST_LOW()        (PORTB &= ~(1 << PIN_RC_RST))
#define RC_RST_HIGH()       (PORTB |=  (1 << PIN_RC_RST))
#define RC_SS_LOW()         (PORTB &= ~(1 << PIN_RC_SS))
#define RC_SS_HIGH()        (PORTB |=  (1 << PIN_RC_SS))

#define NRF_CE_LOW()        (PORTD &= ~(1 << PIN_NRF_CE))
#define NRF_CE_HIGH()       (PORTD |=  (1 << PIN_NRF_CE))
#define BUZZER_ON()         (PORTD |=  (1 << PIN_BUZZER))
#define BUZZER_OFF()        (PORTD &= ~(1 << PIN_BUZZER))
#define LED_VERDE_ON()      (PORTD |=  (1 << PIN_LED_VERDE))
#define LED_VERDE_OFF()     (PORTD &= ~(1 << PIN_LED_VERDE))
#define LED_ROSU_ON()       (PORTD |=  (1 << PIN_LED_ROSU))
#define LED_ROSU_OFF()      (PORTD &= ~(1 << PIN_LED_ROSU))

/* ==================== PARAMETRI TIMER1 & SERVOMOTOR SG90 ==================== */
/* Cu prescaler = 8 @ 16 MHz => 1 tick Timer1 = 0.5 us (2 MHz) */
#define TIMER1_TOP_20MS     39999U      // Perioada PWM 20 ms (50 Hz): 40000 ticks * 0.5 us - 1
#define SERVO_POS_CLOSED    1088U       // Puls ~0.544 ms -> pozitie inchis (0 grade)
#define SERVO_POS_OPEN      2944U       // Puls ~1.472 ms -> pozitie deschis (~90 grade)

/* ==================== CONSTANTE DRIVER LCD1602 (HD44780) ==================== */
#define LCD_CMD_CLEAR           0x01    // Sterge tot ecranul si muta cursorul la (0,0)
#define LCD_CMD_HOME            0x02    // Returneaza cursorul la pozitia initiala
#define LCD_CMD_ENTRY_INC       0x06    // Incrementare automata a cursorului spre dreapta
#define LCD_CMD_DISPLAY_ON      0x0C    // Afisaj pornit, cursor invizibil, fara clipire
#define LCD_CMD_FUNC_4BIT_2L    0x28    // Interfata pe 4 biti, 2 linii, font 5x8 puncte
#define LCD_INIT_NIBBLE_8BIT    0x03    // Secventa hardware de reset in mod 8-bit
#define LCD_INIT_NIBBLE_4BIT    0x02    // Comutare in interfata pe 4 biti
#define LCD_DDRAM_LINE1_BASE    0x80    // Adresa DDRAM de start pentru linia 1
#define LCD_DDRAM_LINE2_BASE    0xC0    // Adresa DDRAM de start pentru linia 2

/* ==================== REGISTRI SI COMENZI RFID MFRC522 ==================== */
#define RC522_REG_COMMAND       0x01    // Porneste/opreste executia comenzilor interne
#define RC522_REG_COM_IRQ       0x04    // Flag-uri de intrerupere comunicatie
#define RC522_REG_DIV_IRQ       0x05    // Flag-uri de intrerupere auxiliare (ex. CRC gata)
#define RC522_REG_ERROR         0x06    // Biti de eroare (coliziune, paritate, buffer, protocol)
#define RC522_REG_FIFO_DATA     0x09    // Intrare/iesire date in bufferul FIFO de 64 octeti
#define RC522_REG_FIFO_LEVEL    0x0A    // Numarul de octeti aflati curent in FIFO
#define RC522_REG_BIT_FRAMING   0x0D    // Ajustari la nivel de bit pentru transmisie/receptie
#define RC522_REG_COLL          0x0E    // Detectie coliziuni RF
#define RC522_REG_MODE          0x11    // Moduri generale de transmisie/receptie
#define RC522_REG_TX_CONTROL    0x14    // Controlul pinilor de antena TX1 si TX2
#define RC522_REG_TX_ASK        0x15    // Setare modulatie 100% ASK
#define RC522_REG_CRC_RES_H     0x21    // Octetul superior al rezultatului CRC
#define RC522_REG_CRC_RES_L     0x22    // Octetul inferior al rezultatului CRC
#define RC522_REG_T_MODE        0x2A    // Setari timer intern MFRC522
#define RC522_REG_T_PRESCALER   0x2B    // Prescaler timer intern MFRC522
#define RC522_REG_T_RELOAD_H    0x2C    // Valoare reincarcare timer (octet superior)
#define RC522_REG_T_RELOAD_L    0x2D    // Valoare reincarcare timer (octet inferior)

#define RC522_CMD_IDLE          0x00    // Opreste comanda curenta
#define RC522_CMD_CALC_CRC      0x03    // Activeaza coprocesorul hardware CRC
#define RC522_CMD_TRANSCEIVE    0x0C    // Transmite date din FIFO si activeaza receptia
#define RC522_CMD_SOFT_RESET    0x0F    // Resetare software MFRC522
#define RC522_FIFO_FLUSH_BIT    0x80    // Goleste bufferul FIFO intern
#define RC522_ERROR_MASK        0x13    // Masca erori critice (BufferOvfl | ParityErr | ProtocolErr)

/* Comenzi standard ISO/IEC 14443-3A pentru carduri MIFARE */
#define PICC_CMD_REQA           0x26    // Request Command Type A (7 biti)
#define PICC_CMD_ANTICOLL_CL1   0x93    // Cascade Level 1 (Anti-coliziune / Selectie)
#define PICC_CMD_ANTICOLL_NVB   0x20    // Number of Valid Bits = 2 octeti
#define PICC_CMD_SELECT_NVB     0x70    // Number of Valid Bits = 7 octeti
#define PICC_CMD_HLTA           0x50    // Halt Command Type A (pune cardul in repaus)

/* ==================== REGISTRI SI COMENZI RADIO nRF24L01+ ==================== */
#define NRF_REG_CONFIG          0x00    // Configurare generala (PWR_UP, PRIM_RX, CRC)
#define NRF_REG_EN_AA           0x01    // Activare Auto-Acknowledgement
#define NRF_REG_EN_RXADDR       0x02    // Activare adrese RX (Pipe 0..5)
#define NRF_REG_SETUP_AW        0x03    // Latime adresa (0x03 = 5 octeti)
#define NRF_REG_SETUP_RETR      0x04    // Configurare retrimiteri automate
#define NRF_REG_RF_CH           0x05    // Canalul de frecventa RF (2400 + CH MHz)
#define NRF_REG_RF_SETUP        0x06    // Rata de date si puterea RF
#define NRF_REG_STATUS          0x07    // Registru de stare (RX_DR, TX_DS, MAX_RT)
#define NRF_REG_RX_ADDR_P0      0x0A    // Adresa de receptie pentru Pipe 0
#define NRF_REG_TX_ADDR         0x10    // Adresa destinatie pentru transmisie
#define NRF_REG_RX_PW_P0        0x11    // Dimensiune payload pe Pipe 0 (in octeti)
#define NRF_REG_FIFO_STATUS     0x17    // Stare FIFO TX/RX (RX_EMPTY = bit 0)

#define NRF_CMD_R_REGISTER      0x00    // Citire registru (masca 0x1F)
#define NRF_CMD_W_REGISTER      0x20    // Scriere registru (masca 0x1F)
#define NRF_CMD_R_RX_PAYLOAD    0x61    // Citire date receptionate din FIFO RX
#define NRF_CMD_W_TX_PAYLOAD    0xA0    // Scriere date de transmis in FIFO TX
#define NRF_CMD_FLUSH_TX        0xE1    // Goleste FIFO TX
#define NRF_CMD_FLUSH_RX        0xE2    // Goleste FIFO RX
#define NRF_CMD_NOP             0xFF    // No Operation (returneaza registrul STATUS)

#define NRF_CFG_POWER_DOWN      0x00    // Cip oprit (consum minim)
#define NRF_CFG_TX_MODE         0x0E    // PWR_UP=1, PRIM_RX=0 (TX), EN_CRC=1, CRCO=1 (CRC 16-bit)
#define NRF_CFG_RX_MODE         0x0F    // PWR_UP=1, PRIM_RX=1 (RX), EN_CRC=1, CRCO=1 (CRC 16-bit)
#define NRF_RF_CHANNEL          76      // Canal 76 -> 2476 MHz
#define NRF_RF_250KBPS_0DBM     0x26    // RF_DR_LOW=1 (250 kbps) + RF_PWR=11 (0 dBm)
#define NRF_PAYLOAD_SIZE        32      // Dimensiune fixa pachet radio (32 octeti)
#define NRF_ADDR_WIDTH          5       // Lungime adresa radio (5 octeti)
#define NRF_STATUS_IRQ_CLEAR    0x70    // Sterge flag-urile RX_DR (bit 6), TX_DS (bit 5), MAX_RT (bit 4)
#define NRF_STATUS_RX_DR        0x40    // Flag: Date noi disponibile in FIFO RX
#define NRF_STATUS_TX_DS        0x20    // Flag: Pachet transmis cu succes
#define NRF_FIFO_RX_EMPTY       0x01    // Bit 0 din FIFO_STATUS: 1 = FIFO RX gol

/* ==================== CONSTANTE APLICATIE & TIMPI ==================== */
#define ACK_WAIT_STEPS          600U    // 600 pasi * 100 us = ~60 ms fereastra asteptare ecou ACK
#define ACK_RETRY_DELAY_MS      40U     // Pauza intre retrimiterile pachetului neconfirmat
#define DOOR_OPEN_TIME_MS       2500U   // Cat timp ramane usa descuiata la acces permis
#define INTRUDER_LED_TIME_MS    1500U   // Cat timp ramane aprins LED-ul rosu la intrus

#define UID_LENGTH          4U          // Lungimea UID-ului standard MIFARE Classic (4 octeti)
static const uint8_t VALID_UID[UID_LENGTH] = {0x76, 0x4B, 0x2E, 0x03};
static const uint8_t RF_ADDR[NRF_ADDR_WIDTH] = {'0', '0', '0', '0', '1'};

/* ==================== FUNCTII INTARZIERE & SIRURI ==================== */
static void delay_us(uint16_t n) {
  while (n--) {
    /* 12 instructiuni NOP + bucla = ~1 us @ 16 MHz */
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}

static void delay_ms(uint16_t n) {
  while (n--) delay_us(1000);
}

static void mem_zero(uint8_t *p, uint8_t n) {
  while (n--) *p++ = 0;
}

static bool str_eq(const char *a, const char *b) {
  while (*a && *b) {
    if (*a++ != *b++) return false;
  }
  return (*a == *b);
}

/* ==================== DRIVER LCD1602 PARALEL (4-BIT, PORTC) ==================== */
static void lcd_pulse_enable(void) {
  PORTC |= (1 << PIN_LCD_E);
  delay_us(2);
  PORTC &= ~(1 << PIN_LCD_E);
  delay_us(50);
}

static void lcd_write_nibble(uint8_t nib) {
  PORTC = (PORTC & ~LCD_DATA_MASK) | (nib & LCD_DATA_MASK);
  lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t val, bool is_data) {
  if (is_data) PORTC |=  (1 << PIN_LCD_RS);
  else         PORTC &= ~(1 << PIN_LCD_RS);
  lcd_write_nibble(val >> 4);
  lcd_write_nibble(val & LCD_DATA_MASK);
}

static void lcd_cmd(uint8_t cmd) {
  lcd_send_byte(cmd, false);
  if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME) delay_ms(2);
}

static void lcd_char(char c) {
  lcd_send_byte((uint8_t)c, true);
}

static void lcd_str(const char *s) {
  while (*s) lcd_char(*s++);
}

static void lcd_hex(uint8_t v) {
  const char *hx = "0123456789ABCDEF";
  lcd_char(hx[v >> 4]);
  lcd_char(hx[v & 0x0F]);
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
  uint8_t addr = (row == 0) ? (LCD_DDRAM_LINE1_BASE | col) : (LCD_DDRAM_LINE2_BASE | col);
  lcd_cmd(addr);
}

static void lcd_show_lines(const char *l1, const char *l2) {
  lcd_cmd(LCD_CMD_CLEAR);
  lcd_set_cursor(0, 0);
  lcd_str(l1);
  lcd_set_cursor(0, 1);
  lcd_str(l2);
}

static void lcd_init(void) {
  DDRC  |= LCD_PORTC_MASK;  // Configureaza PC0..PC5 ca iesiri digitale (Atmel-7810 Cap. 13.2.1)
  PORTC &= ~LCD_PORTC_MASK; // Initializeaza pinii pe LOW
  delay_ms(50);             // Stabilizare alimentare LCD (>40 ms)

  lcd_write_nibble(LCD_INIT_NIBBLE_8BIT); delay_ms(5);
  lcd_write_nibble(LCD_INIT_NIBBLE_8BIT); delay_us(150);
  lcd_write_nibble(LCD_INIT_NIBBLE_8BIT); delay_us(150);
  lcd_write_nibble(LCD_INIT_NIBBLE_4BIT); delay_us(150);

  lcd_cmd(LCD_CMD_FUNC_4BIT_2L);
  lcd_cmd(LCD_CMD_DISPLAY_ON);
  lcd_cmd(LCD_CMD_ENTRY_INC);
  lcd_cmd(LCD_CMD_CLEAR);
}

/* ==================== DRIVER UART0 (ATmega328P Cap. 19) ==================== */
static void uart_init(void) {
  UBRR0  = UART_UBRR_VAL;                         // Seteaza baud rate 9600 (UBRR0H:UBRR0L)
  UCSR0B = (1 << TXEN0);                          // Activeaza transmitatorul UART0 (bit TXEN0)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);         // Format cadru: 8 biti date, fara paritate, 1 bit stop
}

static void uart_char(char c) {
  while (!(UCSR0A & (1 << UDRE0)));               // Asteapta golirea bufferului de transmisie (UDRE0=1)
  UDR0 = c;
}

static void uart_str(const char *s) {
  while (*s) uart_char(*s++);
}

static void uart_hex(uint8_t v) {
  const char *hx = "0123456789ABCDEF";
  uart_char(hx[v >> 4]);
  uart_char(hx[v & 0x0F]);
}

static void uart_ln(const char *s) {
  uart_str(s);
  uart_str("\r\n");
}

static void uart_dec(uint16_t v) {
  char t[5];
  uint8_t n = 0;
  do { t[n++] = '0' + (v % 10); v /= 10; } while (v);
  while (n) uart_char(t[--n]);
}

/* ==================== DRIVER SPI MASTER (ATmega328P Cap. 18) ==================== */
static void spi_init(void) {
  DDRB  |= (1 << PIN_NRF_CSN) | (1 << PIN_RC_RST) | (1 << PIN_RC_SS) | (1 << PIN_SPI_MOSI) | (1 << PIN_SPI_SCK);
  DDRB  &= ~(1 << PIN_SPI_MISO);                  // MISO setat ca intrare
  PORTB |= (1 << PIN_NRF_CSN) | (1 << PIN_RC_SS); // Ambele cipuri SPI deselectate (HIGH)
  SPCR   = (1 << SPE) | (1 << MSTR) | (1 << SPR0);// SPI Enable, Master, ceas f_osc/16 = 1 MHz (Tabel 18-5)
}

static uint8_t spi_xfer(uint8_t b) {
  SPDR = b;                                       // Incarca octetul in registrul de date SPI
  while (!(SPSR & (1 << SPIF)));                  // Asteapta finalizarea transferului (flag SPIF=1)
  return SPDR;
}

/* ==================== DRIVER TIMER1 PWM SERVO (ATmega328P Cap. 15) ==================== */
static volatile uint16_t servo_ticks = SERVO_POS_CLOSED;

ISR(TIMER1_COMPA_vect) {
  OCR1B  = servo_ticks;                           // Actualizeaza durata pulsului pentru ciclul curent
  PORTD |= (1 << PIN_SERVO_PWM);                  // Front crescator PWM la inceputul perioadei de 20 ms
}

ISR(TIMER1_COMPB_vect) {
  PORTD &= ~(1 << PIN_SERVO_PWM);                 // Front descrescator PWM la atingerea duratei dorite
}

static void servo_init(void) {
  DDRD  |= (1 << PIN_SERVO_PWM);                  // Pinul D3 ca iesire
  TCCR1A = 0x00;                                  // Mod normal port, CTC controlat prin TCCR1B
  TCCR1B = (1 << WGM12) | (1 << CS11);            // Mod CTC (TOP = OCR1A), prescaler = 8 (Tabel 15-4, 15-5)
  OCR1A  = TIMER1_TOP_20MS;                       // Perioada 20 ms
  OCR1B  = SERVO_POS_CLOSED;                      // Pozitie initiala inchis
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);         // Activeaza intreruperile Compare Match A si B
}

static void servo_set(uint16_t t) {
  uint8_t sreg_save = SREG;
  cli();                                          // Sectiune critica pentru scriere atomica pe 16 biti
  servo_ticks = t;
  SREG = sreg_save;
}

static void beep(uint16_t ms) {
  BUZZER_ON();
  delay_ms(ms);
  BUZZER_OFF();
}

/* ==================== DRIVER RFID MFRC522 ==================== */
static void rc_write(uint8_t reg, uint8_t val) {
  RC_SS_LOW();
  spi_xfer((reg << 1) & 0x7E);
  spi_xfer(val);
  RC_SS_HIGH();
}

static uint8_t rc_read(uint8_t reg) {
  RC_SS_LOW();
  spi_xfer(((reg << 1) & 0x7E) | 0x80);
  uint8_t v = spi_xfer(0x00);
  RC_SS_HIGH();
  return v;
}

static void rc_setbits(uint8_t r, uint8_t m) { rc_write(r, rc_read(r) | m); }
static void rc_clrbits(uint8_t r, uint8_t m) { rc_write(r, rc_read(r) & ~m); }

static void rc_init(void) {
  RC_RST_LOW(); delay_us(10); RC_RST_HIGH(); delay_ms(50);
  rc_write(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET); delay_ms(50);
  rc_write(RC522_REG_T_MODE, 0x80);
  rc_write(RC522_REG_T_PRESCALER, 0xA9);
  rc_write(RC522_REG_T_RELOAD_H, 0x03);
  rc_write(RC522_REG_T_RELOAD_L, 0xE8);
  rc_write(RC522_REG_TX_ASK, 0x40);
  rc_write(RC522_REG_MODE, 0x3D);
  rc_setbits(RC522_REG_TX_CONTROL, 0x03);
}

static bool rc_transceive(const uint8_t *tx, uint8_t txLen, uint8_t *rx, uint8_t *rxLen, uint8_t txBits) {
  rc_write(RC522_REG_COMMAND, RC522_CMD_IDLE);
  rc_write(RC522_REG_COM_IRQ, 0x7F);
  rc_write(RC522_REG_FIFO_LEVEL, RC522_FIFO_FLUSH_BIT);

  for (uint8_t i = 0; i < txLen; i++) rc_write(RC522_REG_FIFO_DATA, tx[i]);
  rc_write(RC522_REG_BIT_FRAMING, txBits);
  rc_write(RC522_REG_COMMAND, RC522_CMD_TRANSCEIVE);
  rc_setbits(RC522_REG_BIT_FRAMING, 0x80);

  uint16_t n = 3000;
  for (;;) {
    uint8_t irq = rc_read(RC522_REG_COM_IRQ);
    if (irq & 0x30) break;
    if (irq & 0x01) return false;
    if (--n == 0) return false;
  }

  if (rc_read(RC522_REG_ERROR) & RC522_ERROR_MASK) return false;

  if (rx && rxLen) {
    uint8_t len = rc_read(RC522_REG_FIFO_LEVEL);
    if (len > *rxLen) return false;
    *rxLen = len;
    for (uint8_t i = 0; i < len; i++) rx[i] = rc_read(RC522_REG_FIFO_DATA);
  }
  return true;
}

static void rc_crc(const uint8_t *d, uint8_t n, uint8_t *out) {
  rc_write(RC522_REG_COMMAND, RC522_CMD_IDLE);
  rc_write(RC522_REG_DIV_IRQ, 0x04);
  rc_write(RC522_REG_FIFO_LEVEL, RC522_FIFO_FLUSH_BIT);
  for (uint8_t i = 0; i < n; i++) rc_write(RC522_REG_FIFO_DATA, d[i]);
  rc_write(RC522_REG_COMMAND, RC522_CMD_CALC_CRC);

  uint16_t t = 5000;
  while (!(rc_read(RC522_REG_DIV_IRQ) & 0x04) && --t);
  rc_write(RC522_REG_COMMAND, RC522_CMD_IDLE);
  out[0] = rc_read(RC522_REG_CRC_RES_L);
  out[1] = rc_read(RC522_REG_CRC_RES_H);
}

static bool rc_read_uid(uint8_t *uid) {
  uint8_t rx[5], len;
  rc_clrbits(RC522_REG_COLL, 0x80);

  uint8_t reqa = PICC_CMD_REQA;
  len = 2;
  if (!rc_transceive(&reqa, 1, rx, &len, 7) || len != 2) return false;

  uint8_t ac[2] = {PICC_CMD_ANTICOLL_CL1, PICC_CMD_ANTICOLL_NVB};
  len = 5;
  if (!rc_transceive(ac, 2, rx, &len, 0) || len != 5) return false;
  if ((rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) return false;

  uint8_t sel[9] = {PICC_CMD_ANTICOLL_CL1, PICC_CMD_SELECT_NVB, rx[0], rx[1], rx[2], rx[3], rx[4], 0, 0};
  rc_crc(sel, 7, sel + 7);
  uint8_t sak[3];
  len = 3;
  rc_transceive(sel, 9, sak, &len, 0);

  for (uint8_t i = 0; i < 4; i++) uid[i] = rx[i];
  return true;
}

static void rc_halt(void) {
  uint8_t h[4] = {PICC_CMD_HLTA, 0x00, 0, 0};
  rc_crc(h, 2, h + 2);
  rc_transceive(h, 4, 0, 0, 0);
  rc_write(RC522_REG_COMMAND, RC522_CMD_IDLE);
}

static bool is_valid_card(const uint8_t *uid) {
  for (uint8_t i = 0; i < UID_LENGTH; i++) {
    if (uid[i] != VALID_UID[i]) return false; // Orice octet diferit -> respins imediat (INTRUS)
  }
  return true; // Doar daca toti cei 4 octeti coincid cu 76 4B 2E 03 -> SAFE
}

/* ==================== DRIVER RADIO nRF24L01+ ==================== */
static void nrf_wreg(uint8_t reg, uint8_t val) {
  NRF_CSN_LOW();
  spi_xfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  spi_xfer(val);
  NRF_CSN_HIGH();
}

static void nrf_wreg_buf(uint8_t reg, const uint8_t *b, uint8_t n) {
  NRF_CSN_LOW();
  spi_xfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < n; i++) spi_xfer(b[i]);
  NRF_CSN_HIGH();
}

static uint8_t nrf_rreg(uint8_t reg) {
  NRF_CSN_LOW();
  spi_xfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
  uint8_t v = spi_xfer(NRF_CMD_NOP);
  NRF_CSN_HIGH();
  return v;
}

static uint8_t nrf_cmd(uint8_t c) {
  NRF_CSN_LOW();
  uint8_t s = spi_xfer(c);
  NRF_CSN_HIGH();
  return s;
}

static void nrf_init(void) {
  DDRD |= (1 << PIN_NRF_CE);
  NRF_CE_LOW();
  NRF_CSN_HIGH();
  delay_ms(100);

  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_POWER_DOWN);
  delay_ms(5);
  nrf_wreg(NRF_REG_EN_AA, 0x00);                  // Dezactiveaza Auto-ACK hardware
  nrf_wreg(NRF_REG_EN_RXADDR, 0x01);              // Activeaza Pipe 0
  nrf_wreg(NRF_REG_SETUP_AW, 0x03);               // Latime adresa = 5 octeti
  nrf_wreg(NRF_REG_SETUP_RETR, 0x00);             // Dezactiveaza retrimiterile hardware
  nrf_wreg(NRF_REG_RF_CH, NRF_RF_CHANNEL);        // Canal 76 (2476 MHz)
  nrf_wreg(NRF_REG_RF_SETUP, NRF_RF_250KBPS_0DBM);// 250 kbps, 0 dBm
  nrf_wreg_buf(NRF_REG_TX_ADDR, RF_ADDR, NRF_ADDR_WIDTH);
  nrf_wreg_buf(NRF_REG_RX_ADDR_P0, RF_ADDR, NRF_ADDR_WIDTH);
  nrf_wreg(NRF_REG_RX_PW_P0, NRF_PAYLOAD_SIZE);

  nrf_cmd(NRF_CMD_FLUSH_TX);
  nrf_cmd(NRF_CMD_FLUSH_RX);
  nrf_wreg(NRF_REG_STATUS, NRF_STATUS_IRQ_CLEAR);
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_TX_MODE);
  delay_ms(5);
}

static bool nrf_send_and_wait_ack(const char *msg) {
  uint8_t buf[NRF_PAYLOAD_SIZE];
  mem_zero(buf, NRF_PAYLOAD_SIZE);
  for (uint8_t i = 0; i < (NRF_PAYLOAD_SIZE - 1) && msg[i]; i++) buf[i] = (uint8_t)msg[i];

  NRF_CE_LOW();
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_TX_MODE);
  delay_us(150);
  nrf_cmd(NRF_CMD_FLUSH_TX);
  nrf_wreg(NRF_REG_STATUS, NRF_STATUS_IRQ_CLEAR);

  NRF_CSN_LOW();
  spi_xfer(NRF_CMD_W_TX_PAYLOAD);
  for (uint8_t i = 0; i < NRF_PAYLOAD_SIZE; i++) spi_xfer(buf[i]);
  NRF_CSN_HIGH();

  NRF_CE_HIGH(); delay_us(20); NRF_CE_LOW();

  for (uint16_t t = 0; t < 5000; t++) {
    if (nrf_cmd(NRF_CMD_NOP) & NRF_STATUS_TX_DS) break;
    delay_us(10);
  }
  nrf_wreg(NRF_REG_STATUS, NRF_STATUS_IRQ_CLEAR);

  /* Comuta imediat pe receptie pentru a prinde ecoul ACK de la Nod 2 */
  nrf_cmd(NRF_CMD_FLUSH_RX);
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_RX_MODE);
  NRF_CE_HIGH();

  bool ack_ok = false;
  for (uint16_t t = 0; t < ACK_WAIT_STEPS; t++) {
    uint8_t st   = nrf_cmd(NRF_CMD_NOP);
    uint8_t fifo = nrf_rreg(NRF_REG_FIFO_STATUS);
    if ((st & NRF_STATUS_RX_DR) || !(fifo & NRF_FIFO_RX_EMPTY)) {
      char rx_msg[NRF_PAYLOAD_SIZE + 1];
      NRF_CSN_LOW();
      spi_xfer(NRF_CMD_R_RX_PAYLOAD);
      for (uint8_t i = 0; i < NRF_PAYLOAD_SIZE; i++) rx_msg[i] = (char)spi_xfer(NRF_CMD_NOP);
      NRF_CSN_HIGH();
      rx_msg[NRF_PAYLOAD_SIZE] = '\0';
      nrf_wreg(NRF_REG_STATUS, NRF_STATUS_RX_DR);

      if (str_eq(rx_msg, msg)) {
        ack_ok = true;
        break;
      }
    }
    delay_us(100);
  }

  NRF_CE_LOW();
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_TX_MODE);
  return ack_ok;
}

static void nrf_send_reliable(const char *msg, bool silent) {
  uint16_t incercari = 0;
  while (!nrf_send_and_wait_ack(msg)) {
    incercari++;
    delay_ms(ACK_RETRY_DELAY_MS);
  }
  if (!silent) {
    uart_str("    [ACK PRIMIT] Mesaj confirmat dupa ");
    uart_dec(incercari + 1);
    uart_ln(" incercari.");
  }
}

/* ==================== PROGRAM PRINCIPAL ==================== */
void setup(void) {
  TIMSK0 = 0x00;                                  // Dezactiveaza intreruperile Timer0 Arduino pentru jitter zero pe servo
  DDRD  |= (1 << PIN_BUZZER) | (1 << PIN_LED_VERDE) | (1 << PIN_LED_ROSU);
  BUZZER_OFF();
  LED_VERDE_OFF();
  LED_ROSU_OFF();

  lcd_init();
  lcd_show_lines("Control Acces", "Initializare...");

  uart_init();
  spi_init();
  servo_init();
  sei();                                          // Activeaza intreruperile globale (SREG bit I = 1)
  delay_ms(400);

  uart_ln("\r\n=== NOD 1: SISTEM CONTROL ACCES ===");
  LED_VERDE_ON(); LED_ROSU_ON();
  beep(100); delay_ms(100); beep(100);
  LED_VERDE_OFF(); LED_ROSU_OFF();

  rc_init();
  nrf_init();

  uint8_t cfg = nrf_rreg(NRF_REG_CONFIG);
  if (cfg == NRF_CFG_TX_MODE) uart_ln("[OK] Hardware SPI & Radio gata.");
  else uart_ln("[EROARE] Verifica conexiunile SPI ale antenei!");

  lcd_show_lines("Conectare RF...", "Astept Nod 2");
  uart_ln("[...] Astept conexiunea cu receptorul (Nod 2)...");
  nrf_send_reliable("Alex e tare", true);
  uart_ln("[CONECTAT] Legatura radio confirmata cu Nod 2!\r\n");

  LED_VERDE_ON(); beep(80); delay_ms(150); LED_VERDE_OFF();
  lcd_show_lines("Sistem Pregatit", "Apropiati cardul");
}

void loop(void) {
  uint8_t uid[4];
  if (!rc_read_uid(uid)) {
    delay_ms(50);
    return;
  }

  bool safe = is_valid_card(uid);
  const char *prefix = safe ? "ACCES PERMIS: " : "ALERTA INTRUS: ";
  char msg[NRF_PAYLOAD_SIZE];
  uint8_t k = 0;
  while (prefix[k]) { msg[k] = prefix[k]; k++; }

  const char *hx = "0123456789ABCDEF";
  uart_str("Card detectat UID: ");
  for (uint8_t i = 0; i < 4; i++) {
    uart_hex(uid[i]);
    if (i < 3) uart_char(' ');
    msg[k++] = hx[uid[i] >> 4];
    msg[k++] = hx[uid[i] & 0x0F];
  }
  msg[k] = '\0';
  uart_ln("");

  if (safe) {
    uart_ln(" -> [SAFE] Acces permis! Deschid usa...");
    lcd_cmd(LCD_CMD_CLEAR);
    lcd_set_cursor(0, 0); lcd_str("ACCES PERMIS");
    lcd_set_cursor(0, 1); lcd_str("UID: ");
    for (uint8_t i = 0; i < 4; i++) { lcd_hex(uid[i]); if (i < 3) lcd_char(' '); }

    LED_ROSU_OFF(); LED_VERDE_ON();
    beep(120);
    servo_set(SERVO_POS_OPEN);
    nrf_send_reliable(msg, false);
    delay_ms(DOOR_OPEN_TIME_MS);
    servo_set(SERVO_POS_CLOSED);
    LED_VERDE_OFF();
  } else {
    uart_ln(" -> [INTRUS] Card neautorizat! Trimit alerta...");
    lcd_cmd(LCD_CMD_CLEAR);
    lcd_set_cursor(0, 0); lcd_str("ACCES RESPINS!");
    lcd_set_cursor(0, 1); lcd_str("UID: ");
    for (uint8_t i = 0; i < 4; i++) { lcd_hex(uid[i]); if (i < 3) lcd_char(' '); }

    LED_VERDE_OFF(); LED_ROSU_ON();
    beep(100); delay_ms(80); beep(100); delay_ms(80); beep(100);
    nrf_send_reliable(msg, false);
    delay_ms(INTRUDER_LED_TIME_MS);
    LED_ROSU_OFF();
  }

  lcd_show_lines("Sistem Pregatit", "Apropiati cardul");
  rc_halt();
  delay_ms(300);
}