/*
 * NOD 2 (Arduino Uno - ATmega328P) - Receptor Radio + Ecoul ACK + Alarma Buzzer
 * Conform Datasheet Atmel-7810 (ATmega328P) - Registi nativi AVR si constante simbolice
 *
 * Pinout Hardware:
 *   Port D: BUZ=PD5 (D5), NRF_CE=PD7 (D7)
 *   Port B: NRF_CSN=PB0 (D8), HW_SS=PB2 (D10), MOSI=PB3 (D11), MISO=PB4 (D12), SCK=PB5 (D13)
 */

#include <stdint.h>
#include <avr/io.h>        // Definitiile registrilor ATmega328P (PORTB, DDRB, SPCR, UDR0 etc.)

/* ==================== CONFIGURARE CEAS & COMUNICATIE UART0 ==================== */
#define F_CPU_HZ            16000000UL  // Frecventa cristalului extern pe Arduino Uno (16 MHz)
#define UART_BAUD_RATE      9600UL      // Rata de transfer seriala (9600 bps)
#define UART_UBRR_VAL       ((F_CPU_HZ / (16UL * UART_BAUD_RATE)) - 1) // 103 pentru 9600 bps @ 16 MHz (Tabel 19-1)

/* ==================== MAPARE PINI ATMEGA328P (PORTB, PORTD) ==================== */
/* Port B (Digital D8..D13) */
#define PIN_NRF_CSN         PB0         // D8  - Chip Select Not pentru nRF24L01+
#define PIN_SPI_HW_SS       PB2         // D10 - Pin hardware SS (tinut OUTPUT HIGH pentru mod Master SPI - Cap. 18.3.2)
#define PIN_SPI_MOSI        PB3         // D11 - Master Out Slave In (SPI)
#define PIN_SPI_MISO        PB4         // D12 - Master In Slave Out (SPI)
#define PIN_SPI_SCK         PB5         // D13 - Serial Clock (SPI)

/* Port D (Digital D5, D7) */
#define PIN_BUZZER          PD5         // D5  - Buzzer activ pentru alarma si confirmari
#define PIN_NRF_CE          PD7         // D7  - Chip Enable pentru activarea emisiei/receptiei nRF24L01+

/* ==================== MACRO-URI CONTROL PINI ==================== */
#define NRF_CSN_LOW()       (PORTB &= ~(1 << PIN_NRF_CSN))
#define NRF_CSN_HIGH()      (PORTB |=  (1 << PIN_NRF_CSN))
#define NRF_CE_LOW()        (PORTD &= ~(1 << PIN_NRF_CE))
#define NRF_CE_HIGH()       (PORTD |=  (1 << PIN_NRF_CE))
#define BUZZER_ON()         (PORTD |=  (1 << PIN_BUZZER))
#define BUZZER_OFF()        (PORTD &= ~(1 << PIN_BUZZER))

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

#define NRF_CFG_POWER_DOWN      0x00    // Cip oprit
#define NRF_CFG_TX_MODE         0x0E    // PWR_UP=1, PRIM_RX=0 (TX), EN_CRC=1, CRCO=1 (CRC 16-bit)
#define NRF_CFG_RX_MODE         0x0F    // PWR_UP=1, PRIM_RX=1 (RX), EN_CRC=1, CRCO=1 (CRC 16-bit)
#define NRF_RF_CHANNEL          76      // Canal 76 -> 2476 MHz
#define NRF_RF_250KBPS_0DBM     0x26    // RF_DR_LOW=1 (250 kbps) + RF_PWR=11 (0 dBm)
#define NRF_PAYLOAD_SIZE        32      // Dimensiune fixa pachet radio (32 octeti)
#define NRF_ADDR_WIDTH          5       // Lungime adresa radio (5 octeti)
#define NRF_STATUS_IRQ_CLEAR    0x70    // Sterge flag-urile RX_DR, TX_DS, MAX_RT
#define NRF_STATUS_RX_DR        0x40    // Flag: Date noi disponibile in FIFO RX
#define NRF_STATUS_TX_DS        0x20    // Flag: Pachet transmis cu succes
#define NRF_FIFO_RX_EMPTY       0x01    // Bit 0 din FIFO_STATUS: 1 = FIFO RX gol

/* ==================== CONSTANTE APLICATIE & ALARMA ==================== */
#define ACK_TURNAROUND_MS       3U      // Pauza (3 ms) pentru ca Nod 1 sa comute din TX in RX
#define SIREN_CYCLES            10U     // Numar impulsuri sirena la detectie intrus (10 * 300 ms = 3 s)
#define SIREN_HALF_PERIOD_MS    150U    // Durata ON/OFF a unui impuls de sirena

static const uint8_t RF_ADDR[NRF_ADDR_WIDTH] = {'0', '0', '0', '0', '1'};

/* ==================== FUNCTII INTARZIERE & SIRURI ==================== */
static void delay_us(uint16_t n) {
  while (n--) {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n nop\n"
                         "nop\n nop\n nop\n nop\n nop\n nop\n");
  }
}

static void delay_ms(uint16_t n) {
  while (n--) delay_us(1000);
}

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

/* ==================== DRIVER UART0 (ATmega328P Cap. 19) ==================== */
static void uart_init(void) {
  UBRR0  = UART_UBRR_VAL;                         // Baud rate 9600 @ 16 MHz
  UCSR0B = (1 << TXEN0);                          // Activeaza transmitatorul UART0
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);         // 8 biti date, fara paritate, 1 bit stop
}

static void uart_char(char c) {
  while (!(UCSR0A & (1 << UDRE0)));               // Asteapta golirea registrului UDR0
  UDR0 = c;
}

static void uart_str(const char *s) {
  while (*s) uart_char(*s++);
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
  DDRB  |= (1 << PIN_NRF_CSN) | (1 << PIN_SPI_HW_SS) | (1 << PIN_SPI_MOSI) | (1 << PIN_SPI_SCK);
  DDRB  &= ~(1 << PIN_SPI_MISO);                  // MISO setat ca intrare
  PORTB |= (1 << PIN_NRF_CSN) | (1 << PIN_SPI_HW_SS); // CSN si HW_SS tinute HIGH
  SPCR   = (1 << SPE) | (1 << MSTR) | (1 << SPR0);// SPI Enable, Master, ceas f_osc/16 = 1 MHz
}

static uint8_t spi_xfer(uint8_t b) {
  SPDR = b;
  while (!(SPSR & (1 << SPIF)));                  // Asteapta finalizarea transferului SPI
  return SPDR;
}

/* ==================== BUZZER & SIRENA ALARMA ==================== */
static void beep(uint16_t ms) {
  BUZZER_ON();
  delay_ms(ms);
  BUZZER_OFF();
}

static void sirena_intrus(void) {
  for (uint8_t i = 0; i < SIREN_CYCLES; i++) {
    BUZZER_ON();  delay_ms(SIREN_HALF_PERIOD_MS);
    BUZZER_OFF(); delay_ms(SIREN_HALF_PERIOD_MS);
  }
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

static void nrf_init_rx(void) {
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
  nrf_wreg(NRF_REG_RF_CH, NRF_RF_CHANNEL);        // Canal 76 (identic cu Nod 1)
  nrf_wreg(NRF_REG_RF_SETUP, NRF_RF_250KBPS_0DBM);// 250 kbps, 0 dBm
  nrf_wreg_buf(NRF_REG_TX_ADDR, RF_ADDR, NRF_ADDR_WIDTH);
  nrf_wreg_buf(NRF_REG_RX_ADDR_P0, RF_ADDR, NRF_ADDR_WIDTH);
  nrf_wreg(NRF_REG_RX_PW_P0, NRF_PAYLOAD_SIZE);

  nrf_cmd(NRF_CMD_FLUSH_TX);
  nrf_cmd(NRF_CMD_FLUSH_RX);
  nrf_wreg(NRF_REG_STATUS, NRF_STATUS_IRQ_CLEAR);
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_RX_MODE);
  delay_ms(5);

  NRF_CE_HIGH();                                  // Activeaza ascultarea continua
}

static void nrf_send_ack_echo(const uint8_t *raw_payload) {
  delay_ms(ACK_TURNAROUND_MS);                    // Asteapta ca Nod 1 sa comute pe receptie

  NRF_CE_LOW();
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_TX_MODE);      // Comuta temporar pe emisie
  delay_us(150);

  nrf_cmd(NRF_CMD_FLUSH_TX);
  nrf_wreg(NRF_REG_STATUS, NRF_STATUS_IRQ_CLEAR);

  NRF_CSN_LOW();
  spi_xfer(NRF_CMD_W_TX_PAYLOAD);
  for (uint8_t i = 0; i < NRF_PAYLOAD_SIZE; i++) spi_xfer(raw_payload[i]);
  NRF_CSN_HIGH();

  NRF_CE_HIGH(); delay_us(20); NRF_CE_LOW();

  for (uint16_t t = 0; t < 5000; t++) {
    if (nrf_cmd(NRF_CMD_NOP) & NRF_STATUS_TX_DS) break;
    delay_us(10);
  }
  nrf_wreg(NRF_REG_STATUS, NRF_STATUS_IRQ_CLEAR);

  /* Revine imediat in modul de receptie continua */
  nrf_cmd(NRF_CMD_FLUSH_RX);
  nrf_wreg(NRF_REG_CONFIG, NRF_CFG_RX_MODE);
  NRF_CE_HIGH();
}

/* ==================== PROGRAM PRINCIPAL ==================== */
static uint16_t rx_count = 0;
static bool connected_announced = false;

void setup(void) {
  DDRD |= (1 << PIN_BUZZER);
  BUZZER_OFF();

  uart_init();
  spi_init();
  delay_ms(400);

  uart_ln("\r\n=== NOD 2 (RECEPTOR UNO) ===");
  beep(120);

  nrf_init_rx();
  uint8_t cfg = nrf_rreg(NRF_REG_CONFIG);
  if (cfg == NRF_CFG_RX_MODE) {
    uart_ln("[OK] Receptor activ pe canal 76 (250 kbps). Astept Nod 1...");
  } else {
    uart_ln("[EROARE] Verifica firele SPI ale antenei!");
  }
}

void loop(void) {
  uint8_t st   = nrf_cmd(NRF_CMD_NOP);
  uint8_t fifo = nrf_rreg(NRF_REG_FIFO_STATUS);

  if ((st & NRF_STATUS_RX_DR) || !(fifo & NRF_FIFO_RX_EMPTY)) {
    uint8_t raw[NRF_PAYLOAD_SIZE];
    char msg[NRF_PAYLOAD_SIZE + 1];

    NRF_CSN_LOW();
    spi_xfer(NRF_CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < NRF_PAYLOAD_SIZE; i++) {
      raw[i] = spi_xfer(NRF_CMD_NOP);
      msg[i] = (char)raw[i];
    }
    NRF_CSN_HIGH();
    msg[NRF_PAYLOAD_SIZE] = '\0';
    nrf_wreg(NRF_REG_STATUS, NRF_STATUS_RX_DR);

    /* 1. Trimite imediat pachetul inapoi ca ACK catre Nod 1 */
    nrf_send_ack_echo(raw);

<<<<<<< HEAD
    /* 2. Proceseaza mesajul primit */
    if (str_eq(msg, "Alex e tare")) {
=======
    // 2. Proceseaza mesajul primit
    if (str_eq(msg, "Alex te bat cu LCD-ul paralel!")) {
>>>>>>> dd2552648ddac8a81758fbe3e323105b83306e43
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