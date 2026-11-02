#include <SPI.h>
#include <RF24.h>

// Includem doar bibliotecile pentru SPI si antena radio (RF24).
// Declaram pinii pentru a doua antena si pentru al doilea buzzer.
#define PIN_CE      7
#define PIN_CSN     8
#define PIN_BUZZER  5

RF24 radio(PIN_CE, PIN_CSN);
const byte adresa[6] = "00001";   // trebuie sa fie IDENTICA cu adresa din codul de pe Nano

void setup() {
  // Serial.begin(9600) ca sa putem vedea mesajele pe ecranul laptopului in Serial Monitor.
  Serial.begin(9600);

  // Ii explic placii printr-o functie ca buzzerul de paza va scoate sunete, nu va citi date.
  pinMode(PIN_BUZZER, OUTPUT);

  // Voi porni antena (functie), ii setez exact aceeasi frecventa ca la Nano (functie) si ii zic ca ea doar asculta, nu transmite (alta functie).
  radio.begin();
  radio.setChannel(76);              // acelasi canal ca pe Nano
  radio.openReadingPipe(0, adresa);
  radio.startListening();

  // Afisez un prim mesaj in Serial Monitor ca statia de paza a pornit si asteapta semnal (functie).
  Serial.println("Statia de paza a pornit. Astept semnal...");
}

void loop() {
  // Verific printr-o functie daca antena a prins vreun mesaj trimis de Nano.
  if (radio.available()) {

    // Daca a prins semnal, citesc pachetul de date din antena (functie) si il salvez intr-o variabila.
    char mesaj[32] = "";
    radio.read(&mesaj, sizeof(mesaj));

    // Afisez mesajul primit pe laptop in Serial Monitor ca sa il vedem (functie).
    Serial.print("Mesaj primit: ");
    Serial.println(mesaj);

    // Verific printr-o functie daca mesajul primit contine cuvantul de alerta pentru intrus, iar daca da, pornesc alarma la laptop (alta functie).
    if (strstr(mesaj, "INTRUS") != NULL) {
      alarmaPaza();
    }
  }
}

// FUNCTIILE NOASTRE
  // Functie pentru alarma de paza (fara parametri) -> porneste buzzerul pe o frecventa de sirena (functie), asteapta cateva secunde (delay) si apoi il opreste (functie).
void alarmaPaza() {
  tone(PIN_BUZZER, 1000);   // sirena la 1000 Hz
  delay(3000);              // 3 secunde
  noTone(PIN_BUZZER);
}