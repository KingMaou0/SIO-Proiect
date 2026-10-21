// Includem doar bibliotecile pentru SPI si antena radio (RF24).
// Declaram pinii pentru a doua antena si pentru al doilea buzzer.

void setup() {
  // Serial.begin(9600) ca sa putem vedea mesajele pe ecranul laptopului in Serial Monitor.
  // Ii explic placii printr-o functie ca buzzerul de paza va scoate sunete, nu va citi date.
  // Voi porni antena (functie), ii setez exact aceeasi frecventa ca la Nano (functie) si ii zic ca ea doar asculta, nu transmite (alta functie).
  // Afisez un prim mesaj in Serial Monitor ca statia de paza a pornit si asteapta semnal (functie).
}

void loop() {
  // Verific printr-o functie daca antena a prins vreun mesaj trimis de Nano.
  // Daca a prins semnal, citesc pachetul de date din antena (functie) si il salvez intr-o variabila.
  // Afisez mesajul primit pe laptop in Serial Monitor ca sa il vedem (functie).
  // Verific printr-o functie daca mesajul primit contine cuvantul de alerta pentru intrus, iar daca da, pornesc alarma la laptop (alta functie).
}

// FUNCTIILE NOASTRE
  // Functie pentru alarma de paza (fara parametri) -> porneste buzzerul pe o frecventa de sirena (functie), asteapta cateva secunde (delay) si apoi il opreste (functie).