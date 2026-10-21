// Biblioteci, include-uri, declarari de pini

void setup() {
  // Serial.begin(9600) 
  // SPI.begin() ca lucram cu SPI intre RFID si antena radio (e din biblioteca de SPI) 
  // Initializam RFID-ul (functie)
  // Pornim LCD-ul (functie). Pe el vom avea un mesaj de intampinare in genul "Apropie cardul" (alta functie)   
  // Leg servomotorul de pinul de pe placa (functie) si ma asigur ca usa este blocata (alta functie).
  // Ii explic placii printr-o functie ca buzzerul va scoate sunete, nu va citi date.
  // Voi porni antena (functie), ii setez o frecventa (functie) si ii zic ca doar transmite, nu asculta (alta functie).
}

void loop() {
  // Verific printr-o functie daca s-a apropiat un card nou si prin alta functie daca il pot citi (daca nu e niciun card, ies din loop si o iau de la capat).
  // Extrag codul cardului citit si il salvez intr-o variabila text (functie facuta de noi).
  // Verific daca codul citit este cel corect (functie facuta de noi care imi raspunde cu true sau false).
  // Daca cardul este bun (if):
  //   Afisez pe LCD mesajul "Acces Permis" (functie).
  //   Trimit raport prin antena catre Uno ca a intrat cineva valid (functie).
  //   Deschid usa din servomotor, astept cateva secunde si o inchid la loc (functie).
  // Daca cardul NU este bun (else):
  //   Afisez pe LCD mesajul "Acces Respins" (functie).
  //   Pun buzzerul sa tiuie de eroare (functie).
  //   Trimit alerta de intrus prin antena catre Uno (functie).
  // Pun inapoi pe LCD mesajul cu "Apropie cardul" (functie) si ii zic cititorului RFID sa se opreasca din citit cardul actual (functie).
}

//FUNCTIILE NOASTRE
  // Functie care extrage codul (fara parametri) -> citeste bucatile de cod din RFID si le lipeste intr-un text normal (String).
  // Functie care verifica cardul (parametru: codul scanat) -> compara codul scanat cu al nostru si returneaza true sau false.
  // Functie pentru LCD (parametri: text rand 1, text rand 2) -> sterge ecranul (functie), muta cursorul pe randul 1 si scrie (functii), apoi muta pe randul 2 si scrie (functii).
  // Functie care actioneaza usa (fara parametri) -> da un bip scurt din buzzer (functie), roteste servomotorul la 90 de grade (functie), asteapta 3 secunde (functia delay) si roteste servo-ul inapoi la 0 grade (functie).
  // Functie pentru buzzer (parametru: numarul de bipuri) -> porneste curentul pe buzzer (functie), asteapta putin (delay) si opreste curentul (functie).
  // Functie pentru trimis raport radio (parametru: mesajul text) -> impacheteaza textul si il arunca in aer pe frecventa antenei (functie din biblioteca RF24).