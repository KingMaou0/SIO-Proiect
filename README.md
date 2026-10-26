Proiectul descrie proiectarea, construcția și testarea unui sistem distribuit pentru controlul accesului și supravegherea de la distanță a punctelor de trecere restricționate.

Sistemul nu are un creier central: e format din două noduri independente, care comunică prin radio, pe 2.4 GHz.

Nodul 1 – terminalul de acces. Stă chiar la punctul de intrare și e construit în jurul unei plăci Arduino Nano. Citește cartela RFID (modul RC522), comandă zăvorul printr-un servomotor SG90, afișează mesajele pe un LCD1602, semnalizează sonor printr-un buzzer și trimite imediat datele accesului mai departe, prin modulul nRF24L01+.

Nodul 2 – monitorizare. Folosește un Arduino Uno legat la un PC. Receptorul nRF24L01+ prinde pachetele radio, pornește o alertă sonoră când cineva încearcă să intre fără voie și scrie log-urile în timp real pe interfața serială.
