# Sistem embedded bazat pe ESP32 pentru controlul și navigația unui vehicul miniatural

Acest depozit conține codul sursă dezvoltat în cadrul proiectului de licență.

## Autor

Rusu Vasile-Pintilie

## Descriere

Proiectul constă în realizarea unui vehicul miniatural cu patru motoare de curent continuu, controlat de un microcontroler ESP32.

Sistemul permite:
- controlul manual prin Bluetooth Low Energy;
- deplasarea înainte și înapoi;
- virarea la stânga și la dreapta;
- detectarea obstacolelor cu senzori HC-SR04P;
- executarea unei manevre autonome de ocolire;
- măsurarea temperaturii și umidității;
- afișarea informațiilor pe un ecran OLED;
- citirea senzorilor MPU6050, FSR și de prezență.

## Comenzi Bluetooth

- F – deplasare înainte
- B – deplasare înapoi
- L – virare la stânga
- R – virare la dreapta
- S – oprire
- A – activarea modului autonom
- M – revenirea la modul manual

## Fișier principal

`ESP32_Vehicul_Miniatural.ino`

## Platformă de dezvoltare

- ESP32 Dev Module
- Arduino IDE
- ESP32 BLE Terminal
