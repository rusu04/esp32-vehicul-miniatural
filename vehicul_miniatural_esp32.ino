#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// CONFIGURARE GENERALA
// =====================================================

// Lasa false pentru testele actuale.
// Astfel, vibratiile MPU6050 nu pot opri masina.
const bool FOLOSESTE_PROTECTIE_INCLINARE = false;

// =====================================================
// BLE
// =====================================================

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic* txCharacteristic = nullptr;
bool telefonConectat = false;

// =====================================================
// DRIVERE TB6612FNG
// =====================================================

// Partea dreapta
const int PWM_DREAPTA = 25;
const int IN1_DREAPTA = 26;
const int IN2_DREAPTA = 27;

// Partea stanga
const int PWM_STANGA = 14;
const int IN1_STANGA = 16;
const int IN2_STANGA = 17;

// =====================================================
// SENZORI HC-SR04P
// =====================================================

// Senzor frontal
const int TRIG_FATA = 33;
const int ECHO_FATA = 34;

// Senzor lateral montat in stanga
const int TRIG_STANGA = 32;
const int ECHO_STANGA = 35;

// =====================================================
// SENZORI AUXILIARI
// =====================================================

const int PIN_FSR = 36;
const int PREZENTA_OT1 = 39;

// =====================================================
// MAGISTRALA I2C
// =====================================================

const int SDA_I2C = 21;
const int SCL_I2C = 22;

const uint8_t ADRESA_SHT21 = 0x40;
const uint8_t ADRESA_MPU6050 = 0x68;

uint8_t adresaPCF = 0;
bool pcfGasit = false;

// OT2 conectat la P2 al PCF8574
const uint8_t PIN_PCF_OT2 = 2;

// =====================================================
// OLED SH1106 SPI
// =====================================================

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_MOSI 23
#define OLED_CLK  18
#define OLED_DC   19
#define OLED_RST  -1
#define OLED_CS   -1

Adafruit_SH1106G display(
OLED_WIDTH,
OLED_HEIGHT,
OLED_MOSI,
OLED_CLK,
OLED_DC,
OLED_RST,
OLED_CS
);

bool oledPornit = false;

// =====================================================
// PRAGURI
// =====================================================

// Masina incepe oprirea mai devreme pentru a nu lovi obstacolul.
const float PRAG_OBSTACOL_FATA = 40.0f;

// Spatiul minim necesar in partea stanga.
const float PRAG_SPATIU_STANGA = 30.0f;

const float PRAG_INCLINARE = 32.0f;

const int CONFIRMARI_OBSTACOL_NECESARE = 2;
const int CONFIRMARI_STANGA_NECESARE = 2;
const int CONFIRMARI_INCLINARE_NECESARE = 4;

const int ERORI_HC_MAXIME = 8;

const unsigned long DATE_HC_VALABILE_MS = 1500;
const unsigned long TIMP_PREGATIRE_AUTO_MAX = 3000;
const unsigned long TIMP_VERIFICARE_STANGA_MAX = 2500;

// =====================================================
// TIMPI PENTRU OCOLIRE
// =====================================================

// Acesti timpi pot necesita calibrare mecanica,
// in functie de podea, baterie si aderenta.

const unsigned long TIMP_STOP_INITIAL = 400;

const unsigned long TIMP_VIRAJ_STANGA_1 = 650;
const unsigned long TIMP_DEPLASARE_STANGA = 750;
const unsigned long TIMP_VIRAJ_DREAPTA_1 = 650;

const unsigned long TIMP_MERS_PE_LANGA_OBSTACOL = 1300;

const unsigned long TIMP_VIRAJ_DREAPTA_2 = 650;
const unsigned long TIMP_REVENIRE_PE_TRASEU = 750;
const unsigned long TIMP_VIRAJ_STANGA_2 = 650;

// =====================================================
// INTERVALE
// =====================================================

const unsigned long INTERVAL_PING_ULTRASONIC = 65;
const unsigned long INTERVAL_MPU = 100;
const unsigned long INTERVAL_AUXILIARI = 100;
const unsigned long INTERVAL_SHT21 = 2000;
const unsigned long INTERVAL_OLED = 300;
const unsigned long INTERVAL_SERIAL = 1000;

// =====================================================
// VALORI SENZORI
// =====================================================

float distantaFata = -1.0f;
float distantaStanga = -1.0f;

float temperatura = 0.0f;
float umiditate = 0.0f;

float inclinareX = 0.0f;
float inclinareY = 0.0f;

int valoareFSR = 0;

bool prezentaOT1 = false;
bool prezentaOT2 = false;

bool sht21OK = false;
bool mpu6050OK = false;

// =====================================================
// STAREA HC-SR04P
// =====================================================

int eroriConsecutiveFata = 0;
int eroriConsecutiveStanga = 0;

int confirmariObstacolFata = 0;
int confirmariStangaLibera = 0;
int confirmariStangaBlocata = 0;
int confirmariInclinare = 0;

unsigned long ultimaCitireValidaFata = 0;
unsigned long ultimaCitireValidaStanga = 0;

unsigned long ultimulPingUltrasonic = 0;

// Citim frontal de trei ori, apoi lateral o data.
uint8_t contorPing = 0;

// Folosit pentru a nu evalua aceeasi citire laterala de mai multe ori.
unsigned long ultimaCitireStangaEvaluata = 0;

// =====================================================
// INTERVALE GENERALE
// =====================================================

unsigned long ultimaCitireMPU = 0;
unsigned long ultimaCitireAuxiliari = 0;
unsigned long ultimaActualizareOLED = 0;
unsigned long ultimaAfisareSerial = 0;

int paginaOLED = 0;

// =====================================================
// MODURI SI STARI
// =====================================================

bool modAutonom = false;

const char* stareManuala = "STOP";

enum StareAutonoma {
AUTO_OPRIT,
AUTO_PREGATIRE,
AUTO_MERS_INAINTE,
AUTO_STOP_OBSTACOL,
AUTO_VERIFICARE_STANGA,
AUTO_VIRAJ_STANGA_1,
AUTO_DEPLASARE_STANGA,
AUTO_VIRAJ_DREAPTA_1,
AUTO_MERS_PE_LANGA_OBSTACOL,
AUTO_VIRAJ_DREAPTA_2,
AUTO_REVENIRE_PE_TRASEU,
AUTO_VIRAJ_STANGA_2
};

StareAutonoma stareAutonoma = AUTO_OPRIT;

unsigned long momentStare = 0;

// =====================================================
// SHT21 NEBLOCANT
// =====================================================

enum StareSHT21 {
SHT_REPAUS,
SHT_ASTEAPTA_TEMPERATURA,
SHT_ASTEAPTA_UMIDITATE
};

StareSHT21 stareSHT21 = SHT_REPAUS;

unsigned long momentComandaSHT = 0;
unsigned long ultimaPornireSHT = 0;

// =====================================================
// TRIMITERE MESAJ
// =====================================================

void trimiteMesaj(const char* mesaj) {
Serial.println(mesaj);

if (telefonConectat && txCharacteristic != nullptr) {
txCharacteristic->setValue(mesaj);
txCharacteristic->notify();
}
}

// =====================================================
// CONTROL MOTOARE
// =====================================================

void oprireMotoare() {
digitalWrite(PWM_DREAPTA, LOW);
digitalWrite(PWM_STANGA, LOW);

digitalWrite(IN1_DREAPTA, LOW);
digitalWrite(IN2_DREAPTA, LOW);

digitalWrite(IN1_STANGA, LOW);
digitalWrite(IN2_STANGA, LOW);
}

void mersInainte() {
digitalWrite(IN1_DREAPTA, HIGH);
digitalWrite(IN2_DREAPTA, LOW);

digitalWrite(IN1_STANGA, HIGH);
digitalWrite(IN2_STANGA, LOW);

digitalWrite(PWM_DREAPTA, HIGH);
digitalWrite(PWM_STANGA, HIGH);
}

void mersInapoi() {
digitalWrite(IN1_DREAPTA, LOW);
digitalWrite(IN2_DREAPTA, HIGH);

digitalWrite(IN1_STANGA, LOW);
digitalWrite(IN2_STANGA, HIGH);

digitalWrite(PWM_DREAPTA, HIGH);
digitalWrite(PWM_STANGA, HIGH);
}

void virajStanga() {
digitalWrite(IN1_DREAPTA, HIGH);
digitalWrite(IN2_DREAPTA, LOW);

digitalWrite(IN1_STANGA, LOW);
digitalWrite(IN2_STANGA, HIGH);

digitalWrite(PWM_DREAPTA, HIGH);
digitalWrite(PWM_STANGA, HIGH);
}

void virajDreapta() {
digitalWrite(IN1_DREAPTA, LOW);
digitalWrite(IN2_DREAPTA, HIGH);

digitalWrite(IN1_STANGA, HIGH);
digitalWrite(IN2_STANGA, LOW);

digitalWrite(PWM_DREAPTA, HIGH);
digitalWrite(PWM_STANGA, HIGH);
}

// =====================================================
// CITIRE HC-SR04P
// =====================================================

float citesteDistantaBruta(int trigPin, int echoPin) {
digitalWrite(trigPin, LOW);
delayMicroseconds(3);

digitalWrite(trigPin, HIGH);
delayMicroseconds(10);

digitalWrite(trigPin, LOW);

unsigned long durata =
pulseIn(echoPin, HIGH, 25000);

if (durata == 0) {
return -1.0f;
}

float distanta =
durata * 0.0343f / 2.0f;

if (distanta < 2.0f || distanta > 300.0f) {
return -1.0f;
}

return distanta;
}

void proceseazaCitireFata(float citire) {
if (citire <= 0.0f) {
eroriConsecutiveFata++;
return;
}

distantaFata = citire;
eroriConsecutiveFata = 0;
ultimaCitireValidaFata = millis();

if (citire <= PRAG_OBSTACOL_FATA) {
if (
confirmariObstacolFata <
CONFIRMARI_OBSTACOL_NECESARE
) {
confirmariObstacolFata++;
}
} else {
confirmariObstacolFata = 0;
}
}

void proceseazaCitireStanga(float citire) {
if (citire <= 0.0f) {
eroriConsecutiveStanga++;
return;
}

distantaStanga = citire;
eroriConsecutiveStanga = 0;
ultimaCitireValidaStanga = millis();
}

bool dateFataValide() {
if (ultimaCitireValidaFata == 0) {
return false;
}

if (eroriConsecutiveFata >= ERORI_HC_MAXIME) {
return false;
}

return millis() - ultimaCitireValidaFata
<= DATE_HC_VALABILE_MS;
}

bool dateStangaValide() {
if (ultimaCitireValidaStanga == 0) {
return false;
}

if (eroriConsecutiveStanga >= ERORI_HC_MAXIME) {
return false;
}

return millis() - ultimaCitireValidaStanga
<= DATE_HC_VALABILE_MS;
}

void actualizeazaUltrasonice() {
unsigned long acum = millis();

if (
acum - ultimulPingUltrasonic <
INTERVAL_PING_ULTRASONIC
) {
return;
}

ultimulPingUltrasonic = acum;

bool prioritateStanga =
modAutonom &&
stareAutonoma == AUTO_VERIFICARE_STANGA;

if (prioritateStanga) {
float citire =
citesteDistantaBruta(
TRIG_STANGA,
ECHO_STANGA
);


proceseazaCitireStanga(citire);
return;


}

// Trei citiri frontale pentru fiecare citire laterala.
if ((contorPing % 4) == 3) {
float citire =
citesteDistantaBruta(
TRIG_STANGA,
ECHO_STANGA
);


proceseazaCitireStanga(citire);


} else {
float citire =
citesteDistantaBruta(
TRIG_FATA,
ECHO_FATA
);


proceseazaCitireFata(citire);


}

contorPing++;
}

// =====================================================
// I2C
// =====================================================

bool existaDispozitivI2C(uint8_t adresa) {
Wire.beginTransmission(adresa);
return Wire.endTransmission() == 0;
}

// =====================================================
// SHT21
// =====================================================

bool citesteDateSHT21(uint16_t& valoareBruta) {
Wire.requestFrom(
ADRESA_SHT21,
(uint8_t)3
);

if (Wire.available() < 2) {
return false;
}

valoareBruta =
((uint16_t)Wire.read() << 8) |
Wire.read();

if (Wire.available()) {
Wire.read();
}

valoareBruta &= 0xFFFC;

return true;
}

void gestioneazaSHT21() {
if (!sht21OK) {
return;
}

unsigned long acum = millis();

if (stareSHT21 == SHT_REPAUS) {
if (
acum - ultimaPornireSHT <
INTERVAL_SHT21
) {
return;
}


Wire.beginTransmission(ADRESA_SHT21);
Wire.write(0xF3);

if (Wire.endTransmission() == 0) {
  stareSHT21 =
    SHT_ASTEAPTA_TEMPERATURA;

  momentComandaSHT = acum;
} else {
  ultimaPornireSHT = acum;
}

return;


}

if (
stareSHT21 ==
SHT_ASTEAPTA_TEMPERATURA
) {
if (acum - momentComandaSHT < 90) {
return;
}


uint16_t rawTemperatura = 0;

if (citesteDateSHT21(rawTemperatura)) {
  temperatura =
    -46.85f +
    175.72f *
    rawTemperatura /
    65536.0f;
}

Wire.beginTransmission(ADRESA_SHT21);
Wire.write(0xF5);

if (Wire.endTransmission() == 0) {
  stareSHT21 =
    SHT_ASTEAPTA_UMIDITATE;

  momentComandaSHT = acum;
} else {
  stareSHT21 = SHT_REPAUS;
  ultimaPornireSHT = acum;
}

return;


}

if (
stareSHT21 ==
SHT_ASTEAPTA_UMIDITATE
) {
if (acum - momentComandaSHT < 35) {
return;
}


uint16_t rawUmiditate = 0;

if (citesteDateSHT21(rawUmiditate)) {
  umiditate =
    -6.0f +
    125.0f *
    rawUmiditate /
    65536.0f;

  if (umiditate < 0.0f) {
    umiditate = 0.0f;
  }

  if (umiditate > 100.0f) {
    umiditate = 100.0f;
  }
}

stareSHT21 = SHT_REPAUS;
ultimaPornireSHT = acum;


}
}

// =====================================================
// MPU6050
// =====================================================

void initializareMPU6050() {
if (!mpu6050OK) {
return;
}

Wire.beginTransmission(ADRESA_MPU6050);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();

delay(100);

Wire.beginTransmission(ADRESA_MPU6050);
Wire.write(0x1C);
Wire.write(0x00);
Wire.endTransmission();
}

bool citesteMPU6050() {
if (!mpu6050OK) {
return false;
}

Wire.beginTransmission(ADRESA_MPU6050);
Wire.write(0x3B);

if (Wire.endTransmission(false) != 0) {
return false;
}

Wire.requestFrom(
ADRESA_MPU6050,
(uint8_t)6,
(uint8_t)true
);

if (Wire.available() < 6) {
return false;
}

int16_t accelX =
((int16_t)Wire.read() << 8) |
Wire.read();

int16_t accelY =
((int16_t)Wire.read() << 8) |
Wire.read();

int16_t accelZ =
((int16_t)Wire.read() << 8) |
Wire.read();

float ax = accelX / 16384.0f;
float ay = accelY / 16384.0f;
float az = accelZ / 16384.0f;

inclinareX =
atan2(
ay,
sqrt(ax * ax + az * az)
) * 180.0f / PI;

inclinareY =
atan2(
-ax,
sqrt(ay * ay + az * az)
) * 180.0f / PI;

bool depasirePrag =
fabsf(inclinareX) >= PRAG_INCLINARE ||
fabsf(inclinareY) >= PRAG_INCLINARE;

if (depasirePrag) {
if (
confirmariInclinare <
CONFIRMARI_INCLINARE_NECESARE
) {
confirmariInclinare++;
}
} else {
confirmariInclinare = 0;
}

return true;
}

void actualizeazaMPU() {
unsigned long acum = millis();

if (
acum - ultimaCitireMPU <
INTERVAL_MPU
) {
return;
}

ultimaCitireMPU = acum;
citesteMPU6050();
}

bool inclinarePericuloasa() {
if (!FOLOSESTE_PROTECTIE_INCLINARE) {
return false;
}

if (!mpu6050OK) {
return false;
}

return confirmariInclinare >=
CONFIRMARI_INCLINARE_NECESARE;
}

// =====================================================
// PCF8574
// =====================================================

void cautaPCF8574() {
pcfGasit = false;

for (
uint8_t adresa = 0x20;
adresa <= 0x27;
adresa++
) {
if (existaDispozitivI2C(adresa)) {
adresaPCF = adresa;
pcfGasit = true;
break;
}
}

if (!pcfGasit) {
for (
uint8_t adresa = 0x38;
adresa <= 0x3F;
adresa++
) {
if (existaDispozitivI2C(adresa)) {
adresaPCF = adresa;
pcfGasit = true;
break;
}
}
}

if (pcfGasit) {
Wire.beginTransmission(adresaPCF);
Wire.write(0xFF);
Wire.endTransmission();


Serial.print(
  "PCF8574 detectat la 0x"
);
Serial.println(adresaPCF, HEX);


} else {
Serial.println(
"PCF8574 nu a fost detectat"
);
}
}

bool citestePinPCF(uint8_t pin) {
if (!pcfGasit) {
return false;
}

Wire.requestFrom(
adresaPCF,
(uint8_t)1
);

if (!Wire.available()) {
return false;
}

uint8_t valoare = Wire.read();

return bitRead(valoare, pin);
}

// =====================================================
// SENZORI AUXILIARI
// =====================================================

void actualizeazaSenzoriAuxiliari() {
unsigned long acum = millis();

if (
acum - ultimaCitireAuxiliari <
INTERVAL_AUXILIARI
) {
return;
}

ultimaCitireAuxiliari = acum;

valoareFSR = analogRead(PIN_FSR);

prezentaOT1 =
digitalRead(PREZENTA_OT1);

if (pcfGasit) {
prezentaOT2 =
citestePinPCF(PIN_PCF_OT2);
}
}

// =====================================================
// SCHIMBARE STARE AUTONOMA
// =====================================================

void schimbaStareaAutonoma(
StareAutonoma stareNoua
) {
stareAutonoma = stareNoua;
momentStare = millis();

switch (stareNoua) {
case AUTO_OPRIT:
oprireMotoare();
break;


case AUTO_PREGATIRE:
  oprireMotoare();
  confirmariObstacolFata = 0;
  break;

case AUTO_MERS_INAINTE:
  confirmariObstacolFata = 0;
  mersInainte();
  break;

case AUTO_STOP_OBSTACOL:
  oprireMotoare();
  break;

case AUTO_VERIFICARE_STANGA:
  oprireMotoare();

  confirmariStangaLibera = 0;
  confirmariStangaBlocata = 0;
  ultimaCitireStangaEvaluata =
    ultimaCitireValidaStanga;
  break;

case AUTO_VIRAJ_STANGA_1:
  virajStanga();
  break;

case AUTO_DEPLASARE_STANGA:
  mersInainte();
  break;

case AUTO_VIRAJ_DREAPTA_1:
  virajDreapta();
  break;

case AUTO_MERS_PE_LANGA_OBSTACOL:
  mersInainte();
  break;

case AUTO_VIRAJ_DREAPTA_2:
  virajDreapta();
  break;

case AUTO_REVENIRE_PE_TRASEU:
  mersInainte();
  break;

case AUTO_VIRAJ_STANGA_2:
  virajStanga();
  break;


}
}

void opresteModulAutonom(
const char* mesaj
) {
modAutonom = false;
stareAutonoma = AUTO_OPRIT;

oprireMotoare();
trimiteMesaj(mesaj);
}

// =====================================================
// MOD AUTONOM
// =====================================================

void gestioneazaModAutonom() {
if (!modAutonom) {
return;
}

if (inclinarePericuloasa()) {
opresteModulAutonom(
"STOP AUTO: INCLINARE"
);
return;
}

unsigned long timpInStare =
millis() - momentStare;

switch (stareAutonoma) {
case AUTO_OPRIT:
break;


case AUTO_PREGATIRE:
  if (dateFataValide()) {
    if (
      distantaFata <=
      PRAG_OBSTACOL_FATA
    ) {
      trimiteMesaj(
        "OBSTACOL LA PORNIRE"
      );

      schimbaStareaAutonoma(
        AUTO_STOP_OBSTACOL
      );
    } else {
      trimiteMesaj(
        "AUTO: MERS INAINTE"
      );

      schimbaStareaAutonoma(
        AUTO_MERS_INAINTE
      );
    }
  }
  else if (
    timpInStare >=
    TIMP_PREGATIRE_AUTO_MAX
  ) {
    opresteModulAutonom(
      "STOP AUTO: SENZOR FRONTAL"
    );
  }
  break;

case AUTO_MERS_INAINTE:
  if (!dateFataValide()) {
    if (
      eroriConsecutiveFata >=
      ERORI_HC_MAXIME
    ) {
      opresteModulAutonom(
        "STOP AUTO: SENZOR FRONTAL"
      );
    }
  }
  else if (
    confirmariObstacolFata >=
    CONFIRMARI_OBSTACOL_NECESARE
  ) {
    trimiteMesaj(
      "OBSTACOL DETECTAT"
    );

    schimbaStareaAutonoma(
      AUTO_STOP_OBSTACOL
    );
  }
  else {
    mersInainte();
  }
  break;

case AUTO_STOP_OBSTACOL:
  if (
    timpInStare >=
    TIMP_STOP_INITIAL
  ) {
    schimbaStareaAutonoma(
      AUTO_VERIFICARE_STANGA
    );
  }
  break;

case AUTO_VERIFICARE_STANGA:
  if (
    ultimaCitireValidaStanga !=
    ultimaCitireStangaEvaluata
  ) {
    ultimaCitireStangaEvaluata =
      ultimaCitireValidaStanga;

    if (
      distantaStanga >
      PRAG_SPATIU_STANGA
    ) {
      confirmariStangaLibera++;
      confirmariStangaBlocata = 0;
    } else {
      confirmariStangaBlocata++;
      confirmariStangaLibera = 0;
    }
  }

  if (
    confirmariStangaLibera >=
    CONFIRMARI_STANGA_NECESARE
  ) {
    trimiteMesaj(
      "SPATIU LIBER IN STANGA"
    );

    schimbaStareaAutonoma(
      AUTO_VIRAJ_STANGA_1
    );
  }
  else if (
    confirmariStangaBlocata >=
    CONFIRMARI_STANGA_NECESARE
  ) {
    opresteModulAutonom(
      "STOP AUTO: STANGA BLOCATA"
    );
  }
  else if (
    timpInStare >=
    TIMP_VERIFICARE_STANGA_MAX
  ) {
    opresteModulAutonom(
      "STOP AUTO: SENZOR STANGA"
    );
  }
  break;

case AUTO_VIRAJ_STANGA_1:
  if (
    timpInStare >=
    TIMP_VIRAJ_STANGA_1
  ) {
    schimbaStareaAutonoma(
      AUTO_DEPLASARE_STANGA
    );
  }
  break;

case AUTO_DEPLASARE_STANGA:
  if (
    timpInStare >=
    TIMP_DEPLASARE_STANGA
  ) {
    schimbaStareaAutonoma(
      AUTO_VIRAJ_DREAPTA_1
    );
  }
  break;

case AUTO_VIRAJ_DREAPTA_1:
  if (
    timpInStare >=
    TIMP_VIRAJ_DREAPTA_1
  ) {
    schimbaStareaAutonoma(
      AUTO_MERS_PE_LANGA_OBSTACOL
    );
  }
  break;

case AUTO_MERS_PE_LANGA_OBSTACOL:
  if (
    timpInStare >=
    TIMP_MERS_PE_LANGA_OBSTACOL
  ) {
    schimbaStareaAutonoma(
      AUTO_VIRAJ_DREAPTA_2
    );
  }
  break;

case AUTO_VIRAJ_DREAPTA_2:
  if (
    timpInStare >=
    TIMP_VIRAJ_DREAPTA_2
  ) {
    schimbaStareaAutonoma(
      AUTO_REVENIRE_PE_TRASEU
    );
  }
  break;

case AUTO_REVENIRE_PE_TRASEU:
  if (
    timpInStare >=
    TIMP_REVENIRE_PE_TRASEU
  ) {
    schimbaStareaAutonoma(
      AUTO_VIRAJ_STANGA_2
    );
  }
  break;

case AUTO_VIRAJ_STANGA_2:
  if (
    timpInStare >=
    TIMP_VIRAJ_STANGA_2
  ) {
    trimiteMesaj(
      "OCOLIRE FINALIZATA"
    );

    schimbaStareaAutonoma(
      AUTO_MERS_INAINTE
    );
  }
  break;


}
}

// =====================================================
// DENUMIRE STARE
// =====================================================

const char* numeStareAutonoma() {
switch (stareAutonoma) {
case AUTO_PREGATIRE:
return "PREGATIRE";


case AUTO_MERS_INAINTE:
  return "INAINTE";

case AUTO_STOP_OBSTACOL:
  return "STOP";

case AUTO_VERIFICARE_STANGA:
  return "SCAN STANGA";

case AUTO_VIRAJ_STANGA_1:
case AUTO_VIRAJ_STANGA_2:
  return "STANGA";

case AUTO_DEPLASARE_STANGA:
  return "DEPLASARE";

case AUTO_VIRAJ_DREAPTA_1:
case AUTO_VIRAJ_DREAPTA_2:
  return "DREAPTA";

case AUTO_MERS_PE_LANGA_OBSTACOL:
  return "OCOLIRE";

case AUTO_REVENIRE_PE_TRASEU:
  return "REVENIRE";

default:
  return "OPRIT";


}
}

// =====================================================
// OLED
// =====================================================

void afiseazaOLED() {
if (!oledPornit) {
return;
}

display.clearDisplay();
display.setTextColor(SH110X_WHITE);
display.setTextSize(1);
display.setCursor(0, 0);

if (paginaOLED == 0) {
display.print("MOD: ");
display.println(
modAutonom ? "AUTONOM" : "MANUAL"
);


display.print("Stare: ");

if (modAutonom) {
  display.println(numeStareAutonoma());
} else {
  display.println(stareManuala);
}

display.print("Fata: ");

if (dateFataValide()) {
  display.print(distantaFata, 1);
  display.println(" cm");
} else {
  display.println("EROARE");
}

display.print("Stanga: ");

if (dateStangaValide()) {
  display.print(distantaStanga, 1);
  display.println(" cm");
} else {
  display.println("EROARE");
}

display.print("Temp: ");
display.print(temperatura, 1);
display.println(" C");

display.print("Umid: ");
display.print(umiditate, 1);
display.println(" %");


} else {
display.println("SENZORI AUXILIARI");
display.println();


display.print("Unghi X: ");
display.println(inclinareX, 1);

display.print("Unghi Y: ");
display.println(inclinareY, 1);

display.print("FSR: ");
display.println(valoareFSR);

display.print("OT1: ");
display.println(prezentaOT1);

display.print("OT2: ");
display.println(prezentaOT2);

display.print("PCF: ");
display.println(
  pcfGasit ? "OK" : "LIPSA"
);


}

display.display();
}

// =====================================================
// SERIAL MONITOR
// =====================================================

void afiseazaSerial() {
Serial.println();
Serial.println("------------------------");

Serial.print("Mod: ");
Serial.println(
modAutonom ? "AUTONOM" : "MANUAL"
);

Serial.print("Stare: ");

if (modAutonom) {
Serial.println(numeStareAutonoma());
} else {
Serial.println(stareManuala);
}

Serial.print("Fata: ");

if (dateFataValide()) {
Serial.print(distantaFata);
Serial.println(" cm");
} else {
Serial.println("EROARE");
}

Serial.print("Stanga: ");

if (dateStangaValide()) {
Serial.print(distantaStanga);
Serial.println(" cm");
} else {
Serial.println("EROARE");
}

Serial.print("Temperatura: ");
Serial.println(temperatura);

Serial.print("Umiditate: ");
Serial.println(umiditate);

Serial.print("Inclinare X: ");
Serial.println(inclinareX);

Serial.print("Inclinare Y: ");
Serial.println(inclinareY);

Serial.print("FSR: ");
Serial.println(valoareFSR);

Serial.print("OT1: ");
Serial.println(prezentaOT1);

Serial.print("OT2: ");
Serial.println(prezentaOT2);

Serial.print("Erori HC fata: ");
Serial.println(eroriConsecutiveFata);

Serial.print("Erori HC stanga: ");
Serial.println(eroriConsecutiveStanga);
}

// =====================================================
// BLE CALLBACKS
// =====================================================

class ServerCallbacks :
public BLEServerCallbacks {
void onConnect(
BLEServer* server
) override {
telefonConectat = true;


Serial.println(
  "TELEFON CONECTAT"
);


}

void onDisconnect(
BLEServer* server
) override {
telefonConectat = false;


modAutonom = false;
stareAutonoma = AUTO_OPRIT;
stareManuala = "STOP";

oprireMotoare();

Serial.println(
  "TELEFON DECONECTAT"
);

delay(300);
BLEDevice::startAdvertising();


}
};

class RxCallbacks :
public BLECharacteristicCallbacks {
void onWrite(
BLECharacteristic* characteristic
) override {
String text =
characteristic->getValue();


Serial.print("PRIMIT: [");
Serial.print(text);
Serial.println("]");

text.trim();
text.toUpperCase();

if (text.length() == 0) {
  return;
}

char comanda = text.charAt(0);

if (comanda == 'S') {
  modAutonom = false;
  stareAutonoma = AUTO_OPRIT;
  stareManuala = "STOP";

  oprireMotoare();
  trimiteMesaj("STOP");

  return;
}

if (comanda == 'M') {
  modAutonom = false;
  stareAutonoma = AUTO_OPRIT;
  stareManuala = "STOP";

  oprireMotoare();
  trimiteMesaj("MOD MANUAL ACTIV");

  return;
}

if (comanda == 'A') {
  modAutonom = true;
  stareManuala = "STOP";

  schimbaStareaAutonoma(
    AUTO_PREGATIRE
  );

  trimiteMesaj(
    "MOD AUTONOM ACTIV"
  );

  return;
}

if (modAutonom) {
  trimiteMesaj(
    "TRIMITE M PENTRU MANUAL"
  );

  return;
}

switch (comanda) {
  case 'F':
    mersInainte();
    stareManuala = "INAINTE";

    trimiteMesaj("INAINTE");
    break;

  case 'B':
    mersInapoi();
    stareManuala = "INAPOI";

    trimiteMesaj("INAPOI");
    break;

  case 'L':
    virajStanga();
    stareManuala = "STANGA";

    trimiteMesaj("STANGA");
    break;

  case 'R':
    virajDreapta();
    stareManuala = "DREAPTA";

    trimiteMesaj("DREAPTA");
    break;

  default:
    oprireMotoare();
    stareManuala = "STOP";

    trimiteMesaj(
      "COMANDA NECUNOSCUTA"
    );
    break;
}


}
};

// =====================================================
// SETUP
// =====================================================

void setup() {
Serial.begin(115200);
delay(1000);

pinMode(PWM_DREAPTA, OUTPUT);
pinMode(IN1_DREAPTA, OUTPUT);
pinMode(IN2_DREAPTA, OUTPUT);

pinMode(PWM_STANGA, OUTPUT);
pinMode(IN1_STANGA, OUTPUT);
pinMode(IN2_STANGA, OUTPUT);

pinMode(TRIG_FATA, OUTPUT);
pinMode(ECHO_FATA, INPUT);

pinMode(TRIG_STANGA, OUTPUT);
pinMode(ECHO_STANGA, INPUT);

pinMode(PIN_FSR, INPUT);
pinMode(PREZENTA_OT1, INPUT);

digitalWrite(TRIG_FATA, LOW);
digitalWrite(TRIG_STANGA, LOW);

oprireMotoare();

Wire.begin(SDA_I2C, SCL_I2C);
Wire.setClock(100000);

sht21OK =
existaDispozitivI2C(
ADRESA_SHT21
);

mpu6050OK =
existaDispozitivI2C(
ADRESA_MPU6050
);

initializareMPU6050();
cautaPCF8574();

oledPornit =
display.begin(0, true);

if (oledPornit) {
display.clearDisplay();
display.setTextColor(
SH110X_WHITE
);
display.setTextSize(1);
display.setCursor(0, 0);
display.println(
"Pornire sistem..."
);
display.display();
} else {
Serial.println(
"OLED NEPORNIT"
);
}

ultimaPornireSHT =
millis() - INTERVAL_SHT21;

BLEDevice::init("ESP32_MASINA");

BLEServer* server =
BLEDevice::createServer();

server->setCallbacks(
new ServerCallbacks()
);

BLEService* service =
server->createService(
SERVICE_UUID
);

txCharacteristic =
service->createCharacteristic(
TX_UUID,
BLECharacteristic::PROPERTY_READ |
BLECharacteristic::PROPERTY_NOTIFY
);

txCharacteristic->addDescriptor(
new BLE2902()
);

BLECharacteristic* rxCharacteristic =
service->createCharacteristic(
RX_UUID,
BLECharacteristic::PROPERTY_WRITE |
BLECharacteristic::PROPERTY_WRITE_NR
);

rxCharacteristic->setCallbacks(
new RxCallbacks()
);

service->start();

BLEAdvertising* advertising =
BLEDevice::getAdvertising();

advertising->addServiceUUID(
SERVICE_UUID
);

advertising->setScanResponse(true);

BLEDevice::startAdvertising();

Serial.println();
Serial.println("SISTEM PORNIT");
Serial.println(
"Nume BLE: ESP32_MASINA"
);
Serial.println("F = inainte");
Serial.println("B = inapoi");
Serial.println("L = stanga");
Serial.println("R = dreapta");
Serial.println("S = stop");
Serial.println("A = autonom");
Serial.println("M = manual");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
unsigned long acum = millis();

actualizeazaUltrasonice();
actualizeazaMPU();
actualizeazaSenzoriAuxiliari();
gestioneazaSHT21();

// Nu exista oprire automata in modul manual.
// Comanda F continua pana la alta comanda.

gestioneazaModAutonom();

if (
acum - ultimaActualizareOLED >=
INTERVAL_OLED
) {
ultimaActualizareOLED = acum;


paginaOLED =
  (acum / 3000UL) % 2;

afiseazaOLED();


}

if (
acum - ultimaAfisareSerial >=
INTERVAL_SERIAL
) {
ultimaAfisareSerial = acum;


afiseazaSerial();


}

delay(5);
}
