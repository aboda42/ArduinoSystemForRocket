// Arduino kod pro raketu Bingus I

// knihovny potřebné pro modul
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>

#define SERVO_PIN 9
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Servo servoMotor;

Adafruit_BMP3XX bmp;

// nastavení adresy modulu
MPU6050 mpu;
// číslo pinu s LED diodou pro notifikaci
#define LED_PIN 13 

// Vyberte pin pro SD kartu
const int chipSelect = 53;

// inicializace proměnných, do kterých se uloží data
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

int vzletloToUz = 0;
float currentAltitude, previousAltitude = 1000.00;

// promenne pro fazi ASCENT:
float ascentCurrentAltitude = 0;
float maxAltitude = 0;
bool dostatecnyNaklon = false;
bool nakloneni = true;
int srovnani = 0;
int apogeeCounter = 0;
int descentCurrentAltitude, descentFormerAltitude = 0;
int landing = 0;

// inicializace proměnných pro výpočet
Quaternion q;           // [w, x, y, z] kvaternion
VectorFloat gravity;    // [x, y, z] vektor setrvačnosti
float rotace[3];        // rotace kolem os x,y,z

// letové fáze:
bool INIT = true;
bool IDLE, ASCENT, DESCENT, LANDED = false;

// Rutina přerušení
volatile bool mpuInterrupt = false;
void dmpINT() {
  mpuInterrupt = true;
}

// SIM modul

char serialBuffer[100];
bool callReceived = false;
bool hovor = true; 

void getResponse();
void kontrolaKomunikace();
void posliSMS();
void posliSMS2();
void nastavRychlostKomunikace();
void nastaveniChybovychHlaseni();
void kontrolaOdblokovaniSIM();
void kontrolaOperatora();
void kontrolaSignalu();
void nastaveniModuZarizeni();
void zkontrolujPrichoziHovor();

void setup() {
  if (INIT) {
    // nastavení LED jako výstupní
    pinMode(LED_PIN, OUTPUT);
    // nastavení I2C sběrnice
    Wire.begin();
    Serial1.begin(115200);  // Serial1 corresponds to pins 18 (TX) and 19 (RX) on the Arduino Mega
    // komunikace přes sériovou linku rychlostí 115200 baud
    Serial.begin(115200);
    while (!Serial);

    // inicializace čtečky SD karet
    Serial.print("Inicializace SD karty...");
    if (!SD.begin(chipSelect)) {
      Serial.println("Selhalo!");
      return;
    }
    Serial.println("Hotovo.");

    // inicializace MPU9250
    Serial.println(F("Inicializace I2C zarizeni.."));
    mpu.initialize();
    Serial.println(F("Test pripojenych zarizeni.."));
    Serial.println(mpu.testConnection() ? F("Modul pripojeni") : F("Pripojeni modulu selhalo"));
    // incializace DMP
    Serial.println(F("Inicializace DMP..."));
    devStatus = mpu.dmpInitialize();
    // kontrola funkčnosti DMP
    if (devStatus == 0) {
      // zapnutí DMP
      Serial.println(F("Povoleni DMP..."));
      mpu.setDMPEnabled(true);
      // nastavení pinu INT jako přerušovacího, interrupt 0 odpovídá pinu 2
      attachInterrupt(0, dmpINT, RISING);
      mpuIntStatus = mpu.getIntStatus();
      Serial.println(F("DMP pripraveno, cekam na prvni preruseni.."));
      dmpReady = true;
      // načtení velikosti zpráv, které bude DMP posílat
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else {
      // V případě chyby:
      // 1 : selhání připojení k DMP
      // 2 : selhání při nastavení DMP
      Serial.print(F("DMP inicializace selhala (kod "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
    digitalWrite(LED_PIN, LOW);

    //inicializace BMP390L
    Serial.println("Adafruit BMP388 / BMP390 test");

    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
      Serial.println("Could not find a valid BMP3 sensor, check wiring!");
      while (1);
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // inicializace SIM modulu
    nastavRychlostKomunikace();
    kontrolaKomunikace();
    nastaveniChybovychHlaseni();
    nastaveniModuZarizeni();
    kontrolaOperatora();
    kontrolaOdblokovaniSIM();
    kontrolaSignalu();
    registrace();

    // Povolit CLIP
    Serial1.println("AT+CLIP=1");

    // inicializace serva
    servoMotor.attach(SERVO_PIN);
    delay(5000);
    //posliSMS(); wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww odkomentovat na odpalový den
  }
}

void loop() {
  if (INIT) {
    previousAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (hovor) {
      zkontrolujPrichoziHovor();
    }
    if (callReceived) {
      servoMotor.write(70);
      delay(600);
      servoMotor.write(90);
      INIT = false;
      IDLE = true;
    }
  }
  if (IDLE) {
    if (! bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Approx. altitude = ");
    Serial.print(currentAltitude);
    Serial.println(" m");
    if (currentAltitude > previousAltitude) {
      vzletloToUz += 1;
    }
    else {
      vzletloToUz = 0;
    }
    previousAltitude = currentAltitude;
    if (vzletloToUz > 10) {
      IDLE = false;
      ASCENT = true;
    }
    delay(100);
  }
  if (ASCENT) {
    // dokud nepošle DMP přerušení, můžeme provádět ostatní příkazy ve smyčce while níže
    if (!dmpReady) return;
    // tato podmínka čeká na příjem přerušení a můžeme v ní provádět ostatní operace
    while (!mpuInterrupt && fifoCount < packetSize) {
      // místo pro ostatní operace
      // ..
    }

    // získání informace o statusu DSP
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // získání velikosti zásobníku dat
    fifoCount = mpu.getFIFOCount();
    // kontrola přetečení zásobníku dat

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println(F("Preteceni zasobniku dat!"));
      // v případě přetečení zásobníku je nutné
      // častěji vyčítat data
    }
    else if (mpuIntStatus & 0x02) {
      // kontrola délky dat
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // čtení paketu ze zásobníku
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // při větším množství paketů snížíme počítadlo
      fifoCount -= packetSize;
      // přečtení dat z DSP a uložení do proměnných
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(rotace, &q, &gravity);

      // výpis informací o rotacích kolem jednotlivých os
      Serial.print("Rotace \t X ");
      Serial.print(rotace[2] * 180/M_PI);
      Serial.print("st \t Y ");
      Serial.print(rotace[1] * 180/M_PI);
      Serial.print("st \t Z ");
      Serial.print(rotace[0] * 180/M_PI);
      Serial.print("st");
      if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }
      ascentCurrentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      delay(100);
      Serial.print("        Approx. Altitude = ");
      Serial.print(ascentCurrentAltitude);
      Serial.println(" m");

      if (ascentCurrentAltitude > maxAltitude) {
        maxAltitude = ascentCurrentAltitude;
        apogeeCounter = 0;
      }
      else {
        apogeeCounter += 1;
      }

      // Otevření souboru na SD kartě pro zápis
      File dataFile = SD.open("datalog.txt", FILE_WRITE);

      // Pokud se soubor otevře správně, zapíšeme do něj data
      if (dataFile) {
        dataFile.print("Rotace \t X ");
        dataFile.print(rotace[2] * 180/M_PI);
        dataFile.print("st \t Y ");
        dataFile.print(rotace[1] * 180/M_PI);
        dataFile.print("st \t Z ");
        dataFile.print(rotace[0] * 180/M_PI);
        dataFile.print("st");
        dataFile.print("       Approx. Altitude = ");
        dataFile.print(ascentCurrentAltitude);
        dataFile.println(" m");

        // Zavření souboru
        dataFile.close();
      }
      // Pokud se soubor neotevře, vypíšeme chybovou hlášku
      else {
        Serial.println("Chyba pri otevirani souboru datalog.txt");
      }
    }
    // detekce dostatecnpho naklonu:
    if (rotace[1] * 180 / M_PI < -70) {
      srovnani += 1;
    }
    if (srovnani >= 20) {
      if (rotace[1] * 180 / M_PI > -60) {
        dostatecnyNaklon = true;
      }
      if (dostatecnyNaklon && apogeeCounter > 20) {
        servoMotor.write(105);
        delay(600);
        servoMotor.write(90);

        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        dataFile.println("Servo se otocilo!");
        Serial.println("Servo se otocilo!");
        dataFile.close();
        ASCENT = false;
        DESCENT = true;
      } 
    }
  }
  if (DESCENT) {
    // dokud nepošle DMP přerušení, můžeme provádět ostatní příkazy ve smyčce while níže
    if (!dmpReady) return;
    // tato podmínka čeká na příjem přerušení a můžeme v ní provádět ostatní operace
    while (!mpuInterrupt && fifoCount < packetSize) {
      // místo pro ostatní operace
      // ..
    }

    // získání informace o statusu DSP
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // získání velikosti zásobníku dat
    fifoCount = mpu.getFIFOCount();
    // kontrola přetečení zásobníku dat

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println(F("Preteceni zasobniku dat!"));
      // v případě přetečení zásobníku je nutné
      // častěji vyčítat data
    }
    else if (mpuIntStatus & 0x02) {
      // kontrola délky dat
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // čtení paketu ze zásobníku
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // při větším množství paketů snížíme počítadlo
      fifoCount -= packetSize;
      // přečtení dat z DSP a uložení do proměnných
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(rotace, &q, &gravity);

      // výpis informací o rotacích kolem jednotlivých os
      Serial.print("Rotace \t X ");
      Serial.print(rotace[2] * 180/M_PI);
      Serial.print("st \t Y ");
      Serial.print(rotace[1] * 180/M_PI);
      Serial.print("st \t Z ");
      Serial.print(rotace[0] * 180/M_PI);
      Serial.print("st");
      if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }
      descentCurrentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
      delay(100);
      Serial.print("        Approx. Altitude = ");
      Serial.print(ascentCurrentAltitude);
      Serial.println(" m");
    }

    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    if (dataFile) {
        dataFile.print("Rotace \t X ");
        dataFile.print(rotace[2] * 180/M_PI);
        dataFile.print("st \t Y ");
        dataFile.print(rotace[1] * 180/M_PI);
        dataFile.print("st \t Z ");
        dataFile.print(rotace[0] * 180/M_PI);
        dataFile.print("st");
        dataFile.print("       Approx. Altitude = ");
        dataFile.print(ascentCurrentAltitude);
        dataFile.println(" m");

        // Zavření souboru
        dataFile.close();
      }
      // Pokud se soubor neotevře, vypíšeme chybovou hlášku
      else {
        Serial.println("Chyba pri otevirani souboru datalog.txt");
      }
    if (descentCurrentAltitude == descentFormerAltitude || descentCurrentAltitude +1 == descentFormerAltitude || descentCurrentAltitude - 1 == descentFormerAltitude) {
      landing += 1;
    }
    else {
      landing = 0;
    }
    if (landing > 30) {
      DESCENT = false;
      LANDED = true;
    }
    descentFormerAltitude = descentCurrentAltitude;
  }
  if (LANDED) {
    Serial.print("Přistáli jsme!");
    //posliSMS2(); wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww odkomentovat na odpalový den
    LANDED = false;
  }
}

void kontrolaKomunikace() {
  uint8_t var;
  Serial.println("Zahajuji kontrolu komunikace.");
  Serial1.println("AT");
  delay(100);
  var = Serial1.available();
  if (0 < var) {
    Serial1.readBytes(serialBuffer, var);
    Serial.write(serialBuffer, var);
    Serial.println();
  }
}

void posliSMS() {
  Serial.println("Odeslani SMS.");
  // nastavi zpravu jako textovou (ASCII)
  Serial1.println("AT+CMGF=1");
  getResponse();
  Serial1.println("AT+CMGS=\"00420732692060\"");
  getResponse();
  // Zprava
  Serial1.println("Inicializace probehla");
  getResponse();
  // Zakonceni zpravy
  Serial1.write(char(26));
  getResponse();
}

void posliSMS2() {
  Serial.println("Odeslani SMS.");
  // nastavi zpravu jako textovou (ASCII)
  Serial1.println("AT+CMGF=1");
  getResponse();
  Serial1.println("AT+CMGS=\"00420732692060\"");
  getResponse();
  // Zprava
  Serial1.println("Pristali jsme!");
  getResponse();
  // Zakonceni zpravy
  Serial1.write(char(26));
  getResponse();
}

void nastavRychlostKomunikace() {
  Serial.println("Nastavuji rychlost komunikace.");
  /* Dopln pozadovane cislo za znamenko "=", defaultní 9600 */
  Serial1.println("AT+IPR=111520");
  getResponse();
}

void nastaveniChybovychHlaseni() {
  uint8_t var;
  Serial.println("Nastavuji kompletni vypis chybovych hlaseni.");
  Serial1.println("AT+CMEE=2");
  getResponse();
}

void kontrolaOdblokovaniSIM() {
  Serial.println("Kontrola odblokovani SIM.");
  Serial1.println("AT+CPIN?");
  getResponse();
}

void kontrolaOperatora() {
  Serial.println("Kontrola operatora.");
  Serial1.println("AT+COPS?");
  getResponse();
}

void kontrolaSignalu() {
  Serial.println("Kontrola sily signalu.");
  Serial1.println("AT+CSQ");
  getResponse();
}

void registrace() {
  Serial.println("Manualni registrace.");
  Serial1.println("AT+CREG?");
  getResponse();
}

void nastaveniModuZarizeni() {
  Serial.println("Nastaveni modu.");
  /* (1 = plná funkčnost, 2 = pouze příjem, 3 = pouze vysílán?, 4 = zakáže příjem i vysílán?) */
  Serial1.println("AT+CFUN=1");
  getResponse();
}

void zkontrolujPrichoziHovor() {
  uint8_t var;
  Serial1.println("AT+CLCC");
  delay(100);
  var = Serial1.available();
  if (0 < var) {
    Serial1.readBytes(serialBuffer, var);
    Serial.write(serialBuffer, var);
    Serial.println();
    if (strstr(serialBuffer, "RING") != NULL) {
      callReceived = true;
      Serial.println("Prichozi hovor detekovan!");
      hovor = false;
    }
  }
}

void getResponse() {
  uint8_t var;
  delay(100);
  var = Serial1.available();
  if (0 < var) {
    Serial1.readBytes(serialBuffer, var);
    Serial.write(serialBuffer, var);
    Serial.println();
  }
  delay(100);
}
