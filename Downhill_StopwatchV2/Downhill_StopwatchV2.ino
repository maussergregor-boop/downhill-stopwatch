/*
Titel: Downhill Stopwatch 
Author: Gregor Mausser 
*/
//-------------Bibliotheken-------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>          // BLE für ESP32-C3
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>        //Daten-Pakete lesen und schreiben

// BLE-Dienst und Merkmale passend zu data/index.html
static const char* BLE_SERVICE_UUID   = "12345678-1234-5678-1234-56789abcdef0";
static const char* BLE_SETTINGS_UUID = "12345678-1234-5678-1234-56789abcdef1";
static const char* BLE_TIME_UUID     = "12345678-1234-5678-1234-56789abcdef2";
BLECharacteristic* settingsCharacteristic = nullptr;
BLECharacteristic* timeCharacteristic = nullptr;

//-------------Pins------------- 
constexpr int ButtonPin = 2;         //Start/Stopp Taster
constexpr int ButtonResetPin = 3;    //Reset-Taster (nutzt Interrupt)
constexpr int BuzzerPin = 7;         //Akustischer Signalgeber (Piepser)
constexpr int RGB_red   = 4;         //rote LED
constexpr int RGB_green = 5;         //grüne LED
constexpr int RGB_blue  = 6;         //blaue LED
constexpr int HALL_Pin  = 10;        //Sensor am Rad für km/h und Frühstart

//-------------Display-------------
const int ScreenHeight = 64;         //OLED-Display Höhe in Pixeln
const int ScreenWidth  = 128;        //OLED-Display Breite in Pixeln
Adafruit_SSD1306 display(ScreenWidth, ScreenHeight, &Wire, -1); //Display-Objekt erstellen

//-------------Zeit- und Rennvariablen//-------------
unsigned long duration;              //Gesamtzeit des Laufs in Millisekunden
int minute, seconds, milliseconds;   //Aufgeteilte Zeitkomponenten für die Anzeige

unsigned long starttime = 0;         //Exakter Startzeitpunkt (in ms) 
unsigned long endtime   = 0;         //Exakter Zielzeitpunkt (in ms)

bool timerRunning = false;           //Status: Läuft die Stoppuhr aktuell?
bool stopMeasure  = false;           //Status: Wurde die Zeit im Ziel gestoppt?

////-------------Anzeigedauer-Steuerung per Zeitstempel-------------
unsigned long showStart = 0;         
unsigned long notShowStart = 0;
unsigned long showMeasurements = 0;
unsigned long notShowMeasurements = 0;

int setting_endtime   = 5;           //Wie viele Sekunden die Zielzeit sichtbar bleibt
int setting_countdown = 10;          //Startwert für den Countdown (z.B. 10 Sekunden)
int showingSTART      = 5;           //Wie viele Sekunden das Wort "Start!"/grünes Led aufleuchtet
bool setting_buzzer   = true;        // Töne ein-/auszuschalten

//-------------HALL-Sensor & km/h-------------
volatile unsigned long lastPulse = 0;      //Zeitstempel des letzten Rad-Impulses (µs)
volatile unsigned long pulseInterval = 0;  //Zeit für genau eine Radumdrehung (µs)
volatile bool movementDetected = false;    //Status-Flag: Hat sich das Rad gedreht?
volatile unsigned long hallPulseCount = 0;  //Gesamtzahl erkannter Hall-Impulse
unsigned long lastLoggedHallPulseCount = 0; //Für Debug-Ausgabe im loop()
const float WHEEL_CIRCUMFERENCE = 2.2;     //Radumfang in Metern (für die Berechnung)
float currentKmh = 0.0;                    //Speichert das berechnete Tempo im Hintergrund
unsigned long lastPulseTime = 0;           //Zeitstempel für die Stillstandserkennung (ms)

//-------------Start Logik-------------
unsigned long countdownStart = 0;    //Startzeitpunkt des Countdowns
bool startReleased = false;          //Status: Ist die Startlinie offiziell freigegeben?
const int falseStartWindow = 5;      //Die letzten x Sekunden, in denen ein Frühstart droht
int lastRemainingSec = -1;           //Hilfsvariable, damit der Buzzer nur 1x pro Sekunde piept

//-------------Button Entprellung-------------
bool lastReading = HIGH;                //letzter roher Zustand des Startknopfs
bool stableState = HIGH;                //entprellter, stabiler Zustand des Startknopfs
unsigned long lastDebounceTime = 0;     //letzte Zeitänderung des Knopfs
const unsigned long debounceDelay = 30; //Entprell-Verzögerung in Millisekunden

//-------------Zustands-Automat (States)-------------
enum mode {
  READY,         //Wartemodus vor dem Rennen
  START,         //Countdown läuft runter
  TIME_MEASURE,  //Rennen läuft aktiv (Messung im Hintergrund)
  FALSE_START,   //Frühstart erkannt 
  STOP           //Zielzeit wird angezeigt
};

mode state = READY; //Das System startet immer im READY-Modus

const char* stateName(mode value) {
  switch (value) {
    case READY: return "READY";
    case START: return "START/COUNTDOWN";
    case TIME_MEASURE: return "TIME_MEASURE";
    case FALSE_START: return "FALSE_START";
    case STOP: return "STOP";
    default: return "UNKNOWN";
  }
}

//-------------Interrupt-Service-Routinen (ISR)-------------
volatile bool resetPressed = false;
//Wird sofort ausgelöst, wenn der Reset-Knopf gedrückt wird
void IRAM_ATTR myISR() {
  resetPressed = true;
}

//Wird sofort ausgelöst, wenn der Magnet am Hall-Sensor vorbeifliegt
void IRAM_ATTR onPulse() {
  unsigned long now = micros();
  //15ms Sperre verhindert doppelte Fehlsignale durch Vibrationen
  if (now - lastPulse > 15000) { 
    pulseInterval = now - lastPulse;
    lastPulse = now;
    hallPulseCount++;
    movementDetected = true;
  }
}

//-------------Funktion: Zentrierte Schrift auf OLED-------------
void printCentered(const char* text, int textSize, int y) {
  int charWidth = 6 * textSize;
  int x = (ScreenWidth - (int)strlen(text) * charWidth) / 2;
  if (x < 0) x = 0; //Verhindert negative Koordinaten bei zu langem Text

  display.setTextSize(textSize);
  display.setTextColor(WHITE);
  display.setCursor(x, y);
  display.print(text);
}

//-------------Funktion: Start/Stop Taster abfragen-------------
void button_pushed(mode next_state) {
  bool reading = digitalRead(ButtonPin);

  //Wenn sich der Zustand des Pins ändert, Entprell-Timer zurücksetzen
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  //Wenn das Signal länger als 30 ms stabil ist, ist es ein echter Tastendruck
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableState) {
      stableState = reading;
      //Taste wurde gedrückt 
      if (stableState == LOW) {
        Serial.printf("[BUTTON] Start/Stopp gedrueckt in Zustand %s\n", stateName(state));
        mode oldState = state;
        state = next_state; //In den übergebenen Zustand wechseln
        if (oldState != state) {
          Serial.printf("[STATE] %s -> %s\n", stateName(oldState), stateName(state));
        }

        //Spezielle Vorbereitungen, wenn der Countdown gestartet wird
        if (next_state == START) {
          countdownStart = millis();
          Serial.printf("[COUNTDOWN] gestartet, Dauer=%d s\n", setting_countdown);
          startReleased = false;
          lastRemainingSec = -1;
          movementDetected = false;
        }
      }
    }
  }
  lastReading = reading;
}

//-------------Funktion: Hardware-Reset prüfen-------------
void reset_pushed() {
  if (resetPressed) {
    Serial.println("[RESET] Reset-Taster erkannt; Lauf wird zurueckgesetzt.");
    timerRunning = false;
    stopMeasure = false;
    currentKmh = 0.0;
    resetPressed = false;
    state = READY;  //Sofortiger Abbruch und zurück zum Start
  }
}

//-------------Bluetooth Low Energy Funktionen-------------
void parseBluetoothSettings(String json);

class SettingsCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String json = String(characteristic->getValue().c_str());
    Serial.print("[BLE RX] ");
    Serial.println(json);
    parseBluetoothSettings(json);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { Serial.println("[BLE] App verbunden."); }
  void onDisconnect(BLEServer*) override {
    Serial.println("[BLE] App getrennt; Werbung wird neu gestartet.");
    BLEDevice::startAdvertising();
  }
};
//Zerlegt die Nachricht von der App und ändert die Arduino-Variablen
void parseBluetoothSettings(String json) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (!error) {
    //Wenn die App einen Wert schickt, überschreiben wir unsere Variablen
    if (doc.containsKey("show_endtime"))   setting_endtime = doc["show_endtime"];
    if (doc.containsKey("countdown"))      setting_countdown = doc["countdown"];
    if (doc.containsKey("showing_start"))  showingSTART = doc["showing_start"];
    if (doc.containsKey("buzzer_enabled")) setting_buzzer = doc["buzzer_enabled"];

    Serial.printf("[BLE SETTINGS] Endzeit=%d s, Countdown=%d s, Startanzeige=%d s, Buzzer=%s\n",
                  setting_endtime, setting_countdown, showingSTART,
                  setting_buzzer ? "AN" : "AUS");
  } else {
    Serial.print("[BLE JSON FEHLER] ");
    Serial.println(error.c_str());
  }
}

//Schickt die gestoppte Zeit als Paket an die App
void sendTimeToApp(unsigned long finalMs) {
  JsonDocument doc;
  doc["type"] = "NEW_TIME";
  doc["ms"] = finalMs; //Zeit in Millisekunden verpacken
  
  String output;
  serializeJson(doc, output);
  Serial.print("[BLE TX] ");
  Serial.println(output);
  if (timeCharacteristic) {
    timeCharacteristic->setValue(output.c_str());
    timeCharacteristic->notify();
  } else {
    Serial.println("[BLE WARNUNG] Zeit-Merkmal nicht initialisiert.");
  }
}



//-------------Setup-------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] ESP32-C3 Downhill Timer startet.");
  Serial.flush();

  //Pin-Modi festlegen
  pinMode(BuzzerPin, OUTPUT);
  pinMode(ButtonPin, INPUT_PULLUP);
  pinMode(ButtonResetPin, INPUT_PULLUP);
  pinMode(RGB_red, OUTPUT);
  pinMode(RGB_green, OUTPUT);
  pinMode(RGB_blue, OUTPUT);
  pinMode(HALL_Pin, INPUT_PULLUP);

  //Hardware-Interrupts an die Pins koppeln 
  attachInterrupt(digitalPinToInterrupt(ButtonResetPin), myISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_Pin), onPulse, FALLING);

  Wire.begin(8, 9); //I2C-Pins für den ESP32 definieren

  //OLED initialisieren
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED FEHLER] Display bei I2C-Adresse 0x3C nicht gefunden; Setup haelt an.");
    analogWrite(RGB_red, 255);
    analogWrite(RGB_green, 255);
    while (true); //Stoppt das System bei Displayfehler
  }

  Serial.println("[OLED] Display initialisiert.");
  display.clearDisplay();
  display.display();

  BLEDevice::init("ESP_Downhill_Timer");
  BLEDevice::setMTU(128);
  BLEServer* bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());
  BLEService* bleService = bleServer->createService(BLE_SERVICE_UUID);
  settingsCharacteristic = bleService->createCharacteristic(BLE_SETTINGS_UUID, BLECharacteristic::PROPERTY_WRITE);
  settingsCharacteristic->setCallbacks(new SettingsCallbacks());
  timeCharacteristic = bleService->createCharacteristic(BLE_TIME_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  timeCharacteristic->addDescriptor(new BLE2902());
  bleService->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[SETUP] BLE gestartet: ESP_Downhill_Timer; wartet auf App-Verbindung.");
}


//--------------Loop-------------
void loop() {
  reset_pushed();


  // ISR-Ausgaben nicht direkt im Interrupt schreiben: erkannte Impulse hier protokollieren.
  if (hallPulseCount != lastLoggedHallPulseCount) {
    lastLoggedHallPulseCount = hallPulseCount;
    Serial.printf("[HALL] Magnet erkannt! Impulszaehler=%lu, Intervall=%lu us, Zustand=%s\n",
                  hallPulseCount, pulseInterval, stateName(state));
  }

  switch (state) {
    
    //-------------READY-------------
    case READY:
      timerRunning = false;
      digitalWrite(RGB_red, LOW);
      digitalWrite(RGB_green, LOW);
      digitalWrite(RGB_blue, LOW);

      display.clearDisplay();
      printCentered("Ready", 2, 5);
      printCentered("to", 2, 25);
      printCentered("Race", 2, 45);
      display.display();

      button_pushed(START); //Wartet auf Start Button Signal, um Countdown zu starten
      break;

    //-------------START-------------
    case START: {
      unsigned long elapsed = millis() - countdownStart;
      int remaining = setting_countdown - (elapsed / 1000);

      //Countdown groß anzeigen
      display.clearDisplay();
      char buf[4];
      sprintf(buf, "%d", remaining);
      printCentered(buf, 3, 10);
      display.display();

      // Über 5 Sekunden bleibt die rote LED durchgehend an
      if (remaining > 5) {
        digitalWrite(RGB_red, HIGH);
      } else {
        // In den letzten 5 Sekunden blitzt die LED genau zu Beginn jeder Sekunde 
        // für 200 Millisekunden lang auf (synchronisiert mit dem Piepser)
        if (elapsed % 1000 < 200 && remaining > 0) {
          digitalWrite(RGB_red, HIGH);
        } else {
          digitalWrite(RGB_red, LOW);
        }
      }

      //Akustischer Takt: In den letzten 5 Sekunden jede Sekunde 1x kurz piepen
      if (setting_buzzer && remaining <= 5 && remaining > 0 && remaining != lastRemainingSec) {
        tone(BuzzerPin, 1000, 150); 
        lastRemainingSec = remaining;
      }

      //Startfreigabe erfolgt in den letzten 5 Sekunden!
      if (remaining <= 5) {
        startReleased = true;
      }

      //Start-Trigger: Sobald startReleased (in den letzten 5 Sek) TRUE ist UND das Rad rollt
      if (startReleased && movementDetected) {
        Serial.println("[START] Radbewegung nach Startfreigabe erkannt; Zeitmessung startet.");
        starttime = millis(); 
        showStart = millis();
        timerRunning = true;
        movementDetected = false;
        digitalWrite(RGB_red, LOW);     
        state = TIME_MEASURE;
        Serial.println("[STATE] START/COUNTDOWN -> TIME_MEASURE");
      }

      // Automatischer Start, wenn der Countdown regulär auf 0 abgelaufen ist
      if (remaining <= 0) {
        Serial.println("[START] Countdown abgelaufen; Zeitmessung startet automatisch.");
        if (setting_buzzer) {
          tone(BuzzerPin, 2000, 500); //Hoher Startton 
        }
        starttime = millis(); 
        showStart = millis();
        timerRunning = true;
        movementDetected = false;   
        digitalWrite(RGB_red, LOW);     
        state = TIME_MEASURE;
        Serial.println("[STATE] START/COUNTDOWN -> TIME_MEASURE");
      }
      
      break;
    }

    // Zustand: Zeitmessung läuft (Fahrer befindet sich auf der Strecke)
    case TIME_MEASURE: {
      unsigned long currentMillis = millis();

      // Startzeitpunkt beim ersten Eintritt fixieren
      if (!timerRunning) {
        showStart = currentMillis;
        starttime = currentMillis;
        timerRunning = true;
      }

      // UNSICHTBARE km/h BERECHNUNG (Läuft unbemerkt im Hintergrund mit)
      if (movementDetected) {
        movementDetected = false; 
        lastPulseTime = currentMillis; // Zeit für Stillstands-Timer merken
        if (pulseInterval > 0) {
          // Formel: Umfang / (Mikrosekunden zu Sekunden gewandelt) * 3.6 = km/h
          float speedMps = WHEEL_CIRCUMFERENCE / (pulseInterval / 1000000.0);
          currentKmh = speedMps * 3.6;
          Serial.printf("[SPEED] %.2f km/h\n", currentKmh);
        }
      }

      // Stillstands-Timeout: Wenn sich das Rad 2,5 Sek nicht dreht, km/h auf 0 setzen
      if (currentMillis - lastPulseTime > 2500) {
        currentKmh = 0.0;
      }

      notShowStart = currentMillis;

      // Display-Anzeige während der Fahrt
      if (notShowStart - showStart <= (unsigned long)showingSTART * 1000) {
        display.clearDisplay();
        printCentered("Start!", 3, 20); //Zeigt die ersten Sekunden "Start!" an
        display.display();
        digitalWrite(RGB_green, HIGH);
      } else {
        display.clearDisplay();
        display.display();         
        digitalWrite(RGB_green, LOW);
      }

      button_pushed(STOP); //Stopp bei Knopfdruck
      break;
    }

    // Zustand: Ziel durchfahren, Zeit gestoppt
    case STOP:
      if (!stopMeasure) {
        digitalWrite(RGB_green, LOW);
        showMeasurements = millis();
        endtime = millis();

        //Endzeit berechnen und aufsplitten
        duration = endtime - starttime;
        minute   = (duration / 1000) / 60;
        seconds  = (duration / 1000) % 60;
        milliseconds = duration % 1000;
        
        sendTimeToApp(duration);

        Serial.printf("[STOP] Lauf beendet: %lu ms (%d:%02d.%03d)\n",
                      duration, minute, seconds, milliseconds);

        stopMeasure = true;
      }

      // Zeitstempel JETZT erst aktualisieren, um Fehlberechnungen zu verhindern
      notShowMeasurements = millis();

      // Endzeit für die Dauer von 'setting_endtime' auf dem OLED anzeigen
      if (notShowMeasurements - showMeasurements <= (unsigned long)setting_endtime * 1000) {
        digitalWrite(RGB_blue, HIGH);

        char timeStr[16];
        sprintf(timeStr, "%d:%02d.%03d", minute, seconds, milliseconds);

        display.clearDisplay();
        printCentered(timeStr, 2, 25); // Cleane Rennzeit ohne km/h ausgeben
        display.display();

        // WICHTIG: Erst nach einer kurzen Sperre (z.B. 500ms) einen Neustart erlauben,
        // damit ein langer Tastendruck nicht sofort den nächsten State triggert!
        if (notShowMeasurements - showMeasurements > 500) {
          button_pushed(READY); 
        }

      } else {
        // Nach Ablauf der Anzeigezeit automatisch zurück in die Startbereitschaft
        digitalWrite(RGB_blue, LOW);
        stopMeasure = false;
        state = READY;
      }
      break;

    
    case FALSE_START:
      Serial.println("[FALSE START] Fruehstart-Zustand aktiv.");
      if (setting_buzzer) {
        tone(BuzzerPin, 300, 1000); //Tiefer, langer Fehlstart-Ton
      }
      display.clearDisplay();
      printCentered("FALSE START", 2, 20);
      display.display();

      //Farbe: orange
      analogWrite(RGB_red, 255);
      analogWrite(RGB_green, 165);
      delay(2000);                
      movementDetected = false;
      state = READY;              //zurück in den "Warte" State
      break;
  }

  delay(10); //macht Abläufe stabiler (entschleunigt ihn)
}
