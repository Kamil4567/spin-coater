/*
 * =====================================================================================
 *  STANOWISKO TESTOWE (spin-coater) - PROGRAM SERWISOWY / TEST SPRZETU
 *  Plytka: ESP32 DevKit 38-pin (ESP32-WROOM-32), Arduino-ESP32 2.x (espressif32 6.x)
 * =====================================================================================
 *
 *  PRZYPISANIE PINOW
 *  -------------------------------------------------------------------------------------
 *  GPIO21  - OLED SSD1306 128x64 I2C  SDA   (adres 0x3C, zasilanie 3.3V)
 *  GPIO22  - OLED SSD1306 128x64 I2C  SCL
 *
 *  GPIO27  - przycisk STOP  (NC, zwiera do GND)  INPUT_PULLUP + przerwanie RISING
 *              normalnie: zwarty do GND -> LOW,  wcisniety / przerwany przewod -> HIGH = STOP
 *              (fail-safe: urwany kabel = STOP). Przy dlugim kablu dodac zewn. 10k do 3.3V
 *              i 100nF do GND.
 *  GPIO18  - przycisk MINUS (NO, zwiera do GND)  INPUT_PULLUP
 *  GPIO19  - przycisk OK    (NO, zwiera do GND)  INPUT_PULLUP
 *  GPIO23  - przycisk PLUS  (NO, zwiera do GND)  INPUT_PULLUP
 *
 *  GPIO26  - buzzer pasywny (LEDC kanal 2 / timer 1). Buzzer magnetyczny przez tranzystor
 *              NPN (np. S8050/BC547: baza przez 1k, dioda 1N4148 rownolegle do buzzera,
 *              buzzer zasilany z 5V/3.3V). Maly piezo mozna dac bezposrednio przez 100R.
 *
 *  GPIO25  - sygnal do ESC (RED HOBBY AM32 80A 2-4S), PWM serwo, 1 kierunek: 1000 = stop, 1000->2000 gaz (servo ESC: 1000/2000/1500/db30, 3D wylaczony), 50/100/200/400/490 Hz (menu)
 *              (LEDC kanal 0 / timer 0). Masa ESC MUSI byc wspolna z masa ESP32.
 *              Poziom 3.3V jest akceptowany przez AM32.
 *
 *  GPIO13  - wykrycie zasilania ESC, INPUT_PULLUP, aktywne LOW (LOW = ESC zasilony).
 *              Propozycja: transoptor PC817 - dioda z +V baterii/zasilacza przez 3.3k do GND,
 *              kolektor -> GPIO13, emiter -> GND.
 *
 *  GPIO34  - potencjometr (ADC1_CH6, wejscie tylko-input). Skrajne: 3.3V i GND,
 *              suwak -> GPIO34, 100nF do GND.
 *
 *  GPIO35  - pomiar napiecia zasilania (ADC1_CH7) przez dzielnik:
 *              Vin --[ R1 = 120k ]--+--[ R2 = 20k ]-- GND      (+100nF rownolegle do R2)
 *                                   |
 *                                 GPIO35
 *              18.0V -> 2.57V na ADC (max ~21.7V przy 3.1V na ADC, zapas na 4S 16.8V)
 *              Vin = Vadc * (R1+R2)/R2 = Vadc * 7.0.  Rezystory 1%. Kalibracja: VIN_CAL.
 *
 *  GPIO16  - UART2 RX  <- Nextion TX  (NX3224T024)  - tylko przypisanie, nic nie wysylamy
 *  GPIO17  - UART2 TX  -> Nextion RX
 *              Nextion: zasilanie 5V (~90 mA), poziomy UART 3.3V TTL (wejscie RX tolerancyjne
 *              na 5V) -> mozna podlaczyc bezposrednio do ESP32, wspolna masa.
 *
 *  Wolne (na pozniej, np. transoptor RPM): GPIO4, GPIO14, GPIO32, GPIO33.
 *  Unikane: GPIO0/2/5/12/15 (strapping), GPIO6-11 (flash), GPIO1/3 (USB-UART).
 *
 *  OBSLUGA
 *  -------------------------------------------------------------------------------------
 *  Menu:   MINUS/PLUS - wybor,  OK klik - wejscie,  OK przytrzymanie - powrot do menu.
 *  STOP:   w dowolnym momencie natychmiast (w przerwaniu) zatrzymuje silnik (sygnal neutralny 1500 us),
 *          przerywa dzwiek buzzera. Ponowny start silnika wymaga: zwolnienia STOP,
 *          potencjometru na 0 i ponownego nacisniecia OK na stronie ESC.
 *  ESC tester: OK = start/stop, +/- = limit maksymalnego gazu (domyslnie 50%).
 *          Kierunek (tryb 3D) i czestotliwosc PWM: menu "ESC ustawienia".
 *  Programator ESC: most USB (port COM plytki) <-> przewod sygnalowy ESC (GPIO25, 1-wire,
 *          open-drain, half-duplex). PWM jest wtedy wylaczony. Kolejnosc: wejdz w tryb,
 *          odlacz i podlacz zasilanie ESC (AM32 wchodzi w bootloader gdy linia jest wysoko),
 *          polacz konfigurator AM32 ("Direct Connect") z portem COM plytki.
 *          MINUS = echo TAK/NIE, PLUS = predkosc (19200/38400/115200), OK dlugo / STOP = wyjscie.
 *          Tryb jest zapamietywany w NVS - jesli otwarcie portu zresetuje ESP32, wroci do niego.
 *          Start mozliwy tylko gdy pot = 0, STOP zwolniony i ESC zasilony.
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"

#if ESP_ARDUINO_VERSION_MAJOR >= 3
#error "Program napisany pod Arduino-ESP32 2.x (platforma espressif32 6.x) - API LEDC w 3.x jest inne"
#endif

// Log na USB - wylaczany w trybie programatora ESC (USB jest wtedy zajete przez most).
bool g_logEnabled = true;
#define LOGF(...) do { if (g_logEnabled) Serial.printf(__VA_ARGS__); } while (0)
#define LOGLN(x)  do { if (g_logEnabled) Serial.println(x); } while (0)

// ------------------------------------------------------------------ piny
constexpr uint8_t PIN_I2C_SDA    = 21;
constexpr uint8_t PIN_I2C_SCL    = 22;
constexpr uint8_t PIN_BTN_STOP   = 27;
constexpr uint8_t PIN_BTN_MINUS  = 18;
constexpr uint8_t PIN_BTN_OK     = 19;
constexpr uint8_t PIN_BTN_PLUS   = 23;
constexpr uint8_t PIN_BUZZER     = 26;
constexpr uint8_t PIN_ESC        = 25;
constexpr uint8_t PIN_ESC_PWR    = 13;
constexpr uint8_t PIN_POT        = 34;
constexpr uint8_t PIN_VIN        = 35;
constexpr uint8_t PIN_NEXTION_RX = 16;
constexpr uint8_t PIN_NEXTION_TX = 17;

// ------------------------------------------------------------------ konfiguracja
constexpr uint8_t  ESC_LEDC_CH   = 0;      // timer 0
// Czestotliwosc PWM do ESC wybierana w menu "ESC ustawienia" (domyslnie ESC_HZ_LIST[0]).
// Po zmianie odlacz i podlacz zasilanie ESC - AM32 wykrywa typ sygnalu przy starcie.
const uint16_t ESC_HZ_LIST[] = {50, 100, 200, 400, 490};
constexpr uint8_t  ESC_HZ_N      = sizeof(ESC_HZ_LIST) / sizeof(ESC_HZ_LIST[0]);
constexpr uint8_t  ESC_PWM_BITS  = 16;
// Zakres impulsow - MUSI odpowiadac ustawieniom "Servo" w ESC (AM32 config tool):
//   low 1000, high 2000, neutral 1500, dead band 30.
// (Szerszy zakres 810/2190 nie dzialal na tym ESC.)
// AM32 przyjmuje tylko impulsy 800-2200 us - nie przekraczac tych granic!
constexpr uint16_t ESC_US_MIN    = 1000;
constexpr uint16_t ESC_US_MAX    = 2000;
constexpr uint16_t ESC_US_NEUTRAL = 1500;
// Martwa strefa ESC wokol neutralu (tylko tryb 3D): dead band 30 -> 60 jedn. z 1000
// na 500 us = ~30 us. Pierwszy krok gazu startuje tuz za nia.
constexpr uint16_t ESC_DEADBAND_US = 32;
// Tryb ESC - MUSI odpowiadac konfiguracji ESC (AM32 "bidirectional / 3D")!
//   true : 1500 us = stop, 1500->2000 = prawo, 1500->1000 = lewo
//   false: 1000 us = stop, 1000->2000 = gaz w jedna strone  (obecna konfiguracja ESC)
// Celowo stala, a nie opcja w menu: zly tryb = silnik rusza pelnym gazem przy "stopie".
constexpr bool     ESC_BIDIRECTIONAL = false;   // ESC: tryb 3D WYLACZONY
constexpr uint16_t ESC_US_IDLE   = ESC_BIDIRECTIONAL ? ESC_US_NEUTRAL : ESC_US_MIN;
// Pelny zakres jednego kierunku (us): 3D -> 500 w kazda strone, 1-kier. -> 1000.
constexpr uint16_t ESC_US_SPAN   = ESC_BIDIRECTIONAL
    ? ((ESC_US_MAX - ESC_US_NEUTRAL) < (ESC_US_NEUTRAL - ESC_US_MIN) ? ESC_US_MAX - ESC_US_NEUTRAL : ESC_US_NEUTRAL - ESC_US_MIN)
    : ESC_US_MAX - ESC_US_MIN;
constexpr bool     ESC_REQUIRE_PWR = true; // start silnika tylko przy wykrytym zasilaniu ESC

constexpr uint8_t  BUZ_LEDC_CH   = 2;      // timer 1 - inny timer niz ESC!

constexpr float    VIN_R1        = 120000.0f;
constexpr float    VIN_R2        = 20000.0f;
constexpr float    VIN_CAL       = 1.00f;  // korekta po porownaniu z multimetrem

constexpr float    POT_ZERO_PCT  = 2.0f;   // ponizej = zero (martwa strefa / warunek startu)

constexpr uint32_t DEBOUNCE_MS   = 30;
constexpr uint32_t HOLD_MS       = 700;
constexpr uint32_t REPEAT_MS     = 150;
constexpr uint32_t BTN_PAGE_EXIT_MS = 3000; // strona testu przyciskow: OK trzymany 3s = wyjscie

constexpr uint8_t  OLED_ADDR     = 0x3C;

// ------------------------------------------------------------------ OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false;

// ------------------------------------------------------------------ ESC (wspoldzielone z ISR)
portMUX_TYPE escMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     g_motorArmed = false;
volatile uint16_t g_escUs      = ESC_US_IDLE;
volatile uint32_t g_stopEvents = 0;
volatile uint32_t g_escPeriodUs = 1000000UL / ESC_HZ_LIST[0];
uint16_t escPwmHz = ESC_HZ_LIST[0];

static inline uint32_t usToDuty(uint16_t us) {
  return (uint32_t)us * (1UL << ESC_PWM_BITS) / g_escPeriodUs;
}

// Wywolywane z przerwania STOP oraz z petli (gdy STOP aktywny) - natychmiast gasi silnik.
void IRAM_ATTR emergencyStop() {
  portENTER_CRITICAL_SAFE(&escMux);
  g_motorArmed = false;
  g_escUs = ESC_US_IDLE;
  ledcWrite(ESC_LEDC_CH, usToDuty(ESC_US_IDLE));
  g_stopEvents = g_stopEvents + 1;
  portEXIT_CRITICAL_SAFE(&escMux);
}

void IRAM_ATTR stopIsr() { emergencyStop(); }

bool stopActive() { return digitalRead(PIN_BTN_STOP) == HIGH; }  // NC rozwarty

// Zapis wartosci - w sekcji krytycznej, zeby ISR nie zostal "nadpisany" starym gazem.
void escSetUs(uint16_t us) {
  us = constrain(us, ESC_US_MIN, ESC_US_MAX);
  portENTER_CRITICAL(&escMux);
  if (!g_motorArmed) us = ESC_US_IDLE;
  g_escUs = us;
  ledcWrite(ESC_LEDC_CH, usToDuty(us));
  portEXIT_CRITICAL(&escMux);
}

void escDisarm() {
  portENTER_CRITICAL(&escMux);
  g_motorArmed = false;
  g_escUs = ESC_US_IDLE;
  ledcWrite(ESC_LEDC_CH, usToDuty(ESC_US_IDLE));
  portEXIT_CRITICAL(&escMux);
}

// Zmiana czestotliwosci PWM - zawsze z rozbrojonym silnikiem (wyjscie = ESC_US_IDLE).
void escSetFreq(uint16_t hz) {
  escDisarm();
  ledcSetup(ESC_LEDC_CH, hz, ESC_PWM_BITS);   // tylko timer; kanal pozostaje podpiety
  portENTER_CRITICAL(&escMux);
  g_escPeriodUs = 1000000UL / hz;
  ledcWrite(ESC_LEDC_CH, usToDuty(ESC_US_IDLE));
  portEXIT_CRITICAL(&escMux);
  escPwmHz = hz;
  LOGF("[ESC] PWM %u Hz\n", hz);
}

bool escArm() {
  bool ok;
  portENTER_CRITICAL(&escMux);
  ok = (digitalRead(PIN_BTN_STOP) == LOW);   // sprawdzone atomowo wzgledem ISR
  if (ok) g_motorArmed = true;
  portEXIT_CRITICAL(&escMux);
  return ok;
}

// ------------------------------------------------------------------ przyciski
struct Button {
  uint8_t pin;
  const char *name;
  bool stable = false;      // true = wcisniety (po debouncingu)
  bool lastRaw = false;
  uint32_t lastChange = 0;
  uint32_t pressedAt = 0;
  uint32_t lastRepeat = 0;
  bool holdFired = false;
  // zdarzenia (kasowane po kazdej iteracji petli)
  bool evClick = false, evHold = false, evRepeat = false;
  uint16_t cntClick = 0, cntHold = 0;

  Button(uint8_t p, const char *n) : pin(p), name(n) {}

  void update(uint32_t now) {
    evClick = evHold = evRepeat = false;
    bool raw = digitalRead(pin) == LOW;
    if (raw != lastRaw) { lastRaw = raw; lastChange = now; }
    if (now - lastChange >= DEBOUNCE_MS && raw != stable) {
      stable = raw;
      if (stable) { pressedAt = now; holdFired = false; }
      else if (!holdFired) { evClick = true; cntClick++; }
    }
    if (stable && !holdFired && now - pressedAt >= HOLD_MS) {
      holdFired = true; evHold = true; cntHold++; lastRepeat = now;
    }
    if (stable && holdFired && now - lastRepeat >= REPEAT_MS) {
      lastRepeat = now; evRepeat = true;
    }
  }
  uint32_t heldFor(uint32_t now) const { return stable ? now - pressedAt : 0; }
};

Button btnMinus(PIN_BTN_MINUS, "MINUS");
Button btnOk(PIN_BTN_OK, "OK");
Button btnPlus(PIN_BTN_PLUS, "PLUS");
Button *buttons[] = {&btnMinus, &btnOk, &btnPlus};

const char *lastEvtName = "-";
const char *lastEvtType = "";

// ------------------------------------------------------------------ buzzer (nieblokujacy)
struct Note { uint16_t hz; uint16_t ms; };
const Note MEL_START[] = {{1047, 90}, {0, 30}, {1319, 90}, {0, 30}, {1568, 90}, {0, 30}, {2093, 250}};
const Note MEL_ALARM[] = {{2500, 120}, {1500, 120}, {2500, 120}, {1500, 120}};
const Note MEL_OK[]    = {{2000, 60}};
const Note MEL_ERR[]   = {{400, 150}, {0, 60}, {400, 150}};
const Note MEL_CLICK[] = {{3000, 12}};
const Note MEL_T500[]  = {{500, 500}};
const Note MEL_T2K[]   = {{2000, 500}};
const Note MEL_T4K[]   = {{4000, 500}};
#define MEL(x) x, (uint8_t)(sizeof(x) / sizeof(x[0]))

const Note *bzSeq = nullptr;
uint8_t bzLen = 0, bzIdx = 0;
uint32_t bzT0 = 0;
bool bzSweep = false;
uint16_t bzCurHz = 0xFFFF;
constexpr uint32_t SWEEP_MS = 2500;

void bzTone(uint16_t hz) {
  if (hz == bzCurHz) return;
  bzCurHz = hz;
  ledcWriteTone(BUZ_LEDC_CH, hz);  // 0 = cisza
}
void bzStop() { bzSeq = nullptr; bzSweep = false; bzTone(0); }
bool bzBusy() { return bzSeq || bzSweep; }
void bzPlay(const Note *s, uint8_t n) {
  bzSweep = false; bzSeq = s; bzLen = n; bzIdx = 0; bzT0 = millis();
  bzCurHz = 0xFFFF; bzTone(s[0].hz);
}
void bzStartSweep() { bzSeq = nullptr; bzSweep = true; bzT0 = millis(); }
void bzUpdate(uint32_t now) {
  if (bzSweep) {
    uint32_t t = now - bzT0;
    if (t >= SWEEP_MS) { bzStop(); return; }
    bzTone(200 + (uint32_t)4800 * t / SWEEP_MS);
    return;
  }
  if (!bzSeq) return;
  if (now - bzT0 >= bzSeq[bzIdx].ms) {
    if (++bzIdx >= bzLen) { bzStop(); return; }
    bzT0 = now;
    bzTone(bzSeq[bzIdx].hz);
  }
}
void keyClick() { if (!bzBusy()) bzPlay(MEL(MEL_CLICK)); }

// ------------------------------------------------------------------ wejscia analogowe
float potRawF = 0;     // 0..4095 (filtrowane)
float potPct = 0;      // 0..100 po martwej strefie
float vinAdcMv = 0;    // mV na pinie
float vinV = 0;        // napiecie zasilania
bool escPwr = false;
uint32_t lastAnalog = 0;
uint32_t nextionRxBytes = 0;

void readInputs(uint32_t now, bool init = false) {
  escPwr = digitalRead(PIN_ESC_PWR) == LOW;
  if (!init && now - lastAnalog < 5) return;
  lastAnalog = now;
  float pr = analogRead(PIN_POT);
  float mv = analogReadMilliVolts(PIN_VIN);
  const float a = init ? 1.0f : 0.15f;
  potRawF += (pr - potRawF) * a;
  vinAdcMv += (mv - vinAdcMv) * a;
  float pct = potRawF * 100.0f / 4095.0f;
  potPct = (pct < POT_ZERO_PCT) ? 0.0f : pct;
  vinV = vinAdcMv / 1000.0f * (VIN_R1 + VIN_R2) / VIN_R2 * VIN_CAL;
}

// ------------------------------------------------------------------ menu / strony
enum Page { PG_MENU, PG_INPUTS, PG_BUTTONS, PG_BUZZER, PG_ESC, PG_ESCFREQ, PG_ESCPROG, PG_INFO };
Page page = PG_MENU;
const char *MENU_ITEMS[] = {"Stan wejsc", "Test przyciskow", "Test buzzera", "ESC tester (servo)", "ESC ustawienia", "Programator ESC", "Piny / info"};
const Page  MENU_PAGES[] = {PG_INPUTS, PG_BUTTONS, PG_BUZZER, PG_ESC, PG_ESCFREQ, PG_ESCPROG, PG_INFO};
uint8_t escHzSel = 0;
int8_t  escDir = 1;          // tryb 3D: +1 = prawo (1500->2000), -1 = lewo (1500->1000)
uint8_t escSetRow = 0;
constexpr uint8_t MENU_N = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);
uint8_t menuSel = 0;

const char *BUZ_ITEMS[] = {"Ton  500 Hz", "Ton 2000 Hz", "Ton 4000 Hz", "Sweep 200-5000 Hz", "Melodia startowa", "Sygnal alarmu"};
constexpr uint8_t BUZ_N = sizeof(BUZ_ITEMS) / sizeof(BUZ_ITEMS[0]);
uint8_t buzSel = 0;

uint8_t escLimitPct = 50;
bool escStoppedByStop = false;
char escMsg[22] = "";
uint32_t escMsgUntil = 0;
uint32_t lastStopEventsSeen = 0;

// ------------------------------------------------------------------ programator ESC
// Przezroczysty most: USB (UART0) <-> UART1 na GPIO25 (jeden przewod, open-drain).
// Linia 1-wire odbiera tez wlasne nadawane bajty (echo) - tak jak przerobiony adapter USB-UART.
// Echo mozna wylaczyc (MINUS), gdyby konfigurator go nie oczekiwal.
const uint32_t PROG_BAUDS[] = {19200, 38400, 115200};
constexpr uint8_t PROG_BAUD_N = sizeof(PROG_BAUDS) / sizeof(PROG_BAUDS[0]);
uint8_t  progBaudIdx = 0;
bool     progEcho = true;
bool     progActive = false;
uint32_t progToEsc = 0, progFromEsc = 0, progPendingEcho = 0, progLastAct = 0;
Preferences prefs;

void progConfigPin() {
  // UART1 TX i RX na tym samym pinie, wyjscie open-drain + podciagniecie do 3.3V.
  // gpio_set_direction() podpina zwykle wyjscie GPIO, wiec UART podlaczamy ponownie po nim.
  gpio_set_direction((gpio_num_t)PIN_ESC, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode((gpio_num_t)PIN_ESC, GPIO_PULLUP_ONLY);
  pinMatrixOutAttach(PIN_ESC, U1TXD_OUT_IDX, false, false);
  pinMatrixInAttach(PIN_ESC, U1RXD_IN_IDX, false);
}

void progStart() {
  escDisarm();
  bzStop();
  ledcDetachPin(PIN_ESC);                       // koniec PWM na GPIO25
  uint32_t baud = PROG_BAUDS[progBaudIdx];
  LOGF("[PROG] start, %lu baud - log USB wylaczony\n", (unsigned long)baud);
  Serial.flush();
  g_logEnabled = false;
  Serial.updateBaudRate(baud);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(baud, SERIAL_8N1, PIN_ESC, PIN_ESC);
  progConfigPin();
  while (Serial.available()) Serial.read();
  while (Serial1.available()) Serial1.read();
  progToEsc = progFromEsc = progPendingEcho = progLastAct = 0;
  progActive = true;
  prefs.putBool("prog", true);
}

void progStop() {
  if (!progActive) return;
  Serial1.end();
  pinMatrixInDetach(U1RXD_IN_IDX, true, false);
  ledcAttachPin(PIN_ESC, ESC_LEDC_CH);          // z powrotem PWM, od razu sygnal neutralny
  escDisarm();
  Serial.flush();
  Serial.updateBaudRate(115200);
  progActive = false;
  prefs.putBool("prog", false);
  g_logEnabled = true;
  LOGLN("[PROG] koniec trybu programatora, PWM przywrocony");
}

void progSetBaud(uint8_t idx) {
  progBaudIdx = idx;
  uint32_t baud = PROG_BAUDS[idx];
  Serial.flush();
  Serial1.flush();
  Serial.updateBaudRate(baud);
  Serial1.updateBaudRate(baud);
  progPendingEcho = 0;
  prefs.putUChar("progBaud", idx);
}

void progUpdate(uint32_t now) {
  if (!progActive) return;
  uint8_t buf[128];
  int n;
  while ((n = Serial.available()) > 0) {        // PC -> ESC
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    for (int i = 0; i < n; i++) buf[i] = Serial.read();
    Serial1.write(buf, n);
    progToEsc += n;
    progPendingEcho += n;                       // te bajty wroca jako echo linii 1-wire
    progLastAct = now;
  }
  while ((n = Serial1.available()) > 0) {       // ESC -> PC
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    int m = 0;
    for (int i = 0; i < n; i++) {
      uint8_t b = Serial1.read();
      if (progPendingEcho) { progPendingEcho--; if (!progEcho) continue; }
      else progFromEsc++;
      buf[m++] = b;
    }
    if (m) Serial.write(buf, m);
    progLastAct = now;
  }
}

void setEscMsg(const char *m, uint32_t ms = 2000) {
  strncpy(escMsg, m, sizeof(escMsg) - 1);
  escMsg[sizeof(escMsg) - 1] = 0;
  escMsgUntil = millis() + ms;
}

void goPage(Page p) {
  LOGF("[MENU] strona %d\n", (int)p);
  if (page == PG_ESCPROG && p != PG_ESCPROG) progStop();
  if (page == PG_ESC || p == PG_ESC) escDisarm();
  if (page == PG_BUZZER) bzStop();
  page = p;
  if (p == PG_ESC) { escStoppedByStop = false; setEscMsg("OK=start +/-=limit", 3000); }
  if (p == PG_ESCPROG && !progActive) progStart();
}

// ------------------------------------------------------------------ logika stron
void handleMenu() {
  if (btnMinus.evClick || btnMinus.evRepeat) menuSel = (menuSel + MENU_N - 1) % MENU_N;
  if (btnPlus.evClick || btnPlus.evRepeat)   menuSel = (menuSel + 1) % MENU_N;
  if (btnOk.evClick) goPage(MENU_PAGES[menuSel]);
}

void handleButtonsPage(uint32_t now) {
  if (btnOk.heldFor(now) >= BTN_PAGE_EXIT_MS && page == PG_BUTTONS) goPage(PG_MENU);
}

void handleEscFreqPage() {
  if (btnOk.evHold) { goPage(PG_MENU); return; }
  const uint8_t rows = ESC_BIDIRECTIONAL ? 2 : 1;
  if (btnMinus.evClick) escSetRow = (escSetRow + rows - 1) % rows;
  if (btnPlus.evClick)  escSetRow = (escSetRow + 1) % rows;
  if (btnOk.evClick) {
    if (escSetRow == 0) {
      escHzSel = (escHzSel + 1) % ESC_HZ_N;
      escSetFreq(ESC_HZ_LIST[escHzSel]);
    } else {
      escDir = -escDir;
      LOGF("[ESC] kierunek %s\n", escDir > 0 ? "PRAWO" : "LEWO");
    }
    bzPlay(MEL(MEL_OK));
  }
}

void handleProgPage() {
  if (btnOk.evHold) { goPage(PG_MENU); return; }
  if (btnMinus.evClick) { progEcho = !progEcho; progPendingEcho = 0; prefs.putBool("progEcho", progEcho); }
  if (btnPlus.evClick)  progSetBaud((progBaudIdx + 1) % PROG_BAUD_N);
}

void handleBuzzerPage() {
  if (btnOk.evHold) { goPage(PG_MENU); return; }
  if (btnMinus.evClick || btnMinus.evRepeat) buzSel = (buzSel + BUZ_N - 1) % BUZ_N;
  if (btnPlus.evClick || btnPlus.evRepeat)   buzSel = (buzSel + 1) % BUZ_N;
  if (btnOk.evClick) {
    switch (buzSel) {
      case 0: bzPlay(MEL(MEL_T500)); break;
      case 1: bzPlay(MEL(MEL_T2K)); break;
      case 2: bzPlay(MEL(MEL_T4K)); break;
      case 3: bzStartSweep(); break;
      case 4: bzPlay(MEL(MEL_START)); break;
      case 5: bzPlay(MEL(MEL_ALARM)); break;
    }
  }
}

void handleEscPage() {
  if (btnOk.evHold) { goPage(PG_MENU); return; }

  if (btnMinus.evClick || btnMinus.evRepeat) { if (escLimitPct > 10) escLimitPct -= 5; }
  if (btnPlus.evClick || btnPlus.evRepeat)   { if (escLimitPct < 100) escLimitPct += 5; }

  if (btnOk.evClick) {
    if (g_motorArmed) {
      escDisarm();
      setEscMsg("Zatrzymano");
      bzPlay(MEL(MEL_OK));
      LOGLN("[ESC] stop (OK)");
    } else if (stopActive()) {
      setEscMsg("Zwolnij STOP!"); bzPlay(MEL(MEL_ERR));
    } else if (potPct > 0.0f) {
      setEscMsg("Ustaw pot na 0!"); bzPlay(MEL(MEL_ERR));
    } else if (ESC_REQUIRE_PWR && !escPwr) {
      setEscMsg("Brak zasilania ESC!"); bzPlay(MEL(MEL_ERR));
    } else if (escArm()) {
      escStoppedByStop = false;
      setEscMsg("PRACA - OK=stop", 1500);
      bzPlay(MEL(MEL_OK));
      LOGLN("[ESC] start");
    }
  }

  if (g_motorArmed && ESC_REQUIRE_PWR && !escPwr) {
    escDisarm();
    setEscMsg("Zanik zasil. ESC!", 3000);
    bzPlay(MEL(MEL_ERR));
    LOGLN("[ESC] zanik zasilania ESC -> stop");
  }

  float thr = potPct * escLimitPct / 100.0f;                 // 0..limit %
  if (ESC_BIDIRECTIONAL) {  // 1500 +/- (32..500) us, 0% = dokladnie 1500
    int off = 0;
    if (thr > 0.0f) off = ESC_DEADBAND_US + (int)(thr * (ESC_US_SPAN - ESC_DEADBAND_US) / 100.0f + 0.5f);
    escSetUs((uint16_t)(ESC_US_IDLE + escDir * off));
  } else {                  // 1000 .. 2000 us
    escSetUs(ESC_US_MIN + (uint16_t)(thr * ESC_US_SPAN / 100.0f + 0.5f));
  }
}

// ------------------------------------------------------------------ rysowanie
void drawTitle(const char *t) {
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(t);
  display.drawFastHLine(0, 9, 128, SSD1306_WHITE);
}

void drawList(const char *const *items, uint8_t n, uint8_t sel, int16_t y0, uint8_t rowH) {
  uint8_t vis = (64 - y0 + 1) / rowH;                    // ile wierszy miesci sie na ekranie
  uint8_t first = (sel >= vis) ? sel - vis + 1 : 0;      // przewijanie
  for (uint8_t i = first; i < n && i < first + vis; i++) {
    int16_t y = y0 + (i - first) * rowH;
    if (i == sel) {
      display.fillRect(0, y - 1, 128, rowH, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(2, y);
    display.print(items[i]);
  }
  display.setTextColor(SSD1306_WHITE);
}

void drawMenu() {
  drawTitle("MENU SERWISOWE");
  if (stopActive()) { display.setCursor(98, 0); display.print("STOP"); }
  drawList(MENU_ITEMS, MENU_N, menuSel, 12, 9);   // przewijana lista, bez stopki
}

void drawEscFreq() {
  drawTitle("ESC USTAWIENIA");
  char buf[22];
  snprintf(buf, sizeof(buf), "PWM:   %3u Hz", escPwmHz);
  const char *rows[2] = {buf, escDir > 0 ? "Kier.: PRAWO (>1500)" : "Kier.: LEWO  (<1500)"};
  drawList(rows, ESC_BIDIRECTIONAL ? 2 : 1, escSetRow, 12, 9);
  display.setCursor(0, 33); display.print(ESC_BIDIRECTIONAL ? "Tryb: 3D, stop=1500us" : "Tryb: 1-kier,stop=1000");
  display.setCursor(0, 42); display.print("-/+ wybor, OK zmiana");
  display.setCursor(0, 51); display.print("Po zmianie Hz: ESC");
  display.setCursor(0, 58); display.print("zasilanie OFF -> ON");
}

void drawInputs(uint32_t now) {
  drawTitle("STAN WEJSC");
  display.setCursor(0, 11);
  display.print(stopActive() ? "STOP: WCISNIETY!" : "STOP: zwolniony");
  display.setCursor(0, 20);
  display.printf("-:%d  OK:%d  +:%d", btnMinus.stable, btnOk.stable, btnPlus.stable);
  display.setCursor(0, 29);
  display.printf("ESC zasilanie: %s", escPwr ? "TAK" : "NIE");
  display.setCursor(0, 38);
  display.printf("Pot:%4d  %5.1f%%", (int)potRawF, potPct);
  display.setCursor(0, 47);
  display.printf("Vin:%5.2fV (%4dmV)", vinV, (int)vinAdcMv);
  display.setCursor(0, 56);
  display.printf("NexRX:%luB up:%lus", (unsigned long)nextionRxBytes, (unsigned long)(now / 1000));
}

void drawButtons(uint32_t now) {
  drawTitle("TEST PRZYCISKOW");
  for (uint8_t i = 0; i < 3; i++) {
    Button *b = buttons[i];
    display.setCursor(0, 11 + i * 9);
    display.printf("%-5s %s K:%-3u H:%-3u", b->name, b->stable ? "[#]" : "[ ]", b->cntClick, b->cntHold);
  }
  display.setCursor(0, 38);
  display.printf("STOP  %s zdarzen:%lu", stopActive() ? "[#]" : "[ ]", (unsigned long)g_stopEvents);
  display.setCursor(0, 47);
  display.printf("Ost: %s %s", lastEvtName, lastEvtType);
  display.setCursor(0, 56);
  uint32_t h = btnOk.heldFor(now);
  if (h > HOLD_MS) display.printf("wyjscie za %.1fs", (BTN_PAGE_EXIT_MS - min(h, BTN_PAGE_EXIT_MS)) / 1000.0f);
  else display.print("OK trzymaj 3s=powrot");
}

void drawBuzzer() {
  drawTitle(bzBusy() ? "TEST BUZZERA  >gra" : "TEST BUZZERA");
  drawList(BUZ_ITEMS, BUZ_N, buzSel, 11, 9);
}

void drawEsc(uint32_t now) {
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (ESC_BIDIRECTIONAL) display.printf("ESC %s lim:%3u%%", escDir > 0 ? "PRAWO" : "LEWO ", escLimitPct);
  else                   display.printf("ESC TESTER  lim:%3u%%", escLimitPct);
  display.drawFastHLine(0, 9, 128, SSD1306_WHITE);

  display.setCursor(0, 11);
  display.print("Stan: ");
  if (stopActive())        display.print("STOP AWARYJNY");
  else if (g_motorArmed)   display.print("PRACA");
  else if (escStoppedByStop) display.print("STOP->uzbroj");
  else                     display.print("ROZBROJONY");

  uint16_t us = g_escUs;
  float gaz = ((int)us - ESC_US_IDLE) * 100.0f / ESC_US_SPAN;  // +/- % zakresu impulsu
  display.setCursor(0, 20);
  display.printf("Pot:%5.1f%% Gaz:%+6.1f%%", potPct, gaz);
  display.setCursor(0, 29);
  display.printf("PWM: %4u us @%uHz", us, escPwmHz);
  display.setCursor(0, 38);
  display.printf("ESC zas:%s Vin:%4.1fV", escPwr ? "TAK" : "NIE", vinV);

  // pasek gazu + znacznik limitu
  display.drawRect(0, 47, 128, 7, SSD1306_WHITE);
  if (ESC_BIDIRECTIONAL) {   // od srodka: w prawo = PRAWO, w lewo = LEWO
    int w = (int)(fabsf(gaz) * 62 / 100);
    display.fillRect(gaz >= 0 ? 64 : 64 - w, 49, w, 3, SSD1306_WHITE);
    display.drawFastVLine(64, 47, 7, SSD1306_WHITE);
    int lx = 64 + escDir * (escLimitPct * 62 / 100);
    display.drawFastVLine(constrain(lx, 1, 126), 45, 11, SSD1306_WHITE);
  } else {
    int w = (int)(gaz * 124 / 100);
    display.fillRect(2, 49, w, 3, SSD1306_WHITE);
    int lx = 2 + escLimitPct * 124 / 100;
    display.drawFastVLine(min(lx, 126), 45, 11, SSD1306_WHITE);
  }

  display.setCursor(0, 56);
  if ((int32_t)(escMsgUntil - now) > 0) display.print(escMsg);
  else if (g_motorArmed) display.print("OK=stop  STOP=awaria");
  else display.print("OK=start OKdlugo=wr");
}

void drawProg(uint32_t now) {
  drawTitle("PROGRAMATOR ESC");
  display.setCursor(0, 11);
  display.printf("USB<->GPIO25 %6lu", (unsigned long)PROG_BAUDS[progBaudIdx]);
  display.setCursor(0, 20);
  display.printf("Echo:%s %s", progEcho ? "TAK" : "NIE",
                 (progLastAct && now - progLastAct < 300) ? "<transmisja>" : "");
  display.setCursor(0, 29);
  display.printf("PC->ESC:%8lu B", (unsigned long)progToEsc);
  display.setCursor(0, 38);
  display.printf("ESC->PC:%8lu B", (unsigned long)progFromEsc);
  display.setCursor(0, 47);
  display.print("1.ESC zasil. OFF->ON");
  display.setCursor(0, 56);
  display.print("2.AM32 tool: Direct");
}

void drawInfo() {
  drawTitle("PINY / INFO");
  display.setCursor(0, 11); display.print("OLED SDA21 SCL22");
  display.setCursor(0, 20); display.print("STOP27 -18 OK19 +23");
  display.setCursor(0, 29); display.print("BUZ26 ESC25 ESCdet13");
  display.setCursor(0, 38); display.print("POT34 VIN35 R120k/20k");
  display.setCursor(0, 47); display.print("Nextion RX16 TX17");
  display.setCursor(0, 56); display.printf("%s %s", __DATE__, __TIME__);
}

void drawScreen(uint32_t now) {
  if (!oledOk) return;
  display.clearDisplay();
  switch (page) {
    case PG_MENU:    drawMenu(); break;
    case PG_INPUTS:  drawInputs(now); break;
    case PG_BUTTONS: drawButtons(now); break;
    case PG_BUZZER:  drawBuzzer(); break;
    case PG_ESC:     drawEsc(now); break;
    case PG_ESCFREQ: drawEscFreq(); break;
    case PG_ESCPROG: drawProg(now); break;
    case PG_INFO:    drawInfo(); break;
  }
  display.display();
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  // 1) ESC najpierw: od razu sygnal neutralny (gaz zero), zanim cokolwiek innego.
  ledcSetup(ESC_LEDC_CH, escPwmHz, ESC_PWM_BITS);
  ledcAttachPin(PIN_ESC, ESC_LEDC_CH);
  escDisarm();

  // 2) STOP + przerwanie
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_STOP), stopIsr, RISING);

  Serial.setRxBufferSize(1024);   // zapas dla mostu programatora (OLED blokuje petle ~25 ms)
  Serial.begin(115200);
  LOGLN("\n=== Stanowisko testowe - program serwisowy ===");

  for (Button *b : buttons) pinMode(b->pin, INPUT_PULLUP);
  pinMode(PIN_ESC_PWR, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_POT, ADC_11db);
  analogSetPinAttenuation(PIN_VIN, ADC_11db);

  // Buzzer
  ledcSetup(BUZ_LEDC_CH, 2000, 10);
  ledcAttachPin(PIN_BUZZER, BUZ_LEDC_CH);
  bzTone(0);

  // Nextion - tylko konfiguracja UART2 (nic nie wysylamy, jedynie zliczamy odebrane bajty)
  Serial2.begin(9600, SERIAL_8N1, PIN_NEXTION_RX, PIN_NEXTION_TX);

  // OLED
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    display.setTextWrap(false);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 10); display.print("TEST");
    display.setCursor(10, 30); display.print("BENCH");
    display.setTextSize(1);
    display.setCursor(10, 54); display.print("program serwisowy");
    display.display();
  } else {
    LOGLN("[OLED] brak odpowiedzi pod 0x3C!");
  }

  readInputs(millis(), true);
  for (Button *b : buttons) { b->lastRaw = b->stable = (digitalRead(b->pin) == LOW); b->holdFired = true; }

  if (stopActive()) {
    emergencyStop();
    LOGLN("[STOP] STOP aktywny przy starcie (wcisniety lub przerwany obwod)");
  }
  lastStopEventsSeen = g_stopEvents;

  // Powrot do trybu programatora po resecie (np. reset przez DTR/RTS przy otwarciu portu COM)
  prefs.begin("bench", false);
  progBaudIdx = prefs.getUChar("progBaud", 0) % PROG_BAUD_N;
  progEcho = prefs.getBool("progEcho", true);
  if (prefs.getBool("prog", false)) {
    LOGLN("[PROG] wznowienie trybu programatora ESC");
    goPage(PG_ESCPROG);
    bzPlay(MEL(MEL_OK));
    return;
  }

  bzPlay(MEL(MEL_START));
  uint32_t t0 = millis();
  while (millis() - t0 < 1200) { bzUpdate(millis()); delay(1); }
}

void loop() {
  uint32_t now = millis();
  progUpdate(now);

  // --- STOP: dodatkowo odpytywany (gdyby zbocze umknelo), silnik nie moze pracowac przy STOP
  bool stopNow = stopActive();
  if (stopNow && g_motorArmed) emergencyStop();
  if (g_stopEvents != lastStopEventsSeen) {
    lastStopEventsSeen = g_stopEvents;
    escStoppedByStop = true;
    bzStop();                       // przerwij "zadanie" buzzera
    bzPlay(MEL(MEL_ALARM));
    lastEvtName = "STOP"; lastEvtType = "AWARIA";
    if (page == PG_ESCPROG) goPage(PG_MENU);   // STOP przerywa tez tryb programatora
    LOGLN("[STOP] zatrzymanie awaryjne!");
  }

  // --- wejscia
  readInputs(now);
  while (Serial2.available()) { Serial2.read(); nextionRxBytes++; }
  progUpdate(now);

  for (Button *b : buttons) {
    b->update(now);
    if (b->evClick) { lastEvtName = b->name; lastEvtType = "KLIK";  LOGF("[BTN] %s klik\n", b->name); }
    if (b->evHold)  { lastEvtName = b->name; lastEvtType = "PRZYTRZ"; LOGF("[BTN] %s przytrzymanie\n", b->name); }
    if ((b->evClick || b->evHold) && page != PG_BUZZER) keyClick();
  }

  // --- logika stron
  switch (page) {
    case PG_MENU:    handleMenu(); break;
    case PG_BUTTONS: handleButtonsPage(now); break;
    case PG_BUZZER:  handleBuzzerPage(); break;
    case PG_ESC:     handleEscPage(); break;
    case PG_ESCFREQ: handleEscFreqPage(); break;
    case PG_ESCPROG: handleProgPage(); break;
    case PG_INPUTS:
    case PG_INFO:    if (btnOk.evHold) goPage(PG_MENU); break;
  }
  if (page != PG_ESC && g_motorArmed) escDisarm();   // silnik pracuje tylko na stronie ESC

  bzUpdate(now);

  // --- OLED ~20 Hz
  static uint32_t lastDraw = 0;
  if (now - lastDraw >= (page == PG_ESCPROG ? 250u : 50u)) { lastDraw = now; drawScreen(now); }
  progUpdate(millis());

  // --- log co 1 s
  static uint32_t lastLog = 0;
  if (now - lastLog >= 1000) {
    lastLog = now;
    LOGF("STOP:%d -:%d OK:%d +:%d ESCpwr:%d pot:%.1f%% vin:%.2fV esc:%uus armed:%d\n",
                  stopNow, btnMinus.stable, btnOk.stable, btnPlus.stable, escPwr, potPct, vinV,
                  (unsigned)g_escUs, (int)g_motorArmed);
  }
}
