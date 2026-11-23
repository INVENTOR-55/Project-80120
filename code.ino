#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_system.h>

// ============================================================================
// --- VOLBA FONTU PRO RYCHLOST NA LCD ---
// ============================================================================
// HD44780 má jen 8 vlastních znaků (CGRAM sloty 0..7) a BigFont01 si je v begin()
// zabere všech 8 pro sebe, takže vedle něj se žádná vlastní ikona nevejde. Proto:
//   0 = (VÝCHOZÍ) rychlost vlastním 3x2 fontem ve stylu BigFont01 (4 sloty) a
//       všechny ikony jsou skutečné vlastní znaky (4 sloty):
//         blinkr L, blinkr P, světlo (potkávací/dálková sdílí 1 slot) a ruční brzda
//   1 = rychlost přes BigFont01_I2C, ikony jsou jen náhradní ROM znaky displeje
//       (<  >  -D  =D  (P)), protože žádný slot nezbývá
#ifndef RYCHLOST_BIGFONT01
#define RYCHLOST_BIGFONT01 0
#endif

#if RYCHLOST_BIGFONT01
#include <BigFont01_I2C.h>
#endif

// ============================================================================
// --- DEFINICE HARDWAROVÝCH KONEKTORŮ (Piny počítány od GND jako 1) ---
// ============================================================================

// PMOD 1
#define PMOD1_1  4   // IO04 (1. pin od GND)
#define PMOD1_2  5   // IO05 (2. pin od GND)
#define PMOD1_3  15  // IO15 (3. pin od GND)
#define PMOD1_4  16  // IO16 (4. pin od GND) -> PWM plyn

// PMOD 2
#define PMOD2_1  2   // IO02 (1. pin od GND)
#define PMOD2_2  10  // IO10 (2. pin od GND)
#define PMOD2_3  42  // IO42 (3. pin od GND) -> I2C_SCL
#define PMOD2_4  40  // IO40 (4. pin od GND)

// PMOD 3
#define PMOD3_1  6   // IO06 (1. pin od GND - horní řada)
#define PMOD3_2  8   // IO08 (2. pin od GND - horní řada)
#define PMOD3_3  47  // IO47 (3. pin od GND - horní řada) -> I2C_SDA
#define PMOD3_4  12  // IO12 (4. pin od GND - horní řada)
#define PMOD3_5  7   // IO07 (1. pin od GND - dolní řada)
#define PMOD3_6  9   // IO09 (2. pin od GND - dolní řada)
#define PMOD3_7  11  // IO11 (3. pin od GND - dolní řada)
#define PMOD3_8  13  // IO13 (4. pin od GND - dolní řada)

// DISPLAY CONNECTOR
#define DISPLAY_D0 38
#define DISPLAY_D1 39
#define DISPLAY_D2 41
#define DISPLAY_D3 45
#define DISPLAY_CS 1
#define DISPLAY_CK 3

// PINS PRO PLYN A BRZDU
#define PIN_POT_PLYNU      17        // Analogový vstup pro plyn
#define PIN_POT_BRZDY      18        // Analogový vstup pro brzdu
#define PIN_PWM_PLYNU      PMOD1_4   // GPIO 16 - PWM výstup pro plyn

// HALL SONDA PRO RYCHLOST
#define PIN_HALL_RYCHLOST  21        // IO21 - jedna ze tří hall sond hoverboard motoru

// I2C SBERNICE
#define I2C_SDA            PMOD3_3   // IO47
#define I2C_SCL            PMOD2_3   // IO42

// 20x04 I2C LCD
#define LCD_ADDR           0x3F
#define LCD_COLS           20
#define LCD_ROWS           4

// ROZLOŽENÍ PALUBNÍHO DISPLEJE (sloupec, řádek; počítáno od 0)
#define LCD_RYCHLOST_RADEK       0   // velká rychlost: řádky 0-1, od sloupce 0
#define LCD_RYCHLOST_SLOUPEC     0
#define LCD_IKONY_RADEK          0   // ikony světel a blinkrů
#define LCD_IKONA_BLINKR_L       8
#define LCD_IKONA_SVETLO         12  // potkávací / dálková (jedna buňka, ROM varianta 2 buňky)
#define LCD_IKONA_RUCNI_BRZDA    15  // zatažená ruční brzda = neutral (ROM varianta 3 buňky)
#define LCD_IKONA_BLINKR_P       19

// PALUBNÍ PINY
#define ONBOARD_ILED       48

// ============================================================================
// --- PŘIŘAZENÍ FUNKCÍ K PINŮM ---
// ============================================================================

#define STAVOVA_LED_PIN    ONBOARD_ILED // IO48
#define ZADNI_PIN          PMOD2_4      // IO40
#define PREDNI_PIN         PMOD1_2      // IO05
#define BOCNI_PIN          PMOD1_3      // IO15 - oba 8-LED boční pásky v sérii (celkem 16 LED: 0..7 pravý, 8..15 levý)

#define LED_COUNT          46
#define BOCNI_LED_COUNT    16
#define AKTIVNI_ZONA_LEDS  10

// Nastavení PWM (ESP32 Arduino core 2.x)
const int PWM_CH               = 0;
const int PWM_FREQ             = 1000;
const int PWM_RES_BITS         = 8;  // 0..255
const int PWM_ZPATECKA_VYPNUTO = 0;  // Pedál plynu není stisklý -> PWM 0
const int PWM_ZPATECKA_MIN     = 81; // Min. hodnota při stisku plynu na zpátečku
const int PWM_ZPATECKA_MAX     = 63; // Max. hodnota při plném plynu na zpátečku
const int PRAH_STISKU_PLYNU    = 5;  // Práh v % (ochrana proti šumu na kraji potenciometru)
int aktualniPwmPlynu           = 0;

// --- RYCHLOST Z HALL SONDY ---
// 1 = do Serial Monitoru (115200) se každých 200 ms píše stav Hall sondy (místo výpisu PWM)
#define HALL_DEBUG 1

const float    KOLO_PRUMER_PALCE    = 6.5f;
const int      HALL_PULSU_NA_OTACKU = 15;       // pólové páry = vzestupné hrany jedné sondy na otáčku
const float    KOLO_OBVOD_M         = KOLO_PRUMER_PALCE * 0.0254f * PI;
const float    METRU_NA_PULZ        = KOLO_OBVOD_M / HALL_PULSU_NA_OTACKU;
const uint32_t HALL_MIN_PERIODA_US  = 1000;     // filtr šumu (odpovídá cca 120 km/h)
const uint32_t HALL_TIMEOUT_US      = 500000;   // bez pulzu 0,5 s = stojí (~0,25 km/h)
const float    HALL_FILTR_ALFA      = 0.3f;     // vyhlazení (0..1, menší = hladší)

volatile uint32_t hallPosledniUs = 0;
volatile uint32_t hallPeriodaUs  = 0;
volatile uint32_t hallPocitadlo  = 0;
uint32_t hallPredchoziPocitadlo  = 0;
float    rychlostFiltrKmh        = 0.0f;
portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;

// Spoušť rebootu při výstražných blinkrech + dlouhém držení plynu i brzdy na max
unsigned long casStartRebootKombinace = 0;
const unsigned long CAS_PRO_REBOOT_MS = 2000;

// Obnovovací interval hodnot na palubním LCD (500 ms šetří I2C čas)
unsigned long posledniObnovaLcd = 0;
const unsigned long INTERVAL_LCD_MS = 500;

Adafruit_NeoPixel stripPredni(LED_COUNT, PREDNI_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripZadni(LED_COUNT, ZADNI_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripBocni(BOCNI_LED_COUNT, BOCNI_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripStavova(1, STAVOVA_LED_PIN, NEO_GRB + NEO_KHZ800);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

#if RYCHLOST_BIGFONT01
BigFont01_I2C big(&lcd);
#endif

// --- VÝSTUPY PRO RELÉ ---
const int PIN_RELE_BRZDY[6]   = {DISPLAY_D0, DISPLAY_D1, DISPLAY_D2, DISPLAY_D3, DISPLAY_CS, DISPLAY_CK};
const int PIN_RELE_BLDC       = PMOD2_2; // IO10
const int PIN_RELE_ZVUK       = PMOD2_1; // IO02

// --- VSTUPY PRO SPÍNAČE ---
const int PIN_SMER_VPRED      = PMOD3_1; // IO06
const int PIN_SMER_VZAD       = PMOD3_2; // IO08
const int PIN_BLINKR_L        = PMOD3_4; // IO12
const int PIN_BLINKR_P        = PMOD3_5; // IO07
const int PIN_SVETLA_VYP      = PMOD3_6; // IO09
const int PIN_SVETLA_DALK     = PMOD3_7; // IO11
const int PIN_VYSTRAZNE       = PMOD3_8; // IO13
const int PIN_PREPINAC_MAJAKY = PMOD1_1; // IO04

// --- GLOBÁLNÍ STAVY PRO SVĚTLA A REŽIMY ---
bool stavDenniSviceni = false;
bool stavDalkova      = false;
bool stavBrzda        = false;
bool stavMlhovka      = false;
bool stavZpatecka     = false;
bool stavMajaky       = false;
bool jeNeutral        = true;

bool minulyStavDenniSviceni = false;
bool minulyStavDalkova      = false;
bool minulyStavMajaky       = false;

// --- DATA PRO PALUBNÍ DISPLEJ ---
// Záporná hodnota = zatím neznámá -> na LCD se vypíše 00.
// Až budeš měřit, stačí do proměnné zapsat skutečnou hodnotu (např. napetiTrakcniV = 48.3f;)
int   aktualniRychlost = 0;       // km/h (0-99), počítá aktualizujRychlost() z Hall sondy
float napetiTrakcniV   = -1.0f;   // napětí trakční baterie [V]
float napetiPalubniV   = -1.0f;   // napětí palubní baterie [V]
float proudA           = -1.0f;   // proud [A]
float dojezdKm         = -1.0f;   // odhadovaný dojezd [km]

// Stav blinkrů pro ikony na LCD (nastavuje aktualizujSvetla -> ikona bliká synchronně s LED)
bool lcdBlinkL = false;
bool lcdBlinkP = false;

// --- DYNAMIKY A OCHRANY PRO BLDC DRIVER ---
bool bldcMaBytSepnuto          = false;
bool minulyBldcStav            = false;
bool pulzProbiha               = false;
bool veStavuNeutral            = false;

unsigned long casStartPulzu          = 0;
unsigned long casVstupuDoNeutralu    = 0;
unsigned long casKoncePoslednihoPulzu = 0;

const unsigned long DELKA_PULZU_BLDC      = 300;
const unsigned long ZPOZDENI_NEUTRALU     = 500;
const unsigned long MIN_PAUZA_MEZI_PULSY  = 2000;

// Pomocné proměnné pro blinkry a majáky
unsigned long casMajaku = 0;
unsigned long casBlinkru = 0;
unsigned long posledniCasBlinkruL = 0;
unsigned long posledniCasBlinkruP = 0;
unsigned long posledniCasMajakuVyp = 0;

// Žárovkový efekt bočních blinkrů (naběhnutí vlákna a dohasnutí)
float zarovkaPravy = 0.0;
float zarovkaLevy  = 0.0;
unsigned long posledniCasZarovky = 0;
const unsigned long ZAROVKA_NABEH_MS   = 60;
const unsigned long ZAROVKA_DOHAS_MS   = 220;

const unsigned long TRVANI_CYKLU_BLINKRU   = 700;
const unsigned long TRVANI_ANIMACE_BLINKRU  = 450;
const unsigned long TRVANI_FADE_IN          = 500;

// --- FUNKCE PRO STAVOVOU LED ---
void nastavStavovouLed(uint8_t r, uint8_t g, uint8_t b) {
  stripStavova.setPixelColor(0, stripStavova.Color(r, g, b));
  stripStavova.show();
}

// --- POMOCNÁ FUNKCE PRO ZVUK ---
void pipniZvuk(unsigned long delkaMs) {
  digitalWrite(PIN_RELE_ZVUK, LOW);
  delay(delkaMs);
  digitalWrite(PIN_RELE_ZVUK, HIGH);
}

// --- NASTAVENÍ PWM PLYNU ---
void nastavPwmPlynu(int duty) {
  duty = constrain(duty, 0, 255);
  aktualniPwmPlynu = duty;
  ledcWrite(PWM_CH, duty);
}

// ============================================================================
// --- RYCHLOST Z HALL SONDY (přerušení na IO14) ---
// ============================================================================

void IRAM_ATTR hallISR() {
  uint32_t nyni = micros();
  portENTER_CRITICAL_ISR(&hallMux);
  uint32_t perioda = nyni - hallPosledniUs;
  if (perioda >= HALL_MIN_PERIODA_US) {
    // první pulz po zastavení nemá platnou periodu -> 0
    hallPeriodaUs = (perioda < HALL_TIMEOUT_US) ? perioda : 0;
    hallPosledniUs = nyni;
    hallPocitadlo++;
  }
  portEXIT_CRITICAL_ISR(&hallMux);
}

void aktualizujRychlost() {
  uint32_t posledni, perioda, pocitadlo;

  portENTER_CRITICAL(&hallMux);
  uint32_t nyni = micros();
  posledni  = hallPosledniUs;
  perioda   = hallPeriodaUs;
  pocitadlo = hallPocitadlo;
  if (perioda != 0 && (uint32_t)(nyni - posledni) >= HALL_TIMEOUT_US) {
    hallPeriodaUs = 0;   // timeout -> stojíme
    perioda = 0;
  }
  portEXIT_CRITICAL(&hallMux);

  if (perioda == 0) {
    rychlostFiltrKmh = 0.0f;
  } else if (pocitadlo != hallPredchoziPocitadlo) {   // přišel nový pulz
    float v = (1000000.0f / (float)perioda) * METRU_NA_PULZ * 3.6f;
    if (rychlostFiltrKmh < 0.01f) rychlostFiltrKmh = v;   // rozjezd: bez rozběhu filtru
    else rychlostFiltrKmh += HALL_FILTR_ALFA * (v - rychlostFiltrKmh);
  }
  hallPredchoziPocitadlo = pocitadlo;

  aktualniRychlost = (int)(rychlostFiltrKmh + 0.5f);
}

// ============================================================================
// --- PALUBNÍ DISPLEJ 20x4: ZÁKLADNÍ POMOCNÉ FUNKCE ---
// ============================================================================

void lcdZnakNa(uint8_t sloupec, uint8_t radek, uint8_t znak) {
  lcd.setCursor(sloupec, radek);
  lcd.write(znak);
}

// Celý řádek (doplní mezerami na 20 znaků)
void vypisRadekLcd(uint8_t radek, const char* text) {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-20s", text);
  lcd.setCursor(0, radek);
  lcd.print(buf);
}

// Přepíše část řádku jen když se text změnil (šetří I2C čas a nebliká)
void lcdPrepisDiff(uint8_t sloupec, uint8_t radek, uint8_t delka, const char* text, char* cache) {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-*.*s", (int)delka, (int)delka, text);
  if (strcmp(buf, cache) == 0) return;
  strcpy(cache, buf);
  lcd.setCursor(sloupec, radek);
  lcd.print(buf);
}

void inicializujLcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

// ============================================================================
// --- LOADING ANIMACE NA LCD (progress bar z vlastních znaků + rychlé hlášky) ---
// ============================================================================
// CGRAM během loadingu: sloty 0..5 = výplň 0..5 sloupců, 6 = levý okraj, 7 = pravý okraj

#define LOADING_BAR_BUNEK   14      // buňky ve sloupcích 1..14 (okraje ve sloupcích 0 a 15)
#define LOADING_BAR_RADEK   3
#define LOADING_PCT_SLOUPEC 16      // "100%" ve sloupcích 16..19
#define LOAD_SLOT_OKRAJ_L   6
#define LOAD_SLOT_OKRAJ_P   7

// Odhad celkové délky animace (dynamicBlinkers + náběh) - podle něj se počítají procenta
const unsigned long ODHAD_DELKY_ANIMACE_MS = 2450;
// Jak často se mění hláška (a obnovují procenta)
const unsigned long INTERVAL_LOADING_MS    = 30;

// Hlášky jen pro efekt (max 18 znaků, bez diakritiky - HD44780 ji nemá)
const char* const LOADING_HLASKY[] = {
  "Init ESP32-S3",
  "Start FreeRTOS",
  "Kontrola pameti",
  "Watchdog timer",
  "Cteni EEPROM",
  "Mapovani PMOD",
  "I2C sbernice",
  "LCD driver",
  "Kalibrace plynu",
  "Kalibrace brzdy",
  "Cteni pedalu",
  "PWM kanal 0",
  "Test rele brzdy",
  "Brzdovy okruh",
  "BLDC handshake",
  "Ochrana BLDC",
  "Test neutralu",
  "Kontrola spinacu",
  "Predni LED pasek",
  "Zadni LED pasek",
  "Bocni LED pasky",
  "Stavova LED",
  "Zvukove rele",
  "Majaky",
  "Vystrazne blinkry",
  "Dalkova svetla",
  "Denni sviceni",
  "Napeti trakcni",
  "Napeti palubni",
  "Snimac proudu",
  "Vypocet dojezdu",
  "Rychlostni snimac",
  "Sync hodin",
  "Telemetrie",
  "Self-test"
};
const int POCET_HLASEK = sizeof(LOADING_HLASKY) / sizeof(LOADING_HLASKY[0]);

unsigned long casStartAnimace   = 0;
unsigned long posledniTickLoading = 0;
int posledniHlaska              = -1;
int posledniPctLoading          = -1;
uint8_t cacheBarLevel[LOADING_BAR_BUNEK];

void nahrajGlyfyProgressBaru() {
  // 0..5 = buňka s N sloupci výplně (nahoře a dole je rámeček)
  for (int k = 0; k <= 5; k++) {
    uint8_t vypln = (uint8_t)((0x1F << (5 - k)) & 0x1F);
    uint8_t g[8] = {0x00, 0x1F, vypln, vypln, vypln, vypln, 0x1F, 0x00};
    lcd.createChar(k, g);
  }
  uint8_t okrajL[8] = {0x00, 0x1F, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x00};
  uint8_t okrajP[8] = {0x00, 0x1F, 0x01, 0x01, 0x01, 0x01, 0x1F, 0x00};
  lcd.createChar(LOAD_SLOT_OKRAJ_L, okrajL);
  lcd.createChar(LOAD_SLOT_OKRAJ_P, okrajP);
  lcd.home();
}

void vykresliProgress(int pct) {
  pct = constrain(pct, 0, 100);
  if (pct == posledniPctLoading) return;
  posledniPctLoading = pct;

  int kroky = (pct * LOADING_BAR_BUNEK * 5) / 100;   // 14 buněk x 5 sloupců = 70 kroků
  for (int i = 0; i < LOADING_BAR_BUNEK; i++) {
    int level = constrain(kroky - i * 5, 0, 5);
    if (level != cacheBarLevel[i]) {
      cacheBarLevel[i] = (uint8_t)level;
      lcdZnakNa(1 + i, LOADING_BAR_RADEK, (uint8_t)level);
    }
  }

  char buf[8];
  snprintf(buf, sizeof(buf), "%3d%%", pct);
  lcd.setCursor(LOADING_PCT_SLOUPEC, LOADING_BAR_RADEK);
  lcd.print(buf);
}

void loadingStart() {
  nahrajGlyfyProgressBaru();
  lcd.clear();
  vypisRadekLcd(0, "== Project 80120 ==");
  vypisRadekLcd(1, "   initialization   ");
  vypisRadekLcd(2, "> Start");

  lcdZnakNa(0, LOADING_BAR_RADEK, LOAD_SLOT_OKRAJ_L);
  for (int i = 0; i < LOADING_BAR_BUNEK; i++) {
    cacheBarLevel[i] = 0;
    lcdZnakNa(1 + i, LOADING_BAR_RADEK, 0);
  }
  lcdZnakNa(LOADING_BAR_BUNEK + 1, LOADING_BAR_RADEK, LOAD_SLOT_OKRAJ_P);

  posledniPctLoading = -1;
  posledniHlaska = -1;
  vykresliProgress(0);

  casStartAnimace = millis();
  posledniTickLoading = casStartAnimace;
}

// Volá se opakovaně během animace; sama hlídá interval, takže je levná
void loadingTick() {
  unsigned long nyni = millis();
  if (nyni - posledniTickLoading < INTERVAL_LOADING_MS) return;
  posledniTickLoading = nyni;

  // 1) náhodná hláška (nikdy dvakrát za sebou stejná)
  int idx;
  do {
    idx = (int)random(POCET_HLASEK);
  } while (idx == posledniHlaska);
  posledniHlaska = idx;

  char radek[32];
  snprintf(radek, sizeof(radek), "> %s", LOADING_HLASKY[idx]);
  vypisRadekLcd(2, radek);

  // 2) procenta podle uplynulého času (do 99 %, 100 % nastaví loadingHotovo)
  int pct = (int)(((nyni - casStartAnimace) * 100UL) / ODHAD_DELKY_ANIMACE_MS);
  if (pct > 99) pct = 99;
  if (pct < posledniPctLoading) pct = posledniPctLoading;   // nikdy nejde zpět
  vykresliProgress(pct);
}

// Náhrada delay() v animaci: LCD tick se vejde do čekání, takže se LED animace nezpomalí
void cekejSLoadingem(unsigned long ms) {
  unsigned long start = millis();
  loadingTick();
  while (millis() - start < ms) {
    delay(1);
  }
}

void loadingHotovo() {
  vypisRadekLcd(2, "> Success!");
  vykresliProgress(100);
  delay(300);
}

// ============================================================================
// --- PALUBNÍ DISPLEJ 20x4: DASHBOARD ---
// ============================================================================

// Sloty CGRAM pro vlastní font + ikony (jen RYCHLOST_BIGFONT01 = 0)
#define SLOT_CIF_HORNI      0   // horní pruh
#define SLOT_CIF_DOLNI      1   // dolní pruh
#define SLOT_CIF_OBA        2   // horní + dolní pruh
#define SLOT_CIF_PLNY       3   // plný blok
#define SLOT_BLINKR_L       4
#define SLOT_BLINKR_P       5
#define SLOT_SVETLO         6   // sdílený slot: potkávací a dálková se nikdy nezobrazují zároveň,
                                // takže se glyf v tomto slotu při změně stavu přepíše
#define SLOT_RUCNI_BRZDA    7

bool dashboardAktivni = false;

int    cacheRychlost = -1;
char   cacheRadek1[LCD_COLS + 1] = "";
char   cacheRadek2[LCD_COLS + 1] = "";
char   cacheRadek3[LCD_COLS + 1] = "";
int8_t cacheIkonaBlinkL  = -1;
int8_t cacheIkonaBlinkP  = -1;
int8_t cacheIkonaSvetlo  = -1;   // 0 = nic, 1 = potkávací, 2 = dálková
int8_t cacheIkonaBrzda   = -1;

// Glyfy ikon (5x8 px). Nekonstantní pole, protože createChar() v některých verzích
// LiquidCrystal_I2C bere uint8_t[] bez const.
static uint8_t GLYF_BLINKR_L[8]   = {0x02, 0x06, 0x0E, 0x1E, 0x0E, 0x06, 0x02, 0x00};   // šipka vlevo
static uint8_t GLYF_BLINKR_P[8]   = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00};   // šipka vpravo
static uint8_t GLYF_DALKOVA[8]    = {0x02, 0x1B, 0x03, 0x1B, 0x03, 0x1B, 0x02, 0x00};   // reflektor + rovné paprsky
static uint8_t GLYF_POTKAVACI[8]  = {0x02, 0x0B, 0x13, 0x0B, 0x13, 0x0B, 0x12, 0x00};   // reflektor + paprsky šikmo dolů
static uint8_t GLYF_RUCNI_BRZDA[8] = {0x0E, 0x11, 0x15, 0x15, 0x11, 0x15, 0x11, 0x0E};  // (!) v kruhu

#if !RYCHLOST_BIGFONT01
// Vlastní velké číslice 3 sloupce x 2 řádky
enum { GH = SLOT_CIF_HORNI, GD = SLOT_CIF_DOLNI, GO = SLOT_CIF_OBA, GP = SLOT_CIF_PLNY, GM = ' ' };
const uint8_t VLASTNI_CISLICE[10][2][3] = {
  {{GP, GH, GP}, {GP, GD, GP}},   // 0
  {{GH, GP, GM}, {GD, GP, GD}},   // 1
  {{GO, GO, GP}, {GP, GD, GD}},   // 2
  {{GH, GO, GP}, {GD, GD, GP}},   // 3
  {{GP, GD, GP}, {GM, GM, GP}},   // 4
  {{GP, GO, GO}, {GD, GD, GP}},   // 5
  {{GP, GO, GO}, {GP, GD, GP}},   // 6
  {{GH, GH, GP}, {GM, GP, GM}},   // 7
  {{GP, GO, GP}, {GP, GD, GP}},   // 8
  {{GP, GO, GP}, {GD, GD, GP}}    // 9
};

void nahrajGlyfyDashboardu() {
  uint8_t horni[8]  = {0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t dolni[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F};
  uint8_t oba[8]    = {0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x1F, 0x1F, 0x1F};
  uint8_t plny[8]   = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};
  lcd.createChar(SLOT_CIF_HORNI, horni);
  lcd.createChar(SLOT_CIF_DOLNI, dolni);
  lcd.createChar(SLOT_CIF_OBA, oba);
  lcd.createChar(SLOT_CIF_PLNY, plny);
  lcd.createChar(SLOT_BLINKR_L, GLYF_BLINKR_L);
  lcd.createChar(SLOT_BLINKR_P, GLYF_BLINKR_P);
  lcd.createChar(SLOT_SVETLO, GLYF_POTKAVACI);        // výchozí glyf, přepíše se podle stavu světel
  lcd.createChar(SLOT_RUCNI_BRZDA, GLYF_RUCNI_BRZDA);
  lcd.home();
}

void vlastniCislice(uint8_t radek, uint8_t sloupec, uint8_t cislice) {
  for (uint8_t r = 0; r < 2; r++) {
    lcd.setCursor(sloupec, radek + r);
    for (uint8_t c = 0; c < 3; c++) {
      lcd.write(VLASTNI_CISLICE[cislice][r][c]);
    }
  }
}
#endif

void resetCacheDashboardu() {
  cacheRychlost = -1;
  cacheRadek1[0] = 0;
  cacheRadek2[0] = 0;
  cacheRadek3[0] = 0;
  cacheIkonaBlinkL = -1;
  cacheIkonaBlinkP = -1;
  cacheIkonaSvetlo = -1;
  cacheIkonaBrzda  = -1;
}

// Přepne displej z loadingu na dashboard
void inicializujDashboard() {
  lcd.clear();
#if RYCHLOST_BIGFONT01
  big.begin();               // nahraje segmenty velkého fontu do CGRAM (přepíše glyfy progress baru)
#else
  nahrajGlyfyDashboardu();
#endif
  resetCacheDashboardu();
  dashboardAktivni = true;
  posledniObnovaLcd = millis() - INTERVAL_LCD_MS;   // první vykreslení hned
}

// Velká rychlost, 2 číslice (00-99)
void kresliRychlost(int rychlost) {
  rychlost = constrain(rychlost, 0, 99);
#if RYCHLOST_BIGFONT01
  // BigFont01_I2C::writeint(radek, sloupec, cislo, pocetCifer, uvodniNuly)
  big.writeint(LCD_RYCHLOST_RADEK, LCD_RYCHLOST_SLOUPEC, rychlost, 2, true);
#else
  vlastniCislice(LCD_RYCHLOST_RADEK, LCD_RYCHLOST_SLOUPEC, rychlost / 10);
  vlastniCislice(LCD_RYCHLOST_RADEK, LCD_RYCHLOST_SLOUPEC + 4, rychlost % 10);
#endif
}

// --- IKONY ---
// Všechny ikony jsou vlastní znaky (RYCHLOST_BIGFONT01 = 0). V režimu BigFont01 zbývají
// jen náhradní ROM znaky, protože BigFont01 zabere všech 8 slotů CGRAM.

void kresliBlinkrL(bool zap) {
#if RYCHLOST_BIGFONT01
  lcd.setCursor(LCD_IKONA_BLINKR_L, LCD_IKONY_RADEK);
  lcd.print(zap ? "<" : " ");
#else
  lcdZnakNa(LCD_IKONA_BLINKR_L, LCD_IKONY_RADEK, zap ? SLOT_BLINKR_L : ' ');
#endif
}

void kresliBlinkrP(bool zap) {
#if RYCHLOST_BIGFONT01
  lcd.setCursor(LCD_IKONA_BLINKR_P, LCD_IKONY_RADEK);
  lcd.print(zap ? ">" : " ");
#else
  lcdZnakNa(LCD_IKONA_BLINKR_P, LCD_IKONY_RADEK, zap ? SLOT_BLINKR_P : ' ');
#endif
}

// stav: 0 = nic, 1 = potkávací, 2 = dálková
void kresliSvetlo(int8_t stav) {
#if RYCHLOST_BIGFONT01
  lcd.setCursor(LCD_IKONA_SVETLO, LCD_IKONY_RADEK);
  lcd.print(stav == 2 ? "=D" : (stav == 1 ? "-D" : "  "));
#else
  if (stav == 0) {
    lcdZnakNa(LCD_IKONA_SVETLO, LCD_IKONY_RADEK, ' ');
    return;
  }
  // jeden slot, dva glyfy: při změně stavu se glyf ve slotu přepíše (createChar mění CGRAM,
  // proto až potom znovu nastavujeme kurzor v lcdZnakNa)
  lcd.createChar(SLOT_SVETLO, (stav == 2) ? GLYF_DALKOVA : GLYF_POTKAVACI);
  lcdZnakNa(LCD_IKONA_SVETLO, LCD_IKONY_RADEK, SLOT_SVETLO);
#endif
}

void kresliRucniBrzdu(bool zap) {
#if RYCHLOST_BIGFONT01
  lcd.setCursor(LCD_IKONA_RUCNI_BRZDA, LCD_IKONY_RADEK);
  lcd.print(zap ? "(P)" : "   ");
#else
  lcdZnakNa(LCD_IKONA_RUCNI_BRZDA, LCD_IKONY_RADEK, zap ? SLOT_RUCNI_BRZDA : ' ');
#endif
}

// Ikony se kontrolují při každém průchodu loop(), ale I2C se používá jen při změně stavu
void aktualizujIkonyLcd() {
  int8_t blL    = lcdBlinkL ? 1 : 0;
  int8_t blP    = lcdBlinkP ? 1 : 0;
  int8_t svetlo = stavDalkova ? 2 : (stavDenniSviceni ? 1 : 0);   // dálková má přednost před potkávacími
  int8_t brzda  = jeNeutral ? 1 : 0;                               // zatažená ruční brzda = neutral

  if (blL != cacheIkonaBlinkL) {
    cacheIkonaBlinkL = blL;
    kresliBlinkrL(blL);
  }
  if (blP != cacheIkonaBlinkP) {
    cacheIkonaBlinkP = blP;
    kresliBlinkrP(blP);
  }
  if (svetlo != cacheIkonaSvetlo) {
    cacheIkonaSvetlo = svetlo;
    kresliSvetlo(svetlo);
  }
  if (brzda != cacheIkonaBrzda) {
    cacheIkonaBrzda = brzda;
    kresliRucniBrzdu(brzda);
  }
}

// Neznámá (záporná) hodnota -> "00"
void formatHodnota(char* out, size_t n, float hodnota, bool desetinne) {
  if (hodnota < 0.0f) {
    snprintf(out, n, "00");
  } else if (desetinne) {
    snprintf(out, n, "%.1f", constrain(hodnota, 0.0f, 99.9f));
  } else {
    snprintf(out, n, "%d", (int)constrain(hodnota + 0.5f, 0.0f, 999.0f));
  }
}

void aktualizujDashboardLcd(unsigned long nyni) {
  if (!dashboardAktivni) return;

  aktualizujIkonyLcd();

  if (nyni - posledniObnovaLcd < INTERVAL_LCD_MS) return;
  posledniObnovaLcd = nyni;

  // Rychlost (velká čísla)
  int rychlost = constrain(aktualniRychlost, 0, 99);
  if (rychlost != cacheRychlost) {
    cacheRychlost = rychlost;
    kresliRychlost(rychlost);
  }

  char t[32];

  // Řádek 1 (vpravo od velkých číslic): jednotka a směr jízdy
  const char* rezim = jeNeutral ? "" : (stavZpatecka ? "RWD" : "FWD");
  snprintf(t, sizeof(t), "km/h  %-6s", rezim);
  lcdPrepisDiff(8, 1, 12, t, cacheRadek1);

  // Řádek 2: napětí baterií (TRK = trakční, PAL = palubní)
  char a[12], b[12], c[12], d[12];
  formatHodnota(a, sizeof(a), napetiTrakcniV, true);
  formatHodnota(b, sizeof(b), napetiPalubniV, true);
  snprintf(t, sizeof(t), "TRK %sV  PAL %sV", a, b);
  lcdPrepisDiff(0, 2, LCD_COLS, t, cacheRadek2);

  // Řádek 3: proud a odhadovaný dojezd (DOJ)
  formatHodnota(c, sizeof(c), proudA, true);
  formatHodnota(d, sizeof(d), dojezdKm, false);
  snprintf(t, sizeof(t), "I %sA  DOJ %skm", c, d);
  lcdPrepisDiff(0, 3, LCD_COLS, t, cacheRadek3);
}

// --- VÝPIS DŮVODU POSLEDNÍHO RESETU ESP32 ---
void vypisDuvodResetu() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Duvod posledniho startu/resetu: ");
  switch (reason) {
    case ESP_RST_BROWNOUT:
      Serial.println("BROWNOUT");
      break;
    case ESP_RST_POWERON:
      Serial.println("POWERON");
      break;
    case ESP_RST_SW:
      Serial.println("SW (softwarovy reset)");
      break;
    case ESP_RST_PANIC:
      Serial.println("PANIC");
      break;
    default:
      Serial.print("Kod: ");
      Serial.println((int)reason);
      break;
  }
}

// --- ZPRACOVÁNÍ PLYNU, BRZDY A STUPŇOVITÉHO RELE ---
void zpracujPlynABrzdu() {
  int surovejPlyn = analogRead(PIN_POT_PLYNU);
  int surovaBrzda = analogRead(PIN_POT_BRZDY);

  int procentaPlynu = map(surovejPlyn, 0, 4095, 0, 100);
  int procentaBrzdy = map(surovaBrzda, 0, 4095, 0, 100);

  procentaPlynu = constrain(procentaPlynu, 0, 100);
  procentaBrzdy = constrain(procentaBrzdy, 0, 100);

  bool vystrazneAktivni = (digitalRead(PIN_VYSTRAZNE) == LOW);
  bool obaNaMax = (procentaPlynu >= 95 && procentaBrzdy >= 95);
  unsigned long nyni = millis();

  if (vystrazneAktivni && obaNaMax) {
    if (casStartRebootKombinace == 0) {
      casStartRebootKombinace = nyni;
    } else if (nyni - casStartRebootKombinace >= CAS_PRO_REBOOT_MS) {
      Serial.println("REBOOT: Vystrazne + plyn MAX + brzda MAX podrzeny -> ESP.restart()");
      Serial.flush();
      nastavPwmPlynu(0);
      delay(50);
      ESP.restart();
    }
  } else {
    casStartRebootKombinace = 0;
  }

  if (jeNeutral) {
    nastavPwmPlynu(0);
    for (int i = 0; i < 6; i++) {
      digitalWrite(PIN_RELE_BRZDY[i], HIGH);
    }
    stavBrzda = (procentaBrzdy > 5);
    return;
  }

  if (procentaBrzdy > 5) {
    int koeficientUbrani = map(procentaBrzdy, 0, 100, 100, 0);
    procentaPlynu = (procentaPlynu * koeficientUbrani) / 100;
  }

  if (!pulzProbiha) {
    int dutyPWM = 0;
    if (stavZpatecka) {
      if (procentaPlynu > PRAH_STISKU_PLYNU) {
        dutyPWM = map(procentaPlynu, PRAH_STISKU_PLYNU, 100, PWM_ZPATECKA_MIN, PWM_ZPATECKA_MAX);
      } else {
        dutyPWM = PWM_ZPATECKA_VYPNUTO;
      }
    } else if (procentaPlynu > 0) {
      dutyPWM = map(procentaPlynu, 0, 100, 0, 255);
    }
    nastavPwmPlynu(dutyPWM);
  }

  int pocetAktivnichReleBrzdy = map(procentaBrzdy, 0, 100, 6, 0);

  for (int i = 0; i < 6; i++) {
    if (i < pocetAktivnichReleBrzdy) {
      digitalWrite(PIN_RELE_BRZDY[i], LOW);
    } else {
      digitalWrite(PIN_RELE_BRZDY[i], HIGH);
    }
  }

  stavBrzda = (procentaBrzdy > 5);
}

// --- POMOCNÉ FUNKCE PRO ŽÁROVKOVÝ EFEKT BOČNÍCH PÁSKŮ ---
float aktualizujZarovku(float aktualniJas, bool cilSviti, unsigned long dtMs) {
  if (dtMs == 0) return aktualniJas;
  if (cilSviti) {
    aktualniJas += (float)dtMs / (float)ZAROVKA_NABEH_MS;
    if (aktualniJas > 1.0f) aktualniJas = 1.0f;
  } else {
    aktualniJas -= (float)dtMs / (float)ZAROVKA_DOHAS_MS;
    if (aktualniJas < 0.0f) aktualniJas = 0.0f;
  }
  return aktualniJas;
}

void vykresliBocniZarovkovyBlinkr(unsigned long nyni, bool svitiPravy, bool svitiLevy) {
  unsigned long dtMs = (posledniCasZarovky == 0) ? 0 : (nyni - posledniCasZarovky);
  posledniCasZarovky = nyni;

  zarovkaPravy = aktualizujZarovku(zarovkaPravy, svitiPravy, dtMs);
  zarovkaLevy  = aktualizujZarovku(zarovkaLevy, svitiLevy, dtMs);

  float krivkaPravy = zarovkaPravy * zarovkaPravy;
  float krivkaLevy  = zarovkaLevy * zarovkaLevy;

  uint32_t barvaPravyBok = stripBocni.Color(
    (uint8_t)(255.0f * krivkaPravy),
    (uint8_t)(100.0f * krivkaPravy),
    0
  );
  uint32_t barvaLevyBok = stripBocni.Color(
    (uint8_t)(255.0f * krivkaLevy),
    (uint8_t)(100.0f * krivkaLevy),
    0
  );

  for (int i = 0; i < 8; i++) {
    stripBocni.setPixelColor(i, barvaPravyBok);
  }
  for (int i = 8; i < BOCNI_LED_COUNT; i++) {
    stripBocni.setPixelColor(i, barvaLevyBok);
  }
  stripBocni.show();
}

// --- EFEKTY PŘI STARTU ---
// (delay() je nahrazen cekejSLoadingem(), který mezitím obnovuje loading animaci na LCD)
void dynamicBlinkers() {
  posledniCasZarovky = millis();
  zarovkaPravy = 0.0f;
  zarovkaLevy  = 0.0f;

  for (int b = 0; b < 2; b++) {
    digitalWrite(PIN_RELE_ZVUK, LOW);
    unsigned long startSvitu = millis();
    for (int i = 0; i < AKTIVNI_ZONA_LEDS; i++) {
      int levyIdx  = (AKTIVNI_ZONA_LEDS - 1) - i;
      int pravyIdx = (LED_COUNT - AKTIVNI_ZONA_LEDS) + i;

      if (levyIdx >= 0 && levyIdx < LED_COUNT) {
        stripPredni.setPixelColor(levyIdx, stripPredni.Color(255, 100, 0));
        stripZadni.setPixelColor(levyIdx, stripZadni.Color(255, 100, 0));
      }
      if (pravyIdx >= 0 && pravyIdx < LED_COUNT) {
        stripPredni.setPixelColor(pravyIdx, stripPredni.Color(255, 100, 0));
        stripZadni.setPixelColor(pravyIdx, stripZadni.Color(255, 100, 0));
      }
      stripPredni.show();
      stripZadni.show();
      vykresliBocniZarovkovyBlinkr(millis(), true, true);
      cekejSLoadingem(35);
    }
    digitalWrite(PIN_RELE_ZVUK, HIGH);

    while (millis() - startSvitu < (AKTIVNI_ZONA_LEDS * 35 + 150)) {
      vykresliBocniZarovkovyBlinkr(millis(), true, true);
      cekejSLoadingem(10);
    }

    for (int i = 0; i < AKTIVNI_ZONA_LEDS; i++) {
      int levyIdx  = (AKTIVNI_ZONA_LEDS - 1) - i;
      int pravyIdx = (LED_COUNT - AKTIVNI_ZONA_LEDS) + i;

      if (levyIdx >= 0 && levyIdx < LED_COUNT) {
        stripPredni.setPixelColor(levyIdx, 0);
        stripZadni.setPixelColor(levyIdx, 0);
      }
      if (pravyIdx >= 0 && pravyIdx < LED_COUNT) {
        stripPredni.setPixelColor(pravyIdx, 0);
        stripZadni.setPixelColor(pravyIdx, 0);
      }
    }
    stripPredni.show();
    stripZadni.show();

    unsigned long startZhasnuti = millis();
    while (millis() - startZhasnuti < 200) {
      vykresliBocniZarovkovyBlinkr(millis(), false, false);
      cekejSLoadingem(10);
    }
  }

  while (zarovkaPravy > 0.001f || zarovkaLevy > 0.001f) {
    vykresliBocniZarovkovyBlinkr(millis(), false, false);
    cekejSLoadingem(10);
  }
}

void runOnSequence() {
  dynamicBlinkers();
  int mid = LED_COUNT / 2;
  for (int i = 0; i <= mid; i++) {
    for (int j = 0; j <= i; j++) {
      int left = mid - j;
      int right = mid + j;
      if (left >= 0) {
        stripPredni.setPixelColor(left, stripPredni.Color(178, 178, 178));
        stripZadni.setPixelColor(left, stripZadni.Color(255, 0, 0));
      }
      if (right < LED_COUNT) {
        stripPredni.setPixelColor(right, stripPredni.Color(178, 178, 178));
        stripZadni.setPixelColor(right, stripZadni.Color(255, 0, 0));
      }
    }
    stripPredni.show();
    stripZadni.show();
    cekejSLoadingem(40);
  }
  pipniZvuk(60);
}

// --- LOGIKA ŘÍZENÍ IMPULZU BLDC DRIVERU S BEZPEČNOSTNÍM UBRANÍM PLYNU ---
void kontrolaBLDCDriveru() {
  unsigned long nyni = millis();

  if ((bldcMaBytSepnuto != minulyBldcStav) && !pulzProbiha) {
    if (nyni - casKoncePoslednihoPulzu >= MIN_PAUZA_MEZI_PULSY) {
      pulzProbiha = true;
      casStartPulzu = nyni;

      nastavPwmPlynu(0);
      delay(500);
      digitalWrite(PIN_RELE_BLDC, LOW);
      minulyBldcStav = bldcMaBytSepnuto;
    }
  }

  if (pulzProbiha) {
    if (nyni - casStartPulzu >= (DELKA_PULZU_BLDC + 500)) {
      digitalWrite(PIN_RELE_BLDC, HIGH);
      pulzProbiha = false;
      casKoncePoslednihoPulzu = millis();
    }
  } else {
    digitalWrite(PIN_RELE_BLDC, HIGH);
  }
}

// --- LOGIKA SMĚRU A NEUTRÁLU (PROHOZENÉ POLOHY PŘEPÍNAČE) ---
void masterSmerAManagement() {
  bool polohaVpred = (digitalRead(PIN_SMER_VZAD) == LOW);
  bool polohaVzad  = (digitalRead(PIN_SMER_VPRED) == LOW);
  unsigned long nyni = millis();

  if (polohaVpred) {
    jeNeutral = false;
    bldcMaBytSepnuto = true;
    stavZpatecka = false;
    veStavuNeutral = false;
  }
  else if (polohaVzad) {
    jeNeutral = false;
    bldcMaBytSepnuto = true;
    stavZpatecka = true;
    veStavuNeutral = false;
  }
  else {
    jeNeutral = true;
    nastavPwmPlynu(0);
    if (!veStavuNeutral) {
      veStavuNeutral = true;
      casVstupuDoNeutralu = nyni;
    }
    if (nyni - casVstupuDoNeutralu >= ZPOZDENI_NEUTRALU) {
      bldcMaBytSepnuto = false;
    }
    stavZpatecka = false;
  }
}

void nactiPrepinace() {
  bool novyStavDenni = false;
  bool novyStavDalkova = false;

  if (digitalRead(PIN_SVETLA_VYP) == LOW) {
    novyStavDenni = false;
    novyStavDalkova = false;
  }
  else if (digitalRead(PIN_SVETLA_DALK) == LOW) {
    novyStavDenni = true;
    novyStavDalkova = true;
  }
  else {
    novyStavDenni = true;
    novyStavDalkova = false;
  }

  bool novyStavMajaky = (digitalRead(PIN_PREPINAC_MAJAKY) == LOW);

  if (novyStavDenni != minulyStavDenniSviceni || novyStavDalkova != minulyStavDalkova || novyStavMajaky != minulyStavMajaky) {
    pipniZvuk(50);
  }

  if (minulyStavMajaky && !novyStavMajaky) {
    posledniCasMajakuVyp = millis();
  }

  minulyStavDenniSviceni = novyStavDenni;
  minulyStavDalkova = novyStavDalkova;
  minulyStavMajaky = novyStavMajaky;

  stavDenniSviceni = novyStavDenni;
  stavDalkova = novyStavDalkova;
  stavMajaky = novyStavMajaky;
}

void rizeniZvukuRele(bool aktivniBlinkry, unsigned long casVCyklu) {
  if (aktivniBlinkry) {
    if (casVCyklu < TRVANI_ANIMACE_BLINKRU) digitalWrite(PIN_RELE_ZVUK, LOW);
    else digitalWrite(PIN_RELE_ZVUK, HIGH);
  } else {
    digitalWrite(PIN_RELE_ZVUK, HIGH);
  }
}

void aktualizujBocniPasky(unsigned long nyni, bool aktivniL, bool aktivniP, bool blinkrSviti, bool majakLevaCervena, bool majakPravaModra) {
  unsigned long dtMs = (posledniCasZarovky == 0) ? 0 : (nyni - posledniCasZarovky);
  posledniCasZarovky = nyni;

  bool cilPravyBlinkr = (aktivniP && blinkrSviti);
  bool cilLevyBlinkr  = (aktivniL && blinkrSviti);

  zarovkaPravy = aktualizujZarovku(zarovkaPravy, cilPravyBlinkr, dtMs);
  zarovkaLevy  = aktualizujZarovku(zarovkaLevy, cilLevyBlinkr, dtMs);

  float krivkaPravy = zarovkaPravy * zarovkaPravy;
  float krivkaLevy  = zarovkaLevy * zarovkaLevy;

  uint32_t barvaPravyBok = stripBocni.Color(0, 0, 0);
  uint32_t barvaLevyBok  = stripBocni.Color(0, 0, 0);

  if (aktivniP || zarovkaPravy > 0.001f) {
    uint8_t rP = (uint8_t)(255.0f * krivkaPravy);
    uint8_t gP = (uint8_t)(100.0f * krivkaPravy);
    barvaPravyBok = stripBocni.Color(rP, gP, 0);
  } else if (stavMajaky) {
    barvaPravyBok = majakPravaModra ? stripBocni.Color(0, 0, 255) : stripBocni.Color(0, 0, 0);
  }

  if (aktivniL || zarovkaLevy > 0.001f) {
    uint8_t rL = (uint8_t)(255.0f * krivkaLevy);
    uint8_t gL = (uint8_t)(100.0f * krivkaLevy);
    barvaLevyBok = stripBocni.Color(rL, gL, 0);
  } else if (stavMajaky) {
    barvaLevyBok = majakLevaCervena ? stripBocni.Color(255, 0, 0) : stripBocni.Color(0, 0, 0);
  }

  for (int i = 0; i < 8; i++) {
    stripBocni.setPixelColor(i, barvaPravyBok);
  }
  for (int i = 8; i < BOCNI_LED_COUNT; i++) {
    stripBocni.setPixelColor(i, barvaLevyBok);
  }

  stripBocni.show();
}

void aktualizujSvetla() {
  unsigned long nyni = millis();

  int cyklusMajaku = (nyni - casMajaku) % 600;
  bool majakLevaCervena = false;
  bool majakPravaModra  = false;

  if (stavMajaky) {
    if ((cyklusMajaku >= 0 && cyklusMajaku < 50) || (cyklusMajaku >= 100 && cyklusMajaku < 150) || (cyklusMajaku >= 200 && cyklusMajaku < 250)) {
      majakLevaCervena = true;
    }
    else if ((cyklusMajaku >= 300 && cyklusMajaku < 350) || (cyklusMajaku >= 400 && cyklusMajaku < 450) || (cyklusMajaku >= 500 && cyklusMajaku < 550)) {
      majakPravaModra = true;
    }
  }

  if (nyni - casBlinkru >= TRVANI_CYKLU_BLINKRU) casBlinkru = nyni;
  unsigned long casVCyklu = nyni - casBlinkru;
  bool blinkrSviti = (casVCyklu < TRVANI_ANIMACE_BLINKRU);
  int sviticichLeds = 0;

  if (blinkrSviti) {
    sviticichLeds = map(casVCyklu, 0, TRVANI_ANIMACE_BLINKRU, 1, AKTIVNI_ZONA_LEDS + 1);
    if (sviticichLeds > AKTIVNI_ZONA_LEDS) sviticichLeds = AKTIVNI_ZONA_LEDS;
  }

  bool blinkrL = (digitalRead(PIN_BLINKR_L) == LOW);
  bool blinkrP = (digitalRead(PIN_BLINKR_P) == LOW);
  bool vystrazne = (digitalRead(PIN_VYSTRAZNE) == LOW);

  bool aktivniL = (blinkrL || vystrazne);
  bool aktivniP = (blinkrP || vystrazne);

  // Ikony blinkrů na LCD bliká stejně jako LED (výstražné = obě)
  lcdBlinkL = (aktivniL && blinkrSviti);
  lcdBlinkP = (aktivniP && blinkrSviti);

  if (aktivniL) posledniCasBlinkruL = nyni;
  if (aktivniP) posledniCasBlinkruP = nyni;

  float factorL = 1.0f;
  float factorP = 1.0f;
  float factorMajaky = 1.0f;

  if (nyni - posledniCasBlinkruL < TRVANI_FADE_IN) factorL = (float)(nyni - posledniCasBlinkruL) / (float)TRVANI_FADE_IN;
  if (nyni - posledniCasBlinkruP < TRVANI_FADE_IN) factorP = (float)(nyni - posledniCasBlinkruP) / (float)TRVANI_FADE_IN;
  if (!stavMajaky && (nyni - posledniCasMajakuVyp < TRVANI_FADE_IN)) {
    factorMajaky = (float)(nyni - posledniCasMajakuVyp) / (float)TRVANI_FADE_IN;
  }

  rizeniZvukuRele((aktivniL || aktivniP), casVCyklu);

  const int POCET_PREDNI_MAJAK_BLOK = 8;
  const int MEZERA_PREDNI_MAJAK     = 3;
  const int SIRKA_PREDNI_MAJAK_CELE = POCET_PREDNI_MAJAK_BLOK * 2 + MEZERA_PREDNI_MAJAK;
  const int ZACATEK_PREDNI_MAJAK_L  = (LED_COUNT - SIRKA_PREDNI_MAJAK_CELE) / 2;
  const int KONEC_PREDNI_MAJAK_L    = ZACATEK_PREDNI_MAJAK_L + POCET_PREDNI_MAJAK_BLOK;
  const int ZACATEK_PREDNI_MAJAK_P  = KONEC_PREDNI_MAJAK_L + MEZERA_PREDNI_MAJAK;
  const int KONEC_PREDNI_MAJAK_P    = ZACATEK_PREDNI_MAJAK_P + POCET_PREDNI_MAJAK_BLOK;

  const int POCET_STRED_BLOK = 4;
  const int MEZERA_STRED     = 3;
  const int SIRKA_STRED_CELE = POCET_STRED_BLOK * 2 + MEZERA_STRED;
  const int ZACATEK_STRED_L  = (LED_COUNT - SIRKA_STRED_CELE) / 2;
  const int KONEC_STRED_L    = ZACATEK_STRED_L + POCET_STRED_BLOK;
  const int ZACATEK_STRED_P  = KONEC_STRED_L + MEZERA_STRED;
  const int KONEC_STRED_P    = ZACATEK_STRED_P + POCET_STRED_BLOK;

  uint8_t jasBrzdy = ((nyni / 60) % 2 == 0) ? 255 : 45;

  uint8_t jasPredni = 0;
  if (stavDalkova) jasPredni = 255;
  else if (stavDenniSviceni) jasPredni = 178;

  for (int i = 0; i < LED_COUNT; i++) {
    bool jeLevaStrana = (i < AKTIVNI_ZONA_LEDS);
    bool jePravaStrana = (i >= LED_COUNT - AKTIVNI_ZONA_LEDS);
    bool jePredniMajakLevy = (i >= ZACATEK_PREDNI_MAJAK_L && i < KONEC_PREDNI_MAJAK_L);
    bool jePredniMajakPravy = (i >= ZACATEK_PREDNI_MAJAK_P && i < KONEC_PREDNI_MAJAK_P);

    if (jeLevaStrana && aktivniL) {
      int indexVeZone = (AKTIVNI_ZONA_LEDS - 1) - i;
      if (indexVeZone < sviticichLeds) stripPredni.setPixelColor(i, stripPredni.Color(255, 100, 0));
      else stripPredni.setPixelColor(i, stripPredni.Color(0, 0, 0));
    }
    else if (jePravaStrana && aktivniP) {
      int indexVeZone = i - (LED_COUNT - AKTIVNI_ZONA_LEDS);
      if (indexVeZone < sviticichLeds) stripPredni.setPixelColor(i, stripPredni.Color(255, 100, 0));
      else stripPredni.setPixelColor(i, stripPredni.Color(0, 0, 0));
    }
    else if (stavMajaky && (jePredniMajakLevy || jePredniMajakPravy)) {
      if (jePredniMajakLevy && majakLevaCervena) stripPredni.setPixelColor(i, stripPredni.Color(255, 0, 0));
      else if (jePredniMajakPravy && majakPravaModra) stripPredni.setPixelColor(i, stripPredni.Color(0, 0, 255));
      else stripPredni.setPixelColor(i, stripPredni.Color(0, 0, 0));
    }
    else {
      float f = jeLevaStrana ? factorL : (jePravaStrana ? factorP : 1.0f);
      if (jePredniMajakLevy || jePredniMajakPravy) {
        f *= factorMajaky;
      }
      uint8_t finalJas = (uint8_t)(jasPredni * f);
      stripPredni.setPixelColor(i, stripPredni.Color(finalJas, finalJas, finalJas));
    }
  }

  uint8_t jasZadniObrys = stavDenniSviceni ? 80 : 0;

  for (int i = 0; i < LED_COUNT; i++) {
    bool jeLevaStrana = (i < AKTIVNI_ZONA_LEDS);
    bool jePravaStrana = (i >= LED_COUNT - AKTIVNI_ZONA_LEDS);
    bool jeStredLevy = (i >= ZACATEK_STRED_L && i < KONEC_STRED_L);
    bool jeStredPravy = (i >= ZACATEK_STRED_P && i < KONEC_STRED_P);
    bool jeZpateckaLed = stavZpatecka && (jeStredLevy || jeStredPravy);

    uint32_t barvaPozadi;

    if (stavMajaky && (jeStredLevy || jeStredPravy)) {
      if (jeStredLevy && majakLevaCervena) barvaPozadi = stripZadni.Color(255, 0, 0);
      else if (jeStredPravy && majakPravaModra) barvaPozadi = stripZadni.Color(0, 0, 255);
      else barvaPozadi = stripZadni.Color(0, 0, 0);
    }
    else if (jeZpateckaLed) {
      barvaPozadi = stripZadni.Color(255, 255, 255);
    } else if (stavBrzda) {
      barvaPozadi = stripZadni.Color(jasBrzdy, 0, 0);
    } else {
      barvaPozadi = stripZadni.Color(jasZadniObrys, 0, 0);
    }

    if (jeLevaStrana && aktivniL) {
      int indexVeZone = (AKTIVNI_ZONA_LEDS - 1) - i;
      if (indexVeZone < sviticichLeds) stripZadni.setPixelColor(i, stripZadni.Color(255, 100, 0));
      else stripZadni.setPixelColor(i, stripZadni.Color(0, 0, 0));
    }
    else if (jePravaStrana && aktivniP) {
      int indexVeZone = i - (LED_COUNT - AKTIVNI_ZONA_LEDS);
      if (indexVeZone < sviticichLeds) stripZadni.setPixelColor(i, stripZadni.Color(255, 100, 0));
      else stripZadni.setPixelColor(i, stripZadni.Color(0, 0, 0));
    }
    else {
      float f = jeLevaStrana ? factorL : (jePravaStrana ? factorP : 1.0f);
      if (jeStredLevy || jeStredPravy) {
        f *= factorMajaky;
      }
      uint8_t r = (uint8_t)(((barvaPozadi >> 16) & 0xFF) * f);
      uint8_t g = (uint8_t)(((barvaPozadi >> 8) & 0xFF) * f);
      uint8_t b = (uint8_t)((barvaPozadi & 0xFF) * f);
      stripZadni.setPixelColor(i, stripZadni.Color(r, g, b));
    }
  }

  stripPredni.show();
  stripZadni.show();

  aktualizujBocniPasky(nyni, aktivniL, aktivniP, blinkrSviti, majakLevaCervena, majakPravaModra);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  vypisDuvodResetu();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  inicializujLcd();

  stripStavova.begin();
  stripStavova.setBrightness(100);
  nastavStavovouLed(255, 100, 0);

  pinMode(PIN_SMER_VPRED, INPUT_PULLUP);
  pinMode(PIN_SMER_VZAD, INPUT_PULLUP);
  pinMode(PIN_BLINKR_L, INPUT_PULLUP);
  pinMode(PIN_BLINKR_P, INPUT_PULLUP);
  pinMode(PIN_SVETLA_VYP, INPUT_PULLUP);
  pinMode(PIN_SVETLA_DALK, INPUT_PULLUP);
  pinMode(PIN_VYSTRAZNE, INPUT_PULLUP);
  pinMode(PIN_PREPINAC_MAJAKY, INPUT_PULLUP);

  pinMode(PIN_POT_PLYNU, INPUT);
  pinMode(PIN_POT_BRZDY, INPUT);

  // Hall sonda rychlosti (externí pull-up na 3,3 V doporučen, INPUT_PULLUP je jen záloha)
  pinMode(PIN_HALL_RYCHLOST, INPUT_PULLUP);
  hallPosledniUs = micros() - HALL_TIMEOUT_US;   // první pulz se bere jako neplatný
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_RYCHLOST), hallISR, RISING);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_PWM_PLYNU, PWM_CH);
  nastavPwmPlynu(0);

  pinMode(PIN_RELE_BLDC, OUTPUT);
  digitalWrite(PIN_RELE_BLDC, HIGH);

  pinMode(PIN_RELE_ZVUK, OUTPUT);
  digitalWrite(PIN_RELE_ZVUK, HIGH);

  for (int i = 0; i < 6; i++) {
    pinMode(PIN_RELE_BRZDY[i], OUTPUT);
    digitalWrite(PIN_RELE_BRZDY[i], HIGH);
  }

  stripPredni.begin();
  stripZadni.begin();
  stripBocni.begin();
  stripPredni.show();
  stripZadni.show();
  stripBocni.show();

  randomSeed(esp_random());

  minulyStavDenniSviceni = (digitalRead(PIN_SVETLA_VYP) == HIGH);
  minulyStavDalkova      = (digitalRead(PIN_SVETLA_DALK) == LOW);
  minulyStavMajaky       = (digitalRead(PIN_PREPINAC_MAJAKY) == LOW);

  loadingStart();        // LCD: titulek + progress bar + procenta
  runOnSequence();       // LED animace; během ní běží rychlé hlášky a procenta na LCD
  loadingHotovo();       // 100 %
  inicializujDashboard();

  nastavStavovouLed(0, 255, 0);

  casMajaku = millis();
  casBlinkru = millis();
  posledniCasZarovky = millis();
}

void loop() {
  unsigned long nyni = millis();

  masterSmerAManagement();
  nactiPrepinace();
  zpracujPlynABrzdu();
  kontrolaBLDCDriveru();
  aktualizujSvetla();
  aktualizujRychlost();
  aktualizujDashboardLcd(nyni);

#if HALL_DEBUG
  static unsigned long posledniDebug = 0;
  if (nyni - posledniDebug >= 200) {
    posledniDebug = nyni;
    Serial.print("HALL pin=");
    Serial.print(digitalRead(PIN_HALL_RYCHLOST));
    Serial.print("  pulzu=");
    Serial.print(hallPocitadlo);
    Serial.print("  perioda_us=");
    Serial.print(hallPeriodaUs);
    Serial.print("  v=");
    Serial.print(rychlostFiltrKmh, 2);
    Serial.println(" km/h");
  }
#else
  Serial.print("PWM driver (0-255): ");
  Serial.print(aktualniPwmPlynu);
  Serial.print(" (");
  Serial.print((aktualniPwmPlynu * 100) / 255);
  Serial.print("%)  Rychlost: ");
  Serial.print(aktualniRychlost);
  Serial.println(" km/h");
#endif

  delay(10);
}