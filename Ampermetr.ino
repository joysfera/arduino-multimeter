#include <EnableInterrupt.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <SD.h>

#define BUTTON_DEBOUNCE_TIME    200

#define SD_CS    4  // Chip select line for SD card

#define TFT_RST    7
#define TFT_DC     8
#define TFT_CS     10

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

#define TASKER_MAX_TASKS 3
#include <Tasker.h>
Tasker tasker;

#define MOSFET_PIN 9

enum PREC { PR10, PR11, PR12 };
const byte PRECCOUNT[] = { 1, 4, 16 };
PREC precision = PR10;

#define ANALOG_MAX 1023

#define DIVIDER_U1 253UL / 50
#define DIVIDER_U2 253UL / 50

const byte PORTS[] = { A6, 0, A0, A1, A2, A3 }; // internal ref., VCC, U1, I1, U2, I2

bool lastInternalState[sizeof(PORTS)] = { true, false, true, true, true, true };
byte lastReference = 255; // impossible value

unsigned int refVoltage10;
unsigned int vccVoltage10;

unsigned int refVoltage;
unsigned int vccVoltage;

unsigned long mAh;

bool sdcard;

int rrd[120*2];
byte rrdptr;

#define MENU_MAX 2
enum MENU_OPTIONS { FREQ };
const byte MENU_OPTION_MAX[MENU_MAX] = { 6, 20 };
const unsigned long freqValues[] = { 0, 4*2, 30*2, 4*60*2, 30*60*2, 4*3600*2 };
volatile byte menuOption;
volatile byte menuValue[MENU_MAX];

unsigned int readVcc()
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    lastReference = 255;
    delay(2); // Wait for Vref to settle

    unsigned int result = 0;
    for(byte i = 0; i < PRECCOUNT[precision]; i++) {
        ADCSRA |= _BV(ADSC); // Start conversion
        while (bit_is_set(ADCSRA,ADSC)); // measuring
        result += ADCW;
    }

    return result;
}

unsigned int mujAnalogRead(byte port, byte reference)
{
    ADMUX = (reference << 6) | (port & 0x07);
    delay(5);
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
    // return ADCW; // zahodit
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
    return ADCW;
}

inline int setReferenceAndRead(byte port, bool isInternalReference)
{
#if 0
    if ((isInternalReference && lastReference != INTERNAL) || (!isInternalReference && lastReference != DEFAULT)) {
        lastReference = isInternalReference ? INTERNAL : DEFAULT;
        analogReference(lastReference);
        analogRead(port); // dummy read to stabilize the new reference
        analogRead(port); // dummy read to stabilize the new reference
    }
    return analogRead(port);
#else
    return mujAnalogRead(port-14, isInternalReference ? INTERNAL : DEFAULT);
#endif
}

int AnalogRead(byte index)
{
    byte port = PORTS[index];
    if (port == 0)
        return readVcc();

    bool internal = lastInternalState[index];
    int value = setReferenceAndRead(port, internal);

    if (internal && value == ANALOG_MAX) { // overflow
        internal = false;
        lastInternalState[index] = internal;
        value = setReferenceAndRead(port, internal);
    }
    else if (!internal && value < ((unsigned long)refVoltage * ANALOG_MAX / vccVoltage)) {
        internal = true;
        lastInternalState[index] = internal;
        value = setReferenceAndRead(port, internal);
    }

    for(byte i = 1; i < PRECCOUNT[precision]; i++)
        value += analogRead(port);

    return internal ? value : -value;
}

void updateReferenceVoltages()
{
    int portValues[sizeof(PORTS)];
    for(byte i = 0; i < sizeof(PORTS); i++) {
        portValues[i] = AnalogRead(i);
    }

    refVoltage10 = ANALOG_MAX * 10000UL * (1 << precision) / (portValues[0] >> precision);
    vccVoltage10 = (unsigned long)refVoltage10 * ANALOG_MAX * (1 << precision) / (portValues[1] >> precision);

    refVoltage = (refVoltage10 + 5) / 10;
    vccVoltage = (vccVoltage10 + 5) / 10;

    tft.setCursor(40, 0);
    tft.print(portValues[0]);
    tft.print(F("  "));
    tft.print(refVoltage10 / 10);
    tft.print('.');
    tft.print(refVoltage10 % 10);
    tft.print(F("mV "));

    tft.setCursor(40, 10);
    tft.print(portValues[1]);
    tft.print(F("  "));
    tft.print(vccVoltage10 / 10);
    tft.print('.');
    tft.print(vccVoltage10 % 10);
    tft.print(F("mV "));   
}

void button1(void)
{
    static unsigned long lastTime;
    unsigned long curTime = millis();
    if (curTime - lastTime > BUTTON_DEBOUNCE_TIME) {
        if (!digitalRead(3)) {
            if (++menuOption >= MENU_MAX)
                menuOption = 0;
        }
    }
    lastTime = curTime;
}

void button2(void)
{
    static unsigned long lastTime;
    unsigned long curTime = millis();
    if (curTime - lastTime > BUTTON_DEBOUNCE_TIME) {
        if (!digitalRead(5)) {
            if (++menuValue[menuOption] >= MENU_OPTION_MAX[menuOption])
                menuValue[menuOption] = 0;
        }
    }
    lastTime = curTime;
}

void setup(void)
{
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, HIGH);

    if (SD.begin(SD_CS))
        sdcard = true;

    tft.initR(INITR_BLACKTAB);

    pinMode(3, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);

    enableInterrupt(3, button1, CHANGE);
    enableInterrupt(5, button2, CHANGE);

    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);

    tft.setCursor(0, 0);
    tft.print(F("Refer: "));

    tft.setCursor(0, 10);
    tft.print(F("VCC: "));

    tft.setCursor(0, 20);
    tft.print(F("Jack U: "));

    tft.setCursor(0, 30);
    tft.print(F("Jack I: "));

    tft.setCursor(0, 40);
    tft.print(F("USB U: "));

    tft.setCursor(0, 50);
    tft.print(F("USB I: "));

    tft.setCursor(0, 80);
    tft.print(F("FPS:"));

    updateReferenceVoltages();

    rrd[rrdptr] = 0; // just test

    tasker.setInterval(measure, 1000);
    tasker.setInterval(menu, 100);
    //tasker.setInterval(displayValues, 1000);
    tasker.run();
}

void loop() { }

byte updateReferenceVoltagesTimer = 0; // always

int portValues[sizeof(PORTS)];
unsigned long milli[4];

void measure(int)
{
    static byte updateRefCounter;
    if (updateRefCounter++ > updateReferenceVoltagesTimer) {
        updateReferenceVoltages();
        updateRefCounter = 0;
    }

    for(byte i = 2; i < sizeof(PORTS); i++) {
        portValues[i] = AnalogRead(i);
    }

    for(byte i = 0; i < 4; i++) {
        byte idx = i + 2;
        long val = portValues[idx];
        milli[i] = (val < 0) ? (-val * vccVoltage) : (val * refVoltage);
    }

    displayValues();
}

void displayValues()
{
    tft.setCursor(40, 20);
    tft.print(portValues[2]);
    tft.print(F("  "));
    unsigned long u1 = milli[0] * DIVIDER_U1 / ANALOG_MAX;
    tft.print(u1);
    tft.print(F("mV  "));

    tft.setCursor(40, 30);
    tft.print(portValues[3]);
    tft.print(F("  "));
    tft.print(milli[1] / ANALOG_MAX);
    tft.print(F("mA  "));

    tft.setCursor(40, 40);
    tft.print(portValues[4]);
    tft.print(F("  "));
    unsigned long u2 = milli[2] * DIVIDER_U2 / ANALOG_MAX;
    tft.print(u2);
    tft.print(F("mV  "));

    tft.setCursor(40, 50);
    tft.print(portValues[5]);
    tft.print(F("  "));
    tft.print(milli[3] / ANALOG_MAX);
    tft.print(F("mA  "));

    mAh += milli[3] / ANALOG_MAX;

    tft.setCursor(0, 60);
    tft.print(mAh / 3600);
    tft.print(F(" mAh "));
/*
    tft.setCursor(30, 80);
    unsigned fps = 1000 / delta;
    tft.print(fps);
    tft.print(F("  "));
*/    
/*
    for(byte i = 0; i < sizeof(PORTS); i++) {
        tft.setCursor(120, i*10);
        tft.print(lastInternalState[i] ? "I" : "V");
    }
*/
}

void menu(int)
{
    static byte lastMenu = 255;
    static byte lastValue = 255;
    if (lastMenu != menuOption) {
        lastMenu = menuOption;
        lastValue = 255;
        tft.setCursor(0, 100);
        const char *menuOptions[] = { "Freq", "File" };
        tft.print(menuOptions[lastMenu]);
        tft.print(F(" "));
    }
    if (lastValue != menuValue[lastMenu]) {
        lastValue = menuValue[lastMenu];
        tft.setCursor(50, 100);
        tft.print(lastValue);
        tft.print(F(" "));
    }
}  

