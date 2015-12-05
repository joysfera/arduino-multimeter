/*
 * Arduino Dual Channel Digital Multimeter

 * created by Petr Stehlik in 2015

 * petr@pstehlik.cz

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <EnableInterrupt.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <SD.h>

#define PROFILE     1
#ifdef PROFILE
unsigned long measureTime;
#endif

#define BUTTON_DEBOUNCE_TIME    100

#define SD_CS    4  // Chip select line for SD card

#define TFT_RST    7
#define TFT_DC     8
#define TFT_CS     10

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

#define TASKER_MAX_TASKS 3
#include <Tasker.h>
Tasker tasker;

#define MOSFET_PIN 9

enum PREC { PR10 = 0, PR11, PR12 };
const byte PRECSHIFT[] = { 0, 2, 4 };
const byte PRECCOUNT[] = { 1, 4, 16 };

PREC precision = PR10;

#define DIVIDER_U1 253UL / 50
#define DIVIDER_U2 253UL / 50

const byte PORTS[] = { A6, 0, A0, A1, A2, A3 }; // internal ref., VCC, U1, I1, U2, I2

bool lastInternalState[sizeof(PORTS)] = { true, false, true, true, true, true };
byte lastReference = 255; // impossible value

unsigned ref;
unsigned vcc;

unsigned long refVoltage10;
unsigned long vccVoltage10;

unsigned int refVoltage;
unsigned int vccVoltage;

unsigned long mAs1, mAs2;

volatile bool active[4] = { false, false, true, true };
volatile bool largeDisplay;
volatile bool printHeader;
bool displayReference;

bool sdcard;

int rrd[120*2];
byte rrdptr;

#ifdef USE_MENU
#define MENU_MAX 2
enum MENU_OPTIONS { FREQ };
const byte MENU_OPTION_MAX[MENU_MAX] = { 6, 20 };
const unsigned long freqValues[] = { 0, 4*2, 30*2, 4*60*2, 30*60*2, 4*3600*2 };
volatile byte menuOption;
volatile byte menuValue[MENU_MAX];
#endif

volatile bool clearPrintedData;

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
    for(byte i = PRECCOUNT[precision]; i--; i) {
        ADCSRA |= _BV(ADSC); // Start conversion
        while (bit_is_set(ADCSRA,ADSC)); // measuring
        result += ADCW;
    }

    return result;
}

unsigned int myAnalogRead(byte port, byte reference)
{
    ADMUX = (reference << 6) | (port & 0x07);
    delay(5);
#if 1 // throwaway
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
    //(void)ADCW; // throw away
#endif
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
    return myAnalogRead(port-14, isInternalReference ? INTERNAL : DEFAULT);
#endif
}

int AnalogRead(byte index)
{
    byte port = PORTS[index];
    if (port == 0)
        return readVcc();

    bool internal = lastInternalState[index];
    int value = setReferenceAndRead(port, internal);

    if (internal && value == 1023) { // overflow
        internal = false;
        lastInternalState[index] = internal;
        value = setReferenceAndRead(port, internal);
    }
    else if (!internal && value < (((long)refVoltage << 10) / vccVoltage)) {
        internal = true;
        lastInternalState[index] = internal;
        value = setReferenceAndRead(port, internal);
    }

    for(byte i = PRECCOUNT[precision] - 1; i--; i) {
        // value += analogRead(port);
        value += setReferenceAndRead(port, internal);
    }

    return internal ? value : -value;
}

void updateReferenceVoltages()
{
    ref = AnalogRead(0);  // voltage on pin A6 - 1.000 V ~ 932 in 10bit precision
    vcc = AnalogRead(1);  // voltage of internal reference (cca 1.1 V) against VCC ~ 228 in 10bit precision

    refVoltage10 = (10000UL << (10 + precision)) / (ref >> precision); // in 100 microVolt units
    vccVoltage10 = ((unsigned long)refVoltage10 << (10 + precision)) / (vcc >> precision); // in 100 microVolt units

    refVoltage = (refVoltage10 + 5) / 10; // rounded up, in milliVolts
    vccVoltage = (vccVoltage10 + 5) / 10; // rounded up, in milliVolts

    displayReference = true;
}

void button1(void)
{
    static unsigned long lastTime;
    unsigned long curTime = millis();
    if (curTime - lastTime > BUTTON_DEBOUNCE_TIME) {
        if (!digitalRead(3)) {
#ifdef USE_MENU
            if (++menuOption >= MENU_MAX)
                menuOption = 0;
#else
            bool x = active[0];
            bool y = active[2];
            if (x == y) {
                x = true;
                y = false;
            }
            else if (x) {
                x = false;
                y = true;
            }
            else {
                x = y = true;
            }
            active[0] = active[1] = x;
            active[2] = active[3] = y;
            clearPrintedData = true;
#endif
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
#ifdef USE_MENU
            if (++menuValue[menuOption] >= MENU_OPTION_MAX[menuOption])
                menuValue[menuOption] = 0;
#else
            largeDisplay = !largeDisplay;
            printHeader = true;
#endif
        }
    }
    lastTime = curTime;
}

void measure(int);
void menu(int);
void displayValues(int);

void setup(void)
{
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, HIGH);

    if (SD.begin(SD_CS))
        sdcard = true;

    tft.initR(INITR_BLACKTAB);
    tft.setTextWrap(false);

    pinMode(3, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);

    enableInterrupt(3, button1, CHANGE);
    enableInterrupt(5, button2, CHANGE);

    printHeader = true;

    updateReferenceVoltages();

    rrd[rrdptr] = 0; // just test

    tasker.setInterval(measure, 1000);
#ifdef USE_MENU
    tasker.setInterval(menu, 100);
#endif
    tasker.setInterval(displayValues, 1000);
    tasker.run();
}

// loop() is unused as all tasks are called by Tasker
void loop() { }

// right aligned print
void rprint(unsigned long number, byte digits, bool leadingZero = false)
{
    if (digits > 1 && digits < 10) {
        char lead = leadingZero ? '0' : ' ';
        unsigned long range = 10;
        for(byte i = digits-1; i--; i) {
            if (number < range)
                tft.print(lead);
            range *= 10;
        }
    }
    tft.print(number);
}

// right aligned print of value / 10
void rprint10(unsigned long value, byte digits, bool leadingZero = false)
{
    rprint(value / 10, digits);
    tft.print('.');
    tft.print(value % 10);
}

byte updateReferenceVoltagesTimer = 10;

int portValues[4];
unsigned long milli[4];

void measure(int)
{
#ifdef PROFILE
    measureTime = millis();
#endif
    static byte updateRefCounter;
    if (++updateRefCounter > updateReferenceVoltagesTimer) {
        updateReferenceVoltages();
        updateRefCounter = 0;
    }

    for(byte i = 0; i < 4; i++) {
		if (!active[i]) continue;
        portValues[i] = AnalogRead(2+i);
        long val = portValues[i];
        milli[i] = (val < 0) ? (-val * vccVoltage) : (val * refVoltage);
    }
#ifdef PROFILE
    measureTime = millis() - measureTime;
#endif
}

void displayValues(int)
{
    unsigned long u1 = (milli[0] * DIVIDER_U1) >> (PRECSHIFT[precision] + 10);
    unsigned int i1 = milli[1] >> (PRECSHIFT[precision] + 10);
    unsigned int u2 = (milli[2] * DIVIDER_U2) >> (PRECSHIFT[precision] + 10);
    unsigned int i2 = milli[3] >> (PRECSHIFT[precision] + 10);
    mAs1 += i1;
    mAs2 += i2;

    if (printHeader) {
        tft.fillScreen(ST7735_BLACK);
    }
    
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    if (largeDisplay) {
        printHeader = false;
        tft.setTextSize(1);
        tft.setCursor(4, 1);
        unsigned long val;
        bool mA = false;
        unsigned long mAs;
        if (active[0]) {
            val = u1;
            mAs = mAs1;
            tft.print(F("Jack: voltage [mV]"));
        }
        else if (active[1]) {
            val = i1;
            mA = true;
            mAs = mAs1;
            tft.print(F("Jack: current [mA]"));
        }
        else if (active[2]) {
            val = u2;
            mAs = mAs2;
            tft.print(F("USB: voltage [mV] "));
        }
        else if (active[3]) {
            val = i2;
            mA = true;
            mAs = mAs2;
            tft.print(F("USB: current [mA] "));
        }

        tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
        tft.setTextSize(4);
        tft.setCursor(8, 12);
        rprint(val, 5);

        tft.setCursor(8, 56);
        if (mA) {
            rprint(mAs / 3600, 5);
        }
    }
    else {
        tft.setTextSize(1);
        if (printHeader) {
            tft.setCursor(0, 0);
            tft.print(F("Ref: "));

            tft.setCursor(0, 10);
            tft.print(F("VCC: "));

            tft.setCursor(0, 20);
            tft.print(F("JackU: "));

            tft.setCursor(0, 30);
            tft.print(F("JackI: "));

            tft.setCursor(0, 40);
            tft.print(F("USB U: "));

            tft.setCursor(0, 50);
            tft.print(F("USB I: "));

            tft.setCursor(0, 80);
            tft.print(F("Time:"));

            printHeader = false;
        }

        if (displayReference) {
            tft.setCursor(30, 0);
            rprint(ref, 5);
            rprint10(refVoltage10, 6);
            tft.print(F(" mV"));
            tft.clreol();

            tft.setCursor(30, 10);
            rprint(vcc, 5);
            rprint10(vccVoltage10, 6);
            tft.print(F(" mV"));

            displayReference = false;
        }

        tft.setCursor(36, 20);
    if (active[0]) {
        rprint(portValues[0], 5);
        rprint(u1, 7);
        tft.print(F(" mV"));
    }
    else if (clearPrintedData) {
        tft.clreol();
    }

        tft.setCursor(36, 30);
    if (active[1]) {
        rprint(portValues[1], 5);
        rprint(i1, 6);
        tft.print(F(" mA"));
    }
    else if (clearPrintedData) {
        tft.clreol();
    }

        tft.setCursor(36, 40);
    if (active[2]) {
        rprint(portValues[2], 5);
        rprint(u2, 6);
        tft.print(F(" mV"));
    }
    else if (clearPrintedData) {
        tft.clreol();
    }

        tft.setCursor(36, 50);
    if (active[3]) {
        rprint(portValues[3], 5);
        rprint(i2, 6);
        tft.print(F(" mA"));

        tft.setCursor(0, 60);
        rprint(mAs2 / 3600, 6);
        tft.print(F(" mAh"));
    }
    else if (clearPrintedData) {
        tft.clreol();
    }

        clearPrintedData = false;

        // debug info
        tft.setCursor(30, 80);
        tft.print(measureTime);
        tft.print(F(" ms"));
        tft.clreol();

/*
        for(byte i = 0; i < sizeof(PORTS); i++) {
            tft.setCursor(120, i*10);
            tft.print(lastInternalState[i] ? "I" : "V");
        }
*/
    }
}

#ifdef USE_MENU
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
#endif
