/*

DDS code based on Pete Juliano's super simple DDS code
http://www.jessystems.com/Images/Arduino/AD9850_Signal_Generator.txt

Rotary Encoder code based on my ATmega328 port
http://blog.templaro.com/reading-a-rotary-encoder-demo-code-for-atmega328/
of Oleg Mazurov's code written for ATmega644p
https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros

 
  The circuit:
 * 5V to Arduino 5V pin
 * GND to Arduino GND pin
 
 * CLK to digital #4
 * FQ to digital #5
 * DAT to digital #6
 * RST to digital #7
 
 * Rotary Encoder left tab to digital #3
 * Rotary Encoder right tab to digital #2
 * Rotary Encoder middle tab to ground
 * Rotary Switch to digital #1
 
*/

// include the library code:
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
#include <EEPROM.h>

// set rotary encoder pins
#define ENC_RD	PIND	//encoder port read
#define RotEncSwPin 1 // digital pin 4
#define A_PIN 2  // INT0 vector; digital pin 2
#define B_PIN 3  // INT1 vector; digital pin 3

// set up AD9850 pins
#define W_CLK 4
#define FQ_UD 5
#define DATAPIN 6
#define RESET 7

// set up memory push buttons
#define MEM_BUTTON_START_PIN  8  // digital pin where mem buttons start
#define NUMBER_MEM_BUTTONS    2  // how many mem buttons

const double bandStart = 100000;  // start of Gnerator at 100 KHZ
const double bandEnd = 40000000; // End of Generator at 40 MHz --signal will be a bit flaky!
const double bandInit = 7150000; // where to initially set the frequency for tetsting Part II

double freq = 0 ;  // this is a variable (changes) - set it to the beginning of the band
double freqDelta = 10000; // how much to change the frequency by, clicking the rotary encoder will change this.

byte counter = 0;
long lastClick = 0;
long lastBlink = 0;
boolean blinkState = false;

boolean mem_armed[NUMBER_MEM_BUTTONS];
long last_mem[NUMBER_MEM_BUTTONS];

int eeAddressStart = 0;

// Connect via i2c, default address #1 (nothing jumpered)
Adafruit_LiquidCrystal lcd(0);

void setup() {
  lcd.begin(16,2);
  lcd.setCursor(0,1); // This places a display on the LCD at turn on at the 2nd line
  lcd.print(" 5R8SV Sig Gen");
  
  // Set up DDS
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATAPIN, OUTPUT);
  pinMode(RESET, OUTPUT);
  
  // Set up Rotary Encoder
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  attachInterrupt(0, evaluateRotary, CHANGE);
  attachInterrupt(1, evaluateRotary, CHANGE);
  pinMode(RotEncSwPin, INPUT_PULLUP);
  
  // Set up memory buttons
  for (int buttons=0; buttons < NUMBER_MEM_BUTTONS; buttons++) {
    mem_armed[buttons] = false;
    last_mem[buttons] = 0;
    pinMode(MEM_BUTTON_START_PIN + buttons,INPUT_PULLUP);  
  }
    
 // start on M1 freq
  EEPROM.get(eeAddressStart,freq);
  if(freq < bandStart || freq > bandEnd) {
    freq = bandInit;
    EEPROM.put(eeAddressStart,freq);
  }
  
 // start up the DDS... 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD); 
  // start the oscillator...
  send_frequency(freq);     
  display_frequency(freq);
  
  delay(500);
  lcd.setCursor(0,1);
  lcd.print("              ");
}

void loop() {
  int freqCursorPosition = 4;
  
  // change freq dependent on rotary encoder spin direction
  if(counter != 0) {
    if (counter == 1) {
      freq=constrain(freq-freqDelta,bandStart,bandEnd);
    } else {
      freq=constrain(freq+freqDelta,bandStart,bandEnd);
    }
    counter = 0;
    display_frequency(freq); // push the frequency to LCD display
    send_frequency(freq);  // set the DDS to the new frequency  
  }
  
  // check for the click of the rotary encoder 
  if (!digitalRead(RotEncSwPin) && millis() - lastClick > 200) { // push button debounce
    // if user clicks rotary encoder, change the size of the increment
    if (freqDelta == 10) {
      freqDelta = 1000000;
    } else {
      freqDelta /= 10;
    }
    lastClick = millis();
  }
  
  //blink the cursor to show selected frequency resolution
  if (millis() - lastBlink > 300) {
    freqCursorPosition = 8;
    for (long tempDelta = 10; tempDelta/freqDelta != 1; tempDelta *= 10) {
      freqCursorPosition--;
      if (freqCursorPosition == 2 || freqCursorPosition == 6) {  //hop the punctuation
        freqCursorPosition--;
      }
    }
    lcd.setCursor(freqCursorPosition,0);
    if(blinkState) {
      lcd.noBlink();
    } else {
      lcd.blink();
    }
    blinkState = !blinkState;
    lastBlink = millis();
  }
  
  // handle memory buttons
  for (int buttons = 0; buttons < NUMBER_MEM_BUTTONS; buttons++) {
    if(!mem_armed[buttons] && !digitalRead(MEM_BUTTON_START_PIN+buttons) && millis() - last_mem[buttons] > 50) {
      last_mem[buttons] = millis();
      mem_armed[buttons] = true;
    }
    if(mem_armed[buttons] && digitalRead(MEM_BUTTON_START_PIN+buttons) && millis() - last_mem[buttons] > 50) {  
      //pushed, but for how long?
      lcd.setCursor(0,1);
      if(millis() - last_mem[buttons] > 500) {
      // long push = commit to memory
      EEPROM.put(eeAddressStart+buttons*sizeof(double),freq);
      lcd.print("m");
      lcd.print(buttons);
      lcd.print(" stored");
      } else {
      // short push = qsy to memory frequency
      EEPROM.get(eeAddressStart+buttons*sizeof(double),freq);
      lcd.print("recall m");
      lcd.print(buttons);
      send_frequency(freq);     
      display_frequency(freq);
      }
      mem_armed[buttons] = false;
      last_mem[buttons] = millis();
    }
  }
  
  
}

// subroutine to display the frequency...
void display_frequency(double frequency) {  
  int currentCursor;
  byte currentDigit;
  
  lcd.noBlink(); // suppress blinking after printing Mhz
  currentCursor = 0;
  for (long freqDiv = 10000000; freqDiv > 1; freqDiv /=10) {
    currentDigit = ((long(freq)/freqDiv)%10);
    if (currentCursor == 2) {
      lcd.setCursor(currentCursor,0);
      lcd.print(".");
      currentCursor++; 
    } else if (currentCursor == 6) {
      lcd.setCursor(currentCursor,0);
      lcd.print(",");
      currentCursor++;
    } 
    lcd.setCursor(currentCursor,0);
    if (currentCursor == 0 && currentDigit == 0) {
      lcd.print(" ");
    } else {
      lcd.print(currentDigit);
    }
    currentCursor++;
  } 
  lcd.print("0 MHz");
}  

// Subroutine to generate a positive pulse on 'pin'...
void pulseHigh(int pin) {
  digitalWrite(pin, HIGH); 
  digitalWrite(pin, LOW); 
}

// calculate and send frequency code to DDS Module...
void send_frequency(double frequency) {
  int32_t freq = (frequency) * 4294967295/124997797;
  for (int b=0; b<4; b++, freq>>=8) {
    shiftOut(DATAPIN, W_CLK, LSBFIRST, freq & 0xFF);
  } 
  shiftOut(DATAPIN, W_CLK, LSBFIRST, 0x00);  
  pulseHigh(FQ_UD); 
}

void evaluateRotary() {
/* encoder routine. Expects encoder with four state changes between detents */
/* and both pins open on detent */

  static uint8_t old_AB = 3;  //lookup table index
  static int8_t encval = 0;   //encoder value  
  static const int8_t enc_states [] PROGMEM = 
  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table
  /**/
  old_AB <<=2;  //remember previous state
  old_AB |= (( ENC_RD >>2 ) & 0x03 );
  encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
  
  if( encval > 3 ) {  //four steps forward
    counter = 1;
    encval = 0;
  }
  else if( encval < -3 ) {  //four steps backwards
   counter = 2;
   encval = 0;
  } 
}
