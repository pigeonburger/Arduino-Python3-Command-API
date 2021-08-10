#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
LiquidCrystal* lcd = nullptr;

void Version(){
  Serial.println(F("V0.8"));
}


SoftwareSerial *sserial = NULL;
Servo servos[8];
int servo_pins[] = {0, 0, 0, 0, 0, 0, 0, 0};
boolean connected = false;

int Str2int (String Str_value)
{
  char buffer[10]; //max length is three units
  Str_value.toCharArray(buffer, 10);
  int int_value = atoi(buffer);
  return int_value;
}

void split(String results[], int len, String input, char spChar) {
  String temp = input;
  for (int i=0; i<len; i++) {
    int idx = temp.indexOf(spChar);
    results[i] = temp.substring(0,idx);
    temp = temp.substring(idx+1);
  }
}

uint8_t readCapacitivePin(String data) {
  int pinToMeasure = Str2int(data);
  // readCapacitivePin
  //  Input: Arduino pin number
  //  Output: A number, from 0 to 17 expressing
  //  how much capacitance is on the pin
  //  When you touch the pin, or whatever you have
  //  attached to it, the number will get higher
  //  http://playground.arduino.cc/Code/CapacitiveSensor
  //
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
       if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  //return cycles;
  Serial.println(cycles);
}

// Melody() function
void Tone(String data){
  int idx = data.indexOf('%');
  int len = Str2int(data.substring(0,idx));
  String data2 = data.substring(idx+1);
  int idx2 = data2.indexOf('%');
  int pin = Str2int(data2.substring(0,idx2));
  String data3 = data2.substring(idx2+1);
  String melody[len*2];
  split(melody,len*2,data3,'%');

  for (int thisNote = 0; thisNote < len; thisNote++) {
    int noteDuration = 1000/Str2int(melody[thisNote+len]);
    int note = Str2int(melody[thisNote]);
    tone(pin, note, noteDuration);
    int pause = noteDuration * 1.30;
    delay(pause);
    noTone(pin);
  }
}

// tone() function
void NormalTone(String data){
    int idx = data.indexOf('%');
    int pin = Str2int(data.substring(0,idx));
    String data2 = data.substring(idx+1);
    if (data2.indexOf('%') == -1){
        int freq = Str2int(data.substring(idx+1));
        tone(pin, freq);
    }
    else {
        int idx2 = data2.indexOf('%');
        int freq = Str2int(data2.substring(0,idx2));
        int duration = Str2int(data2.substring(idx2+1));
        tone(pin, freq, duration);
    }
}

// noTone() function
void ToneNo(String data){
  int pin = Str2int(data);
  noTone(pin);
}

/////////////////////////////////
////// BEGIN LCD FUNCTIONS //////
/////////////////////////////////
// See https://www.arduino.cc/en/Reference/LiquidCrystal for reference to the LiquidCrystal library
// @TODO: Add support for LiquidCrystal createChar() operation.

// Creates an object of type LiquidCrystal. (currently only support 4 data pins)
void LCDSet(String data){
    // This string parsing is dogshit but I don't know much C++ so this is the best I could do
    int idx = data.indexOf('%');
    int RS = Str2int(data.substring(0,idx));
    String data2 = data.substring(idx+1);
    int idx2 = data2.indexOf('%');
    int EN = Str2int(data2.substring(0,idx2));
    String data3 = data2.substring(idx2+1);
    int idx3 = data3.indexOf('%');
    int D4 = Str2int(data3.substring(0,idx3));
    String data4 = data3.substring(idx3+1);
    int idx4 = data4.indexOf('%');
    int D5 = Str2int(data4.substring(0,idx4));
    String data5 = data4.substring(idx4+1);
    int idx5 = data5.indexOf('%');
    int D6 = Str2int(data5.substring(0,idx5));
    String data6 = data5.substring(idx5+1);
    int D7 = Str2int(data6);
    lcd = new LiquidCrystal(RS, EN, D4, D5, D6, D7);
}

// Initializes the interface to the LCD screen, and specifies the dimensions of the display.
void LCDBegin(String data){
    int idx = data.indexOf('%');
    int cols = Str2int(data.substring(0,idx));
    int rows = Str2int(data.substring(idx+1));
    lcd->begin(cols, rows);
}

// Deletes LCD object from system memory
void LCDCleanup(){
    delete lcd; lcd = nullptr;
}

// Clears the LCD screen and positions the cursor in the upper-left corner. 
void LCDClear(){
    lcd->clear();
}

// Positions the cursor in the upper-left corner of the LCD.
void LCDHome(){
    lcd->home();
}

// Prints text to the LCD screen
void LCDPrint(String data){
    lcd->print(data);
}

// Sets the location at which subsequent text written to the LCD will be displayed. 
void LCDCursorHandler(String data){
    int idx = data.indexOf('%');
    int col = Str2int(data.substring(0,idx));
    int row = Str2int(data.substring(idx+1));
    lcd->setCursor(col, row);
}

// Displays or hides the LCD cursor
void LCDCursor(int mode){
    if (mode == 1){ //Show
        lcd->cursor();
    } else{ //Hide
        lcd->noCursor();
    }
}

// Displays or hides the blinking LCD cursor.
void LCDBlink(int mode){
    if (mode == 1){ //Show
        lcd->blink();
    } else{ //Hide
        lcd->noBlink();
    }
}

// Turns the LCD display on or off
void LCDDisplay(int mode){
    if (mode == 1){ //Show
        lcd->display();
    } else{ //Hide
        lcd->noDisplay();
    }
}

// Scroll text on the LCD screen
void LCDScroll(int mode){
    /* Modes
    0 = Left
    1 = Right
    2 = Autoscroll
    3 = No Autoscroll */
    if (mode == 0){
        lcd->scrollDisplayLeft();
    } else if (mode == 1){
        lcd->scrollDisplayRight();
    } else if (mode == 2){
        lcd->autoscroll();
    } else if (mode == 3){
        lcd->noAutoscroll();
    }
}

// Sets the direction for text to be written to the LCD
void LCDWritedir(int mode){
    if (mode == 1){ // Left-Right
        lcd->leftToRight();
    } else{ // Right-Left
        lcd->rightToLeft();
    }
}

///////////////////////////////
////// END LCD FUNCTIONS //////
///////////////////////////////


void DigitalHandler(int mode, String data){
      int pin = Str2int(data);
    if(mode<=0){ //read
        Serial.println(digitalRead(pin));
    }else{
        if(pin <0){
            digitalWrite(-pin,LOW);
        }else{
            digitalWrite(pin,HIGH);
        }
        //Serial.println('0');
    }
}

void AnalogHandler(int mode, String data){
     if(mode<=0){ //read
        int pin = Str2int(data);
        Serial.println(analogRead(pin));
    }else{
        String sdata[2];
        split(sdata,2,data,'%');
        int pin = Str2int(sdata[0]);
        int pv = Str2int(sdata[1]);
        analogWrite(pin,pv);
    }
}

void ConfigurePinHandler(String data){
    int pin = Str2int(data);
    if(pin <=0){
        pinMode(-pin,INPUT);
    }else{
        pinMode(pin,OUTPUT);
    }
}

void shiftOutHandler(String data) {
    String sdata[4];
    split(sdata, 4, data, '%');
    int dataPin = sdata[0].toInt();
    int clockPin = sdata[1].toInt();
    String bitOrderName = sdata[2];
    byte value = (byte)(sdata[3].toInt());
    if (bitOrderName == "MSBFIRST") {
       shiftOut(dataPin, clockPin, MSBFIRST, value);
    } else {
       shiftOut(dataPin, clockPin, LSBFIRST, value);
    }
}

void shiftInHandler(String data) {
    String sdata[3];
    split(sdata, 3, data, '%');
    int dataPin = sdata[0].toInt();
    int clockPin = sdata[1].toInt();
    String bitOrderName = sdata[2];
    int incoming;
    if (bitOrderName == "MSBFIRST") {
       incoming = (int)shiftIn(dataPin, clockPin, MSBFIRST);
    } else {
       incoming = (int)shiftIn(dataPin, clockPin, LSBFIRST);
    }
    Serial.println(incoming);
}

void SS_set(String data){
  delete sserial;
  String sdata[3];
  split(sdata,3,data,'%');
  int rx_ = Str2int(sdata[0]);
  int tx_ = Str2int(sdata[1]);
  int baud_ = Str2int(sdata[2]);
  sserial = new SoftwareSerial(rx_, tx_);
  sserial->begin(baud_);
  Serial.println("ss OK");
}

void SS_write(String data) {
 int len = data.length()+1;
 char buffer[len];
 data.toCharArray(buffer,len);
 Serial.println("ss OK");
 sserial->write(buffer);
}
void SS_read(String data) {
 char c = sserial->read();
 Serial.println(c);
}

void pulseInHandler(String data){
    int pin = Str2int(data);
    long duration;
    if(pin <=0){
          pinMode(-pin, INPUT);
          duration = pulseIn(-pin, LOW);
    }else{
          pinMode(pin, INPUT);
          duration = pulseIn(pin, HIGH);
    }
    Serial.println(duration);
}

void pulseInSHandler(String data){
    int pin = Str2int(data);
    long duration;
    if(pin <=0){
          pinMode(-pin, OUTPUT);
          digitalWrite(-pin, HIGH);
          delayMicroseconds(2);
          digitalWrite(-pin, LOW);
          delayMicroseconds(5);
          digitalWrite(-pin, HIGH);
          pinMode(-pin, INPUT);
          duration = pulseIn(-pin, LOW);
    }else{
          pinMode(pin, OUTPUT);
          digitalWrite(pin, LOW);
          delayMicroseconds(2);
          digitalWrite(pin, HIGH);
          delayMicroseconds(5);
          digitalWrite(pin, LOW);
          pinMode(pin, INPUT);
          duration = pulseIn(pin, HIGH);
    }
    Serial.println(duration);
}

void SV_add(String data) {
    String sdata[3];
    split(sdata,3,data,'%');
    int pin = Str2int(sdata[0]);
    int min = Str2int(sdata[1]);
    int max = Str2int(sdata[2]);
    int pos = -1;
    for (int i = 0; i<8;i++) {
        if (servo_pins[i] == pin) { //reset in place
            servos[pos].detach();
            servos[pos].attach(pin, min, max);
            servo_pins[pos] = pin;
            Serial.println(pos);
            return;
            }
        }
    for (int i = 0; i<8;i++) {
        if (servo_pins[i] == 0) {pos = i;break;} // find spot in servo array
        }
    if (pos == -1) {;} //no array position available!
    else {
        servos[pos].attach(pin, min, max);
        servo_pins[pos] = pin;
        Serial.println(pos);
        }
}

void SV_remove(String data) {
    int pos = Str2int(data);
    servos[pos].detach();
    servo_pins[pos] = 0;
}

void SV_read(String data) {
    int pos = Str2int(data);
    int angle;
    angle = servos[pos].read();
    Serial.println(angle);
}

void SV_write(String data) {
    String sdata[2];
    split(sdata,2,data,'%');
    int pos = Str2int(sdata[0]);
    int angle = Str2int(sdata[1]);
    servos[pos].write(angle);
}

void SV_write_ms(String data) {
    String sdata[2];
    split(sdata,2,data,'%');
    int pos = Str2int(sdata[0]);
    int uS = Str2int(sdata[1]);
    servos[pos].writeMicroseconds(uS);
}

void sizeEEPROM() {
    Serial.println(E2END + 1);
}

void EEPROMHandler(int mode, String data) {
    String sdata[2];
    split(sdata, 2, data, '%');
    if (mode == 0) {
        EEPROM.write(Str2int(sdata[0]), Str2int(sdata[1]));
    } else {
        Serial.println(EEPROM.read(Str2int(sdata[0])));
    }
}

void SerialParser(void) {
  char readChar[64];
  Serial.readBytesUntil(33,readChar,64);
  String read_ = String(readChar);
  //Serial.println(readChar);
  int idx1 = read_.indexOf('%');
  int idx2 = read_.indexOf('$');
  // separate command from associated data
  String cmd = read_.substring(1,idx1);
  String data = read_.substring(idx1+1,idx2);

  // determine command sent
  if (cmd == "dw") {
      DigitalHandler(1, data);
  }
  else if (cmd == "dr") {
      DigitalHandler(0, data);
  }
  else if (cmd == "aw") {
      AnalogHandler(1, data);
  }
  else if (cmd == "ar") {
      AnalogHandler(0, data);
  }
  else if (cmd == "pm") {
      ConfigurePinHandler(data);
  }
  else if (cmd == "ps") {
      pulseInSHandler(data);
  }
  else if (cmd == "pi") {
      pulseInHandler(data);
  }
  else if (cmd == "ss") {
      SS_set(data);
  }
  else if (cmd == "sw") {
      SS_write(data);
  }
  else if (cmd == "sr") {
      SS_read(data);
  }
  else if (cmd == "sva") {
      SV_add(data);
  }
  else if (cmd == "svr") {
      SV_read(data);
  }
 else if (cmd == "svw") {
      SV_write(data);
  }
 else if (cmd == "svwm") {
      SV_write_ms(data);
  }
  else if (cmd == "svd") {
      SV_remove(data);
  }
  else if (cmd == "version") {
      Version();
  }
  else if (cmd == "to") {
      Tone(data);
  }
  else if (cmd == "ton") {
      NormalTone(data);
  }
  else if (cmd == "nto") {
      ToneNo(data);
  }
  else if (cmd == "lcdh"){
      LCDSet(data);
  }
  else if (cmd == "lcdb"){
      LCDBegin(data);
  }
  else if (cmd == "lcdcl"){
      LCDCleanup();
  }
  else if (cmd == "lcdc"){
      LCDClear();
  }
  else if (cmd == "lcdho"){
      LCDHome();
  }
  else if (cmd == "lcdp"){
      LCDPrint(data);
  }
  else if (cmd == "lcdch"){
      LCDCursorHandler(data);
  }
  else if (cmd == "lcdcc"){
      LCDCursor(1);
  }
  else if (cmd == "lcdco"){
      LCDCursor(0);
  }
  else if (cmd == "lcdbc"){
      LCDBlink(1);
  }
  else if (cmd == "lcdbo"){
      LCDBlink(0);
  }
  else if (cmd == "lcddc"){
      LCDDisplay(1);
  }
  else if (cmd == "lcddo"){
      LCDDisplay(0);
  }
  else if (cmd == "lcdsl"){
      LCDScroll(0);
  }
  else if (cmd == "lcdsr"){
      LCDScroll(1);
  }
  else if (cmd == "lcdsa"){
      LCDScroll(2);
  }
  else if (cmd == "lcdsb"){
      LCDScroll(3);
  }
  else if (cmd == "lcdwl"){
      LCDWritedir(1);
  }
  else if (cmd == "lcdwr"){
      LCDWritedir(0);
  }
  else if (cmd == "cap") {
      readCapacitivePin(data);
  }
  else if (cmd == "so") {
      shiftOutHandler(data);
  }
  else if (cmd == "si") {
      shiftInHandler(data);
  }
  else if (cmd == "eewr") {
      EEPROMHandler(0, data);
  }
  else if (cmd == "eer") {
      EEPROMHandler(1, data);
  }
  else if (cmd == "sz") {
      sizeEEPROM();
  }
}

void setup()  {
  Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
    }
  Serial.println("connected");
}

void loop() {
  SerialParser();
}
