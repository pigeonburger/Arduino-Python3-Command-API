# pyArduinoAPI

A Python API for communicating with your Arduino board.

A lightweight Python library for
communicating with [Arduino microcontroller boards](http://www.arduino.cc/) from a connected computer using
standard serial IO, either over a physical wire
or wirelessly. It is written using a custom protocol, similar to [Firmata](http://firmata.org/wiki/Main_Page).

This is my own fork of the [original](https://github.com/mkals/Arduino-Python3-Command-API) version of this repo. I've added some new commands to suit my needs and use cases for Arduino, and will continue adding my modifications here as I need them.

This library allows a user to quickly prototype programs for Arduino using Python code, or to
simply read/control/troubleshoot/experiment
with hardware connected to an Arduino board without ever having to recompile and reload sketches to the board itself.

Method names within the Arduino-Python3 Command API are designed to be as close
as possible to their Arduino programming language counterparts. This allows for Arduino code to quickly be transcribed into Python, or vice-versa.

## Requirements:
- [Python](http://python.org/) 3.7 or above (tested on Windows, Linux and macOS).
- [pyserial](http://pyserial.sourceforge.net/) 2.6 or higher (`pip install pyserial`)
- Any [Arduino compatible microcontroller](https://www.sparkfun.com/categories/242) with at least 14KB of flash memory


# Installation + Setup
1. To install, run `pip install pyArduinoAPI`.

2. Load the `prototype.ino` sketch onto your Arduino board, using the Arduino IDE.

3. Set up some kind of serial I/O communication between the Arduino board and your computer (via physical USB cable,
Bluetooth, xbee, etc. + associated drivers)

4. Add `from Arduino import Arduino` into your python script to communicate with your Arduino.
---

**What I've added so far that is different to the version I adapted this from:**
- Added support for `tone()` and `noTone()` operations.
- Added (almost) complete support for the [`LiquidCrystal`](https://www.arduino.cc/en/Reference/LiquidCrystal) library for controlling LCD screens (missing `write()` and `createChar()` functions)

# Examples
## Simple usage example (LED blink):
```python
from Arduino import Arduino
import time

board = Arduino()
board.pinMode(13, "OUTPUT")

while True:
    board.digitalWrite(13, "LOW")
    time.sleep(1)
    board.digitalWrite(13, "HIGH")
    time.sleep(1)
```

## Python adaptation of [Spaceship Interface](https://create.arduino.cc/projecthub/SBR/spaceship-interface-4e616d):
```python
from Arduino import Arduino
import time

board = Arduino()

switchState = 0

board.pinMode(3, "OUTPUT")
board.pinMode(4, "OUTPUT")
board.pinMode(5, "OUTPUT")
board.pinMode(2, "INPUT")

while True:
    switchState = board.digitalRead(2)

    if switchState == 0:
        board.digitalWrite(3, "HIGH")
        board.digitalWrite(4, "LOW")
        board.digitalWrite(5, "LOW")
    else:
        board.digitalWrite(3, "LOW")
        board.digitalWrite(4, "LOW")
        board.digitalWrite(5, "HIGH")

        time.sleep(0.25)
        board.digitalWrite(4, "HIGH")
        board.digitalWrite(5, "LOW")
        time.sleep(0.25)
```

## Simple Servo Motor Control on pin 9:
```python
from Arduino import Arduino
import time

board = Arduino()

board.Servos.attach(9)

while True:
    for pos in range(180):
        board.Servos.write(9, pos)
        time.sleep(0.015)

    for pos in range(180):
        pos = 180 - pos

        board.Servos.write(9, pos)
        time.sleep(0.015)
```

For a collection of examples, see `examples.py`. This file contains methods which replicate
the functionality of many Arduino demo sketches.

## Classes
- `Arduino(baud)` - Set up communication with currently connected and powered
Arduino.

```python
board = Arduino("115200") #Example
```

The device name / COM port of the connected Arduino will be auto-detected.
If there are more than one Arduino boards connected,
the desired COM port can be also be passed as an optional argument:

```python
board = Arduino("115200", port="COM3") #Windows example
```
```python
board = Arduino("115200", port="/dev/tty.usbmodemfa141") #OSX example
```

A time-out for reading from the Arduino can also be specified as an optional
argument:

```python
board = Arduino("115200", timeout=2) #Serial reading functions will
#wait for no more than 2 seconds
```

## Methods

**Digital I/O**

- `Arduino.digitalWrite(pin_number, state)` turn digital pin on/off
- `Arduino.digitalRead(pin_number)` read state of a digital pin

```python
#Digital read / write example
board.digitalWrite(13, "HIGH") #Set digital pin 13 voltage
state_1 = board.digitalRead(13) #Will return integer 1
board.digitalWrite(13, "LOW") #Set digital pin 13 voltage
state_2 = board.digitalRead(13) #Will return integer 0
```

- `Arduino.pinMode(pin_number, io_mode)` set pin I/O mode
- `Arduino.pulseIn(pin_number, state)` measures a pulse
- `Arduino.pulseIn_set(pin_number, state)` measures a pulse, with preconditioning

```python
#Digital mode / pulse example
board.pinMode(7, "INPUT") #Set digital pin 7 mode to INPUT
duration = board.pulseIn(7, "HIGH") #Return pulse width measurement on pin 7
```

**Analog I/O**

- `Arduino.analogRead(pin_number)` returns the analog value
- `Arduino.analogWrite(pin_number, value)` sets the analog value

```python
#Analog I/O examples
val=board.analogRead(5) #Read value on analog pin 5 (integer 0 to 1023)
val = val / 4 # scale to 0 - 255
board.analogWrite(11) #Set analog value (PWM) based on analog measurement
```

**Shift Register**

- `Arduino.shiftIn(dataPin, clockPin, bitOrder)` shift a byte in and returns it
- `Arduino.shiftOut(dataPin, clockPin, bitOrder, value)` shift the given byte out

`bitOrder` should be either `"MSBFIRST"` or `"LSBFIRST"`

**Servo Library Functionality**
Support is included for up to 8 servos.

- `Arduino.Servos.attach(pin, min=544, max=2400)` Create servo instance. Only 8 servos can be used at one time.
- `Arduino.Servos.read(pin)` Returns the angle of the servo attached to the specified pin
- `Arduino.Servos.write(pin, angle)` Move an attached servo on a pin to a specified angle
- `Arduino.Servos.writeMicroseconds(pin, uS)` Write a value in microseconds to the servo on a specified pin
- `Arduino.Servos.detach(pin)` Detaches the servo on the specified pin

```python
#Servo example
board.Servos.attach(9) #declare servo on pin 9
board.Servos.write(9, 0) #move servo on pin 9 to 0 degrees
print board.Servos.read(9) # should be 0
board.Servos.detach(9) #free pin 9
```

**Software Serial Functionality**

- `Arduino.SoftwareSerial.begin(ss_rxPin, ss_txPin, ss_device_baud)` initialize software serial device on
specified pins.
Only one software serial device can be used at a time. Existing software serial instance will
be overwritten by calling this method, both in Python and on the Arduino board.
- `Arduino.SoftwareSerial.write(data)` send data using the Arduino 'write' function to the existing software
serial connection.
- `Arduino.SoftwareSerial.read()` returns one byte from the existing software serial connection

```python
#Software serial example
board.SoftwareSerial.begin(0, 7, "19200") # Start software serial for transmit only (tx on pin 7)
board.SoftwareSerial.write(" test ") #Send some data
response_char = board.SoftwareSerial.read() #read response character
```

**EEPROM**

- `Arduino.EEPROM.read(address)` reads a byte from the EEPROM
- `Arduino.EEPROM.write(address, value)` writes a byte to the EEPROM
- `Arduino.EEPROM.size()` returns size of the EEPROM

```python
#EEPROM read and write examples
location = 42
value = 10 # 0-255(byte)

board.EEPROM.write(location, 10)
print(board.EEPROM.read(location))
print('EEPROM size {size}'.format(size=board.EEPROM.size()))
```

**LCD Screen LiquidCrystal Support Library (only supports 1 display currently)**

*See https://www.arduino.cc/en/Reference/LiquidCrystal for reference to the LiquidCrystal library.*

- `Arduino.LCD.LiquidCrystal(rs, en, d4, d5, d6, d7)` Creates an object of type LiquidCrystal.
- `Arduino.LCD.begin(cols, rows)` Initializes the interface to the LCD screen, and specifies the dimensions of the display.
- `Arduino.LCD.cleanup()` Deletes LCD object from Arduino system memory
- `Arduino.LCD.clear()` Clears the LCD screen and positions the cursor in the upper-left corner.
- `Arduino.LCD.home()` Positions the cursor in the upper-left corner of the LCD.
- `Arduino.LCD.print(text)` Prints text to the LCD screen.
- `Arduino.LCD.setCursor(col, row)` Sets the location at which subsequent text written to the LCD will be displayed.
- `Arduino.LCD.cursor()` Displays the LCD cursor.
- `Arduino.LCD.noCursor()` Hides the LCD cursor.
- `Arduino.LCD.blink()` Displays the blinking LCD cursor.
- `Arduino.LCD.noBlink()` Hides the blinking LCD cursor.
- `Arduino.LCD.display()` Turns the LCD display on.
- `Arduino.LCD.noDisplay()` Turns the LCD display off.
- `Arduino.LCD.scrollDisplayLeft()` Scrolls the contents of the display (text and cursor) one space to the left.
- `Arduino.LCD.scrollDisplayRight()` Scrolls the contents of the display (text and cursor) one space to the right.
- `Arduino.LCD.autoscroll()` Turns on automatic scrolling of the LCD.
- `Arduino.LCD.noAutoscroll()` Turns off automatic scrolling of the LCD.
- `Arduino.LCD.leftToRight()` Set the direction for text written to the LCD to left-to-right.
- `Arduino.LCD.rightToLeft()` Set the direction for text written to the LCD to right-to-left.

```python
#LCD "Hello World" and time since start example
#Ported from https://www.arduino.cc/en/Tutorial/LibraryExamples/HelloWorld
import time
start_time = time.time()

board.LCD.LiquidCrystal(12, 11, 5, 4, 3, 2)
board.LCD.begin(16, 2)
board.LCD.print("hello, world!")

while True:
    board.LCD.setCursor(0, 1)

    board.LCD.print(round(time.time() - start_time))
```


**Misc**

- `Arduino.close()` closes serial connection to the Arduino.

## To-do list:
- Add simple reset functionality that zeros out all pin values
- Include a wizard which generates 'prototype.ino' with selected serial baud rate and Arduino function support
(to help reduce memory requirements).
- Add `write()` and `createChar()` functions for the LCD library.
- Anything else that I come across that I need will be added here.
