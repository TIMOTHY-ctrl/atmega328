![Screenshot 2025-10-11 145524](https://github.com/user-attachments/assets/a7749750-eb30-4479-bda8-4f195800c6ef)
![Screenshot 2025-10-11 145719](https://github.com/user-attachments/assets/aa6c583c-f686-4816-9b92-b8e92cd909dd)

**ky-017**

![WhatsApp Image 2025-10-08 at 21 24 08_57d78a6e](https://github.com/user-attachments/assets/277e9fd3-8754-4235-823f-cc4fecac1c7b)

***KY-020***

![WhatsApp Image 2025-10-08 at 21 24 09_45275609](https://github.com/user-attachments/assets/e953e49d-0c62-47ba-8f76-a47991520bd9)


***https://www.youtube.com/watch?v=0V7PkoYPm8w***


***https://www.youtube.com/watch?v=xkNt-Av0KBk***


https://www.youtube.com/watch?v=ciD3ILxgXzU


understand the soil moisture sensor  https://projecthub.arduino.cc/Aswinth/soil-moisture-sensor-with-arduino-91c818?_gl=1*chy7td*_up*MQ..*_ga*MjEwNDQ2NTM2LjE3NjA4ODk4NDU.*_ga_NEXN8H46L5*czE3NjA4ODk4NDMkbzEkZzAkdDE3NjA4ODk4NDMkajYwJGwwJGg2MDA4NzcyMDc.


Digital Pins
What they do: Digital pins are designed to read or write only two distinct voltage levels.

LOW: 0 Volts (or close to 0V, typically < 1.5V)

HIGH: 5 Volts (or 3.3V on some boards like the Arduino Due or ESP32, typically > 3.0V)

How you use them:

As an INPUT: To detect the state of a component.

Example: Reading if a button is pressed (HIGH) or not pressed (LOW). Detecting if a motion sensor has been triggered.

As an OUTPUT: To control the state of a component.

Example: Turning an LED ON (HIGH) or OFF (LOW). Controlling a relay to switch a high-power device.

Key Limitation: They only understand ON or OFF. They cannot perceive "how much."

Common Functions:

pinMode(pin, INPUT/OUTPUT) - Sets the pin direction.

digitalRead(pin) - Reads the state (returns HIGH or LOW).

digitalWrite(pin, HIGH/LOW) - Sets the state.

Analog Pins
What they do: Analog pins are designed to read or generate a range of values, not just two states.

1. Analog INPUT (Reading a Range)
This is the most common use. Arduino has a built-in Analog-to-Digital Converter (ADC) that measures a voltage and converts it to a number.

Voltage Range: It can measure a voltage between 0V and the board's operating voltage (usually 5V).

Resolution: The Arduino Uno (and similar) has a 10-bit ADC. This means it divides this voltage range into 2¹⁰ = 1024 discrete steps.

0 represents 0V.

1023 represents 5V.

512 represents approximately 2.5V.

Example: You connect a potentiometer (a variable resistor) to an analog pin. As you turn the knob, the voltage on the pin changes smoothly from 0V to 5V. The analogRead() function will return a value from 0 to 1023, giving you a precise reading of the knob's position.

Common Function:

analogRead(pin) - Reads the voltage (returns a value from 0 to 1023).

2. Analog OUTPUT (Generating a Range)
Most Arduino pins cannot output a true analog voltage. Instead, they use a clever trick called Pulse-Width Modulation (PWM) to simulate an analog output.

PWM Pins: On an Arduino Uno, these are the pins marked with a tilde (~): 3, 5, 6, 9, 10, 11.

How it works: The pin switches very rapidly between HIGH and LOW. The proportion of time the signal is HIGH (the "duty cycle") determines the average voltage. A 50% duty cycle results in an average of 2.5V.

Resolution: The analogWrite() function uses an 8-bit value, meaning it has a range from 0 to 255.

analogWrite(0) = 0% duty cycle = 0V average.

analogWrite(127) = ~50% duty cycle = ~2.5V average.

analogWrite(255) = 100% duty cycle = 5V average.

Example: Controlling the brightness of an LED. Instead of just ON/OFF, you can set it to be dim, medium, or full brightness.

Common Function:

analogWrite(pin, value) - Writes a PWM signal (value from 0 to 255).
