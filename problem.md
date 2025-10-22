1. analog = measure/generate value

2\. digital = on \& off/ 0s \& 1s

3\. relay model(SRD-05VDC-SL-C module) = allows low power signal from arduino

4\. led-switch = 0 or 1 on tilt

5\. lcd (2PCS IIC/IC2 Interface LCD1602 LCD Display Adapter Board Plate Module    5V Monitor Current Panel Supply) = display status depending on the readings form sensors (a)KY-017 (B)soil moisture sensor

6\. soil moisture sensor = get readings of amount of moisture in the soil

7\. water pump (Micro Submersible Water Pump DC 3V-5V) = water the plant

8\. using USAIT communication and interrupts

9\. Arduino(atmega328)





### problem(smart plant monitoring)

water sensor is fixed in the soil sends real time moisture levels to the Arduino .which also print it on lcd. when the moisture level go below 10 yellow led lights then a word "moisture is low". below 10 red lights and word

"water the plant" is printed on the lcd. when the level reaches 4 one is send to relay model hence putting on water pump and led ligth green led. the tilt switch is fixed on the leaves to detect the movement of the leaves if it puts on and off 10 times flug an interrupt hence a word "wind available" is shown on the lcd

