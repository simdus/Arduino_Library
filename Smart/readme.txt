This is currently a work in progress.

Currently it is possible to communicate with a smartcard directly connected to a Arduino or via a TDA8024T (or compatible chips. e.g. DS8024).

All tests have been done with a "Smart Card Reader" removed from an old settop box (PACE 210KP) and with a smart card slot connected directly to the Aruino board. 

Currently the SCLib uses TIMER1A(OC1A) to generate the needed CLK signal for the smart card. When using an Arduino UNO the CLK signal is found on digital pin 9, Arduino Mega based boards 
provide this signal on digital pin 11.

Please check your board documentation to find out the correct pin for the CLK signal.

Working:
- Activation of asynchronous and synchronous smart cards 
- Exchanging data with supported Smart card (Sending / Receiving)
- SLE4432 / SLE4442 / SLE4441 / SLE4440 and compatible support class for easy communication and integration in own projects.

Planned:
- Configuration based on ATR data
- Make library more portable.
