Project Description
===================
his is a Graphical Software used to control a Motor for postion or speed control, depending on selection
It is recommended to use the L298N H-Bridge to control the Motor if the Motor is Used in Fwd and Rev mode. for this the Code needs to be adapted to output on Pin B


Getting started - Working with the Software
------------

Connect the Motor dependently on the Motor type used for the Testbench. The software was tested using a L298N H-Bridge
Pinout is found in Datashets/stm-pinout
#### For now the Motor out is set up to be B6 for ENA and B2/D13 for IN1/IN2, these define the direction
#### A AS5600 I2C Encoder is used on I2C1, for this the SCL and SDA pins PB8/PB9 are used 
these can be modified in the constants.h and needs to be uploaded to the stm
If a normal rotatory encoder is used, the code needs to be adapted. There is already an encoder class, that could handle normal rotatory encoders, for this Project a AS5600 was selected for efficency.
Depending on the Encoder Resolution, the Encoder-Resolution constant in the Same file need to be adapted depending on the Encoder used

The Main Software runs with Python 3.8
Run the Python Software and connect the Motor accordingly.

The Software is found under
Core-visu-main.Py
 

Getting started - Editing the Embedded Software
------------

Be sure to have the necessary files installed. 
The Software for the Motor controller and Embedded System runs in CPP
MinGW is needed for Windows.
For the VCP communication and Upload it is recommended to use OpenOCD


The Software for the Embedded System is found in
Core/Inc for the headers and Core/Src for the .cpp files
