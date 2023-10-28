===========
Structure
===========


STM32 Standalone Version 
===========================
Paremeters are currently hardcoded in "/software/microcontroller/define.h"
sensor data is used in STM32 to calculate steps with fixed gains

System Loop 
----------------------
- Send gains ans setpoint to STM32
- Encoder read for current position
- PID control for H-Bridge controller
- apply new dCurrenPosition to controler
- send current values 

Flowcharts
----------------------



