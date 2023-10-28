# DriveControl

This is a Graphical Software used to control a Motor for postion or speed control, depending on selection
It is recommended to use the L298N H-Bridge to control the Motor if the Motor is Used in Fwd and Rev mode. for this the Code needs to be adapted to output on the Analog Pins found in the define.h
## Connecting the STM32:
    
    Connect the L298N Bridge:
    HBridge  | NUCLEO (CN7)
        ENA -> PD13
        IN1 -> PB2
        IN2 -> PB6
    Connect the AS5600:
    HBridge  | NUCLEO (CN7)
        SCL -> 
        SDA -> 

## Getting started:
To simply work with the software:
    
    >cd visu/
    \visu>python3 main.py

To *view* the Documentation in order to work with the Software open the index.html file under:
    
    docs/_build/html/index.html

## Building the Docu

### 1. Install all the dependencies:in the command prompt:
    pip install -r requirements.txt
### 2. install doxygen and sphinx

#### https://www.doxygen.nl/manual/install.html
check if doxygen has been properly installed:

    >doxy
    error: Doxyfile not found and no input file specified!
    Doxygen version 1.9.5 (2f6875a5ca481a69a6f32650c77a667f87d25e88)
    Copyright Dimitri van Heesch 1997-2021
if doxygen is installed, some error should appear, that only states that there is no doxyfile in this Folder.
In order to be able to compile the File, firstly
install Sphinx:

    pip install -U sphinx
### 3. Building 
in the command prompt enter the docs folder:
    > cd docs/
and insert these commands to build the docu.

    ..\docs>.\/make htmldoxyge+n
    ..\docs>.\/make htmlmake html

### 4. Open the file in *docs/_build/html/index.html* in your Browser
    If the documentation was build successfully, the File is found under docs/_build/html/index.html.
    The Documentation explains the SOftware and all Variables
