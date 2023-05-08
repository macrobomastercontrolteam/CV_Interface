# CV_Interface
Communication API for CV to use

CV requirement for python version: 3.6.9

Different but equivalent bench setups
- TWO_ARDUINO_SETUP
    - Two arduinos connected to each other and also computer
    - Scanner_Arduino is the specialized code for scanner arduino
- ARDUINO_RPI_SETUP
    - RPi connected to Arduino
## Arduino Test Circuit Diagram
Button setup is the same as the one described in https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
Whenever the button is pressed, arduino will send a set-mode request.

## Jetson TX2 Circuit Diagram
Serial Port : /dev/ttyTHS2
![6634a59717a0defd36d039292c8d643](https://user-images.githubusercontent.com/56321690/236954118-339b6e05-28cb-4140-a3d6-6123518752b6.jpg)

![Arduino Test Circuit (Schematics)](https://github.com/macrobomastercontrolteam/CV_Interface/blob/main/Arduino_Test_Circuit.png)

![Arduino Test Circuit (Real)](https://user-images.githubusercontent.com/57267209/212556064-896bd52c-37dd-4e50-95ff-976f00145a35.jpg)
