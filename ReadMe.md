# The Other Things

*The Other Things* is a collection of experiments in code for Arduino, Adafruit Feather, and other nrf52840 based boards to explore how sensor data od the body can be captured usefully for creative prortotyping.
These explroations are part of research activities investigating wearable Internet of Things and the Self within [OpenDoTT](https://opendott.org.).

# Repository Contents

* **/Arduio** - C/C++ code for Arduino and Adafruit Feather boards
* **/Circuit Pyhton** - Prototypes written in Circuit Python for the Adafruit Feather Sense board enabling quicker iterations between code and testting.
* **/tools/** - Contains custom tools and utility scripts for data handling and analysis.

# Notes

To enable Circuit Python on the Adafruit nrf52840 Feather Sense, follow this guide: https://learn.adafruit.com/adafruit-feather-sense/circuitpython-on-feather-sense

Exmaple code for the *Adafruit Feather nRF52840 Sense* can found / is copied form here: https://learn.adafruit.com/adafruit-feather-sense

To test the *Adafruit Feather nRF52840 Sense* board this example sketch from the Adafruit tutorial: https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/main/Adafruit_Feather_Sense/feather_sense_sensor_demo/feather_sense_sensor_demo.ino

Some sketches are using an RTC clock. The module mus be initial set to a spcific date and time; also repeat after every battery change. The **/tools** folder contains the ubiquitous `RTCSetClock` sketch which can be found on many forums and tutorials. However, there is no URL where the code is hosted. Hence it is included in this repository with its respective copyright and license in the header of the file.

# Licenses

Source code written by the researcher in this repository is licensed under the Gnu Public License (GPL).

External examples or used libraries are covered under their own respective licenses; please the folders or follow up through the respective source code files.

Any other original content is licensed under a [Creative Commons Attribution-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-sa/4.0/).

[![Creative Commons Licence](https://i.creativecommons.org/l/by-sa/4.0/88x31.png)](http://creativecommons.org/licenses/by-sa/4.0/)


# Acknowledgments

🇪🇺 This project is part of [Open Design of Trusted Things (OpenDoTT) doctoral training network](https://opendott.org.) and has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 813508.