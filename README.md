# LaserPiano

## Overview
  The goal of the project is to build an one octave piano. Instead pressing the keys, sound plays when pianist breaks the laser beam.
## Description
Schematic:

<a href="https://ibb.co/Mpd1vjv"><img src="https://i.ibb.co/5vZj0z0/schemeit-project-1.png" alt="schemeit-project-1" border="0" height="600"></a>

Project contains interruption service on pins PE0-PE7. Sound is generated using CS43L22 module. Every note is a composition of three sine waves. Their frequencies referens to their first harmonics.
## Tools
STM32F4407VG Discovery Microcontroller
8 Laser diodes
8 Fototransistors
Cables
Wires

## How to run
Connect all required tools as table below says:

| PIN  | TOOL |
| ------------- | ------------- |
| PE3 | Fototransistor GND |
| PE4 | Fototransistor GND |
| PE5 | Fototransistor GND |
| PE6 | Fototransistor GND |
| PD8 | Fototransistor GND |
| PD9 | Fototransistor GND |
| PD10 | Fototransistor GND |
| PD11 | Fototransistor GND |
| VCC 3V | Fototransistors VCC |

It is also needed to power up the laser diodes with an external power supply.

## How to Compile
Run program in System Workbench Developers C/C++ for STM32.
## Future improvements
Making more compact box.
Adding complex sounds.
Developing the number of notes.
## Attributions
CS43L22 module library - [CS43L22 Tutorial](https://www.youtube.com/watch?v=QIPQOnVablY)
## Licence
Our project is based on [MIT](https://opensource.org/licenses/mit-license.php) license.
## Credits
The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.
Supervisor: Tomasz Mańkowski
