# Alticraft Flight Data Recorder
Abstract from SRS

## Product scope

Alticraft Flight Data Recorder software is a tool that controls the launch and staging of a rocket. Furthermore, the software captures and saves data about the projectile’s movement and environmental data for later research and other analysis.
It does this by interfacing with various real-time sensors and saves their data to an external drive. Stage control is achieved by using an ignition coil and a servo motor.

## Product Perspective

Alticraft Flight Data Recorder was developed for everyone interested in analyzing moving objects throughout their complete motion and for those who want to add basic flight controlling features to their projectiles.
It was developed to run on Arduino/Geniuno compatible boards but optimized for the Arduino Nano.

## Operating Enviroment

* Arduino Nano
* Arduino Uno
* Arduino Due
* Arduino Mega

## System Features

Alticraft Flight Data Recorder has three main operational modes. The desired mode must be selected in the source code’s main file under program preferences.

The three core operation modes:
* PASSIVE_LOG: Only log flight data. Disables launch and staging control.
* ONLY_LAUNCH_AND_LOG: Enables only launch control and flight data logging.
* AUNCH_LOG_STAGE: Enables all features including launch, staging control and flight data logging.

If a mode is selected, which supports staging, a trigger must be selected too. This can be done in the source code’s main file under the operation mode selector. There are two trigger modes.

The three staging trigger modes:
* PRESSURE: Activates staging at a specific altitude (m), derived from the barometric pressure readings.
* ALTITUDE: Activates staging at a specific pressure (kPa).

#### Accelerometer and gyroscope data measurements

##### Description and Priority

Read data about the environment from the BME280 and save it to a file, including barometric pressure, temperature and humidity.

##### Stimulus/Response Sequences

The reading and saving process happens on a set time interval determined by the specific application of the system and the available hardware specifications.

##### Functional Requirements

Data should be written to the file in a manor that makes post flight analysis as simple as possible (easily importable into Microsoft Access® and Microsoft Excel®).

#### Environmental data measurements

##### Description and Priority

Read data about the environment from the BME280 and save it to a file, including barometric pressure, temperature and humidity.

##### Stimulus/Response Sequences

The reading and saving process happens on a set time interval determined by the specific application of the system and the available hardware specifications.

##### Functional Requirements

Data should be written to the file in a manor that makes post flight analysis as simple as possible (easily importable into Microsoft Access® and Microsoft Excel®).

#### Basic flight process management

##### Description and Priority

Manage basic flight processes by controlling a digital launch signal and by controlling the position of a staging servo motor. Activate a warning buzzer before sending the launch signal.

##### Stimulus/Response Sequences

The launch signal is activated by a push button, while operation of the staging motor is triggered by reaching the apoapsis height.

##### Functional Requirements

A push button must be installed, and the barometric pressure sensor must be operating nominally.

## More information

For more information please read the software requirements specification document.

## Contact

If you have any questions or suggestions, don't hesitate to contact me at rohu747@gmail.com
