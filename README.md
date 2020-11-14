# Alticraft Flight Data Recorder
Abstract from SRS

## Product scope

Alticraft Flight Data Recorder software is a tool that controls the launch and staging of a rocket or other projectile. Furthermore, the software captures and saves data about the projectile’s movement and environmental data for later research and other analysis. It does this by interfacing with various real-time sensors and saves their data to an external drive.

Staging can be controlled with a servo motor, which is activated in a specified manner when the given trigger criteria is reached.

Alticraft Flight Data Recorder software is part of the Smart Crew Alticraft Flight Data recorder product and should only be used alongside specified and tested Alticraft FDR hardware.

## Product Perspective

Alticraft Flight Data Recorder was developed for everyone interested in analyzing moving objects throughout their complete motion and for those who want to add basic flight controlling features to their airborne systems.

Alticraft Flight Data Recorder software is designed to operate on Arduino Nano V3 compatible 5V microcontrollers. The software is only tested to operate on such hardware (see performance requirements).

## Operating Enviroment

* Arduino Nano V3 compatible boards which meet memory requirements and operate on 5V

Please note, Alticraft Flight Data Recorder software is designed to run on Arduino Nano V3 compatible boards. Although it may run on other devices, it has only been tested to function properly on Arduino Nano V3 compatible microcontrollers

## System Features

Alticraft Flight Data Recorder has three main operational modes. The desired mode must be selected in the source code’s main file under program preferences.

The three core operation modes:
* PASSIVE_LOG: Only log flight data. Disables launch and staging control.
* ONLY_LAUNCH_AND_LOG: Enables only launch control and flight data logging.
* LAUNCH_LOG_STAGE: Enables all features including launch, staging control and flight data logging.

If a mode is selected which supports staging, a staging trigger must be selected too. This can be done in the source code’s main file under the operation mode selector. There are four trigger modes. All trigger modes operate completely independently from each other, which means multiple triggers can be activated at the same time.

The three staging trigger modes:
* ALTITUDE: Activates staging at a specific altitude (m), derived from the barometric pressure readings.
* PRESSURE: Activates staging at a specific pressure (Pa).
* TEMPERATURE: Activates staging when the measured temperature leaves a given range (°C).
* APOAPSIS: Activates staging when a constant decline in altitude is detected.

Additional features and modes
* LAUNCH_STABILITY_ABORT:This feature monitors accelerometer and gyroscope data for anomalies during the launch countdown and aborts launch if a change in attitude is detected.
* TOUCHDOWN_DETECTION:This feature watches sensor data and attempts to detect touchdown. If this is detected, recording is finished, and the system lingers in low power mode until it is shut down using the push button.
* SAR_HELPER: This activates the buzzer upon touchdown to help direct search and rescue teams towards the fallen rocket. Note: SAR_HELPER will only work if TOUCHDOWN_DETECTION is also activated.

#### Accelerometer and gyroscope data measurements

##### Description and Priority

Read accelerometer, gyroscope, and compass data from the MPU9250 and save it to a file.

##### Stimulus/Response Sequences

The reading and saving process happens on a set time interval determined by the specific application of the system and the available hardware specifications.

##### Functional Requirements

Data should be written to the file in a manor that makes post flight analysis as simple as possible (easily importable into Microsoft Access® and Microsoft Excel®).

#### Environmental data measurements

##### Description and Priority

Read data about the environment from the BME280 and save it to a file, including barometric pressure, temperature and humidity.

#### Basic flight process management

##### Description and Priority

Manage basic flight processes by controlling a digital launch signal and by controlling the position of a staging servo motor. Activate a warning buzzer before sending the launch signal.

##### Stimulus/Response Sequences

The launch signal is activated by a push button, while operation of the staging motor is triggered by reaching a specified launch trigger altitude/temperature/pressure or apoapsis height.

Trigger criteria:
* Pressure:	Trigger condition is met, when the ambient pressure is LOWER than the trigger pressure set (pressure decreases upwards).
* Temperature: 	Trigger condition is met when the ambient temperature leaves a given range (bound by min and max). The range can be set with two temperature values. (TRIGGER_TEMPERATURE_MIN and TRIGGER_TEMPERATURE_MAX) To invert the trigger range, enter the max temperature value as the min one, and the min one as the max temperature value.
* Altitude:	Trigger condition is met when the measured altitude is HIGHER than the trigger altitude set.
* Apoapsis: Trigger condition is met, when the measured altitude is constantly decreasing for given time set in the STAGE_TRIGGER_TIME option.

##### Functional Requirements

A push button must be installed, and the barometric pressure sensor must be operating nominally.

## More information

For more information please read the software requirements specification document.

## Contact

If you have any questions or suggestions, don't hesitate to contact me at rohu747@gmail.com
