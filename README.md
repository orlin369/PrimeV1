# Prime V1

# Educational Robot System

**1. Review**

This robot system is dedicated to focus on a few different aspects of education of robotics electronics and programming.
The system provides modular robotics hardware that can be separated on a modular education blocks. Robot system covers:

- Mechanical part that helps youngsters to understand how to build the robot and assemble the robotic parts together
- Modular electronics that is extendable of various parts.
- Modular software that is open source and easy to rewrite.

**2. Commands**


# Robot Commands

## Commands
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Reset the robot | AT+RESET | OK | Will turnoff all actuators. Reset all settings to default. Clear all previous presets.|
|Mode of the robot. |AT+MODE|OK|Returns the robot mode.|
|Preset the robot in specified mode.|AT+MODE <0 to N>|OK|Will preset the robot in specified mode.|
|Run the robot in following mode.|AT+RUN|OK|Will run the robot in the following mode.|
|Stop the robot.|AT+STOP|OK|Stop the robot if it is running in the some mode.|

# Sensors

## Line
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Calibrate the line sensor.|AT+LINE+CALIBRATE|OK (Note 2)|The sensor values will be calibrate by local minimum and maximum of the trace.|
|Get line sensor data.|AT+LINE|d0 d2 d3 ... dn (Note 1)|Will read the line sensor data.|
|Get line position.|AT+LINE+POS|<-1.0 to 1.0>|This will be possible only if the sensor is calibrated correctly.|

## Ultra Sonic
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Get ultra-sonic distance.|AT+GET+US <0 to 180>|<0 to 2000>|Distance to object in mm with specified angle of reading in deg.|


## Encoders
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Get encoders count since last robot turn on.|AT+GET+ENCODERS|<-n to N>,<-n to N>|Count of the encoders.|

# Actuators

## Motors

| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Get motor state.|AT+GET+MOTORS|<0 to 255> <0 to 255>|Gets the motors PWMs.|
|Set motor state.|AT+SET+MOTORS <0 to 255> <0 to 255>|OK|Sets the motors PWMs.|

# User

## Button
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Set the button mode.|AT+SET+BUTTON+MODE <0 to N>|OK|Set the button mode. (note 3)|
|Get the button state.|AT+GET+BUTTON|<0/1>|Gets the button state.|

## LED
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Set the LED mode.|AT+SET+LED+MODE <0 to N>|OK|Set the led mode.|
|Set the LED state.|AT+SET+LED <0/1>|OK|Turn ON or OFF the LED|

## Events
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|Enable events to popup in the console.|AT+SET+EVENTS <0 to N>|OK|Will print all events that accrued. Where "n" is level of events.|



**Example**
| Command | Syntax | Response | Result |
|---------|:-------|:---------|:-------|
|||||

Notes
1. Array with all values from the line sensor. The size of array depends on count of the array. It depends on firmware and hardware revision. The values will be from 0 to 1023 if the sensor is not calibrated and 0 to 100 if the sensor is calibrated.
2. The sensor is specially calibrated.
3. Describe buttons mode.
* **All commands are delimited by (space)**
* **All commands ends with \r\n.**
* **All responses are delimited by (space)**
* **All responses ends with \r\n.**
