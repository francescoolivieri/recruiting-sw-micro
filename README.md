# recruiting-sw-microcontrollers

Implementation of a Finite State Machine which monitors three inputs (temperature sensor, system voltage and a button) and acts accordingly to those values (turning on LEDs or sending messages via UART).

# 🚧STILL WORK IN PROGRESS🚧

## FSM STATES

## STATE RUNNING
When the system is in *STATE_RUNNING* it:
  - reads from the analog sensor (temperature sensor) every 200ms (TIM10) 
  - checks the system voltage (potentiometer) every 350ms (TIM11)
In fact this state represent a "wellness state": the system is running while having no troubles.

## STATE DANGER
We enter in this state when the system voltage read by our Nucleo board exceeds the set limits:
  - system_voltage < 1.8 -> case of UNDERVOLTAGE 
  - system_voltage > 2.7 -> case of OVERVOLTAGE
After the detection of an "anomaly", a LED is turned on (red for undevoltage and yellow for overvoltage) and a message in send via UART saying what is the problem.
The machine is still acquiring the system voltage every 350ms, if the voltage level returned in the "safe zone" the state will be switched to *STATE_RUNNING* otherwise we remain in *STATE_DANGER*.


## STATE WAITING
This state is enabled when the system detects that the button is pressed. This event put the machine in *STATE_WAITING*, turn off all LED and while in this state it performs a single task: print on UART the message "Board in waiting state - please press the emergency button" every 500ms.
This happens until the button is pressed again, at this point the state of the machine is switched to *STATE_RUNNING* where we restart monitoring our devices.
