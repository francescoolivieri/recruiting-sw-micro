# recruiting-sw-microcontrollers

#### 🚧STILL WORK IN PROGRESS🚧

Implementation of a Finite State Machine which monitors three inputs (temperature sensor, system voltage and a button) and acts accordingly to those values (turning on LEDs or sending messages via UART).

## FSM STATES
![FSM](https://user-images.githubusercontent.com/113623927/197071812-668e9959-a0ae-4777-900b-ca0018745cb4.png)

## STATE RUNNING
When the system is in **STATE_RUNNING** it:
  - reads from the analog sensor (temperature sensor) every 200ms (TIM10) 
  - checks the system voltage (potentiometer) every 350ms (TIM11), if this value overcome certains limits the state is changed to **STATE_DANGER**

At each reading of an input, the machine prints on UART the value and the timing (using System Tick timer).  
In fact this state represent a "wellness state": the system is running while having no troubles.

##### 😴 SLEEP MODE 😴
Note that if we want our stm32 to enter sleep mode while doing nothing, System Tick will be disabled and can no longer be used for timing. Instead, we could use RTC (Real Time Clock). 

## STATE DANGER
We enter in this state when the system voltage read by our Nucleo board exceeds the set limits:
  - system_voltage < 1.8 -> case of UNDERVOLTAGE 
  - system_voltage > 2.7 -> case of OVERVOLTAGE

After the detection of an "anomaly", a LED is turned on (red for undevoltage and yellow for overvoltage) and a message in send via UART saying what is the problem. 
The machine is still acquiring the system voltage every 350ms, if the voltage level returned in the "safe zone" the state will be switched to **STATE_RUNNING** otherwise we remain in **STATE_DANGER**.


## STATE WAITING
This state is enabled when the system detects that the button is pressed. This event put the machine in **STATE_WAITING**, turn off all LED and while in this state it performs a single task: print on UART the message "Board in waiting state - please press the emergency button" every 500ms.
This happens until the button is pressed again, at this point the state of the machine is switched to **STATE_RUNNING** where we restart monitoring our devices.
