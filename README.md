# 900 MHz transmitter and receiver with PPM for drop in replacement
![Transmitter and Receiver](/images/rc900.png)  
The receiver wires are the PPM, 5V, Gnd.  

## Intermittent PPM issue RESOLVED
Switched to using the hardware PWM capability on pin 5.  
The initial timer ISR was not working because the RF95 interrupts for reading data took too long and blocked the PPM ISR.  
This explains why I only saw the issue when sending wireless 900 MHz data.  

## Initial Intermittent PPM issue, invalid pulse length  
The transmitter board only sends radio packets for non-neutral joystick positions.  
This is currently for saving battery power and also helps with debugging.  
I have a working Sabertooth robot that currently uses a micro to receive a PPM signal from a 2.4 GHz receiver.  
When I try to swap in the 900 MHz receiver PPM signal, I get intermittent jolt responses in the left and right motors.  
NOTE, the jolts do NOT occur if I am not sending data (neutral joystick positions).  
To re-create the issue, I hold the left joystick to the right to continuously send data.  
We expect the 8 PPM channels to be 1500 1500 1500 2000 1500 1500 1500 1500 usec.  
For now, I believe I have correctly concluded that the issue is an interrmittent bad PPM signal.  
I connected a logic analyzer to the PPM input pin on the micro (vs the PPM output on this new receiver).  
This revealed that a 1500 usec pulse is sometimes actually 1950 usec.  
This is due to the low pulse being 850 usec when it should be a fixed 400 usec.  
Because this happens only when sending 900MHz data, I believe the antenna/wireless signal is causing timer errors on the receiver board.  
I receive the PPM signal using this library: [https://github.com/Nikkilae/PPM-reader](https://github.com/Nikkilae/PPM-reader)  

![850 usec low pulse should be 400 usec](/images/bad_pulse_length1.png)  
![850 usec low pulse should be 400 usec](/images/bad_pulse_length2.png)  
