# Reaction Wheel Backpack Project

## Project progress:
Small-scale prototype testing. Code not completed.

## TODOs:
- Reverse motor function
- Tune PID gains
- Adjust trigger timing
- Update credits

## What is this?
It is a project to create **backpack-styled assistive device** for people with **defective balancing abilities**,
 using **reaction wheels** to prevent them from fall injuries.


## How does it work?
The project uses a **motion sensor** to sense the body position and when senses a fall 
it will activate the reaction wheels to provide counter acting force and stop the fall (or lessen the impact by 
reducing the speed).

The project incorperates **PID controllers** to calculate throttle values for the motors and 
uses off-market **ESCs** to drive **BLDC motors** which are commonly used for RC aircrafts. The 
reaction wheels are attached to the motors. 

Basically follows the sequence:   
*motion sensor --> PID controller --> ESC --> Motor --> Reaction wheel --> motion of the person*


## Parts employed
Currently used parts:
- Arduino Mini
- MPU6050 module
- 2204 BLDC motors*2
- Blheli_S ESC*2
- 3D printed base
- 3D printed reaction wheels
- Wires

## Credits:
- to be done.