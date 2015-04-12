# Hexapod Simulator
A simulator for Hexapods.  
[Demo video](https://www.youtube.com/watch?v=JohDGAX7GWw)


## How to build

```
./configure.sh
cd Hexapod
make
```

## How to run
```
cd Hexapod
./Hexapod
```
# How to control the servos
```
telnet localhost 5555
#{servoId}P{PWM}  
```
{servoId} is from 1 to 18, {PWM} is from 0 to 3000.

Sample
```
#1P1500#2P3000
```
