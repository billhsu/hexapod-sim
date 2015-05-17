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
{servoId} is from 1 to 9 and from 32 to 24, {PWM} is from 0 to 3000.

You can config the servo mapping in Hexapod/config.txt.

Sample
```
#1P1500#2P3000
```

# How to run multiple commands
Save the commands in a file(e.g. testData.txt)
```
#1P1500#2P2000#3P1277#29P1500#28P1000#27P1722#7P1500#8P2000#9P1277#32P1833#31P1611#30P1277#4P1166#5P1388#6P1722#26P1833#25P1611#24P1277T400\r\n
#1P1500#2P1388#3P1722#29P1500#28P1611#27P1277#7P1500#8P1388#9P1722#32P1833#31P1000#30P1722#4P1166#5P2000#6P1277#26P1833#25P1000#24P1722T400\r\n
#1P1166#2P1388#3P1722#29P1833#28P1611#27P1277#7P1166#8P1388#9P1722#32P1500#31P1000#30P1722#4P1500#5P2000#6P1277#26P1500#25P1000#24P1722T400\r\n

```

Then run the following command
```
cd Hexapod
./run_command.rb testData.txt
```
