# scanse_sweep_lidar_mcu
implementation of stm32 based code for getting data from SCANSE sweep lidar.

The setup sees the lidar sensor connected to the microcontroller, and the latter connected
to the Raspberry Pi.

This implementation only considers these 3 devices (MCU, lidar, RPi) even though the
application of this would need a connection to other sensor (IMU, indoor GPS, wheel encoders, ...).


## Features
This implementation uses the following:
- UART, SCANSE SWEEP LIDAR - baud rate (115200 bit/s) - not changeable
- UART, debug - baud rate (115200 bit/s) \[can be removed\]
- I2C, fast mode (400 kHz) - slave mode
- freeRTOS

#### UART
the scanse lidar is connected using it's connector with loose wires on one hand.
These wires are connected to the microcontroller as per the table below description.

color | what it is | description
--- | --- | ---
red | power | this MUST be (5 +-0.5)V!
black | ground | same ground as microcontroller
orange | enable | if pulled HIGH will enable the motor to spin and to get measurements. if set LOW will stop the motor. A sequence of LOW and HIGH will reset the sensor.
yellow | synch/device ready | not used in this implementation
green | UART RX | sensor RX line for serial
blue | UART TX | sensor TX line for serial

If the enable line is pulled HIGH, the motor will start to spin. Until the spin matches the goal speed,
the sensor will not start acquiring measurements, even if requested to do so.

To start the acquisition, the command `DS\n` must be sent and the reply `DS00` confirms it.

After, the sensor by default will send each measurement as soon as it is available. This means
that there is no way to know when a measurement is coming. An IT - interrupt - is set up to read,
byte by byte, every time the interrupt is triggered, and each byte is saved in a circular buffer.

The default values of motor spin (5Hz) and measurements sample rate (500Hz) is used.

#### UART debug
for debugging purposes, a second uart is being used but can be removed.

#### I2C
The I2C implementation for this project is quite messed up.. unfortunately we cannot set the
microcontroller as master, as the raspberry pi cannot be slave.

We need to set this I2C instance as slave (slave address 0x04) with IT - interrupt - set up.
The I2C interrupt is not as simple as the uart interrupt to set up, mainly due to the many
operations the I2C allows to do.

This implementation expects the I2C master to **_only_** request data, thus a read-only I2C operation.
Any other operation will fail, meaning:
- no effect involved, e.g. if try to write something, nothing happens;
- may break the software in the MCU, thus needing a reset after.

#### freeRTOS
freeRTOS version provided by StLink and included when using the St CubeMx Application (described below).

Only 1 task is set up and running at 50Hz.

This task goes 10 times faster as the default motor spin rate (5Hz). This makes sure that
we start processing data in the circular buffer fast enough to allow using a small enough buffer
for the circular buffer itself.

At each iteration, the task will look for available packets from the sensor (7 bytes total),
decipher the information and put the into the i2c output buffer at the correct position.

If a synch packet is found, the I2C output buffer is copied to the I2C interface. The next
time the I2C master is reading from this microcontroller, that buffer will be sent.




## Requirements
1. STM32Cube MX (st microcontroller initialization code generator)
2. System Workbench (based on Eclipse)
3. ST-Link v2 in-circuit debugger and programmer
4. Raspberry Pi unit (tested with Raspberry Pi Zero)
5. Scanse Sweep Lidar sensor (can be acquired from SparkFun)


### Installation
Here the steps to easily set up the prerequisites stated above.

#### STM32Cube MX
By default is possible to install the package for Windows.
Can be downloaded from st.com at the following address:
http://www.st.com/en/development-tools/stm32cubemx.html

the zip folder downloaded will include an .exe installer and also an installer for mac and linux.
##### Windows
Just run the installer, it works.

##### Mac
At the current time the mac installer seems to not work, but there is a way around to
make the installation work on Mac as well.

Note: STM32CubeMX is purely java code, thus a Java JDK must be installed
prior to installation.

The way around to install on Mac is as follow:
- after installing java open a terminal
- unzip downloaded folder
- type: ```sudo /Library/Internet\ Plug-Ins/JavaAppletPlugin.plugin/Contents/Home/bin/java -jar /Users/<<$username>>/Downloads/en.stm32cubemx/SetupSTM32CubeMX-4.23.0.exe```
- follow installer GUI

Now the application is installed and can be launch as any others.

#### System Workbench
To avoid have any problems in having to separately install the corret toolchain for ARM processors or other shit,
it is very convenient to use a IDE which already includes it and that is specifically made by a STM32 community.

http://openstm32.org/ is a website that provides such platform, along with a forum to discuss any problems related
to the platform itself, or how to use it when program the ST mcus.

Their platform - System Workbench - is based on Eclipse, and is disributed for Windows, Mac and Linux OS.

In order to download the System Workbench, it is first needed to create an account on the website.
There are dedicated downloads for Windows, Mac and Linux.

##### Windows
The installation is straightforward on Windows, by running the installer.

##### Mac
On Mac is less obvious, as the installer has a *.run extension.
The installation can be however easily run with the command:
```
/bin/bash install_sw4stm32.run
```
This will launch the GUI for the installation.

Please note that at any time the password may be requested on the command line,
so if the GUI is not moving forward anymore, just go back to command line, and input the password when requested.

Note that for Mac, this installer will also provide the drivers for the ST-Link v2 debugger, which is not obtainable direactly from the ST website.


### RPi I2C configuration
This implementation expects the RPi running Raspian OS.

Access the RPi either using  monitor and keyboard, or by ssh into it (USB cable or wireless connectivity).

#### Enabling I2C
On a terminal type:
```
sudo raspi-config
```
this will show an interactive interface (yes, on a terminal) for all the configuration tools available on the RPi.
- Choose option 5 - Interfacing Options
- Choose option P5 - I2C
- Confirm you want to enable the I2C
- Press tap to exit the interactive interface
- type `sudo reboot` to make sure the changes are properly enabled in the system.

#### Installing I2C tools
On a terminal type the following to make sure the system is up-to-date. Note this may take considerable time if never done recently.
```
sudo apt update
sudo apt upgrade -y
```

Once the system is up-to-date, start the installation with the tools to
communicate with i2c devices in Python (python-smbus) and diagnostic tools (i2c-tools).
```
sudo apt-get install -y python-smbus i2c-tools
```

#### Changing I2C Baudrate (Clock frequency)
The default clock frequency of the RPi Zero and other RPi products needs is by default 100kHz.
It needs to be adjusted to match the clock frequency expected on the microcontroller side, which in this case is set
to 400kHz.

To my knowledge there is no way to set this in a software packet while in operation. It must be set on the OS level.
I found the following way to work.

Edit the following file:
```
sudo vi /boot/config.txt
```
and add the following line:
```
dtparam=i2c1_baudrate=400000
```
where 400000 is the wanted clock frequency.

As that is a boot configuration file, to be effective, a reboot is needed: `sudo reboot`.

#### Check I2C working
After properly connecting SDA, SCL and GND lines between microcontroller and RPi, when give power to both boards,
it is possible to check the functionality of I2C by running the following command:

```
i2cdetect -y 1
```
This command will automatically send a I2C short write (meaning no byte sent) over the bus for each slave device address
from 0x03 to 0x77. If any device acknowledges the address (pulls the data line low), we are notified that device is alive.

Specifically for this implementation, slave address 0x04, the above command should output the following:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- 04 -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

#### I2C packet size
As clock frequency is the time need to transfer 1 bit, it is connected to the bps (bit per second) quantity, but need
to consider additional bits sent as part of the I2C protocol.

In general the I2C doesn't require a huge overlay, so it makes no real difference the size of the packet sent.

Note however that on the RPi side, the I2C software implementation is very limitative:
- it is not possible (to my knowledge) to use the RPi as I2C slave device, meaning it must be the master;
- the most cited python package for I2C operation is `smbus` which, as its name states, is actually relative to
the SMbus - System Management bus - which shares similarities with the I2C, but also some differences.
An example of difference is for instance the maximum size per transfer, 32bytes as opposed to the pure
 I2C protocol definitions where no such restriction is posed. As this is however a purely software implementation,
 a different package can be used, which removes this limitation. An example is the `smbus2` package.

#### Python test program
```
import time
from smbus2 import SMBus, i2c_msg

bus = SMBus(1)

msg = i2c_msg.read(0x04, 150)
bus.i2c_rdwr(msg)
count = 1
print count

while 1:
    time.sleep(.2)

    msg = i2c_msg.read(0x04, 110)
    bus.i2c_rdwr(msg)
    count += 1
    print count
```

can see the read data with the following:
```
msg = i2c_msg.read(0x04, 150)
bus.i2c_rdwr(msg)

for k in range(msg.len):
    print("0x%2.2x" % ord(msg.buf[k]))      # print hex
    print("%d" % ord(msg.buf[k]))           # print uint
```