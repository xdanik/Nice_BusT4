# ESPHOME component to control Nice drives via Bus T4 protocol
# Nice Bus T4 protocol

There was a desire to understand the protocol for controlling Nice gates.
Perspective - cheap esp8266-based devices for smart home control.

Modern drive control units have a BusT4 connector, to which GND, + 24V, Can-Rx, Can-Tx are output.

# Current capabilities
* Sending commands: "Open", "Stop", "Close", "Partial opening", "Step by step (SBS)" and others via buttons.
* Sending arbitrary HEX commands via the "raw_command" service. The team must be formed in advance or peeped somewhere. Byte separators can be periods or spaces. Example: 55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c or 55.0D.00.FF.00.66.08.06.97.00.04.99.00.00.9D.0D
* Formation and sending of arbitrary GET/SET requests through the "send_inf_command" service. Allows you to configure the device or get its status.
* Display packets from all devices in the busT4 network.

# BusT4:

This is a modified UART 19200 8n1 with a uart break duration of 519us-590us before each burst.
You can connect several devices; for this, CAN-BUS transceivers are added to the physical layer.
Physical transmission often occurs through CAN transceivers, but there are no CAN frames.

# What is done:
* Connected FTDI232 to GND, Can-Rx, Can-Tx. Packets are visible and decryptable.
* With a logic analyzer, I saw the shape of the signal and the composition of the packages, picked up the uart parameters.
* Successfully simulated a read packet through Arduino Mega, the drive responds.
* Received commands OPEN CLOSE and so on
* Got drive status byte
* I read the main commands, partially deciphered the meaning of the bytes.
* Assembled a prototype device, tested the work.
* Compiled a sniffer to catch packets between OVIEW and busT4 devices
* Wrote a component that has the ability to control drives and receivers using the BusT4 protocol
* I checked the work on Wingo5000 with MCA5 block, Robus RB500HS, SO2000, Rd400.

![alt text](img/Schematic_esphome_bust4_adapter.png "bus-t4 adapter diagram")


ESP8266 does not match BUS T4 level, add a 3.3V -> 5V level converter for Tx on the transistor.
Rx ESP is 5V tolerant, but needs a diode for stable operation. It works for me with random germanium, maybe silicon will do.

Later the scheme was modified.
![alt text](img/Schematic_busT4adapter_xl.png "Bus-t4 adapter diagram with a modified power supply")

![alt text](img/hassio-bust4.png "Bus-t4 component test")


The component supports sending an arbitrary command to the drive via the ESPHome service: nice_bust4_uart_raw_command in Home assystant.
```
SBS:   55 0c 00 03 00 81 01 05 86 01 82 01 64 e6 0c
Open:  55 0c 00 03 05 81 01 05 83 01 82 03 64 e4 0c
Close: 55 0c 00 03 05 81 01 05 83 01 82 04 64 e3 0c
Stop:  55 0c 00 03 00 81 01 05 86 01 82 02 64 e5 0c
```
![alt text](img/IMG_20220113_160221.jpg "Appearance of the prototype device")

During startup and operation, ESP polls the devices connected to the BusT4 bus and displays information about them in the log.
![log](img/log.png "Лог")
![log](img/log2.png "Лог2")

# Updates
* Services have been added to the component interface to make it easier to start the leaf length recognition procedure and the BlueBus device recognition procedure without disassembling the drive housing (and even being remotely).
* Added output to the configuration log of the states L1, L2, L3 read from the device (Automatic closing, close after photocell, always close)

If you are interested in the project, you can [buy me a beer or coffee](https://yoomoney.ru/to/4100117927279918)
