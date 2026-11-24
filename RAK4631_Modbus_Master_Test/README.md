# RAK4631 Modbus master test

Upload this sketch to a RAK4631 with the RAK5802 in its IO slot. Upload
`RS485_Slave_Test` to a XIAO with node ID 1. Connect the RS485 A and B
lines consistently and provide an appropriate shared signal ground. Both
devices need power. Use the RAK USB serial monitor at 115200 baud.

The test polls node 1 every five seconds, reading 121 input registers in one
request and printing the node ID and decoded message. To poll a second XIAO,
set its `NODE_ID` to 2 and change `NODE_IDS` to `{1, 2}`. The RS485 bus uses
9600 baud, 8N1. A timeout or other Modbus error is printed with its code.
For diagnosis, the master sends a wake byte and waits 30 ms before each request;
this is an experiment, not a standard Modbus solution.

The `src` folder contains project-local copies of ModbusRTUMaster 2.0.2,
ModbusADU 1.0.2 and ModbusRTUComm 1.3.2. The communication copy is modified
to send each frame as one write for this experiment. It is used only by this
sketch and does not change the globally installed libraries.
The RAK5802 automatically switches direction; WB_IO2 enables its switched
3.3 V rail. The sketch assumes the RAK4631 Arduino board package and Serial1.
This is a communication test, not a data logger.
