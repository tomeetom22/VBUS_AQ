# Modbus string slave test

Upload this sketch to a XIAO RP2040 using the Arduino-Pico core. It requires
ModbusRTUSlave 3.x and its dependencies ModbusADU, ModbusRTUComm, and
ModbusSlaveLogic. Connect the RS485 breakout RX to D5, TX to D4, direction
control to D2, and share signal ground. Set `NODE_ID` to 1 or 2; each node
must have a unique address. The test bus uses 9600 baud, 8N1.

Call `node.setMessage(myString)` to publish the latest message. It returns
false without changing the message if it exceeds 240 bytes. Call `node.poll()`
frequently from the main loop. The message is held in RAM, not sent until the
logger requests it. Blocking sensor reads may delay or miss Modbus requests.

The logger reads **121 input registers** starting at zero with function 04 in
one request. Register 0 is the byte length; registers 1–120 contain two bytes
each, high byte first, with zero padding. Decode only the specified number of
bytes. The Modbus unit address identifies the node; it is not prepended to the
text. Reading the whole record in one request avoids mixing separate updates.
Initially length is zero until the first successful `setMessage`. No queue or
delivery acknowledgement is provided; a new message replaces the previous one.
