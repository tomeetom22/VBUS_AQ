# ADC + OPC RS485 test

Upload to the XIAO RP2040, and use `RAK4631_Raw_Modbus_Test` on the RAK.
Node address 1, 9600 baud, 8N1. USB monitor: 115200 baud. The latest scan
is published as CSV: scan interval in ms, four ADC voltages, OPC serial,
and the library's complete histogram string, matching `ADC_OPC_Combined_Test`.
The logger polls the cached message; scans are approximately ten seconds apart.
After initialization the first handled Modbus request returns the CSV header;
subsequent polls return the latest measurement (or a waiting/error message).
The `b0` through `b23` columns are OPC histogram bins. The header is held
until a request is handled, but Modbus provides no acknowledgement that the
logger received that response; a lost response can therefore lose the header.

Messages are limited to 240 bytes. An oversized row replaces the published
message with an error and is printed in full on USB. Initialization and invalid
OPC reads also publish error text instead of leaving an old measurement looking
current. Initialization errors require a reset. Polling can time out during
blocking sensor operations, and this test has no queue or data acknowledgement.
The raw RAK's wake byte is experimental and not standard Modbus.
