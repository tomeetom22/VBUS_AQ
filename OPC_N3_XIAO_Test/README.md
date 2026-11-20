# OPC-N3 test with XIAO RP2040

Open `OPC_N3_XIAO_Test.ino` in Arduino IDE, select **Seeed XIAO RP2040**
from your RP2040 board package, upload, and open Serial Monitor at **115200 baud**.
Requires **OPC-N3 Library** (`OPCN3.h`), the same library used by
`AQ_Sense/reference/OPC_N3_DataLogger_V1.ino`. The installed version is 0.1.0
and exposes `initialize()`, `getSerialNumber()`, `readHistogramData()`,
`lastResponse()`, and `HistogramData.valid` / `toString()`.

| OPC-N3 pin | Signal | XIAO RP2040 connection |
| --- | --- | --- |
| 1 | Vcc | External regulated 5 V |
| 2 | SCK | D8 / GPIO2 |
| 3 | SDO / MISO | D9 / GPIO4 |
| 4 | SDI / MOSI | D10 / GPIO3 |
| 5 | /SS / CS | D7 / GPIO1 |
| 6 | GND | XIAO GND and external supply GND |

Use the XIAO silkscreen **D** labels when wiring. For example, D7 is GPIO1.
Facing the sensor's connector with the sensor upright as in `OPC_Wiring.pptx`,
pins run **6 to 1 from left to right**. SPI logic is 3.3 V; the OPC supply is 5 V.

The sketch follows Alexei Perelet's reference logger: construct `OPCN3`, call
`initialize()`, obtain and trim the serial number, then call `readHistogramData()`
approximately once per second and print `hist.toString()`. Initialization and
SPI protocol handling belong to the installed `OPCN3` library. That installed
library manages its own mode-1, 500 kHz transactions, so this sketch does not
wrap its calls in additional SPI transactions. It uses the board's default SPI
pins and D7 as CS. The previous custom `OPC_N3.h` driver has been removed from
this test folder.

SD initialization, files, file rotation, and SD mode-0 transactions are removed.
No RTC is required: the reference's six year/month/day/hour/minute/second
columns are replaced by one `elapsed_ms` column (milliseconds since XIAO reset).
Each measurement row is:

```text
elapsed_ms,OPC_serial,<hist.toString()>
```

`hist.toString()` preserves the installed library's field order and formatting:
24 bin counts; four time-of-flight values; sampling period (s); sample flow
(ml/s); temperature (C); humidity (%); four rejection counts; fan revolutions;
laser status; checksum; PM1; PM2.5; PM10.

Startup messages and errors begin with `#`. Invalid histograms produce an error
instead of a CSV measurement row, and the next scheduled read proceeds normally.
If initialization fails, reset the XIAO to retry. Resetting the XIAO does not
power-cycle the OPC. The fan and laser can stay on if the OPC still has power.

Pin reference: [Seeed XIAO RP2040 pin map](https://wiki.seeedstudio.com/XIAO-RP2040/#pin-map).
Connector reference: [Yoctopuce OPC-N3 wiring guide](https://www.yoctopuce.com/EN/article/mesures/yocto-spi-et-alphasense-opc-n3).

Hardware communication must be checked on the connected OPC after upload.

## Architecture

`To Be Implemented`

Planned architecture: one WisBlock logger with a RAK5802 RS485 interface polls
two XIAO RP2040 sensor nodes. Each node measures both particulate matter with
an OPC-N3 and ozone-interface voltages with an ADS1115. This section describes
work to implement; the current sketch remains a one-second OPC-to-USB-Serial test.

### Node identities and bus protocol

Use Modbus RTU with the WisBlock as the sole client/master and each RP2040 as a
server/slave. Both RP2040s run the same firmware with a configurable address.

| Location | Logical unit | Modbus address |
| --- | --- | --- |
| Bottom, 2 m | Unit 0 | 1 |
| Top, 5 m | Unit 1 | 2 |

Modbus address 0 is reserved for broadcasts and must not be assigned to either
node. Start with 19,200 baud and matching parity/stop-bit settings on all three
devices. Finalize the register map, numeric scaling, serial framing, response
timeout, and retry limits during implementation.

The WisBlock requests a completed record from address 1, waits for its response
or a timeout, then requests address 2. Only the addressed node transmits.
Nodes do not send unsolicited measurements, startup text, or debug messages
onto the RS485 bus; local diagnostics use USB Serial. The incoming request is
a serial message over A/B, so the breakout's reserved INT terminal is unused.

### Sampling and buffering

Each RP2040 samples independently of polling and keeps the OPC fan and laser
running. Read the OPC histogram every 10 seconds; the manufacturer specifies
a histogram period of 1–30 seconds. Each read clears the accumulated histogram,
so store the actual reported integration period with the measurement. Use the
OPC's supplied PM1, PM2.5, and PM10 values in ug/m3 rather than summing bin counts
to estimate mass concentration.

Provisionally sample the ozone ADC once per second and average the readings
into the same 10-second records. Confirm this cadence, ADC gain, input voltage
range, and conversion to ozone concentration against the sensor/interface
specifications and calibration before implementation. Preserve raw ADC values
or voltages alongside any calibrated concentration.

Publish a complete measurement snapshot only after its fields and status are
ready. A poll returns that cached snapshot without triggering a new histogram
read or waiting for the next measurement window. Use millis-based scheduling
and bounded communication handling rather than `delay(10000)`; account for
blocking OPC library calls when setting communication timeouts.

Include these fields in the record/register map:

- Sample sequence number and a restart indicator so the logger can distinguish
  repeated polls, new measurements, and node resets.
- Sample timing/age and the actual OPC integration period. The logger should
  timestamp the measurement window rather than treating receipt time as sample time.
- PM1, PM2.5, PM10, ozone ADC readings/voltages, and calibrated ozone when available.
- Sensor validity/error flags, with stale or failed measurements explicitly identified.

Keep a bounded RAM ring buffer for short communication outages. Define how the
logger requests records after its last saved sequence number, how records are
acknowledged, and how overwritten records are reported. RAM records are lost on
power loss/reset; persistent outage storage would require an additional design.
Periodically synchronize node timing if matching the two measurement windows
is required.

### RP2040 pin allocation and RS485 wiring

| Function | XIAO RP2040 pins |
| --- | --- |
| ADS1115 I2C | SDA D0 / GPIO26; SCL D1 / GPIO27 |
| RS485 breakout serial | TX D4 / GPIO6; RX D5 / GPIO7 |
| RS485 V1.0 direction | D2 / GPIO28 |
| OPC-N3 SPI | CS D7; SCK D8; MISO D9; MOSI D10 |

Move the ADC SDA/SCL wires to D0/D1 and configure `Wire.setSDA(D0)` and
`Wire.setSCL(D1)` before `Wire.begin()`. Power the ADC at 3.3 V for direct XIAO
logic compatibility, subject to confirming the ozone-interface output range.
D4/D5 are not hardware UART TX/RX pins on the RP2040, so use the Arduino-Pico
core's PIO serial facility for the breakout.

The XIAO RS485 breakout is V1.0 and requires software direction control:
hold D2 LOW while receiving, set it HIGH to reply, wait for the final transmitted
byte to leave the serial port, then return D2 LOW. The RAK5802 switches direction
automatically. Configure Modbus library callbacks accordingly.

Connect the bus as a trunk from the logger through the bottom node to the top
node, with short node stubs. Connect A/B consistently and provide the appropriate
signal-ground connection. A short 3 m link may work without termination, but the
permanent design should provide 120-ohm termination at the two physical bus ends,
accounting for resistors already present. Leave termination off at an intermediate
node. Set the breakout's 5V switch to IN when external 5 V enters that terminal;
INT and the unused 12V terminal remain disconnected.

Architecture references:

- [RAK5802 interface and automatic direction control](https://github.com/RAKWireless/rakwireless-docs-archive/blob/master/Datasheet/rakproducts/36%20RAK5802%20WisIO%20Module.md)
- [Modbus serial-line specification](https://www.modbus.org/file/secure/modbusoverserial.pdf)
- [XIAO RS485 breakout and direction control](https://wiki.seeedstudio.com/XIAO-RS485-Expansion-Board/)
- [Arduino-Pico PIO serial](https://arduino-pico.readthedocs.io/en/latest/piouart.html)
- [Arduino-Pico pin configuration](https://arduino-pico.readthedocs.io/en/latest/pins.html)
- [OPC-N3 specifications](https://www.alphasense.com/products/view-by-target-gas/opc-n3)
