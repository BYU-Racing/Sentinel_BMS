# RP2040 Pinout

## Full Pinout

| RP2040 Signal | Package Pin | Function            | Notes / Connection                 |
|---------------|-------------|---------------------|------------------------------------|
| GPIO0         | 2           | SPI0 MISO           | To CAN0/CAN1 (`can_spi.miso`)      |
| GPIO1         | 3           | SK6812 LED Data     | Via 150 Ω resistor                 |
| GPIO2         | 4           | SPI0 SCLK           | To CAN0/CAN1                       |
| GPIO3         | 5           | SPI0 MOSI           | To CAN0/CAN1                       |
| GPIO4         | 6           | SPI0 CS0            | CAN0 chip select (nCS)             |
| GPIO5         | 7           | SPI0 CS1            | CAN1 chip select (nCS)             |
| GPIO6         | 8           | SPI1 CS0            | ADBMS6832                          |
| GPIO7         | 9           | SPI1 CS1            | ADBMS6832                          |
| GPIO8         | 11          | GPIO                | Not used                           |
| GPIO9         | 12          | GPIO                | Not used                           |
| GPIO10        | 13          | SPI1 SCLK           | ADBMS6832                          |
| GPIO11        | 14          | SPI1 MOSI           | ADBMS6832                          |
| GPIO12        | 15          | SPI1 MISO           | ADBMS6832                          |
| GPIO13        | 16          | Charge Enable Sense | Via transistor / pull-up network   |
| GPIO14        | 17          | Drive Enable Sense  | Via transistor / pull-up network   |
| GPIO15        | 18          | BMS Status Output   | Drives transistor base             |
| GPIO16        | 27          | UART0 TX            | To UART0 + header pin 5            |
| GPIO17        | 28          | UART0 RX            | To UART0 + header pin 6            |
| GPIO18        | 29          | CAN0 Interrupt      | nINT input                         |
| GPIO19        | 30          | CAN1 Interrupt      | nINT input                         |
| GPIO20        | 31          | I²C SDA             | To I²C bus + header pin 11         |
| GPIO21        | 32          | I²C SCl             | To I²C bus + header pin 12         |
| GPIO22        | 34          | GPIO                | Header pin 13                      |
| GPIO23        | 35          | GPIO                | Header pin 14                      |
| GPIO24        | 36          | GPIO                | Header pin 15                      |
| GPIO25        | 37          | GPIO                | Header pin 16                      |
| GPIO26 / ADC0 | 38          | Analog Input        | Header pin 19                      |
| GPIO27 / ADC1 | 39          | Analog Input        | Header pin 20                      |
| GPIO28 / ADC2 | 40          | Analog Input        | Header pin 21                      |
| GPIO29 / ADC3 | 41          | Analog Input        | Header pin 22                      |
| USB_DP        | 47          | USB D+              | USB-C + box connector + test point |
| USB_DM        | 46          | USB D−              | USB-C + box connector + test point |

## SPI Lines

### CAN (MCP2517FD)

| Signal | RP2040 Pin | Connected Devices | Notes           |
|--------|------------|-------------------|-----------------|
| MISO   | GPIO0      | CAN0, CAN1        | Shared bus      |
| MOSI   | GPIO3      | CAN0, CAN1        | Shared bus      |
| SCLK   | GPIO2      | CAN0, CAN1        | Shared clock    |
| CS0    | GPIO4      | CAN0              | Active-low      |
| CS1    | GPIO5      | CAN1              | Active-low      |
| INT0   | GPIO18     | CAN0              | Interrupt input |
| INT1   | GPIO19     | CAN1              | Interrupt input |

### ADBMS6822

| Signal | RP2040 Pin | Connected Device | Notes        |
|--------|------------|------------------|--------------|
| MISO   | GPIO12     | ADBMS6832        | Shared bus   |
| MOSI   | GPIO11     | ADBMS6832        | Shared bus   |
| SCLK   | GPIO10     | ADBMS6832        | Shared clock |
| CS0    | GPIO6      | ADBMS6832 Main   | Active-low   |
| CS1    | GPIO7      | ADBMS6832 AUx    | Active-low   |

## Header Pinout

| Header Pin | Function  | Notes                            |
|------------|-----------|----------------------------------|
| 1          | CAN0 Low  | CAN0 terminating resistor jumper |
| 2          | CAN0 High | CAN0 terminating resistor jumper |
| 3          | CAN1 Low  | CAN1 terminating resistor jumper |
| 4          | CAN1 High | CAN1 terminating resistor jumper |
| 5          | GPIO 16   | UART TX                          |
| 6          | GPIO 17   | UART RX                          |
| 7          | 5V        |                                  |
| 8          | GND       |                                  |
| 9          | 3.3V      |                                  |
| 10         | GND       |                                  |
| 11         | GPIO 20   | I2C SDA                          |
| 12         | GPIO 21   | I2C SCl                          |
| 13         | GPIO 22   |                                  |
| 14         | GPIO 23   |                                  |
| 15         | GPIO 24   |                                  |
| 16         | GPIO 25   |                                  |
| 17         | 5V        |                                  |
| 18         | GND       |                                  |
| 19         | GPIO 26   | ADC                              |
| 20         | GPIO 27   | ADC                              |
| 21         | GPIO 28   | ADC                              |
| 22         | GPIO 29   | ADC                              |
| 23         | 3.3V      |                                  |
| 24         | GND       |                                  |
