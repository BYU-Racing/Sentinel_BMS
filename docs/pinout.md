# RP2040 Pinout

## Full Pinout

| RP2040 Signal | Package Pin | Function            | Notes / Connection                 |
| ------------- | ----------- | ------------------- |------------------------------------|
| GPIO0         | 2           | SPI0 MISO           | To CAN0/CAN1 (`can_spi.miso`)      |
| GPIO1         | 3           | SK6812 LED Data     | Via 150 Ω resistor                 |
| GPIO2         | 4           | SPI0 SCLK           | To CAN0/CAN1                       |
| GPIO3         | 5           | SPI0 MOSI           | To CAN0/CAN1                       |
| GPIO4         | 6           | SPI0 CS0            | CAN0 chip select (nCS)             |
| GPIO5         | 7           | SPI0 CS1            | CAN1 chip select (nCS)             |
| GPIO6         | 8           | SPI1 CS0            | ADBMS6832                          |
| GPIO7         | 9           | SPI1 CS1            | ADBMS6832                          |
| GPIO8         | 11          | GPIO                | Header pin 9                       |
| GPIO9         | 12          | GPIO                | Header pin 10                      |
| GPIO10        | 13          | SPI1 SCLK           | ADBMS6832                          |
| GPIO11        | 14          | SPI1 MOSI           | ADBMS6832                          |
| GPIO12        | 15          | SPI1 MISO           | ADBMS6832                          |
| GPIO13        | 16          | Charge Enable Sense | Via transistor / pull-up network   |
| GPIO14        | 17          | Drive Enable Sense  | Via transistor / pull-up network   |
| GPIO15        | 18          | BMS Status Output   | Drives transistor base             |
| GPIO16        | 27          | UART0 TX            | To UART0 + header pin 1            |
| GPIO17        | 28          | UART0 RX            | To UART0 + header pin 2            |
| GPIO18        | 29          | CAN0 Interrupt      | nINT input                         |
| GPIO19        | 30          | CAN1 Interrupt      | nINT input                         |
| GPIO20        | 31          | I²C0 SDA            | To I²C bus + header pin 7          |
| GPIO21        | 32          | I²C0 SCL            | To I²C bus + header pin 8          |
| GPIO22        | 34          | GPIO                | Header pin 18                      |
| GPIO26 / ADC0 | 38          | Analog Input        | Header pin 15                      |
| GPIO27 / ADC1 | 39          | Analog Input        | Header pin 16                      |
| GPIO28 / ADC2 | 40          | Analog Input        | Header pin 17                      |
| USB_DP        | 47          | USB D+              | USB-C + box connector + test point |
| USB_DM        | 46          | USB D−              | USB-C + box connector + test point |

## SPI Lines

### CAN

| Signal | RP2040 Pin | Connected Devices | Notes           |
| ------ |------------| ----------------- | --------------- |
| MISO   | GPIO0      | CAN0, CAN1        | Shared bus      |
| MOSI   | GPIO3      | CAN0, CAN1        | Shared bus      |
| SCLK   | GPIO2      | CAN0, CAN1        | Shared clock    |
| CS0    | GPIO4      | CAN0              | Active-low      |
| CS1    | GPIO5      | CAN1              | Active-low      |
| INT0   | GPIO18     | CAN0              | Interrupt input |
| INT1   | GPIO19     | CAN1              | Interrupt input |

### ADBMS6822

| Signal | RP2040 Pin | Connected Device | Notes        |
| ------ |------------|------------------| ------------ |
| MISO   | GPIO12     | ADBMS6832        | Shared bus   |
| MOSI   | GPIO11     | ADBMS6832        | Shared bus   |
| SCLK   | GPIO10     | ADBMS6832        | Shared clock |
| CS0    | GPIO6      | ADBMS6832 Main   | Active-low   |
| CS1    | GPIO7      | ADBMS6832 AUx    | Active-low   |
