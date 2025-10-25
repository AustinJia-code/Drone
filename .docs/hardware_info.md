# Hardware
| Component         | Name                                                                                    | Info                              |
| ------------      | --------                                                                                | -------                           |
| Computer          | [Raspberry Pi 5 4GB](https://www.amazon.com/dp/B0CK3L9WD3/)                             | Companion Computer                |
| AI Coprocessor    | [Hat M.2+](https://www.amazon.com/dp/B0D6GMXH73/)                                       | M.2 port, Run AI models (YOLOv4)  |
| Heat Sink         | [Generic](https://www.amazon.com/dp/B0CLM77BRL/)                                        | AI Kit cooling                    |
| Flight Controller | [Pixhawk 6C Mini](https://holybro.com/products/pixhawk-6c-mini?variant=44511517475005)  | Flight control + IMU + GPS + PDB  |
| ESC               | [Tekko32 F4](https://holybro.com//products/tekko32-f4-4in1-50a-esc)                     | ESC 4-in-1                        |
| Motor             | [2212 920KV](https://www.aliexpress.us/item/2255800307898838.html?)                     | Motors                            |
| Propeller         | [1045](https://www.aliexpress.us/item/2251832363284871.html?)                           | Propellers                        |
| Camera            | [OV9715 Stereo](https://www.amazon.com/dp/B07R8LQKV4)               | Vision and relative positioning   |
| Frame             | [S500 Quadcopter](https://www.aliexpress.us/item/2251832118132126.html?)                | Frame                             |
| Battery           | [4s 5000mAh (14.8V)](https://www.amazon.com/dp/B06XK8WWX1/)                             | Battery                           |
| RPi Stepdown      | [Generic](https://www.aliexpress.us/item/3256805630251942.html?)                        | Buck down for RPi (to 5V/5A)      |
| Serial to USB     | [FT232RL](https://www.amazon.com/DSD-TECH-Adapter-FT232RL-Compatible/dp/B07BBPX8B8/)    | Serial to USB for Pixhawk to RPi |
| Power Splitter    | [XT60 1-to-3](https://www.aliexpress.us/item/3256806275642777.html?)                    | Power splitter                    |
| Controller        | [Bluetooth](https://www.aliexpress.us/item/3256807529801372.html?)                      | Manual override (close range)     |

# Wiring
<img src="../.assets/DroneWiring.drawio.svg" width="1000">

Specific wires:
- Solder an XT60 12AWG male to TekkoF32
- Solder 3.5mm Bullet 16AWG to TekkoF32 out pads
- Solder an XT60 12AWG male to 5V5A BEC
- Strip a Pixhawk TELEM wire, solder female DuPont to all but 5V, connect to Serial to USB:

| TELEM2 Pin | Signal     | FTDI Pin | FTDI Signal            |
| ---------- | ---------  | -------- | ---------------------- |
| 1          | +5V (red)  | NONE     | DO NOT CONNECT!        |
| 2          | Tx (out)   | 5        | FTDI RX (yellow) (in)  |
| 3          | Rx (in)    | 4        | FTDI TX (orange) (out) |
| 4          | CTS (in)   | 6        | FTDI RTS (green) (out) |
| 5          | RTS (out)  | 2        | FTDI CTS (brown) (in)  |
| 6          | GND        | 1        | FTDI GND (black)       |

- DuPont female to TekkoF32 communication wires (m1-m4, GND)

### Power Estimate
**Hover Power:** ~45A
- 4x Motors @ 12V, ~10A -> 40A
- Nano @ 5V, ~10W -> ~2A
- Pixhawk @ 5.2V -> ~3A

**Flight Time:** ~5:20min

Flight Time (min) = (Battery Capacity (mAh) * efficiency) / current (mA) * 60

Assuming 5000mAh LiPo, 0.8 efficiency:

(5,000 * 0.8) / (45,000) * 60 = ~5:20min