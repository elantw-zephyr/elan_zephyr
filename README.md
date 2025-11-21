# elan-zephyr
This project extends the official Zephyr RTOS to support Elan’s custom board `32f967_dv`.

## 📁 Directory Structure
This project follows the Zephyr module structure. All necessary files are organized to integrate seamlessly with the official Zephyr RTOS.

```
elan-zephyr/
├── .git/
├── boards/
├── drivers/
├── dts/
├── samples/
├── soc/
├── zephyr/
├── CMakeLists.txt
├── Kconfig
└── README.md
```

## Build Instructions
1. First, refer to the [Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to download and initialize the Zephyr project.
2. Clone this repository into the `modules` directory of your Zephyr workspace.
3. Use `west` to build elan-zephyr project.

```
TEST=west build -b 32f967_dv -p always ./samples/elanspi

The SPI sample will show the sansor id as bellow:
*** Booting Zephyr OS build v4.1.0-6262-g2f2eaf7b6f7f ***
Elan SPI test!
ELAN Read Success 0xb1
```
