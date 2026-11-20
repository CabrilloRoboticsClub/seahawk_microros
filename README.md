# Seahawk-II Micro-ROS for the RP2040

This repository contains the Micro-ROS firmware for Seahawk-II's Raspberry Pi
Pico (RP2040). The firmware acts as a Micro-ROS node named `pico_node` and
subscribes to the `pwm_values` topic. It expects an `std_msgs/Int16MultiArray`
containing eight PWM values and sends those values to GPIO pins. At startup,
each motor channel is set to 1500 microseconds, neutral PWM.

The Pico communicates with the Raspberry Pi over UART. The Raspberry Pi runs
the Micro-ROS Agent, while the Pico runs the Micro-ROS client in this
repository.

## Pico Connections 

The Pico and Raspberry Pi communicate through UART0 at 115200 baud, 8 data
bits, no parity, and 1 stop bit. The Pi can program the Pico through the SWD 
port.

| Raspberry Pi signal | Pico signal |
| --- | --- | 
| UART TX (GPIO 14) | RX (GP1) | 
| UART RX (GPIO 15) | TX (GP0) |  
| GPIO 24 | SWDIO | 
| GPIO 25 | SWCLK |  

The index to PWM in mapping is: 

| Message Index | PWM Pin | 
| --- | --- | 
| 0 | 20 |
| 1 | 13 |
| 2 | 19 |
| 3 | 11 |
| 4 | 21 |
| 5 | 12 |
| 6 | 18 |
| 7 | 10 |

## Checkout 

After checking out this repo you must: 

```
git submodule init 
git submodule update --init --recursive
```

There are lots and lots of subrepos so this could take a while.

## Ubuntu 26.04 Setup

To setup an Ubuntu Desktop to be able to compile this repo run these steps:

1. Install the necessary packages
    ```
    sudo apt install build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi doxygen git python3
    ```
1. In VS Code, open the palette (Ctrl+Shift+P) and search for:
    1. `CMake: Scan for Kits`. Select it.
    1. `CMake: Select a Kit` and select `arm-none-eabi`
1. Now you can build the project by pressing `F7` while editing a source file.

# Devcontainer Setup 

This project can run in a devcontainer. If you're not using Ubuntu 26.04 this is
the best way to be able to build and test this code. To follow the build 
instructions make sure the `CMake Tools` extension is installed inside the 
devcontainer. 

## Connecting Nodes on the Pico

To connect the Pico to the rest of ROS use Docker on the Raspberry Pi:

```console
$ docker run -it --rm --privileged --net=host \
    -v /dev:/dev \
    --name micro-ros-agent \
    microros/micro-ros-agent:jazzy \
    serial --dev /dev/ttyS0  baudrate=115200
```

## Program the Pico on Seahawk-II

The setup script in the ROS repo compiles and installs `openocd`. Use it to
reprogram the Pico when it's plugged in to the logic board.

```console
$ sudo /usr/local/bin/openocd \
    -f interface/raspberrypi-swd.cfg \
    -f target/rp2040.cfg \
    -c "adapter speed 5000" \
    -c "program build/seahawk.elf verify reset exit"
```

The ROS launch file for the ROV programs the Pico with the version that's
checked in to the [ROS
repository](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/tree/lyrical/pico).

Copy `build/seahawk.elf` into the `pico/` directory to install a new version 
of the Pico node permanently. 

## Program the Pico on Ubuntu Desktop

The easiest way to program a Pico from an Ubuntu desktop is with UF2 mode.
No USB serial permissions or a separate programmer are required.

1. Connect the Pico to the Ubuntu desktop with a USB data cable.
2. Hold the Pico's **BOOTSEL** button while connecting the USB cable. If the
   Pico is already connected, you can also hold **BOOTSEL**, press and release
   **RESET**, and then release **BOOTSEL**.
3. The Pico should appear as a removable drive named `RPI-RP2`.

Build the firmware and copy it to the Pico:

```console
cmake --build build
cp build/seahawk.uf2 /media/$USER/RPI-RP2/
sync
```

The Pico will reboot automatically and run the new firmware. If the drive is
mounted somewhere other than `/media/$USER/RPI-RP2`, determine its mount point
with:

```console
lsblk -f
```

Then copy the UF2 file to that mount point.

Alternative SWD programming from Ubuntu requires `openocd`, a compatible SWD
probe, and connections to the Pico's SWDIO and SWCLK pins. This is generally
not needed when UF2 mounting is available.


