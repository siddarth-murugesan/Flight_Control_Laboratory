# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

Project title - Drone manoeuvring with optimal altitude and obstacle detection irrespective of the terrain\

The project is over 2021.06 release tag of Crazyflie firmware and implemented on Crazyflie 2.1

## Building and Flashing
See the [building and flashing instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/) in the github docs folder.

Build and flash commands used are -
make clean\
make (or) make -j 12\
python.exe -m cfloader flash cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7

## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

## Reference

https://www.bitcraze.io/2022/01/a-modified-ekf-for-constant-altitude-flights/

## Notes

Get started with https://www.bitcraze.io/documentation/tutorials/\

Python application scripts - log_script.py and test_python_script.py\

Checkout logging_variables_info.txt file in the repo for the available new firmware variables to log specific to this project\

Informal mathematical verification of the modified EKF - https://learn.maplesoft.com/d/CPHLNUNUHOOUINMHGRDUFTBOETMQMKNLNHEOMNCILMGGPSJMBNJQCHKTELOSJPJRNIOHPGKMIFMHBJOKKNIIOTPJOJELONJQFLGH
