# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

The project is over 2021.06 release tag of crazyflie firmware and implemented on Crazyflie 2.1

## Building and Flashing
See the [building and flashing instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/) in the github docs folder.

Build and flash commands used are -
make clean
make (or) make -j 12
python.exe -m cfloader flash cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7

## Official Documentation

Check out the [Bitcraze crazyflie-firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) on our website.

Mathematical derivation of the modified EKF - https://learn.maplesoft.com/d/CPHLNUNUHOOUINMHGRDUFTBOETMQMKNLNHEOMNCILMGGPSJMBNJQCHKTELOSJPJRNIOHPGKMIFMHBJOKKNIIOTPJOJELONJQFLGH 

## Reference

https://www.bitcraze.io/2022/01/a-modified-ekf-for-constant-altitude-flights/

## License

The code is licensed under LGPL-3.0
