# gear_a_b_mbed

Gear, Accelerator, and Brake controller for mbed.

## Description

This mbed program is aimed for our autonomous EV project. Following is a list
of hardware components connected to the EV:

- Cool Muscle RD-55T-12-0100-C-N
  - For the brake actuator
- Cool Muscle CM-AB042-010-S2-P2
  - For the steering actuator
- Potentiometer
  - For the brake percentage
- DAC
  - For the accelerator control

TODO: more detailed explanation about the components

## Installation

Run `make deploy` in the project's root directory. This will create the `.bin`
file and copy it over to the mbed. If it doesn't automatically deploy it,
you need to copy it over yourself.

Then, copy this to you catkin workspace and run `catkin_make`. After `source
devel/setup.bash`, you should be able to run `roslaunch gear_a_b_mbed
coms.launch`. Change the port name accordingly.

## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)

## License

MIT License
