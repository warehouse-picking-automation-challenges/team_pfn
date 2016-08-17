Prepare Environment
===================

* Power on controllers and SIO adapter.
* Unless all controllers already have the correct baud rate (230400) set, set it using the PC software:
    * Connect the USB-SIO cable between PC and SIO adapter.
    * Start the IAI configuration software and discover all the controllers. They should have IDs 3 (linear servo) and 0 (rotary servo).
    * Click "Parameters" and set parameter No. 16 to 230400. Upload the settings to the controller and restart it if prompted.
    * Click "disconnect".

Prepare Software
================

* Install requirements: `pip install -r requirements.txt`
* Plug in USB/Serial adapter and connect it with the serial cable to the SIO adapter. This should create a file `/dev/ttyUSB0`.
* Execute `stty -F /dev/ttyUSB0 230400 raw -echo -echoe -echok` to set the baud rate and other parameters.
