# Wireless Vibration Sensing for Structural Health Monitoring

The below flowchart gives an overview of the entire design of this project:

![Design](Images/Design.png)

## Project file organization

* ```Firmware/SHM_bluepill/``` contains the MCU's firmware and ```Firmware/esp32_final.ino``` is the ESP32's firmware.
* ```GUI/``` contains the GUI (```EDL_dashboard.py```) along with a script (```Plot_SDCard.py```) for plotting the SD-card's data.

## Configuring Wi-Fi hotspot/router

* Set your Hotspot's/Router's SSID as ```_SSID_``` and Password as ```_PASSWORD_```, else ESP32's firmware needs to be changed.
* Configure your hotspot/router to 2.4 GHz band, otherwise ESP32 won't be able to connect.

## Using GUI

* Open command prompt, run python ```EDL_dashboard_changed.py```.
* Ctrl + click on the link that comes in cmd.
* With GUI opened you will now see previous 15minutes vibration data in 3 axes by default. You can change this by selecting the appropriate time interval from dropdown.
* If you select, say prev 1 mins, and there is no data in that interval the graphs will not refresh.
* SD-card's data can be plotted by running the ```Plot_SDCard.py``` script in ```GUI/```.

## Few things to note
* The gui will take more time to load in case of bigger intervals such as 6h,12h etc. please be patient. 
* There might be lag in data logging (normally it updates in almost 5-10s). If data is not updating, check cmd prompt. If it is an HTTPS error, it is mostly an InfluxDB server error (wait for sometime and try again).
* Data is not logged into Cloud while the device is running by battery power, it is stored in SD card only, so it wont show up on the GUI.
* Use EDL_dashboard for ```acc_x```, ```acc_y```, ```acc_z```, ```FFT(x,y,z)``` and ```temperature``` plots (total 7 plots).

## Scope for improvement

Following are some possible improvements in our project:

* The LSE of Bluepill wasn't functioning properly. Hence, RTC is being supplied with the LSI clock, which is inaccurate. If the device in use has a functioning LSE in its MCU, then instead supply that to the RTC.

* Writing to InfluxDB cloud-based database is slow. Currently, data is being acquired for intervals of 3 seconds following which the database takes around 5 seconds to write the data. Instead, data could be written at the end of the 2 minute run, before sending the MCU to STANDBY mode. This can be implemented by a handshake between MCU and ESP32 via UART, just before the STANDBY mode.

* The PCB design and connections for the accelerometer aren't correct (one of the GND pins isn't connected). Hence, corrections need to be made before using our PCB design. Also, add regulator for battery power input. Try incorporating the battery charger IC and if using 18650 batteries.

* Currently, no data interpretation/analysis is being performed on the acquired data. ML models can be deployed remotely to analyze the acquired data and accordingly signal in case of hazard.

## Contributors

* Ayush Joshi
* Harsh Shah
* Soham Inamdar
* Aditya Bhangale