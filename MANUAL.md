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
