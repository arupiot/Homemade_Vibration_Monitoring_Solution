# Homemade_Vibration_Monitoring_Solution
Scripts to record data from MEMS accelerometer hooked to an Adafruit Feather M0 Adalogger. Data is written to SD card on the Adafruit feather.

# Description of Scripts
readWritetoSD - Reads X,Y,Z acceleration data and writes to SD card when button is pressed

readWritetoSD_noserial - Same as 'readWritetoSD' but without serial output

readWritetoSD_stripped - Reads Z acceleration and writes only Z acceleration to SD card when button is pressed

readWritetoSD_interrupt - Reads Z accerleration data and only writes to SD card when acceleration exceeds a certain value

'readWritetoSD_interrupt' is the script to be used.

# To do
Enable more than 99 filenames to be created
Take into account DC offset for triggering

Created by Katie.Lampl@arup.com
