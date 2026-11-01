# N2K Dashboard
Monitors the NMEA2000 bus and displays a simple dashboard of relevant sailing info.

* v.1.3.0 and earlier targeted the [Adafruit ESP32-S3 Reverse 1.14" TFT Feather](https://www.adafruit.com/products/5691). This is no longer supported.
* v.2.0.0 and later targets the [Hosyond ESP32-S3 2.8" IPS LCD display](https://a.co/d/04SSH8Cm) with a touch screen.
  * SPI-based 320x240 LCD with the ILI9341 driver.
  * TFT_eSPI GFX library. This project should work pretty seamlessly with any device supported by this library.
  * FT6336 touch device, but the touch logic is very basic and could be adapted to any touch driver.

Relies on the very useful https://github.com/ttlappalainen/NMEA2000 library and a custom version of the [NMEA2000_esp32](https://github.com/offspring/NMEA2000_esp32) library for the ESP32-S3 using the internal CAN controller (external transceiver required).

Approximate power consumption including an external transceiver and 12v to 5v buck converter is about 55mA.

# Overview
Provides top-level pages for wind, logbook and AIS info (at three different ranges). 

All headings and courses are relative to true North and speeds are in knots. Distances are in nautical miles. By default, depth is in feet, pressure in inHg and temperature in Farenheit but the code can be compiled with USE_METRIC defined to switch to meters, millibars and Celcius.

# Wind Info
Displays basic wind and other instrument data.

<img width="320" height="240" alt="wind0" src="https://github.com/user-attachments/assets/f96d91a6-2a20-41db-a9b0-d6ed74407332" />

* The radial dial shows the apparent wind angle (AWA)
* Boat speed (SOG) and course (COG) are in the center in yellow.
* Apparent wind speed (AWS) and angle (AWA) are shown in red at the top with a 1 hour history graph of AWS.
* True wind speed (TWS) and angle (TWA) are shown in blue at the bottom with a 1 hour history graph of TWS.
* Depth is shown to the left in green.

Navigation:
* Tap AWS, TWS, SOG or Depth to view a detailed 60 minute history.
* Tap the Next page icon to the right to cycle to the logbook page.

## History Detail

<img width="320" height="240" alt="aws0" src="https://github.com/user-attachments/assets/83b83bc0-791c-48b2-a7a6-4b3396c406cb" />

The history detail page shows a 60 minute history of TWS, AWS, SOG or Depth with a bar showing the range of values occurring during each minute. The maximum, minimum and average values over the last 60 minutes are also shown. The vertical axis is divided every 10 units.

Navigation:
* Tap the graph title to cycle to another history graph.
* Tap anywhere in the body of the graph to return to the wind info page.

# Logbook Info
This page is intended to provide information for making logbook entries or radio calls.

<img width="320" height="240" alt="pos0" src="https://github.com/user-attachments/assets/3d12e052-c557-4259-af12-19c06c6c3a18" />

Includes:
* Current time (UTC or Local)
* Ambient atmospheric pressure, air/sea temperature when provided.
* GPS position
* Log (trip) distance and time.
* True heading and course.
* Speed over ground and average trip speed.
* Apparent wind speed and angle
* True wind speed and direction (relative to true North).

The trip log is calculated by sampling GPS position every minute. The log distance and time are saved persistently.

Navigation:
* Tap the time to view historical on-the-hour snapshots.
* Tap the Trip Log to reset (tap again to confirm).
* Tap the Next page icon to the right to cycle to the AIS page.
  
## Logbook History
This page displays a snapshot of most of Logbook info on the hour for the last 4 hours. An initial snapshot is also taken when the system is first powered up once GPS position and time are acquired.

<img width="320" height="240" alt="pos_log0" src="https://github.com/user-attachments/assets/8917b3d0-bb54-4f38-a537-77df33699a61" />

Navigation:
* Tap the time to cycle to a previous snapshot.
* Tap anywhere else to return to the main Logbook info page.

# AIS Info
Displays a North-up graphical representation of AIS targets within the selected range.

<img width="320" height="240" alt="ais1" src="https://github.com/user-attachments/assets/32c7b091-601f-4f5d-b353-251a3cc7652f" />

* Speed vectors in yellow show projected position in 5 minutes.
* Your vessel is shown at center in yellow. Selected target is shown in white. Other vessels default to green.
* Targets shown in red are considered potential collision issues (closest point of approach is < 1nm within the next hour).
* Selected vessel's range, true bearing and distance are displayed in green at bottom left.
* Selected vessel's MMSI/name is displayed in green at bottom right.
* If the selected vessel is converging, their closest point of approach (CPA) distance, bearing and time (mins) is shown at top right.
  * CPA is red if your vessel is projected to pass in front of the vessel.
  * CPA is orange if your vessel is projected to pass behind the vessel.
* Targets are removed 3 mins after their last report.

Navigation:
* Tap inside the range circles to cycle between targets.
* Tap MMSI/Name in the bottom right to display additional vessel information.
* Tap the range to cycle between 12nm, 6nm, 3nm, 1nm ranges.
* Tap the Next page icon to the right to cycle to the Wind info page.

## AIS Vessel Info
The additional vessel info subpage includes additional static data for the selected vessel such and length, beam and draft, vessel type, status and destination when provided.

<img width="320" height="240" alt="ais_detail0" src="https://github.com/user-attachments/assets/40f0d3cb-28a2-47e2-a0d1-1b6b91ea4c58" />

Navigation:
* Tap the vessel name to cycle between target.
* Tap anywhere else to return to the main AIS page.



