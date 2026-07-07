# N2K Dashboard
Monitors the NMEA2000 bus and displays a simple dashboard of relevant sailing info.

Implemented for the [Adafruit ESP32-S3 reverse TFT feather](https://www.adafruit.com/product/5691) with a 1.8" TFT display and 3 buttons for driving the UI.

Relies on the very useful https://github.com/ttlappalainen/NMEA2000 library and a custom version of the [NMEA2000_esp32](https://github.com/offspring/NMEA2000_esp32) library for the ESP32-S3 using the internal CAN controller (external transceiver required).

Also uses the Adafruit ST7789 TFT display driver and Adafruit GFX library for drawing primitives and Adafruit BMP5xx/BME280 sensor libraries if optionally defined.

Approximate power consumption including an external transceiver and 12v to 5v buck converter is about 50mA.

# Overview
Provides top-level pages for wind, position and AIS info (at three different ranges). 

<img width="350" height="208" alt="PXL_20260703_210203631_label" src="https://github.com/user-attachments/assets/5b6ef465-827a-4d8b-99ca-ae24ae111907" />

Button usage:
* The D0 (top) button cycles selection of additional info in the current page.
* The D1 (middle) button toggles a subpage displaying additional info.
* The D2 (bottom) button cycles between top-level pages.

All headings and courses are relative to true North and speeds are in knots. Distances are in nautical miles. By default, depth is in feet, pressure in inHg and temperature in Farenheit but the code can be compiled with USE_METRIC defined to switch to meters, millibars and Celcius.

# Wind Info
Displays basic wind and other instrument data.

![PXL_20260303_225928172](https://github.com/user-attachments/assets/d1f87aaa-fd0a-4453-b1ff-37f20c9dc144)

* The radial dial shows the apparent wind angle (AWA) with the boat speed (SOG) and course (COG) in yellow in the center.
* Apparent wind speed (AWS) and angle (AWA) are shown in red at the top with a 1 hour history graph of AWS.
* True wind speed (TWS) and angle are shown in blue at the bottom with a 1 hour history graph of TWS.
* Depth is shown to the right in green.

## History Detail
D1 toggles to a history detail page. The D0 button cycles through 1 hour histories of TWS, AWS, SOG and Depth showing the range of values over each 1 minute interval. The overall maximum, minimum and average values over the last hour are displayed at bottom left. The vertical axis is divided every 10 units.

![TWSHistory](https://github.com/user-attachments/assets/59321a81-251f-41e7-b92b-f3c0120028cd)

# Position Info
This page is intended to provide information for making logbook entries or radio calls.

<img width="350" height="212" alt="PXL_20260707_153206028" src="https://github.com/user-attachments/assets/8d919463-2ce6-4716-92d7-e0ac9d3c3850" />

Includes:
* Current time (UTC or Local)
* Ambient atmospheric pressure, temperature and humidity when supported.
* GPS position
* Log (trip) distance and time.
* True heading and course.
* Speed over ground and average trip speed.
* Apparent wind speed and angle
* True wind speed and direction (relative to true North).

The trip log can be reset by pressing D0, and then again to confirm. The log is calculated by sampling GPS position every minute.

## Logbook History

The D1 button toggles a logbook history page. This displays a snapshot of most of the above data on the hour for the last 4 hours. 

<img width="350" height="201" alt="PXL_20260707_160720517" src="https://github.com/user-attachments/assets/cc07c625-0b99-4fb8-ad6c-0b454a732b7d" />

Use D0 to cycle through each entry.

# AIS Info (12nm, 6nm, 3nm)
Displays a North-up graphical representation of AIS targets within the selected range.

![AISInfo](https://github.com/user-attachments/assets/d307fb4b-1ccf-419e-85a0-05d7123d3347)

* Speed vectors in yellow show projected position in 5 minutes.
* Targets shown in red are considered potential collision issues (closest point of approach is < 1nm within the next hour).
* D0 cycles through the targets (selected vessel is white).
* D1 toggles a subpage displaying additional information for the selected vessel.
* Selected vessel's range, true bearing and name/MMSI are displayed in green at bottom left.
* If the selected vessel is converging, their closest point of approach (CPA) distance, bearing and time (mins) is shown in red at top right.
* Targets are removed 60s after their last report.

## AIS Vessel Info
The additional vessel info subpage includes additional static data for the selected vessel such and length and draft. How much information is available varies for Class A or Class B reports and depends on which messages have been received.

* D0 can also be used on this page to select other vessels.

![AISVesselInfo](https://github.com/user-attachments/assets/633544c8-4bff-455a-8f09-9d6364685911)

