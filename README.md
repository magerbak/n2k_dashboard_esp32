# N2K Dashboard
Monitors the NMEA2000 bus and displays a simple dashboard of useful info.

Implemented for the [Adafruit ESP32-S3 reverse TFT feather](https://www.adafruit.com/product/5691) with a 1.8" TFT display and 3 buttons for driving the UI.

Relies on the very useful https://github.com/ttlappalainen/NMEA2000 library and a custom version of the [NMEA2000_esp32](https://github.com/offspring/NMEA2000_esp32) library for the ESP32-S3 using the internal CAN controller (external transceiver required).

Also uses the Adafruit ST7789 TFT display driver and Adafruit GFX library for drawing primitives.

Approximate power consumption including an external transceiver and 12v to 5V buck converter is about 50mA.

# Overview
Provides top-level pages for wind, position and AIS info (at three different ranges). 

Button usage:
* The D2 (bottom) button cycles between top-level pages.
* The D1 (middle) button toggles a subpage displaying additional info.
* The D0 (top) button cycles selection of additional info in the current page.

All headings and courses are relative to true North and speeds are in knots. Distances are in nautical miles. Depth is in feet.

# Wind Info
Displays basic wind information.

![WindInfo](https://github.com/user-attachments/assets/ce0ab38e-9fd4-4edd-9cf3-04a6386749e4)
* The radial dial shows the apparent wind angle (AWA) with the boat course (COG) and speed (SOG) in yellow in the center.
* Apparent wind angle (AWA) is also shown in red in the bottom left. Apparent wind speed (AWS) is shown in red in the bottom right with a 1 hour history graph.
* True wind angle (TWA) is shown in blue in the top left. True wind speed (TWS) is shown in blue in the top-right with a 1 hour history graph.
* Depth is shown to the left in green.

## History Detail
D1 toggles to a history detail page. The D0 button cycles through 1 hour histories of TWS, AWS, SOG and Depth showing the range of values over each 1 minute interval. The overall maximum, minimum and average values over the last hour are displayed at bottom left. The vertical axis is divided every 10 units.

![TWSHistory](https://github.com/user-attachments/assets/59321a81-251f-41e7-b92b-f3c0120028cd)

# Position Info
Intended to provide information at a glance for making logbook entries.

![PosInfo](https://github.com/user-attachments/assets/c6b03e57-e9e2-4854-a31a-6017e8b9ef8b)

# AIS Info (12nm, 4nm, 1nm)
Displays a North-up graphical representation of AIS targets within the selected range.

![AISInfo](https://github.com/user-attachments/assets/d307fb4b-1ccf-419e-85a0-05d7123d3347)

* Speed vectors in yellow show projected position in 5 minutes.
* Targets shown in red are considered potential collision issues (closest point of approach is < 1nm within the next hour).
* D0 cycles through the targets (selected target is white).
* Selected vessel's range, bearing and name/MMSI are displayed in green at bottom left.
* If the sellected vessel is approaching then their closest point of approach (CPA) distance, bearing and time (mins) is shown in red at top right.

## AIS Vessel Info
D1 toggles displaying additional vessel information. D0 can also be used on this page to cycle through other vessels.

![AISVesselInfo](https://github.com/user-attachments/assets/633544c8-4bff-455a-8f09-9d6364685911)

