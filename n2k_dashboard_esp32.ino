/**************************************************************************
  Monitors NMEA2000 bus and displays a simple dashboard of useful info.

  Works with the Adafruit ESP32-S3 TFT Feather
    ----> http://www.adafruit.com/products/5300

  Uses the Adafruit GFX library and the ST7789 display driver.
  Uses the NEMA2000 library for parsing N2K communications using the ESP32
  internal CAN controller (external transceiver required).

 **************************************************************************/
#include <limits>
#include <list>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "N2kMsg.h"
#include "NMEA2000.h"
#define ESP32_CAN_TX_PIN GPIO_NUM_6
#define ESP32_CAN_RX_PIN GPIO_NUM_5
#include <NMEA2000_esp32.h> // Custom NMEA2000_esp32 support for S3
#include <N2kMessages.h>

#include "debounced_button.h"
#include "n2kvector.h"
#include "n2kpos.h"
#include "n2kunits.h"
#include "n2kaistarget.h"

#define UPDATE_INTERVAL_MS  1000

tNMEA2000 &NMEA2000=*(new tNMEA2000_esp32());

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Display page
enum Page {
    PAGE_WIND,
    PAGE_POSITION,
    PAGE_AIS_12NM,
    PAGE_AIS_4NM,
    PAGE_AIS_1NM,
    NUM_PAGES
} page = PAGE_WIND;

double depth = 0.0;
double awa = 0.0;
double aws = 0.0;
double cog = 0.0;
double sog = 0.0;
double latitude = std::numeric_limits<double>::quiet_NaN();
double longitude = std::numeric_limits<double>::quiet_NaN();

double twa = 0.0;
double tws = 0.0;

bool bPosValid = false;
N2kVector localCog;
N2kPos local_pos;
std::list<N2kAISTarget *> targets;

DebouncedButton buttonD0(0, LOW);
DebouncedButton buttonD1(1);
DebouncedButton buttonD2(2);

uint16_t lastUpdate = 0;

void tftSplashScreen();
void tftUpdate();
void fakeData();

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Starting N2K dashboard"));
  buttonD0.begin();
  buttonD1.begin();
  buttonD2.begin();

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  // TODO: Display splash text.
  tftSplashScreen();
  delay(1000);
  Serial.println(F("TFT Initialized"));

  //NMEA2000.SetN2kCANMsgBufSize(8);
  //NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  NMEA2000.SetForwardStream(&Serial);  // PC output on due native port
  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text
  NMEA2000.EnableForward(false);                 // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();
  Serial.println(F("N2k Initialized"));

}


void loop() {
    //fakeData();

  if (buttonD1.wasPressed()) {
    tft.init(135, 240); // Init ST7789 240x135
    tft.setRotation(3);
    Serial.println("Reset tft");
  }
  if (buttonD2.wasPressed()) {
    int p = (int)page + 1;
    if (p == NUM_PAGES) {
        p = 0;
    }
    Serial.println(p);
    page = (Page)p;
    tftUpdate();
  }

  NMEA2000.ParseMessages();
}

void fakeData() {
    uint16_t now = millis();
    uint16_t diff = now - lastUpdate;

    if (diff > 2000) {
        awa = awa + (random() * M_PI * 15 / 180 / RAND_MAX);
        if (awa > M_PI) {
            awa = -2 * M_PI - awa;
        }
        if (awa < -M_PI) {
            awa = 2 * M_PI + awa;
        }
        aws = 6 + (random() * 10.0 / RAND_MAX);
        sog = 3 + (random() * 2.0 / RAND_MAX);
        cog = M_PI / 6;
        depth = 12 + (random() * 5.0 / RAND_MAX);

        localCog.set(metersPerSec2Kts(sog), rad2Deg(cog));

        tftUpdate();
    }
}


double convertMeters2Ft(double m)
{
    if (m == NAN) {
        return m;
    }

    return m * 3.28084;
}

double convertMeters2Nm(double m)
{
    if (m == NAN) {
        return m;
    }
    // 1852m = 1nm
    return m / 1852.0;
}

double convertMetersPerSec2Kts(double spd)
{
    return msToKnots(spd);
}

double convertRad2Deg(double rad)
{
    return RadToDeg(rad);
}

double convertDeg2Rad(double deg) {
    return DegToRad(deg);
}


bool formatLatLong(char* buf, size_t len, double val, char posSuffix, char negSuffix)
{
    double absVal = fabs(val);
    double degs = floor(absVal);
    double mins = (absVal - degs) * 60.0;

    int rc = snprintf(buf, len, "%.0f %.3f'%c", degs, mins,
                      val >= 0 ? posSuffix : negSuffix);
    if (rc < 0 || rc == (int)len) {
        // Truncation or error
        return false;
    }
    return true;
}

void tftSplashScreen() {
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 60);

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello Michael");
}

void tftPagePosition() {
    char buffer[128];

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);

    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(3);

    buffer[0] = '\0';
    if (!std::isnan(latitude)) {
        formatLatLong(buffer, sizeof(buffer), latitude, 'N', 'S');
    }
    tft.println(buffer);

    buffer[0] = '\0';
    if (!std::isnan(longitude)) {
        formatLatLong(buffer, sizeof(buffer), longitude, 'E', 'W');
    }
    tft.println(buffer);

    buffer[0] = '\0';
    if (!std::isnan(cog)) {
        snprintf(buffer, sizeof(buffer), "COG %.0f %.1fkt",
                 convertRad2Deg(cog), convertMetersPerSec2Kts(sog));
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_RED);
    buffer[0] = '\0';
    if (!std::isnan(awa)) {
        snprintf(buffer, sizeof(buffer), "AWA %.0f %.0fkt", convertRad2Deg(awa), convertMetersPerSec2Kts(aws));
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_GREEN);
    buffer[0] = '\0';
    if (!std::isnan(depth)) {
        snprintf(buffer, sizeof(buffer), "%.1fft", convertMeters2Ft(depth));
    }
    tft.println(buffer);
}

void drawRadial(int x0,  int y0,  int r, int bearing, int len, uint16_t color) {
    double ax = sin(DegToRad(bearing));
    double ay = -cos(DegToRad(bearing));

    tft.drawLine(x0 + round(ax * r), y0 + round(ay * r),
                 x0 + round(ax * (r - len)), y0 + round(ay * (r - len)), color);
}

void tftPageWind() {
    int x0 = 240 / 2;
    int y0 = 135 / 2;

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);

    tft.startWrite();
    // Draw port quadrants in red
    tft.drawCircleHelper(x0, y0, 60, 0x1, ST77XX_RED);
    tft.drawCircleHelper(x0, y0, 60, 0x8, ST77XX_RED);
    tft.drawCircleHelper(x0, y0, 59, 0x1, ST77XX_RED);
    tft.drawCircleHelper(x0, y0, 59, 0x8, ST77XX_RED);
    // Draw starboard quadrants in green
    tft.drawCircleHelper(x0, y0, 60, 0x2, ST77XX_GREEN);
    tft.drawCircleHelper(x0, y0, 60, 0x4, ST77XX_GREEN);
    tft.drawCircleHelper(x0, y0, 59, 0x2, ST77XX_GREEN);
    tft.drawCircleHelper(x0, y0, 59, 0x4, ST77XX_GREEN);
    tft.endWrite();

    for (int d = 0; d < 360; d += 10) {
        drawRadial(x0,  y0,  58,  d,  2,  ST77XX_WHITE);
    }
    for (int d = 0; d < 360; d += 30) {
        drawRadial(x0,  y0,  58,  d,  6,  ST77XX_WHITE);
    }

    drawRadial(x0, y0, 50, convertRad2Deg(awa), 20, ST77XX_WHITE);

    tft.setTextSize(3);
    // True wind angle and speed at top
    tft.setTextColor(ST77XX_BLUE);
    tft.setCursor(0, 0);
    tft.print(convertRad2Deg(twa), 0);
    tft.setCursor(170, 0);
    tft.print(convertMetersPerSec2Kts(tws), 1);

    // Apparent wind angle and speed at bottom
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(0, 112);
    tft.print(convertRad2Deg(awa), 0);
    tft.setCursor(170, 112);
    tft.print(convertMetersPerSec2Kts(aws), 1);

    // Local SOG in center of dial
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(x0 - 25, y0 - 10);
    tft.print(convertMetersPerSec2Kts(sog), 1);

}

// Long range plot is a 12nm radius
// Short range plot is a 4nm radius
// Vectors show projected position in 10min
void tftPageAis(Page pg) {
    const double radius = 60.0;
    double range = pg == PAGE_AIS_12NM ? 12.0 : pg == PAGE_AIS_4NM ? 4.0 : 1.0;
    double range_scale = radius / range;
    double vector_scale = range_scale / 6.0;;
    int x0 = 240 / 2;
    int y0 = 135 / 2;


    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);

    tft.drawCircle(x0, y0, round(range * range_scale), ST77XX_WHITE);
    tft.drawCircle(x0, y0, round(range * 0.75 * range_scale), ST77XX_WHITE);
    tft.drawCircle(x0, y0, round(range * 0.5 * range_scale), ST77XX_WHITE);
    tft.drawCircle(x0, y0, round(range * 0.25 * range_scale), ST77XX_WHITE);

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(3);
    tft.setCursor(0, 0);
    tft.print(range, 0);
    tft.print("nm");

    // Local COG vector
    tft.drawLine(x0, y0, x0 + round(localCog.getX() * vector_scale),
                 y0 + round(localCog.getY() * vector_scale), ST77XX_YELLOW);

    // MA! TODO Skip rest until we have local position

    for (auto t : targets) {

        if (*t->getTimestamp() == 0) {
            continue;
        }
        const N2kVector& p = t->getRelDistance();
        if (p.getMagnitude() > range) {
            continue;
        }

        // MA! TODO Check long/lat. Positions seem inverted somewhere. Targets are on land.
        const N2kVector& v = t->getVelocity();
        tft.drawLine(x0 + round(p.getX() * range_scale),
                     y0 + round(p.getY() * range_scale),
                     x0 + round(p.getX() * range_scale + v.getX() * vector_scale),
                     y0 + round(p.getY() * range_scale + v.getY() * vector_scale),
                     ST77XX_YELLOW);
        tft.drawLine(x0 + round(p.getX() * range_scale),
                     y0 + round(p.getY() * range_scale),
                     x0 + round(p.getX() * range_scale) + 1,
                     y0 + round(p.getY() * range_scale),
                     ST77XX_RED);
        tft.drawLine(x0 + round(p.getX() * range_scale),
                     y0 + round(p.getY() * range_scale) + 1,
                     x0 + round(p.getX() * range_scale) + 1,
                     y0 + round(p.getY() * range_scale) + 1,
                     ST77XX_RED);
    }

}

void tftUpdate() {

  // Age out old AIS entries.


  uint16_t now = millis();
  uint16_t diff = now - lastUpdate;

  if (diff > UPDATE_INTERVAL_MS) {
      switch (page) {
          case PAGE_POSITION:
              tftPagePosition();
              break;

          case PAGE_WIND:
              tftPageWind();
              break;

          case PAGE_AIS_12NM:
          case PAGE_AIS_4NM:
          case PAGE_AIS_1NM:
              tftPageAis(page);
              break;
      }

    lastUpdate = now;
  }
}

N2kAISTarget* getAISTarget(uint32_t mmsi) {
    for (auto t : targets) {
        if (t->getMmsi() == mmsi) {
            return t;
        }
    }

    N2kAISTarget* t = new N2kAISTarget(mmsi);
    targets.push_back(t);
    return t;
}

// Converts a bearing of 0...2PI into +-PI
double getSignedBearing(double bearing)
{
    return bearing > M_PI ? (bearing - 2 * M_PI) : bearing;
}


//
// Src  22, Dst 255, PGN 130306: AWS 12.2kts, AWA -31
// Src  22, Dst 255, PGN 129025: Position 36°51.054'S 174°45.870'W
// Src  22, Dst 255, PGN 129029: GNSS Position 60°26.197'N 22°14.269'E, DateTime 2018-01-24 12:13:20.000Z
// Src  22, Dst 255, PGN 128267: Water Depth 33.8ft (10.3m)
// Src  22, Dst 255, PGN 129026: COG 116T, Speed 0.2kts

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  unsigned char sid = 0;
  double val1, val2;
  tN2kHeadingReference ref;
  tN2kWindReference ref2;
  uint8_t mid;
  tN2kAISRepeat rep;
  uint32_t mmsi;
  uint32_t mothership;
  uint32_t imo;
  char callsign[10];
  char name[32];
  char vendor[32];
  uint8_t vesselType;
  double length;
  double beam;
  double draft;
  double posStbd;
  double posBow;
  uint16_t etaDate;
  double etaTime;
  char dest[32];
  tN2kAISVersion aisVer;
  tN2kGNSStype gnssType;
  tN2kAISDTE dte;
  bool accuracy;
  bool raim;
  uint8_t seconds;
  double aisCog;
  double aisSog;
  double aisHdg;
  double aisRot;
  tN2kAISNavStatus status;
  tN2kAISUnit unit;
  bool display;
  bool dsc;
  bool band;
  bool msg22;
  tN2kAISMode mode;
  bool state;

  switch (N2kMsg.PGN) {
    // Handle various PGNs here
    case 128267:
      if (ParseN2kWaterDepth(N2kMsg, sid, val1, val2)) {
        depth = val1 + val2;
      }
      break;
      
    case 129025:
      if (ParseN2kPositionRapid(N2kMsg, val1, val2)) {
        latitude = val1;
        longitude = val2;
        local_pos.set(val1, val2);
        bPosValid = true;
      }
      break;

    case 129026:
      if (ParseN2kCOGSOGRapid(N2kMsg, sid, ref, val1, val2)) {
        if (ref == 0 && !std::isnan(val1) && !std::isnan(val2)) {
          cog = val1;
          sog = val2;
          localCog.set(metersPerSec2Kts(val2), rad2Deg(val1));
        }
      }
      break;
      
    //case 129029:
    //  break;

    case 129038:
        if (ParseN2kAISClassAPosition(N2kMsg, mid, rep, mmsi, latitude, longitude,
                                      accuracy, raim, seconds,
                                      aisCog, aisSog, aisHdg, aisRot, status)) {
            N2kAISTarget* target = getAISTarget(mmsi);

            N2kPos p(latitude, longitude);
            N2kVector v(metersPerSec2Kts(aisSog), rad2Deg(aisCog));
            target->update(p, v);

            if (bPosValid) {
                target->calcCpa(local_pos, localCog);
            }
        }
        break;

    case 129039:
        if (ParseN2kAISClassBPosition(N2kMsg, mid, rep, mmsi, latitude, longitude,
                                      accuracy, raim, seconds,
                                      aisCog, aisSog, aisHdg,
                                      unit, display, dsc, band, msg22, mode, state)) {
              N2kAISTarget* target = getAISTarget(mmsi);

              N2kPos p(latitude, longitude);
              N2kVector v(metersPerSec2Kts(aisSog), rad2Deg(aisCog));
              target->update(p, v);

              if (bPosValid) {
                  target->calcCpa(local_pos, localCog);
              }
          }
          break;

    case 129794:
        if (ParseN2kAISClassAStatic(N2kMsg, mid, rep, mmsi, imo,
                                    callsign, sizeof(callsign),
                                    name, sizeof(name),
                                    vesselType, length, beam,
                                    posStbd, posBow, etaDate, etaTime, draft,
                                    dest, sizeof(dest), aisVer, gnssType,
                                    dte)) {
            N2kAISTarget* target = getAISTarget(mmsi);

            target->update(vesselType, length, beam, draft,
                           callsign, name, dest);
        }
        break;

    case 129809:
        if (ParseN2kAISClassBStaticPartA(N2kMsg, mid, rep, mmsi, name, sizeof(name))) {
            N2kAISTarget* target = getAISTarget(mmsi);
            target->update(name);
        }
        break;

    case 129810:
        if (ParseN2kAISClassBStaticPartB(N2kMsg, mid, rep, mmsi, vesselType,
                                         vendor, sizeof(vendor), callsign, sizeof(callsign),
                                         length, beam, posStbd, posBow, mothership)) {
            N2kAISTarget* target = getAISTarget(mmsi);

            target->update(vesselType, length,
                           beam, 0,
                           callsign, nullptr, nullptr);

        }
        break;

    case 130306:
      if (ParseN2kWindSpeed(N2kMsg, sid, val1, val2, ref2)) {
        if (ref2 == N2kWind_Apparent && !std::isnan(val1) && !std::isnan(val2)) {

          aws = val1;
          awa = getSignedBearing(val2);

          // Calculate true wind
          N2kVector aw(aws, val2);
          N2kVector boat(sog, 0);

          N2kVector tw;
          tw.setXY(aw.getX() - boat.getX(), aw.getY() - boat.getY());
          tws = tw.getMagnitude();
          twa = getSignedBearing(tw.getBearing());
        }
      }
      break;

    default:
      return;
  }

  // We processed a message so update display. This should probably just be
  // setting a flag that an updates is needed so we can consolidate updates.
  tftUpdate();
}
