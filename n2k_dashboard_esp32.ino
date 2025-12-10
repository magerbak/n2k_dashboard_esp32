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

#define UPDATE_INTERVAL  1
#define AIS_TIMEOUT      60

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

enum Subpage {
    SUBPAGE_NONE,
    SUBPAGE_AIS_INFO,
} subpage = SUBPAGE_NONE;

enum Event {
    EVT_NONE,
    EVT_D0_PRESS,
    EVT_D0_RELEASE,
    EVT_D1_PRESS,
    EVT_D1_RELEASE,
    EVT_D2_PRESS,
    EVT_D2_RELEASE,
};

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
uint32_t sel_target = 0;

DebouncedButton buttonD0(0, LOW);
DebouncedButton buttonD1(1);
DebouncedButton buttonD2(2);

uint16_t lastUpdate = 0;

void tftSplashScreen();
void tftUpdate(bool bForce);
bool isVisibleTarget(const N2kAISTarget* t, Page pg);

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
  tft.cp437(true);    // Use correct code page 437 indices
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  tftSplashScreen();
  delay(1000);
  Serial.println(F("TFT Initialized"));

  //NMEA2000.SetN2kCANMsgBufSize(8);
  //NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  NMEA2000.EnableForward(false);                 // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();
  Serial.println(F("N2k Initialized"));
}

void setTarget(const N2kAISTarget* p) {
    if (p) {
        //char buffer[128];
        //p->toString(buffer,  sizeof(buffer));
        //Serial.println(buffer);

        sel_target = p->getMmsi();
    }
    else {
        //Serial.println("none");
        sel_target = 0;
    }
}

void handleButtonEvents(Event e) {
    bool bNeedUpdate = false;

    if (subpage == SUBPAGE_NONE) {
        // Handle event in a top level page

        if (e == EVT_D2_PRESS) {
            int p = (int)page + 1;
            if (p == NUM_PAGES) {
                p = 0;
            }
            page = (Page)p;
            bNeedUpdate = true;
        }
        else {
            switch (page) {
                case PAGE_AIS_12NM:
                case PAGE_AIS_4NM:
                case PAGE_AIS_1NM:
                    if (targets.size() > 0) {
                        const N2kAISTarget* last = nullptr;

                        switch (e) {
                            case EVT_D0_PRESS:
#if 0
                                // Cycle back through targets
                                for (auto t : targets) {
                                    if (!isVisibleTarget(t, page)) {
                                        continue;
                                    }
                                    if (t->getMmsi() == sel_target) {
                                        // Select previous target
                                        setTarget(last);
                                        bNeedUpdate = true;
                                        goto done;
                                    }
                                    last = t;
                                }
                                // If no match (nothing selected), select the last target
                                setTarget(last);
#else
                                // Cycle forward through targets
                                for (auto t : targets) {
                                    if (!isVisibleTarget(t, page)) {
                                        continue;
                                    }
                                    if (sel_target == 0 || (last && last->getMmsi() == sel_target)) {
                                        // Select next target
                                        setTarget(t);
                                        bNeedUpdate = true;
                                        goto done;
                                    }
                                    last = t;
                                }
                                // If no match (last target was selected), select none.
                                setTarget(nullptr);
#endif
                                break;

                            case EVT_D1_PRESS:
                                if (sel_target) {
                                    subpage = SUBPAGE_AIS_INFO;
                                    bNeedUpdate = true;
                                }
                                break;

                            default:
                                break;
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }
    else {
        if (e == EVT_D1_PRESS) {
            subpage = SUBPAGE_NONE;
            bNeedUpdate = true;
        }
    }

done:
    if (bNeedUpdate) {
        tftUpdate(true);
    }
}

void loop() {
    if (buttonD0.updateState()) {
        handleButtonEvents(buttonD0.isPressed() ? EVT_D0_PRESS : EVT_D0_RELEASE);
    }
    if (buttonD1.updateState()) {
        handleButtonEvents(buttonD1.isPressed() ? EVT_D1_PRESS : EVT_D1_RELEASE);
    }
    if (buttonD2.updateState()) {
        handleButtonEvents(buttonD2.isPressed() ? EVT_D2_PRESS : EVT_D2_RELEASE);
    }

    NMEA2000.ParseMessages();
}

bool formatLatLong(char* buf, size_t len, double val, char posSuffix, char negSuffix)
{
    double absVal = fabs(val);
    double degs = floor(absVal);
    double mins = (absVal - degs) * 60.0;

    int rc = snprintf(buf, len, "%.0f%c %.3f %c", degs, (char)0xf8, mins,
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

void tftPrintJustified(const char* str, int x, int y) {
    int16_t minx;
    int16_t miny;
    uint16_t w;
    uint16_t h;

    tft.getTextBounds(str, 0, 0, &minx, &miny, &w, &h);
    if (x) {
        x -= w + 1;
    }
    if (y) {
        y -= h + 1;
    }
    tft.setCursor(x, y);
    tft.print(str);
}

void tftPrintJustified(double val, int precision, const char* suffix, int x, int y) {
    char str[64];

    if (!suffix) {
        suffix = "";
    }

    int rc = snprintf(str, sizeof(str), "%.*f%s", precision, val, suffix);
    tftPrintJustified(str, x, y);
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

    tft.setCursor(0, 55);
    tft.setTextSize(2);

    buffer[0] = '\0';
    if (!std::isnan(cog)) {
        snprintf(buffer, sizeof(buffer), "COG %.0f%c %.1fkts",
                 rad2Deg(cog), (char)0xf8, metersPerSec2Kts(sog));
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_BLUE);
    buffer[0] = '\0';
    if (!std::isnan(awa)) {
        snprintf(buffer, sizeof(buffer), "TWA %.0f%c %.0fkts",
                 rad2Deg(twa), (char)0xf8, metersPerSec2Kts(tws));
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_RED);
    buffer[0] = '\0';
    if (!std::isnan(awa)) {
        snprintf(buffer, sizeof(buffer), "AWA %.0f%c %.0fkts",
                 rad2Deg(awa), (char)0xf8, metersPerSec2Kts(aws));
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_GREEN);
    buffer[0] = '\0';
    if (!std::isnan(depth)) {
        snprintf(buffer, sizeof(buffer), "Depth %.1fft", meters2Ft(depth));
    }
    tft.println(buffer);
}

void drawRadial(int x0,  int y0,  int r, int bearing, int len, uint16_t color) {
    double ax = sin(deg2Rad(bearing));
    double ay = -cos(deg2Rad(bearing));

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

    drawRadial(x0, y0, 50, rad2Deg(awa), 20, ST77XX_WHITE);

    tft.setTextSize(3);
    // True wind angle and speed at top
    tft.setTextColor(ST77XX_BLUE);
    tft.setCursor(0, 0);
    tft.print(rad2Deg(twa), 0);
    tft.print((char)0xf8);
    tftPrintJustified(metersPerSec2Kts(tws), 1, nullptr, 240, 0);

    // Apparent wind angle and speed at bottom
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(0, 112);
    tft.print(rad2Deg(awa), 0);
    tft.print((char)0xf8);
    tftPrintJustified(metersPerSec2Kts(aws), 1, nullptr, 240, 135);

    // Local SOG in center of dial
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(x0 - 25, y0 - 10);
    tft.print(metersPerSec2Kts(sog), 1);

    // Local COG to left of dial
    tft.setTextSize(2);
    tft.setCursor(0, 60);
    tft.print(rad2Deg(cog), 0);

    // Depth to right of dial
    tft.setTextColor(ST77XX_GREEN);
    tftPrintJustified(meters2Ft(depth), 1, nullptr, 240, 80);
}


bool isVisibleTarget(const N2kAISTarget* t, Page pg) {
    double range = pg == PAGE_AIS_12NM ? 12.0 : pg == PAGE_AIS_4NM ? 4.0 : 1.0;

    if (t->getTimestamp() == 0) {
        // Static data only (no position) so skip
        return false;
    }
    const N2kVector &p = t->getRelDistance();
    if (p.getMagnitude() > range) {
        return false;
    }

    return true;
}

// AIS plot at various ranges defined by page ID.
// Vectors show projected position in 5min
void tftPageAis(Page pg, time_t now) {
    const double radius = 60.0;
    double range = pg == PAGE_AIS_12NM ? 12.0 : pg == PAGE_AIS_4NM ? 4.0 : 1.0;
    double range_scale = radius / range;
    double vector_scale = range_scale / 12.0;;
    int x0 = 240 / 2;
    int y0 = 135 / 2;

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);

    tft.drawCircle(x0, y0, round(range * range_scale), ST77XX_WHITE);
    tft.drawCircle(x0, y0, round(range * 0.75 * range_scale), ST77XX_WHITE);
    tft.drawCircle(x0, y0, round(range * 0.5 * range_scale), ST77XX_WHITE);
    tft.drawCircle(x0, y0, round(range * 0.25 * range_scale), ST77XX_WHITE);

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.print(range, 0);
    tft.print("nm");

    // Local COG vector
    tft.drawLine(x0, y0, x0 + round(localCog.getX() * vector_scale),
                 y0 - round(localCog.getY() * vector_scale), ST77XX_YELLOW);

    // Skip rest until we have local position
    if (bPosValid) {
        const N2kAISTarget* target = nullptr;

        for (auto t : targets) {
            if (!isVisibleTarget(t, page)) {
                continue;
            }

            if (t->getMmsi() == sel_target) {
                target = t;
            }

            const N2kVector &p = t->getRelDistance();
            const N2kVector &v = t->getVelocity();
            tft.fillCircle(x0 + round(p.getX() * range_scale),
                           y0 - round(p.getY() * range_scale),
                           3, ST77XX_RED);
            tft.drawLine(x0 + round(p.getX() * range_scale),
                         y0 - round(p.getY() * range_scale),
                         x0 + round(p.getX() * range_scale + v.getX() * vector_scale),
                         y0 - round(p.getY() * range_scale + v.getY() * vector_scale),
                         ST77XX_YELLOW);

        }
        if (target) {
            double cpad = target->getCpa()->getDistance();
            double cpat = target->getCpa()->getRelTime(now) / 60;
            const N2kVector &p = target->getRelDistance();
            const N2kVector &v = target->getVelocity();

            tft.setTextColor(ST77XX_GREEN);
            tft.setCursor(0, 88);
            tft.print(p.getMagnitude(), 2);
            tft.println("nm");
            tft.print(p.getBearing(), 0);
            tft.println((char)0xf8);
            tft.print(target->getName());

            if (cpat >= 0.0 && !std::isnan(cpad)) {
                tft.setTextColor(ST77XX_RED);
                tftPrintJustified(cpad, 2, "nm", 240,  0);
                tftPrintJustified(cpat, 1, "m", 240,  32);
            }

            tft.fillCircle(x0 + round(p.getX()* range_scale),
                           y0 - round(p.getY()* range_scale),
                           3, ST77XX_GREEN);
            tft.drawLine(x0 + round(p.getX()* range_scale),
                         y0 - round(p.getY()* range_scale),
                         x0 + round(p.getX()* range_scale + v.getX()* vector_scale),
                         y0 - round(p.getY()* range_scale + v.getY()* vector_scale),
                         ST77XX_YELLOW);
        }
    }

}

void tftSubpageAisInfo() {
    char buffer[128];

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);

    if (bPosValid) {
        for (auto t : targets) {
            if (!isVisibleTarget(t, page)) {
                continue;
            }

            if (t->getMmsi() == sel_target) {
                static const int txtSize = 13;
                tft.setTextColor(ST77XX_GREEN);
                tft.setTextSize(2);

                tft.setCursor(0, 0);
                tft.println(t->getName());
                tft.print("MMSI ");
                tft.println(t->getMmsi());
                tft.println("");

                const N2kVector& relPos = t->getRelDistance();
                tft.print("Rng ");
                tft.print(relPos.getMagnitude(), 2);
                tft.print("nm ");
                tft.print("Brg ");
                tft.print(relPos.getBearing(), 0);
                tft.println((char)0xf8);

                const N2kVector& v = t->getVelocity();
                tft.print("SOG ");
                tft.print(v.getMagnitude(), 1);
                tft.print("kts ");
                tft.print("COG ");
                tft.print(v.getBearing(), 0);
                tft.println((char)0xf8);

                tft.print("LOA ");
                tft.print(t->getLength(), 1);
                tft.print("ft ");
                tft.print("Bm ");
                tft.print(t->getBeam(), 1);
                tft.println("ft ");

                tft.print("Dft ");
                tft.print(t->getDraft(), 1);
                tft.println("ft ");
            }
        }
    }
}


void tftUpdate(bool bForce) {

  // Age out old AIS entries.
    time_t now = time(nullptr);
    time_t diff = now - lastUpdate;
    int idx_target = 0;

    // MA! TODO: this should move out of presentation logic
    for (auto t = targets.begin(); t != targets.end(); ) {
        idx_target++;
        if ((*t)->getTimestamp() && now - (*t)->getTimestamp() > AIS_TIMEOUT) {
            if ((*t)->getMmsi() == sel_target) {
                sel_target = 0;
            }
            free(*t);
            t = targets.erase(t);
            bForce = true;
        }
        else {
            ++t;
        }
    }

  if (bForce || diff >= UPDATE_INTERVAL) {
      switch (subpage) {
          case SUBPAGE_NONE:
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
                      tftPageAis(page, now);
                      break;
              }
              break;

          case SUBPAGE_AIS_INFO:
              tftSubpageAisInfo();
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

        bool bIsFirstPos = !bPosValid;
        bPosValid = true;

        if (bIsFirstPos) {
            for (auto t : targets) {
                if (t->getTimestamp() != 0) {
                    t->calcCpa(local_pos, localCog);
                }
            }
        }
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

  tftUpdate(false);
}
