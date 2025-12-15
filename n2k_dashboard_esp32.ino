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
#include "simple_timer.h"
#include "data_history.h"
#include "n2kvector.h"
#include "n2kpos.h"
#include "n2kunits.h"
#include "n2kaistarget.h"

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  135

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

    SUBPAGE_HIST_TWS,
    SUBPAGE_HIST_AWS,
    SUBPAGE_HIST_SOG,
    SUBPAGE_HIST_DEPTH,
    SUBPAGE_HIST_SEPARATOR,

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

enum TextLayout {
    TXT_RIGHT_JUSTIFIED,
    TXT_CENTERED,
};

struct HistWindowContext {
    uint16_t color;
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;

    unsigned int range_x;
    unsigned int range_y;
};

struct HistStatsContext {
    bool bFirst = true;
    size_t len = 0;

    double min = 0.0;
    double max = 1.0;
    double avg = 0.0;
};


double depth = 0.0;
double awa = 0.0;
double aws = 0.0;  // knots
double cog = 0.0;
double sog = 0.0;  // knots
double latitude = std::numeric_limits<double>::quiet_NaN();
double longitude = std::numeric_limits<double>::quiet_NaN();

double twa = 0.0;
double tws = 0.0;  // knots

bool bPosValid = false;
N2kVector localCog;
N2kPos local_pos;
std::list<N2kAISTarget *> targets;
uint32_t sel_target = 0;

DebouncedButton buttonD0(0, LOW);
DebouncedButton buttonD1(1);
DebouncedButton buttonD2(2);

SimpleTimer updateTimer;
SimpleTimer historyTimer;

DataHistory<double> awsHistory;
DataHistory<double> twsHistory;
DataHistory<double> sogHistory;
DataHistory<double> depthHistory;
//DataHistory<double> voltageHistory;

uint16_t lastUpdate = 0;

static const char deg_str[] = { 0xf8, '\0' };

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

  updateTimer.begin(nullptr, 1000, updateCallback);
  historyTimer.begin(nullptr, 1000 * 10, historyCallback);

  // Record history of AWS, TWS and SOG for last 60mins.
  awsHistory.begin(60);
  twsHistory.begin(60);
  sogHistory.begin(60);
  depthHistory.begin(60);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(DISPLAY_HEIGHT, DISPLAY_WIDTH); // Init ST7789 DISPLAY_WIDTHxDISPLAY_HEIGHT
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

bool updateCallback(void* user) {
    bool bChanged = false;
    time_t now = time(nullptr);
    time_t diff = now - lastUpdate;

    (void)user;

    // Age out stale AIS targets
    for (auto t = targets.begin(); t != targets.end();) {
        if ((*t)->getTimestamp() && now - (*t)->getTimestamp() > AIS_TIMEOUT) {
            if ((*t)->getMmsi() == sel_target) {
                sel_target = 0;
            }
            free(*t);
            t = targets.erase(t);
            bChanged = true;
        }
        else {
            ++t;
        }
    }

    static int xxx = 0;
    if (++xxx == 60) {
        awsHistory.updateData(25.0);
        twsHistory.updateData(25.0);
    }
    awsHistory.updateData(aws);
    twsHistory.updateData(tws);
    sogHistory.updateData(sog);
    if (depth > 0.0) {
        depthHistory.updateData(depth);
    }

    if (bChanged) {
        tftUpdate(true);
    }

    return true;
}

bool historyCallback(void* user) {
    awsHistory.updateHistory();
    twsHistory.updateHistory();
    sogHistory.updateHistory();
    depthHistory.updateHistory();

    return true;
}

void handleButtonEvents(Event e) {
    const N2kAISTarget* last = nullptr;
    bool bNeedUpdate = false;
    int p;

    if (subpage == SUBPAGE_NONE) {
        // Handle event in a top level page

        if (e == EVT_D2_PRESS) {
            p = (int)page + 1;
            if (p == NUM_PAGES) {
                p = 0;
            }
            page = (Page)p;
            bNeedUpdate = true;
        }
        else {
            switch (page) {
                case PAGE_WIND:
                    switch (e) {
                        case EVT_D1_PRESS:
                            subpage = SUBPAGE_HIST_TWS;
                            bNeedUpdate = true;
                            break;

                        default:
                            break;
                    }
                    break;

                case PAGE_AIS_12NM:
                case PAGE_AIS_4NM:
                case PAGE_AIS_1NM:
                    if (targets.size() > 0) {
                        switch (e) {
                            case EVT_D0_PRESS:
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
        } else {
            switch (subpage) {
                case SUBPAGE_HIST_TWS:
                case SUBPAGE_HIST_AWS:
                case SUBPAGE_HIST_SOG:
                case SUBPAGE_HIST_DEPTH:
                    switch (e) {
                        case EVT_D0_PRESS:
                            p = (int)subpage + 1;
                            if (p == SUBPAGE_HIST_SEPARATOR) {
                                p = SUBPAGE_HIST_TWS;
                            }
                            subpage = (Subpage)p;
                            bNeedUpdate = true;
                            break;

                        case EVT_D1_PRESS:
                            subpage = SUBPAGE_NONE;
                            bNeedUpdate = true;
                            break;

                        default:
                            break;
                    }
                    break;

                case SUBPAGE_AIS_INFO:
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
                    break;


                default:
                    break;
            }
        }
    }

done:
    if (bNeedUpdate) {
        tftUpdate(true);
    }
}

void loop() {
    uint32_t t = millis();

    updateTimer.tick(t);
    historyTimer.tick(t);

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

void tftPrintJustified(const char* str, int x, int y, TextLayout fmt) {
    int16_t minx;
    int16_t miny;
    uint16_t w;
    uint16_t h;

    tft.getTextBounds(str, 0, 0, &minx, &miny, &w, &h);
    switch (fmt) {
        case TXT_RIGHT_JUSTIFIED:
            if (x) {
                x -= w + 1;
            }
            if (y) {
                y -= h + 1;
            }
            break;

        case TXT_CENTERED:
            x -= (w + 1) / 2;
            break;
    }
    tft.setCursor(x, y);
    tft.print(str);
}

void tftPrintJustified(double val, int precision, const char* suffix, int x, int y, TextLayout fmt) {
    char str[64];

    if (!suffix) {
        suffix = "";
    }

    int rc = snprintf(str, sizeof(str), "%.*f%s", precision, val, suffix);
    tftPrintJustified(str, x, y, fmt);
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
                 rad2Deg(cog), (char)0xf8, sog);
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_BLUE);
    buffer[0] = '\0';
    if (!std::isnan(awa)) {
        snprintf(buffer, sizeof(buffer), "TWA %.0f%c %.0fkts",
                 rad2Deg(twa), (char)0xf8, tws);
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_RED);
    buffer[0] = '\0';
    if (!std::isnan(awa)) {
        snprintf(buffer, sizeof(buffer), "AWA %.0f%c %.0fkts",
                 rad2Deg(awa), (char)0xf8, aws);
    }
    tft.println(buffer);

    tft.setTextColor(ST77XX_GREEN);
    buffer[0] = '\0';
    if (!std::isnan(depth)) {
        snprintf(buffer, sizeof(buffer), "Depth %.1fft", depth);
    }
    tft.println(buffer);
}

void drawRadial(int x0,  int y0,  int r, int bearing, int len, uint16_t color) {
    double ax = sin(deg2Rad(bearing));
    double ay = -cos(deg2Rad(bearing));

    tft.drawLine(x0 + round(ax * r), y0 + round(ay * r),
                 x0 + round(ax * (r - len)), y0 + round(ay * (r - len)), color);
}

void statsHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                          size_t len, size_t offset) {
    HistStatsContext* ctx = (HistStatsContext*) user;

    for (unsigned int i = 0; i < len; i++) {
        if (ctx->bFirst) {
            // Don't allow max to be less than 1.0 to avoid divide by zero and
            // other scaling issues.
            ctx->max = dataMax[i] < 1.0 ? 1.0 : dataMax[i];
            ctx->min = dataMin[i];
        }
        else {
            if (ctx->max < dataMax[i]) {
                ctx->max = dataMax[i];
            }
            if (ctx->min > dataMin[i]) {
                ctx->min = dataMin[i];
            }
        }
        ctx->avg += dataMax[i] + dataMin[i];
        ctx->bFirst = false;
    }

    if (ctx->len && offset + len == ctx->len) {
        // We're finished.
        ctx->avg = ctx->avg / (2 * ctx->len);
    }
}

void drawHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                         size_t len, size_t offset) {
    HistWindowContext* ctx = (HistWindowContext*)user;

    double scale_x = ctx->w / ctx->range_x;
    double scale_y = ctx->h / ctx->range_y;

    for (unsigned int i = 0; i < len; i++) {
        int16_t dmax = round(dataMax[i] * scale_y);
        int16_t dmin = round(dataMin[i] * scale_y);

        tft.drawFastVLine(ctx->x + round((offset + i) * scale_x),
                          ctx->y + ctx->h - dmax,
                          dmax - dmin + 1, ctx->color);
    }
}

void tftPageWind() {
    struct HistWindowContext hist_window;

    int x0 = DISPLAY_WIDTH / 2;
    int y0 = DISPLAY_HEIGHT / 2;

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);

    // TWS history top right
    hist_window.color = ST77XX_BLUE;
    hist_window.x = DISPLAY_WIDTH - twsHistory.getLength();
    hist_window.y = 30;
    hist_window.w = 60;
    hist_window.h = 30;
    hist_window.range_x = twsHistory.getSize();
    hist_window.range_y = hist_window.h;
    twsHistory.forEachData(drawHistoryCallback, &hist_window);

    // AWS history bottom right
    hist_window.color = ST77XX_RED;
    hist_window.x = DISPLAY_WIDTH - twsHistory.getLength();
    hist_window.y = 70;
    hist_window.w = 60;
    hist_window.h = 30;
    hist_window.range_x = awsHistory.getSize();
    hist_window.range_y = hist_window.h;
    awsHistory.forEachData(drawHistoryCallback, &hist_window);

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

    // True wind angle and speed at top
    tft.setTextColor(ST77XX_BLUE);
    tft.setTextSize(3);
    tft.setCursor(0, 0);
    tft.print(rad2Deg(twa), 0);
    tft.println((char)0xf8);
    tft.setTextSize(1);
    tft.print("TWA");

    tft.setTextSize(3);
    tftPrintJustified(tws, 1, nullptr, DISPLAY_WIDTH, 0, TXT_RIGHT_JUSTIFIED);

    // Apparent wind angle and speed at bottom
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(1);
    tft.setCursor(0, 102);
    tft.print("AWA");
    tft.setTextSize(3);
    tft.setCursor(0, 112);
    tft.print(rad2Deg(awa), 0);
    tft.print((char)0xf8);

    tft.setTextSize(3);
    tftPrintJustified(aws, 1, nullptr, DISPLAY_WIDTH, DISPLAY_HEIGHT, TXT_RIGHT_JUSTIFIED);

    // Local SOG and COG in center of dial
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tftPrintJustified(sog, 1, nullptr, x0, y0 - 15, TXT_CENTERED);
    tftPrintJustified(rad2Deg(cog), 0, deg_str, x0, y0 + 5, TXT_CENTERED);

    // Depth to left of dial
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(0, 60);
    tft.println(depth, 1);
    tft.setTextSize(1);
    tft.print("ft");
}

void tftSubpageHistory(const char* title, const DataHistory<double>* hist, uint16_t color) {
    struct HistStatsContext hist_stats;
    struct HistWindowContext hist_window;

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);

    hist_stats.len = hist->getLength();
    hist->forEachData(statsHistoryCallback, &hist_stats);

    // Vertical scale
    int t = ((int)hist_stats.max / 10) * 10;
    while (t > 0) {
        tft.drawFastHLine(0, DISPLAY_HEIGHT - 3 - round(t * (DISPLAY_HEIGHT - 3) / hist_stats.max),
                          DISPLAY_WIDTH, ST77XX_WHITE);
        t -= 10;
    }

    // Horizontal scale
    tft.drawFastHLine(0, DISPLAY_HEIGHT - 3, DISPLAY_WIDTH, ST77XX_WHITE);
    for (t = 0; t < hist->getSize(); t += 10) {
        tft.drawFastVLine(DISPLAY_WIDTH - round(t * DISPLAY_WIDTH / hist->getSize()),
                          DISPLAY_HEIGHT - 2, 2, ST77XX_WHITE);
    }

    // Title
    tft.setTextColor(color);
    tft.setTextSize(3);
    tft.setCursor(0, 0);
    tft.print(title);

    tft.setTextSize(2);
    tft.setCursor(0, 81);
    tft.print("Max ");
    tft.println(hist_stats.max, 0);

    tft.print("Avg ");
    tft.println(hist_stats.avg, 0);

    tft.print("Min ");
    tft.println(hist_stats.min, 0);

    double scale_x = DISPLAY_WIDTH / hist->getSize();
    hist_window.color = color;
    hist_window.x = DISPLAY_WIDTH - round(hist->getLength() * scale_x);
    hist_window.y = 0;
    hist_window.w = DISPLAY_WIDTH;
    hist_window.h = DISPLAY_HEIGHT - 3;
    hist_window.range_x = hist->getSize();
    hist_window.range_y = hist_stats.max;

    hist->forEachData(drawHistoryCallback, &hist_window);
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

bool isDangerousTarget(double d, double t) {
    if (!std::isnan(d) && !std::isnan(t)) {
        return d < 1.0 && t > 0 && t < 60;
    }
    return false;
}


// AIS plot at various ranges defined by page ID.
// Vectors show projected position in 5min
void tftPageAis(Page pg, time_t now) {
    const double radius = 60.0;
    double range = pg == PAGE_AIS_12NM ? 12.0 : pg == PAGE_AIS_4NM ? 4.0 : 1.0;
    double range_scale = radius / range;
    double vector_scale = range_scale / 12.0;;
    int x0 = DISPLAY_WIDTH / 2;
    int y0 = DISPLAY_HEIGHT / 2;

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
            double cpad = t->getCpa()->getDistance();
            double cpat = t->getCpa()->getRelTime(now) / 60;
            bool bDangerous = isDangerousTarget(cpad, cpat);

            tft.fillCircle(x0 + round(p.getX() * range_scale),
                           y0 - round(p.getY() * range_scale),
                           3, bDangerous ? ST77XX_RED : ST77XX_GREEN);
            tft.drawLine(x0 + round(p.getX() * range_scale),
                         y0 - round(p.getY() * range_scale),
                         x0 + round(p.getX() * range_scale + v.getX() * vector_scale),
                         y0 - round(p.getY() * range_scale + v.getY() * vector_scale),
                         ST77XX_YELLOW);

        }
        if (target) {
            double cpad = target->getCpa()->getDistance();
            double cpab = target->getCpa()->getBearing();
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
                tftPrintJustified(cpad, 2, "nm", DISPLAY_WIDTH,  0, TXT_RIGHT_JUSTIFIED);
                tftPrintJustified(cpab, 0, deg_str, DISPLAY_WIDTH,  32, TXT_RIGHT_JUSTIFIED);
                tftPrintJustified(cpat, 1, "m", DISPLAY_WIDTH,  48, TXT_RIGHT_JUSTIFIED);
            }

            tft.fillCircle(x0 + round(p.getX()* range_scale),
                           y0 - round(p.getY()* range_scale),
                           3, ST77XX_WHITE);
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

          case SUBPAGE_HIST_TWS:
              tftSubpageHistory("TWS", &twsHistory, ST77XX_BLUE);
              break;

          case SUBPAGE_HIST_AWS:
              tftSubpageHistory("AWS", &awsHistory, ST77XX_RED);
              break;

          case SUBPAGE_HIST_SOG:
              tftSubpageHistory("SOG", &sogHistory, ST77XX_YELLOW);
              break;

          case SUBPAGE_HIST_DEPTH:
              tftSubpageHistory("Depth", &depthHistory, ST77XX_GREEN);
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
          depth = meters2Ft(val1 + val2);
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
          sog = metersPerSec2Kts(val2);
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

          aws = metersPerSec2Kts(val1);
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
