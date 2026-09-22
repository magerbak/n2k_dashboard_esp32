/**************************************************************************
  Monitors NMEA2000 bus and displays a simple dashboard of useful info.

  Works with the Hosyond ESP32-S3 2.8" IPS LCD display (https://a.co/d/04SSH8Cm)

  Compiles for Arduino using ESP32-S3 Dev Module board config.

  Uses TFT_eSPI and TFT_eWidget GFX libraries with ILI9341 driver and
  FT6336 touch screen driver.

  Uses the NEMA2000 library for parsing N2K communications using the ESP32
  internal CAN controller (external transceiver required).

 **************************************************************************/
#include "n2k_config.h"      // Build options

#include <limits>
#include <memory>
#include <list>
#include <Preferences.h>     // For persistent storage of log data

#include <TFT_eSPI.h>        // The library itself is modified to select a
                             // User_Setup.h for the appropriate display driver.
#include <TFT_eWidget.h>     // For graphing support
#include <SPI.h>

#define TOUCH_FT6336_SCL 15
#define TOUCH_FT6336_SDA 16
#define TOUCH_FT6336_INT 17
#define TOUCH_FT6336_RST 18
#include <FT6336.h>         // Touch screen uses I2C

#include "N2kMsg.h"
#include "NMEA2000.h"
#define ESP32_CAN_SILENT_PIN GPIO_NUM_14
#define ESP32_CAN_TX_PIN GPIO_NUM_2
#define ESP32_CAN_RX_PIN GPIO_NUM_3
#include <NMEA2000_esp32.h> // Custom NMEA2000_esp32 support for S3 (https://github.com/offspring/NMEA2000_esp32)
#include <N2kMessages.h>

#include "simple_timer.h"
#include "data_history.h"
#include "n2kvector.h"
#include "n2kpos.h"
#include "n2kunits.h"
#include "n2kaistarget.h"
#include "touch_rect.h"

#define VERSION_NUM      "v2.0.0"
#define UPDATE_INTERVAL  1
#define AIS_TIMEOUT      (3 * 60 + 10)

// Default magnetic variation, in case this is not provided from the network.
// Modify this value based on the region of operation.
#define DEFAULT_MAGNETIC_VARIATION (-14.0 * (M_PI / 180.0))  // Variation for Boston, MA in 2025.

// Interval that min/max data is sampled.
#define HISTORY_SAMPLE_INTERVAL_MS          1000
// Interval over which sampled min/max data is aggregated per data point in history.
#ifdef TESTING
  #define HISTORY_AGGREGATION_INTERVAL_MS   (10 * 1000)
  #define LOG_SAVE_INTERVAL_MS              (2 * 60 * 1000)
#else
  #define HISTORY_AGGREGATION_INTERVAL_MS   (60 * 1000)
  #define LOG_SAVE_INTERVAL_MS              (10 * 60 * 1000)
#endif
// Default number of data points in history - can be customized in setup().
// Total duration of history = num data points * HISTORY_AGGREGATION_INTERVAL_MS
#define HISTORY_DEF_NUM_DATA_POINTS         60

#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  240

#define ARROW_HEIGHT    25
#define ARROW_WIDTH     12

#define NUM_LOG_ENTRIES 4

// Display page
enum Page {
    PAGE_WIND,
    PAGE_POSITION,
    PAGE_AIS,

    NUM_PAGES
};

enum Subpage {
    SUBPAGE_NONE,

    // Wind supbages
    SUBPAGE_HIST_AWS,
    SUBPAGE_HIST_TWS,
    SUBPAGE_HIST_SOG,
    SUBPAGE_HIST_DEPTH,
    SUBPAGE_HIST_SEPARATOR,

    // Position subpages
    SUBPAGE_POS_LOG,

    // AIS subpages
    SUBPAGE_AIS_INFO,
};

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
    TXT_JUSTIFIED,
    TXT_CENTERED,
};

struct HistStatsContext {
    bool bFirst = true;

    size_t len = 0;     // Num samples of history

    // Stats for current history samples.
    double min = 0.0;
    double max = 1.0;
    double avg = 0.0;
};

// Trip log
struct LogData {
    uint32_t duration;
    float distance;
};

struct LogEntry {
    bool bInUse;

    uint16_t timestamp;     // minutes since midnight
    int16_t tzOffset;       // timezone offset or 0 if unknown.

    N2kPos position;
    double heading;
    N2kVector velocity;
    N2kVector appWind;
    LogData logData;
    float atmPressure;
    float atmTemp;
    float seaTemp;
};

// Abbreviations for TFT_eSPI fonts.
#define GFXFF 1

// These are modified versions of the standard GFX FreeFonts with the same
// filename, that replace the '~' character with the '°' symbol for convenience.
// The Fonts directory includes the 48 fonts from the TFT_eSPI library but we
// only include 3.
#include "Fonts/FreeSansBold9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"

// The font names have a Mod suffix to distinguish them from the stock fonts.
#define FONT_SMALL  &FreeSansBoldMod9pt7b
#define FONT_MEDIUM &FreeSansBoldMod12pt7b
#define FONT_LARGE  &FreeSansBoldMod18pt7b

// fontHeight() returns a line-spacing that accounts for ascending and descending
// lowercase characters. Since we are primarily using uppercase, this additional
// spacing looks overly generous, so we use a more compact spacing instead.
#define FONT_SMALL_ADJ_HEIGHT   (21 - 5)
#define FONT_MEDIUM_ADJ_HEIGHT  (28 - 6)
#define FONT_LARGE_ADJ_HEIGHT   (42 - 9)

// Pin usage as follow:
//             CS  DC/RS  RESET  SDI/MOSI  SCK  SDO/MISO  LED    VCC     GND
// ESP32-S3:   10    2      15      11      12      13     21     5V     GND
TFT_eSPI g_tft;

// Default orientation is portrait (240x320)
FT6336 g_ts = FT6336(TOUCH_FT6336_SDA, TOUCH_FT6336_SCL, TOUCH_FT6336_INT,
                     TOUCH_FT6336_RST, DISPLAY_HEIGHT, DISPLAY_WIDTH);

tNMEA2000 &g_NMEA2000 = *(new tNMEA2000_esp32());

// UI state
Page g_page = PAGE_WIND;
Subpage g_subpage = SUBPAGE_NONE;
double g_aisRange = 12.0;

// Common to all pages
TouchRect g_nextPageRect;

// Wind Page touch rects
TouchRect g_awsRect;
TouchRect g_depthRect;
TouchRect g_twsRect;
TouchRect g_sogRect;

// Position Page touch rects
TouchRect g_timeRect;
TouchRect g_logResetRect;

// AIS Page touch rects
TouchRect g_rangeRect;
TouchRect g_selectRect;
TouchRect g_detailRect;

// Subpage rects
TouchRect g_subTitleRect;
TouchRect g_subBodyRect;

float g_envAtmPressure = std::numeric_limits<float>::quiet_NaN();
float g_envAtmTemp = std::numeric_limits<float>::quiet_NaN();
float g_envAtmHumidity = std::numeric_limits<float>::quiet_NaN();
float g_envSeaTemp = std::numeric_limits<float>::quiet_NaN();;

#ifdef USE_METRIC_PRESSURE
static const int g_precPressure = 0;
static const char g_unitsPressure[] = "mb";
#else
static const int g_precPressure = 2;
static const char g_unitsPressure[] = { '"', '\0' };
#endif
#ifdef USE_METRIC
static const char g_unitsTemp[] = "C";
static const char g_unitsDepth[] = "m";
#else
static const char g_unitsTemp[] = "F";
static const char g_unitsDepth[] = "ft";
#endif

// Log data
Preferences g_prefs;
LogData g_tripLog;          // Duration and distance of current trip
bool g_tripResetConfirmation = false;
N2kPos g_lastPos;           // Lat/Long of last log sample

LogEntry g_logEntries[NUM_LOG_ENTRIES];
unsigned int g_logEntryIndex;   // Index of next entry to store.
unsigned int g_selLogEntry; // Reverse index of displayed entry

uint16_t g_date;            // Days since 1/1/1970
double g_secsSinceMidnight; // Seconds since midnight
int16_t g_tzOffset;         // Timezone offset in minutes, if known

bool g_bPosValid = false;
N2kPos g_localPos;          // Latitude and Longitude
N2kVector g_localVelocity;  // Boat COG (in degrees) and SOG (kts) as a vector
double g_hdg = 0.0;         // degrees
double g_variation = DEFAULT_MAGNETIC_VARIATION; // radians

double g_depth = 0.0;       // ft
N2kVector g_appWind;        // Degrees, knots
N2kVector g_trueWind;       // Degrees, knots

// AIS state
std::list<std::unique_ptr<N2kAISTarget>> g_targets;
uint32_t g_selTarget = 0;   // MMSI of selected AIS target

// 1s timer for refreshing data model (ageing out AIS targets, sampling data history).
SimpleTimer g_updateTimer;

// 60s timer for recording aggregate min/max history and log data.
SimpleTimer g_historyTimer;

// 10min timer to save trip log data
SimpleTimer g_tripLogTimer;

// Timer for recording on-the-hour log entries
SimpleTimer g_hourTimer;

MinMaxDataHistory<double> g_awsHistory;
MinMaxDataHistory<double> g_twsHistory;
MinMaxDataHistory<double> g_sogHistory;
MinMaxDataHistory<double> g_depthHistory;

// Time since last display update in seconds.
time_t g_lastUpdate = 0;

void setup(void) {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("Starting N2K dashboard"));

  g_prefs.begin("n2k_dashboard");

  g_tripLog.duration = g_prefs.getUInt("tripLogTime", 0);
  g_tripLog.distance = g_prefs.getFloat("tripLogDist", 0.0);

  // Optionally, set N2k transceiver to silent mode. This only works if there is
  // more than one device on the N2K network. If testing with a single transmitter,
  // then our transmit must remain enabled so it can ACK messages.
  pinMode(ESP32_CAN_SILENT_PIN, OUTPUT);
  digitalWrite(ESP32_CAN_SILENT_PIN, LOW /* HIGH */); // not silent

  g_updateTimer.begin(nullptr, HISTORY_SAMPLE_INTERVAL_MS, updateCallback);
  g_historyTimer.begin(nullptr, HISTORY_AGGREGATION_INTERVAL_MS, historyCallback);
  g_tripLogTimer.begin(nullptr, LOG_SAVE_INTERVAL_MS, logCallback);

  // Record history of AWS, TWS and SOG for last 60mins.
  g_awsHistory.begin(60);
  g_twsHistory.begin(60);
  g_sogHistory.begin(60);
  g_depthHistory.begin(60);

  // initialize TFT
  g_tft.init();
  g_tft.setRotation(1);
  g_ts.begin();
  g_ts.setRotation(1);

  setupTouchRects();

  displaySplashScreen();

  delay(1000);
  displayUpdate(true);

  //NMEA2000.SetN2kCANMsgBufSize(8);
  //NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  g_NMEA2000.EnableForward(false);  // Disable all msg forwarding to USB (=Serial)
  g_NMEA2000.SetMsgHandler(handleNMEA2000Msg);
  g_NMEA2000.Open();
  Serial.println(F("N2k Initialized"));
}

void loop() {
    uint32_t t = millis();

    handleTouchEvents();

    // Drive our timers
    g_updateTimer.tick(t);
    g_historyTimer.tick(t);
    g_tripLogTimer.tick(t);
    g_hourTimer.tick(t);

    // Handle incoming N2K messages.
    g_NMEA2000.ParseMessages();
}

void setupTouchRects() {
    g_nextPageRect.init(DISPLAY_WIDTH - ARROW_WIDTH - 30, DISPLAY_HEIGHT / 2 - ARROW_HEIGHT / 2 - 10,
                        ARROW_WIDTH + 15 + 15, ARROW_HEIGHT + 10 * 2);
    g_nextPageRect.setReleaseCallback(handleNextPageCallback, nullptr);

    g_awsRect.init(0, 0, 70, 60);
    g_awsRect.setReleaseCallback(handleAwsCallback,  nullptr);

    g_depthRect.init(0, DISPLAY_HEIGHT / 2 - 30, 70, 60);
    g_depthRect.setReleaseCallback(handleDepthCallback,  nullptr);

    g_twsRect.init(0, DISPLAY_HEIGHT - 60, 70, 60);
    g_twsRect.setReleaseCallback(handleTwsCallback,  nullptr);

    g_sogRect.init(DISPLAY_WIDTH / 2 - 40, DISPLAY_HEIGHT / 2 - 40, 80, 80);
    g_sogRect.setReleaseCallback(handleSogCallback,  nullptr);


    g_timeRect.init(0, 0, DISPLAY_WIDTH / 2, 40);
    g_timeRect.setReleaseCallback(handleTimeCallback,  nullptr);

    g_logResetRect.init(0, FONT_MEDIUM_ADJ_HEIGHT * 4 - 5, DISPLAY_WIDTH / 2, FONT_MEDIUM_ADJ_HEIGHT + 15);
    g_logResetRect.setReleaseCallback(handleLogResetCallback,  nullptr);


    g_rangeRect.init(0, 0, DISPLAY_WIDTH / 3, 50);
    g_rangeRect.setReleaseCallback(handleRangeCallback,  nullptr);

    g_selectRect.init(DISPLAY_WIDTH / 2 - 90, DISPLAY_HEIGHT / 2 - 60, 180, 120);
    g_selectRect.setReleaseCallback(handleSelectCallback,  nullptr);

    g_detailRect.init(DISPLAY_WIDTH - DISPLAY_WIDTH / 2, DISPLAY_HEIGHT - 40, DISPLAY_WIDTH / 2, 40);
    g_detailRect.setReleaseCallback(handleDetailCallback,  nullptr);

    g_subTitleRect.init(0, 0, DISPLAY_WIDTH, 50);
    g_subTitleRect.setReleaseCallback(handleSubTitleCallback,  nullptr);

    g_subBodyRect.init(0, 40, DISPLAY_WIDTH, DISPLAY_HEIGHT - 50);
    g_subBodyRect.setReleaseCallback(handleSubBodyCallback,  nullptr);
}


void handleTouchEvents() {
    uint16_t x = 0;
    uint16_t y = 0;
    bool bPressed = false;

    g_ts.read();
    if (g_ts.isTouched) {
        x = g_ts.points[0].x;
        y = g_ts.points[0].y;
        bPressed = true;
    }

    if (g_subpage == SUBPAGE_NONE) {
        // All top level pages have a next page button
        g_nextPageRect.update(bPressed, x, y);

        switch (g_page) {
            case PAGE_WIND:
                g_awsRect.update(bPressed, x, y);
                g_depthRect.update(bPressed, x, y);
                g_twsRect.update(bPressed, x, y);
                g_sogRect.update(bPressed, x, y);
                break;

            case PAGE_POSITION:
                g_timeRect.update(bPressed, x, y);
                g_logResetRect.update(bPressed, x, y);
                break;

            case PAGE_AIS:
                g_rangeRect.update(bPressed, x, y);
                g_selectRect.update(bPressed, x, y);
                g_detailRect.update(bPressed, x, y);
                break;
        }
    }
    else {
        // For now subpages only care about touch events to the first row of
        // text or the rest of the page.
        g_subTitleRect.update(bPressed, x, y);
        g_subBodyRect.update(bPressed, x, y);
    }
}

void handleNextPageCallback(void* user) {
    int pg = (int)g_page + 1;
    if (pg >= NUM_PAGES) {
        pg = PAGE_WIND;
    }
    g_page = (Page)pg;
    g_tripResetConfirmation = false;

    displayUpdate(true);
}

void handleAwsCallback(void* user) {
    g_subpage = SUBPAGE_HIST_AWS;

    displayUpdate(true);
}

void handleDepthCallback(void* user) {
    g_subpage = SUBPAGE_HIST_DEPTH;

    displayUpdate(true);
}

void handleTwsCallback(void* user) {
    g_subpage = SUBPAGE_HIST_TWS;

    displayUpdate(true);
}

void handleSogCallback(void* user) {
    g_subpage = SUBPAGE_HIST_SOG;

    displayUpdate(true);
}

void handleTimeCallback(void* user) {
    g_selLogEntry = 0;
    g_subpage = SUBPAGE_POS_LOG;
    g_tripResetConfirmation = false;

    displayUpdate(true);
}

void handleLogResetCallback(void* user) {
   if (g_tripResetConfirmation) {
        g_tripLog.duration = 0;
        g_tripLog.distance = 0.0;
        g_prefs.putUInt("tripLogTime", g_tripLog.duration);
        g_prefs.putFloat("tripLogDist", g_tripLog.distance);
        g_tripResetConfirmation = false;
    }
    else {
        g_tripResetConfirmation = true;
    }
}

void handleRangeCallback(void* user) {
    // Cycle through ranges
    if (g_aisRange == 1.0) {
        g_aisRange = 12.0;
    }
    else if (g_aisRange == 3.0) {
        g_aisRange = 1.0;
    }
    else {
        g_aisRange = g_aisRange / 2.0;
    }

    displayUpdate(true);
}

void handleSelectCallback(void* user) {
    // Cycle forward through targets
    setTarget(cycleAisTarget(g_targets, g_selTarget));

    displayUpdate(true);
}

void handleDetailCallback(void* user) {
    g_subpage = SUBPAGE_AIS_INFO;

    displayUpdate(true);
}

void handleSubTitleCallback(void* user) {
    switch (g_subpage) {
        case SUBPAGE_HIST_AWS:
        case SUBPAGE_HIST_TWS:
        case SUBPAGE_HIST_SOG:
        case SUBPAGE_HIST_DEPTH:
            {
                int sub = (int)g_subpage + 1;
                if (sub >= SUBPAGE_HIST_SEPARATOR) {
                    sub = SUBPAGE_HIST_AWS;
                }
                g_subpage = (Subpage)sub;
            }
            break;

        case SUBPAGE_POS_LOG:
            {
                unsigned int ix = (g_selLogEntry + 1) % NUM_LOG_ENTRIES;
                if (g_logEntries[ix].bInUse == false) {
                    ix = 0;
                }
                g_selLogEntry = ix;
            }
            break;

        case SUBPAGE_AIS_INFO:
            // Cycle forward through targets
            setTarget(cycleAisTarget(g_targets, g_selTarget));
            break;
    }

    displayUpdate(true);
}

void handleSubBodyCallback(void* user) {
    g_subpage = SUBPAGE_NONE;

    displayUpdate(true);
}


void drawArrowRight(int16_t x, int16_t y) {
    g_tft.drawWideLine(x - ARROW_WIDTH, y - ARROW_HEIGHT / 2, x, y, 5, TFT_DARKGREY);
    g_tft.drawWideLine(x, y, x - ARROW_WIDTH, y + ARROW_HEIGHT / 2, 5, TFT_DARKGREY);
}


const N2kAISTarget* cycleAisTarget(const std::list<std::unique_ptr<N2kAISTarget>>& targets,
                                   uint32_t selMmsi) {
    const N2kAISTarget* last = nullptr;

    for (auto& t : targets) {
        if (!isVisibleTarget(t.get())) {
            continue;
        }

        if (selMmsi == 0 || (last && last->getMmsi() == selMmsi)) {
            return t.get();
        }
        last = t.get();
    }

    return nullptr;
}

// Time-based refresh of our data model. This may update the display based on
// what changes.
bool updateCallback(void* user) {
    bool bChanged = false;
    time_t now = time(nullptr);

    (void)user;

    // Age out stale AIS targets
    for (auto t = g_targets.begin(); t != g_targets.end();) {
        if ((*t)->getTimestamp() && now - (*t)->getTimestamp() > AIS_TIMEOUT) {
            if ((*t)->getMmsi() == g_selTarget) {
                setTarget(nullptr);
            }
            t = g_targets.erase(t);
            bChanged = true;
        }
        else {
            ++t;
        }
    }

    // Update the min/max value for current history data point.
    g_awsHistory.updateData(g_appWind.getMagnitude());
    g_twsHistory.updateData(g_trueWind.getMagnitude());
    g_sogHistory.updateData(g_localVelocity.getMagnitude());
    g_depthHistory.updateData(g_depth);

    if (bChanged) {
        displayUpdate(true);
    }

    // Continue running
    return true;
}

bool historyCallback(void* user) {
    // Record the next data point for each history.
    g_awsHistory.updateHistory();
    g_twsHistory.updateHistory();
    g_sogHistory.updateHistory();
    g_depthHistory.updateHistory();

    // Sample position and accumulate running distance and time
    if (g_bPosValid) {
        N2kVector dist = g_localPos.getRelDistance(g_lastPos);
        g_tripLog.duration += HISTORY_AGGREGATION_INTERVAL_MS / 1000;
        g_tripLog.distance += dist.getMagnitude();
        g_lastPos = g_localPos;
    }

    // Force a display refresh every so often even if we're not receiving data
    displayUpdate(true);

    // Continue running
    return true;
}

bool logCallback(void* user) {
    // Persist the accumulated log data
    g_prefs.putUInt("tripLogTime", g_tripLog.duration);
    g_prefs.putFloat("tripLogDist", g_tripLog.distance);

    // Continue running
    return true;
}

bool logEntryCallback(void* user) {
    // Snapshot logbook entry data on the hour
    addLogEntry();

    // Make sure next interval is 1hr because the first time it is not.
    SimpleTimer* t = (SimpleTimer*)user;
    if (t) {
        t->setInterval(3600 * 1000);
    }
    return true;
}

void displaySplashScreen() {
    g_tft.fillScreen(TFT_BLACK);

    // Version in bottom right
    g_tft.setFreeFont(FONT_SMALL);
    g_tft.setTextSize(1);
    g_tft.setTextColor(TFT_YELLOW);

    g_tft.setTextDatum(BR_DATUM);
    g_tft.drawString(VERSION_NUM, DISPLAY_WIDTH, DISPLAY_HEIGHT, GFXFF);

    // Welcome message centered
    g_tft.setFreeFont(FONT_MEDIUM);
    g_tft.setTextColor(TFT_GREEN, TFT_BLACK);

    g_tft.setTextDatum(MC_DATUM);
    g_tft.drawString("Hello Michael", DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2, GFXFF);

    g_tft.setTextDatum(TL_DATUM);
}

// Helper function to print a floating point value using a calculated starting
// position based on its displayed length and a layout option.
void drawJustifiedVal(double val, int precision, const char* suffix, int x, int y, int datum) {
    char str[64];

    if (!suffix) {
        suffix = "";
    }

    int rc = snprintf(str, sizeof(str), "%.*f%s", precision, val, suffix);

    g_tft.setTextDatum(datum);
    g_tft.drawString(str, x, y);
}


//
// Wind info top-level page.
//
void displayPageWind() {
    const int radius = 90;
    GraphWidget graph1(&g_tft);
    GraphWidget graph2(&g_tft);

    int x0 = DISPLAY_WIDTH / 2;
    int y0 = DISPLAY_HEIGHT / 2;

    g_tft.setTextWrap(false);

    // Wind info in large font
    g_tft.setFreeFont(FONT_LARGE);

    g_tft.fillScreen(TFT_BLACK);

#ifdef TESTING
    // Draw touch rects
    drawTouchRect(&g_nextPageRect);
    drawTouchRect(&g_awsRect);
    drawTouchRect(&g_twsRect);
    drawTouchRect(&g_depthRect);
    drawTouchRect(&g_sogRect);
#endif

    // AWS history top left
    graph1.createGraph(60, 30, TFT_BLACK);

    // x scale units is from 60mins to 0, y scale unit is 0 to max val
    graph1.setGraphScale(0.0, 60.0, 0.0, 30.0);
    graph1.setGraphGrid(0.0, 10.0, 0.0, 10.0, TFT_RED);

    graph1.drawGraph(0, 0 + FONT_LARGE_ADJ_HEIGHT + FONT_SMALL_ADJ_HEIGHT + 1);
    g_awsHistory.forEachData(drawHistoryCallback, &graph1);

    // TWS history bottom left
    graph2.createGraph(60, 30, TFT_BLACK);

    // x scale units is from -60mins to 0, y scale unit is 0 to max val
    graph2.setGraphScale(0.0, 60.0, 0.0, 30.0);
    graph2.setGraphGrid(0.0, 10.0, 0.0, 10.0, TFT_BLUE);

    graph2.drawGraph(0, DISPLAY_HEIGHT - FONT_LARGE_ADJ_HEIGHT - FONT_SMALL_ADJ_HEIGHT - 35);
    g_twsHistory.forEachData(drawHistoryCallback, &graph2);

    g_tft.startWrite();
    // Draw port quadrants in red
    g_tft.drawCircleHelper(x0, y0, radius, 0x1, TFT_RED);
    g_tft.drawCircleHelper(x0, y0, radius, 0x8, TFT_RED);
    g_tft.drawCircleHelper(x0, y0, radius - 1, 0x1, TFT_RED);
    g_tft.drawCircleHelper(x0, y0, radius - 1, 0x8, TFT_RED);
    // Draw starboard quadrants in green
    g_tft.drawCircleHelper(x0, y0, radius, 0x2, TFT_GREEN);
    g_tft.drawCircleHelper(x0, y0, radius, 0x4, TFT_GREEN);
    g_tft.drawCircleHelper(x0, y0, radius - 1, 0x2, TFT_GREEN);
    g_tft.drawCircleHelper(x0, y0, radius - 1, 0x4, TFT_GREEN);
    g_tft.endWrite();

    for (int d = 0; d < 360; d += 10) {
        drawRadial(x0,  y0,  radius - 2,  d,  2,  TFT_WHITE);
    }
    for (int d = 0; d < 360; d += 30) {
        drawRadial(x0,  y0,  radius - 2,  d,  6,  TFT_WHITE);
    }

    drawRadial(x0, y0, radius - 10, g_appWind.getBearing(), radius / 2 - 10, TFT_WHITE);

    // Apparent wind speed and angle at top
    g_tft.setTextColor(TFT_RED);
    drawJustifiedVal(g_appWind.getMagnitude(), 1, "", 0, 0, TL_DATUM);
    drawJustifiedVal(g_appWind.getSignedBearing(), 0, "~", DISPLAY_WIDTH, 0, TR_DATUM);

    // True wind speed and angle at bottom
    g_tft.setTextColor(TFT_BLUE);
    drawJustifiedVal(g_trueWind.getMagnitude(), 1, "", 0, DISPLAY_HEIGHT, BL_DATUM);
    drawJustifiedVal(g_trueWind.getSignedBearing(), 0, "~", DISPLAY_WIDTH, DISPLAY_HEIGHT, BR_DATUM);

    // Local SOG and COG in center of dial
    g_tft.setTextColor(TFT_YELLOW);
    drawJustifiedVal(g_localVelocity.getMagnitude(), 1, nullptr, x0, y0 - FONT_LARGE_ADJ_HEIGHT / 2, MC_DATUM);
    drawJustifiedVal(g_localVelocity.getBearing(), 0, "~", x0, y0 + FONT_LARGE_ADJ_HEIGHT / 2, MC_DATUM);

    // Depth to left of dial
    int precision = g_depth >= 100.0 ? 0 : 1;
    g_tft.setTextColor(TFT_GREEN);
    drawJustifiedVal(g_depth, precision, nullptr, 0, y0 - 10, ML_DATUM);

    // Draw labels and units in smallest font
    g_tft.setFreeFont(FONT_SMALL);

    g_tft.setTextColor(TFT_RED);
    g_tft.setTextDatum(TL_DATUM);
    g_tft.drawString("AWS", 0, FONT_LARGE_ADJ_HEIGHT);
    g_tft.setTextDatum(TR_DATUM);
    g_tft.drawString("AWA", DISPLAY_WIDTH, FONT_LARGE_ADJ_HEIGHT);

    g_tft.setTextColor(TFT_BLUE);
    g_tft.setTextDatum(BL_DATUM);
    g_tft.drawString("TWS", 0, DISPLAY_HEIGHT - FONT_LARGE_ADJ_HEIGHT);
    g_tft.setTextDatum(BR_DATUM);
    g_tft.drawString("TWA", DISPLAY_WIDTH, DISPLAY_HEIGHT - FONT_LARGE_ADJ_HEIGHT);

    g_tft.setTextColor(TFT_GREEN);
    g_tft.setTextDatum(ML_DATUM);
    g_tft.drawString(g_unitsDepth, 0, y0 + FONT_LARGE_ADJ_HEIGHT - 20);

    // Next page arrow center right
    drawArrowRight(DISPLAY_WIDTH - 15, DISPLAY_HEIGHT / 2);
}

// Helper function to draw a partial radial of a circle of radius r at x0, y0.
// Radial extends for len pixels towards center of circle.
void drawRadial(int x0,  int y0,  int r, int bearing, int len, uint16_t color) {
    double ax = sin(deg2Rad(bearing));
    double ay = -cos(deg2Rad(bearing));

    g_tft.drawLine(x0 + round(ax * r), y0 + round(ay * r),
                   x0 + round(ax * (r - len)), y0 + round(ay * (r - len)), color);
}

// Callback to calculate statistics of history data.
void statsHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                          size_t len, size_t offset) {
    HistStatsContext* ctx = (HistStatsContext *)user;

    for (unsigned int i = 0; i < len; i++) {
        if (ctx->bFirst) {
            // Don't allow max to be less than 1.0 to avoid divide by zero and
            // other scaling issues. This presumes that we're not interested in
            // zooming in on negative or very small data values.
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

    if (ctx->len && offset != 0) {
        // We're finished.
        ctx->avg = ctx->avg / (2 * ctx->len);
    }
}

// Callback to draw history data in the graph object provided.
void drawHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                         size_t len, size_t offset) {
    GraphWidget* p_gr = (GraphWidget*)user;
    uint16_t gr_x, gr_y;

    p_gr->getGraphPosition(&gr_x, &gr_y);

    for (unsigned int i = 0; i < len; i++) {
        int16_t x = p_gr->getPointX(offset + i);
        int16_t y = p_gr->getPointY(dataMax[i]);
        int32_t h = p_gr->getPointY(dataMin[i]) - y + 1;

        g_tft.drawFastVLine(x, y, h, TFT_WHITE);
    }
}


//
// Wind subpage to display data history detail.
//
void displaySubpageHistory(const char* title, const MinMaxDataHistory<double>* hist, uint16_t color) {
    struct HistStatsContext hist_stats;
    double maxVal = 0.0;
    GraphWidget graph(&g_tft);
    char buffer[64];

    g_tft.setTextDatum(TL_DATUM);
    g_tft.setTextWrap(false);
    g_tft.setFreeFont(FONT_LARGE);

    // Calculate history statistics.
    hist_stats.len = hist->getLength();
    hist->forEachData(statsHistoryCallback, &hist_stats);

    // Round max up to nearest decade
    maxVal = 10.0 * (int)ceil(hist_stats.max / 10.0);

    g_tft.fillScreen(TFT_BLACK);
#ifdef TESTING
    // Draw touch rects
    drawTouchRect(&g_subTitleRect);
#endif

    graph.createGraph(DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - FONT_LARGE_ADJ_HEIGHT - 1, TFT_BLACK);

    // x scale units is from 0 to 60mins, y scale unit is 0 to max val
    graph.setGraphScale(0.0, 60.0, 0.0, maxVal);
    graph.setGraphGrid(0.0, 10.0, 0.0, 10.0, color);

    graph.drawGraph(0, FONT_LARGE_ADJ_HEIGHT);

    // Title at top-left
    g_tft.setTextColor(color);
    g_tft.drawString(title, 0, 0);

    // Stats at top-right, but aligned to top of graph
    g_tft.setFreeFont(FONT_MEDIUM);
    g_tft.setTextDatum(BR_DATUM);
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "Min %.0f Max %.0f Avg %.0f",
             round(hist_stats.min), round(hist_stats.max), round(hist_stats.avg));
    g_tft.drawString(buffer, DISPLAY_WIDTH, FONT_LARGE_ADJ_HEIGHT - 1);

    // Draw history data
    hist->forEachData(drawHistoryCallback, &graph);
}

//
// Position info top-level page.
//
// This page is intended to provide essential info for making VHF calls or
// logbook entries.
//
void displayPagePosition() {
    char buffer[128];
    int row = 0;
    int offset = 0;
    char atmTemp[4] = "-";
    char seaTemp[4] = "-";
    unsigned int now;

    // Annoyingly, GFXFF fonts use the font baseline instead of top-left as the
    // cursor position. In addition the GNU FreeFonts used by GFXFF have a
    // significant line gap, which makes lines of text feel drafty and is less
    // compact. So for all these reasons, we abandon the use of setCursor and
    // println(), and instead manually keep track of our text rows and use a modified
    // font height.

    // We rely on using custom GFX fonts with '~' replaced by '°' so we can display bearings
    // without jumping through enormous hoops.
    g_tft.setTextDatum(TL_DATUM);
    g_tft.setTextWrap(false);
    g_tft.setFreeFont(FONT_MEDIUM);

    g_tft.fillScreen(TFT_BLACK);

#ifdef TESTING
    // Draw touch rects
    drawTouchRect(&g_timeRect);
    drawTouchRect(&g_logResetRect);
#endif

    g_tft.setTextColor(TFT_WHITE);

    // Current time
    now = g_secsSinceMidnight;
    if (g_tzOffset) {
        now += g_tzOffset * 60;
    }
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d %s",
             now / 3600, (now % 3600) / 60, now % 60,
             g_tzOffset ? "LOC" : "UTC");
    g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row += 2;

    // Atmospheric data in top-right
    g_tft.setTextDatum(TR_DATUM);

    // Temp
    if (!std::isnan(g_envAtmTemp)) {
        snprintf(atmTemp, sizeof(atmTemp), "%.0f", g_envAtmTemp);
    }
    if (!std::isnan(g_envSeaTemp)) {
        snprintf(seaTemp, sizeof(seaTemp), "%.0f", g_envSeaTemp);
    }

    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "%s/%s%s", atmTemp, seaTemp, g_unitsTemp);
    g_tft.drawString(buffer, DISPLAY_WIDTH, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row++;

    // Pressure
    if (!std::isnan(g_envAtmPressure)) {
        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "%.*f%s", g_precPressure, g_envAtmPressure,
                 g_unitsPressure);
        g_tft.drawString(buffer, DISPLAY_WIDTH, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    }
    row++;

    // Humidity
    if (!std::isnan(g_envAtmHumidity)) {
        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "%.0f%%", g_envAtmHumidity);
        g_tft.drawString(buffer, DISPLAY_WIDTH, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    }

    // Position below time on the left
    g_tft.setTextDatum(TL_DATUM);
    row = 2;

    // Position in cyan
    g_tft.setTextColor(TFT_CYAN);

    // GPS Position
    if (g_bPosValid) {
        buffer[0] = '\0';
        g_localPos.toString(buffer, sizeof(buffer), N2kPos::FMT_LAT_ONLY);
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        buffer[0] = '\0';
        g_localPos.toString(buffer, sizeof(buffer), N2kPos::FMT_LON_ONLY);
        g_tft.drawString(buffer, 0, (row + 1) * FONT_MEDIUM_ADJ_HEIGHT + offset);
    }
    row += 2;

    // Add another small gap for improved readability
    offset += 5;

    // Trip data
    buffer[0] = '\0';
    if (g_tripResetConfirmation) {
        snprintf(buffer, sizeof(buffer), "Press again to RESET");
    }
    else {
        snprintf(buffer, sizeof(buffer), "LOG %.1fnm  %u:%02uhrs",
                 g_tripLog.distance,
                 g_tripLog.duration / 3600, (g_tripLog.duration % 3600) / 60);
    }
    g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row++;

    g_tft.setTextColor(TFT_YELLOW);
    // Heading and course
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "HDG %.0f~    COG %.0f~",
             g_hdg, g_localVelocity.getBearing());
    g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row++;

    // SOG and trip average
    float avg = 0.0;
    buffer[0] = '\0';

    if (g_tripLog.duration > 0.0) {
        avg = g_tripLog.distance * 3600 / g_tripLog.duration;
    }
    snprintf(buffer, sizeof(buffer), "SOG %.1fkts  AVG %.1fkts",
             g_localVelocity.getMagnitude(), avg);
    g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row++;

    // Apparent wind angle and speed in red
    g_tft.setTextColor(TFT_RED);
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "AWS %.0fkts    AWA %.0f~",
             g_appWind.getMagnitude(), g_appWind.getSignedBearing());
    g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row++;

    // True wind direction and speed in blue
    g_tft.setTextColor(TFT_BLUE);
    N2kVector trueWind;
    trueWind = calcTrueWind(&g_appWind, g_localVelocity.getMagnitude(), g_hdg);
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "TWS %.0fkts    TWD %.0f~",
             trueWind.getMagnitude(), trueWind.getBearing());
    g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
    row++;


    // Next page arrow center right
    drawArrowRight(DISPLAY_WIDTH - 15, DISPLAY_HEIGHT / 2);
}

//
// Log entry subpage.
//
// Displays saved on-the-hour logbook data.
//
void displaySubpageLog() {
    char buffer[128];
    int row = 0;
    int offset = 0;
    char atmTemp[4] = "-";
    char seaTemp[4] = "-";
    const LogEntry* ent;

    g_tft.setTextDatum(TL_DATUM);
    g_tft.setTextWrap(false);
    g_tft.setFreeFont(FONT_MEDIUM);

    g_tft.fillScreen(TFT_BLACK);

#ifdef TESTING
    // Draw touch rects
    drawTouchRect(&g_subTitleRect);
#endif

    g_tft.setTextColor(TFT_YELLOW);
    ent = getLogEntry(g_selLogEntry);

    if (ent) {
        if (ent->tzOffset) {
            snprintf(buffer, sizeof(buffer), "%02d:%02d LOC",
                     (ent->timestamp + ent->tzOffset) / 60,
                     (ent->timestamp + ent->tzOffset) % 60);
        }
        else {
            snprintf(buffer, sizeof(buffer), "%02d:%02d UTC", ent->timestamp / 60,
                     ent->timestamp % 60);
        }
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row += 2;

        g_tft.setTextColor(TFT_WHITE);

        if (!std::isnan(ent->atmTemp)) {
            snprintf(atmTemp, sizeof(atmTemp), "%.0f", ent->atmTemp);
        }
        if (!std::isnan(ent->seaTemp)) {
            snprintf(seaTemp, sizeof(seaTemp), "%.0f", ent->seaTemp);
        }

        g_tft.setTextDatum(TR_DATUM);
        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "%s/%s%s", atmTemp, seaTemp, g_unitsTemp);
        g_tft.drawString(buffer, DISPLAY_WIDTH, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;

        if (!std::isnan(ent->atmPressure)) {
            buffer[0] = '\0';
            snprintf(buffer, sizeof(buffer), "%.*f%s", g_precPressure, ent->atmPressure,
                     g_unitsPressure);
            g_tft.drawString(buffer, DISPLAY_WIDTH, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
            row++;
        }

        g_tft.setTextDatum(TL_DATUM);
        row = 2;

        buffer[0] = '\0';
        ent->position.toString(buffer, sizeof(buffer), N2kPos::FMT_LAT_ONLY);
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;

        buffer[0] = '\0';
        ent->position.toString(buffer, sizeof(buffer), N2kPos::FMT_LON_ONLY);
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;

        // Add a small gap for improved readability
        offset += 5;

        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "LOG %.1fnm",
                 ent->logData.distance);
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;

        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "SOG %.1fkts COG %.0f~",
                 ent->velocity.getMagnitude(),
                 ent->velocity.getBearing());
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;

        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "AWS %.0fkts   AWA %.0f~",
                 ent->appWind.getMagnitude(),
                 ent->appWind.getSignedBearing());
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;

        N2kVector trueWind;
        trueWind = calcTrueWind(&ent->appWind, ent->velocity.getMagnitude(), ent->heading);

        buffer[0] = '\0';
        snprintf(buffer, sizeof(buffer), "TWS %.0fkts   TWD %.0f~",
                 trueWind.getMagnitude(),
                 trueWind.getBearing());
        g_tft.drawString(buffer, 0, row * FONT_MEDIUM_ADJ_HEIGHT + offset);
        row++;
    }
    else {
        g_tft.drawString("No log entries", 0, 0);
    }
}


N2kVector calcTrueWind(const N2kVector* appWind, double speed, double hdg)
{
    N2kVector boat(speed, hdg);
    N2kVector trueWindDir;

    // Calculate apparent wind direction.
    N2kVector appWindDir(appWind->getMagnitude(), appWind->getSignedBearing() + hdg);

    // Then subtract boat vector to get true wind direction. If hdg is 0, then this
    // is also TWA.
    trueWindDir.setXY(appWindDir.getX() - boat.getX(), appWindDir.getY() - boat.getY());

    return trueWindDir;
}

// Snapshots a new on-the-hour log entry.
bool addLogEntry()
{
    unsigned int timestamp = (unsigned int)g_secsSinceMidnight / 60;

    memset(&g_logEntries[g_logEntryIndex], 0, sizeof(struct LogEntry));

    g_logEntries[g_logEntryIndex].bInUse = true;

    g_logEntries[g_logEntryIndex].timestamp = timestamp; // Minutes since midnight
    g_logEntries[g_logEntryIndex].tzOffset = g_tzOffset; // Timezone offset in minutes
    g_logEntries[g_logEntryIndex].position = g_localPos;
    g_logEntries[g_logEntryIndex].heading = g_hdg;
    g_logEntries[g_logEntryIndex].velocity = g_localVelocity;
    g_logEntries[g_logEntryIndex].appWind = g_appWind;
    g_logEntries[g_logEntryIndex].logData = g_tripLog;
    g_logEntries[g_logEntryIndex].atmPressure = g_envAtmPressure;
    g_logEntries[g_logEntryIndex].atmTemp = g_envAtmTemp;
    g_logEntries[g_logEntryIndex].seaTemp = g_envSeaTemp;

    g_logEntryIndex = (g_logEntryIndex + 1) % NUM_LOG_ENTRIES;

    return true;
}

// Returns pointer to nth log entry in the past (0 represents the most recent).
const LogEntry* getLogEntry(unsigned int n) {
    unsigned int ix = (g_logEntryIndex + NUM_LOG_ENTRIES - (n + 1)) % NUM_LOG_ENTRIES;
    if (g_logEntries[ix].bInUse) {
        return &g_logEntries[ix];
    }
    return nullptr;
}


//
// AIS top-level page.
//
// North-up plot of AIS targets at various ranges defined by page ID.
// Vectors for each target show projected position in 5min.
//
// Dangerous targets are displayed in red (projected to pass within 1nm in the
// next 60mins).
//
// CPA distance and time (mins) is shown in top-right if vessel is
// converging on our position. Displayed in orange if passing ahead of us,
// displayed in red if we pass ahead of them (considered more dangerous).
//
void displayPageAis(time_t now) {
    const double radius = DISPLAY_HEIGHT / 2.0 - 7.0;
    int x0 = DISPLAY_WIDTH / 2;
    int y0 = DISPLAY_HEIGHT / 2;

    double range_scale = radius / g_aisRange;
    double vector_scale = range_scale / 12.0;;

    g_tft.setTextWrap(false);
    g_tft.setFreeFont(FONT_LARGE);

    g_tft.fillScreen(TFT_BLACK);

#ifdef TESTING
    // Draw touch rects
    drawTouchRect(&g_rangeRect);
    drawTouchRect(&g_selectRect);
    drawTouchRect(&g_detailRect);
#endif


    // Draw range circles.
    g_tft.drawCircle(x0, y0, round(g_aisRange * range_scale), TFT_WHITE);
    g_tft.drawCircle(x0, y0, round(g_aisRange * 0.67 * range_scale), TFT_WHITE);
    g_tft.drawCircle(x0, y0, round(g_aisRange * 0.33 * range_scale), TFT_WHITE);

    // Display page max range top-left.
    g_tft.setTextColor(TFT_WHITE);
    drawJustifiedVal(g_aisRange, 0, "nm", 0,  0, TL_DATUM);
    g_tft.setFreeFont(FONT_MEDIUM);

    // Local COG vector at center.
    g_tft.fillCircle(x0, y0, 3, TFT_YELLOW);
    g_tft.drawLine(x0, y0, x0 + round(g_localVelocity.getX() * vector_scale),
                 y0 - round(g_localVelocity.getY() * vector_scale), TFT_YELLOW);


    // Skip targets until we have local position
    if (g_bPosValid) {
        const N2kAISTarget* target = nullptr;

        for (auto& t : g_targets) {
            if (!isVisibleTarget(t.get())) {
                continue;
            }

            // Keep a pointer to the selected target and handle it later. We
            // draw it last so it's on top and stays visible.
            if (t->getMmsi() == g_selTarget) {
                target = t.get();
                continue;
            }

            const N2kVector &p = t->getRelDistance();
            const N2kVector &v = t->getVelocity();
            double cpad = t->getCpa()->getDistance();
            double cpat = t->getCpa()->getRelTime(now) / 60;
            bool bDangerous = isDangerousTarget(cpad, cpat);

            g_tft.fillCircle(x0 + round(p.getX() * range_scale),
                           y0 - round(p.getY() * range_scale),
                           3, bDangerous ? TFT_RED : TFT_GREEN);
            g_tft.drawLine(x0 + round(p.getX() * range_scale),
                         y0 - round(p.getY() * range_scale),
                         x0 + round(p.getX() * range_scale + v.getX() * vector_scale),
                         y0 - round(p.getY() * range_scale + v.getY() * vector_scale),
                         TFT_YELLOW);
        }
        if (target) {
            double cpad = target->getCpa()->getDistance();
            double cpab = target->getCpa()->getBearing();
            double cpat = target->getCpa()->getRelTime(now) / 60;
            bool bDangerous = isDangerousTarget(cpad, cpat);
            bool bTargetPassingAhead = target->getCpa()->getIntersectDeltaTime() < 0;
            const N2kVector &p = target->getRelDistance();
            const N2kVector &v = target->getVelocity();

            // Draw vessel info bottom-left
            g_tft.setTextColor(TFT_GREEN);
            drawJustifiedVal(v.getMagnitude(), 0, "kts", 0, DISPLAY_HEIGHT - FONT_MEDIUM_ADJ_HEIGHT * 2, BL_DATUM);
            drawJustifiedVal(p.getMagnitude(), 2, "nm", 0, DISPLAY_HEIGHT - FONT_MEDIUM_ADJ_HEIGHT, BL_DATUM);
            drawJustifiedVal(p.getBearing(), 0, "~", 0, DISPLAY_HEIGHT, BL_DATUM);

            // Draw CPA info top-right
            if (cpat >= 0.0 && !std::isnan(cpad)) {
                if (bDangerous) {
                    g_tft.setTextColor(bTargetPassingAhead ? TFT_ORANGE : TFT_RED);
                }
                drawJustifiedVal(cpad, 2, "nm", DISPLAY_WIDTH, 0, TR_DATUM);
                drawJustifiedVal(cpab, 0, "~", DISPLAY_WIDTH, FONT_MEDIUM_ADJ_HEIGHT, TR_DATUM);
                drawJustifiedVal(cpat, 1, "m", DISPLAY_WIDTH, FONT_MEDIUM_ADJ_HEIGHT * 2, TR_DATUM);
                g_tft.setFreeFont(FONT_SMALL);
                g_tft.drawString("CPA", DISPLAY_WIDTH, FONT_MEDIUM_ADJ_HEIGHT * 3);
            }

            // Draw vessel name bottom-right
            g_tft.setTextColor(TFT_GREEN);
            g_tft.setFreeFont(FONT_SMALL);
            g_tft.setTextDatum(BR_DATUM);
            g_tft.drawString(target->getName(), DISPLAY_WIDTH, DISPLAY_HEIGHT);

            // Draw selected vessel in white
            g_tft.fillCircle(x0 + round(p.getX()* range_scale),
                           y0 - round(p.getY()* range_scale),
                           3, TFT_WHITE);
            g_tft.drawLine(x0 + round(p.getX()* range_scale),
                         y0 - round(p.getY()* range_scale),
                         x0 + round(p.getX()* range_scale + v.getX()* vector_scale),
                         y0 - round(p.getY()* range_scale + v.getY()* vector_scale),
                         TFT_YELLOW);
        }
    }


    // Next page arrow center right
    drawArrowRight(DISPLAY_WIDTH - 15, DISPLAY_HEIGHT / 2);
}

// Fetch the existing N2kAISTarget object for the specified MMSI, or allocate a
// new object.
N2kAISTarget* getAISTarget(uint32_t mmsi) {
    for (auto& t : g_targets) {
        if (t->getMmsi() == mmsi) {
            return t.get();
        }
    }

    g_targets.push_back(std::make_unique<N2kAISTarget>(mmsi));
    return g_targets.back().get();
}

// Set which AIS target is currently selected.
void setTarget(const N2kAISTarget* p) {
    if (p) {
        //char buffer[128];
        //p->toString(buffer,  sizeof(buffer));
        //Serial.println(buffer);

        g_selTarget = p->getMmsi();
    }
    else {
        //Serial.println("none");
        g_selTarget = 0;
    }
}

// Is the AIS target visible on the current AIS page?
bool isVisibleTarget(const N2kAISTarget* t) {

    if (t->getTimestamp() == 0) {
        // Static data only (no position) so skip
        return false;
    }
    const N2kVector &p = t->getRelDistance();
    if (p.getMagnitude() > g_aisRange) {
        return false;
    }

    return true;
}

// Should the AIS target's closest point of approach (CPA) be considered dangerous?
bool isDangerousTarget(double d, double t) {
    if (!std::isnan(d) && !std::isnan(t)) {
        // Yes, if under 1nm within the next 60mins.
        return d < 1.0 && t > 0 && t < 60;
    }
    return false;
}


//
// AIS subpage to display detailed vessel info.
//
void displaySubpageAisInfo() {
    char buffer[128];
    int row = 0;
    int rowSmall = 0;

    g_tft.setTextWrap(false);
    g_tft.setTextDatum(TL_DATUM);
    g_tft.setFreeFont(FONT_MEDIUM);

    g_tft.fillScreen(TFT_BLACK);

#ifdef TESTING
    // Draw touch rects
    drawTouchRect(&g_subTitleRect);
#endif


    if (g_bPosValid) {
        for (auto& t : g_targets) {
            if (!isVisibleTarget(t.get())) {
                continue;
            }

            if (t->getMmsi() == g_selTarget) {
                static const int txtSize = 13;
                g_tft.setTextColor(TFT_GREEN);

                g_tft.drawString(t->getName(), 0, FONT_MEDIUM_ADJ_HEIGHT * row);
                row++;

                buffer[0] = '\0';
                snprintf(buffer, sizeof(buffer), "MMSI %u", t->getMmsi());
                g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row);
                row += 2;

                const N2kVector &relPos = t->getRelDistance();
                buffer[0] = '\0';
                snprintf(buffer, sizeof(buffer), "Range %.2f  Bearing %.0f~",
                         relPos.getMagnitude(), relPos.getBearing());
                g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row);
                row++;

                const N2kVector& v = t->getVelocity();

                buffer[0] = '\0';
                snprintf(buffer, sizeof(buffer), "SOG %.1fkts  COG %.0f~",
                         v.getMagnitude(), v.getBearing());
                g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row);
                row++;

                buffer[0] = '\0';
                snprintf(buffer, sizeof(buffer), "LOA %.1fft",
                         t->getLength());
                g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row);
                row++;

                buffer[0] = '\0';
                snprintf(buffer, sizeof(buffer), "Beam %.1fft  Draft %.1fft",
                         t->getBeam(), t->getDraft());
                g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row);
                row++;

                if (t->getClass() == N2kAISTarget::CLASS_A) {
                    g_tft.setFreeFont(FONT_SMALL);

                    buffer[0] = '\0';
                    snprintf(buffer, sizeof(buffer), "Type: %s",
                             t->getVesselTypeStr());
                    g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row + 5 +
                                     FONT_SMALL_ADJ_HEIGHT * rowSmall);
                    rowSmall++;

                    buffer[0] = '\0';
                    snprintf(buffer, sizeof(buffer), "Dest: %s",
                             t->getDest());
                    g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row + 5 +
                                     FONT_SMALL_ADJ_HEIGHT * rowSmall);
                    rowSmall++;

                    buffer[0] = '\0';
                    snprintf(buffer, sizeof(buffer), "Status: %s", t->getNavStatusStr());
                    g_tft.drawString(buffer, 0, FONT_MEDIUM_ADJ_HEIGHT * row + 5 +
                                     FONT_SMALL_ADJ_HEIGHT * rowSmall);
                    rowSmall++;
                }

                break;
            }
        }
    }
}

#ifdef TESTING
void drawTouchRect(const TouchRect* rect) {
    int16_t x, y;
    uint16_t w, h;

    rect->getBoundingRect(&x, &y, &w, &h);
    g_tft.drawRect(x, y, w, h, TFT_LIGHTGREY);
}
#endif


// Redraw the display.
//
// This gets called any time a new message is received that updates our data model.
// To avoid excessive calls and the resulting flicker, we avoid updating the
// display if it has been recently redrawn unless the caller forces a redraw.
void displayUpdate(bool bForce) {

    time_t now = time(nullptr);
    time_t diff = now - g_lastUpdate;

    if (bForce || diff >= UPDATE_INTERVAL) {
        // Active page is defined by g_page and g_subpage
        switch (g_subpage) {
            case SUBPAGE_NONE:
                switch (g_page) {
                    case PAGE_WIND:
                        displayPageWind();
                        break;

                    case PAGE_POSITION:
                        displayPagePosition();
                        break;

                    case PAGE_AIS:
                        displayPageAis(now);
                        break;
                }
                break;

            case SUBPAGE_HIST_AWS:
                displaySubpageHistory("AWS", &g_awsHistory, TFT_RED);
                break;

            case SUBPAGE_HIST_TWS:
                displaySubpageHistory("TWS", &g_twsHistory, TFT_BLUE);
                break;

            case SUBPAGE_HIST_SOG:
                displaySubpageHistory("SOG", &g_sogHistory, TFT_YELLOW);
                break;

            case SUBPAGE_HIST_DEPTH:
                displaySubpageHistory("Depth", &g_depthHistory, TFT_GREEN);
                break;

            case SUBPAGE_POS_LOG:
                displaySubpageLog();
                break;

            case SUBPAGE_AIS_INFO:
                displaySubpageAisInfo();
                break;
        }

        g_lastUpdate = now;
    }
}

// PGN 128267: Water Depth 33.8ft (10.3m)
void handlePgn128267Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    double val1, val2;

    if (ParseN2kWaterDepth(N2kMsg, sid, val1, val2)) {
        if (!N2kIsNA(val1) && !N2kIsNA(val2)) {
#ifdef USE_METRIC
            g_depth = val1 + val2;
#else
            g_depth = meters2Ft(val1 + val2);
#endif
        }
    }
}

// PGN 129025: Position 36°51.054'S 174°45.870'W
void handlePgn129025Msg(const tN2kMsg &N2kMsg) {
    double val1, val2;

    if (ParseN2kPositionRapid(N2kMsg, val1, val2)) {
        if (!N2kIsNA(val1) && !N2kIsNA(val2)) {
            g_localPos.set(val1, val2);

            bool bIsFirstPos = !g_bPosValid;
            g_bPosValid = true;

            // If this is the first time we've received our position, update the CPA
            // of all AIS targets and take a log history snapshot.
            if (bIsFirstPos) {
                g_lastPos = g_localPos;
                for (auto& t : g_targets) {
                    t->calcCpa(g_localPos, g_localVelocity);
                }

                // If we already have the time then take an initial log history snapshot
                // once we have a position.
                if (g_hourTimer.isEnabled()) {
                    addLogEntry();
                }
            }
        }
    }
}

// PGN 129026: COG 116T, Speed 0.2kts
void handlePgn129026Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    double val1, val2;
    tN2kHeadingReference ref;

    if (ParseN2kCOGSOGRapid(N2kMsg, sid, ref, val1, val2)) {
        if (ref == 0 && !N2kIsNA(val1) && !N2kIsNA(val2)) {
#ifdef TESTING
            static double fake_sog = 2.0;
            fake_sog += 0.25 - 0.5 * rand() / RAND_MAX;
            fake_sog = std::min(3.0, fake_sog);
            fake_sog = std::max(1.0, fake_sog);
            val2 += fake_sog;
#endif
            g_localVelocity.set(metersPerSec2Kts(val2), rad2Deg(val1));
        }
    }
}

// PGN 129033 TimeDate 2025/11/5T14:33:01.234-04:00
void handlePgn129033Msg(const tN2kMsg &N2kMsg) {
    uint16_t date = 0;
    double secsSinceMidnight = 0.0;
    int16_t tzOffset = 0;

    if (ParseN2kLocalOffset(N2kMsg, date, secsSinceMidnight, tzOffset)) {
        if (date != 0xffff && !N2kIsNA(secsSinceMidnight)) {
            g_date = date;
            g_secsSinceMidnight = secsSinceMidnight;

            if (tzOffset != 0x7fff) {
                // TZ offset in minutes
                g_tzOffset = tzOffset;
            }

            if (!g_hourTimer.isEnabled()) {
                int hrs = (int)trunc(secsSinceMidnight / 3600) + 1;
                int remaining = 3600 * hrs - (int)trunc(secsSinceMidnight) + 30;
                g_hourTimer.begin(&g_hourTimer, remaining * 1000, logEntryCallback);

                // If we already have GPS position and we just got time, then take
                // an initial log history snapshot.
                if (g_bPosValid) {
                    addLogEntry();
                }
            }
        }
    }
}

// PGN 129038: AIS Class A Position MMSI 367513040, 42°21.699'N 71°2.490'W, COG 151T, Speed 8.9kts
void handlePgn129038Msg(const tN2kMsg &N2kMsg) {
    uint8_t mid;
    tN2kAISRepeat rep;
    uint32_t mmsi;
    double latitude;
    double longitude;
    bool accuracy;
    bool raim;
    uint8_t seconds;
    double aisCog;
    double aisSog;
    double aisHdg;
    double aisRot;
    tN2kAISNavStatus status;

    if (ParseN2kAISClassAPosition(N2kMsg, mid, rep, mmsi, latitude, longitude,
                                  accuracy, raim, seconds,
                                  aisCog, aisSog, aisHdg, aisRot, status)) {
        N2kAISTarget* target = getAISTarget(mmsi);

        N2kPos p;
        N2kVector v;

        if (!N2kIsNA(aisSog) && !N2kIsNA(aisCog)) {
            v.set(metersPerSec2Kts(aisSog), rad2Deg(aisCog));
        }
        if (!N2kIsNA(latitude) && !N2kIsNA(longitude)) {
            p.set(latitude, longitude);
            target->update(p, v, (N2kAISTarget::NavStatus)status);

            if (g_bPosValid) {
                target->calcCpa(g_localPos, g_localVelocity);
            }
        }
    }
}

// PGN 129039: AIS Class B Position MMSI 367739760, 42°20.441'N 71°0.612'W, COG 133T, Speed 8.1kts
void handlePgn129039Msg(const tN2kMsg &N2kMsg) {
    uint8_t mid;
    tN2kAISRepeat rep;
    uint32_t mmsi;
    double latitude;
    double longitude;
    bool accuracy;
    bool raim;
    uint8_t seconds;
    double aisCog;
    double aisSog;
    double aisHdg;
    tN2kAISUnit unit;
    bool display;
    bool dsc;
    bool band;
    bool msg22;
    tN2kAISMode mode;
    bool state;

    if (ParseN2kAISClassBPosition(N2kMsg, mid, rep, mmsi, latitude, longitude,
                                  accuracy, raim, seconds,
                                  aisCog, aisSog, aisHdg,
                                  unit, display, dsc, band, msg22, mode, state)) {
        N2kAISTarget* target = getAISTarget(mmsi);
        N2kPos p;
        N2kVector v;

        if (!N2kIsNA(aisSog) && !N2kIsNA(aisCog)) {
            v.set(metersPerSec2Kts(aisSog), rad2Deg(aisCog));
        }
        if (!N2kIsNA(latitude) && !N2kIsNA(longitude)) {
            p.set(latitude, longitude);
            target->update(p, v);

            if (g_bPosValid) {
                target->calcCpa(g_localPos, g_localVelocity);
            }
        }
    }
}

// PGN 129794: AIS Class A Static Info MMSI 367513030, Name INDEPENDENCE, Callsign WDG2186, Type 52, Length 39.0ft, Beam 13.0ft, Draft 6.1ft, Dest BOSTON
void handlePgn129794Msg(const tN2kMsg &N2kMsg) {
    uint8_t mid;
    tN2kAISRepeat rep;
    uint32_t mmsi;
    uint32_t imo;
    char callsign[10];
    char name[32];
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
}

// PGN 129809: AIS Class B Static Info1 MMSI 338444184, Name MENHADEN
void handlePgn129809Msg(const tN2kMsg &N2kMsg) {
    uint8_t mid;
    tN2kAISRepeat rep;
    uint32_t mmsi;
    char name[32];

    if (ParseN2kAISClassBStaticPartA(N2kMsg, mid, rep, mmsi, name, sizeof(name))) {
        N2kAISTarget* target = getAISTarget(mmsi);
        target->update(name);
    }
}

// PGN 129810: AIS Class B Static Info2 MMSI 368310310, Type 37, Callsign , Length 20.0ft, Beam 7.0ft
void handlePgn129810Msg(const tN2kMsg &N2kMsg) {
    uint8_t mid;
    tN2kAISRepeat rep;
    uint32_t mmsi;
    uint8_t vesselType;
    char vendor[32];
    char callsign[10];
    double length;
    double beam;
    double posStbd;
    double posBow;
    uint32_t mothership;

    if (ParseN2kAISClassBStaticPartB(N2kMsg, mid, rep, mmsi, vesselType,
                                     vendor, sizeof(vendor), callsign, sizeof(callsign),
                                     length, beam, posStbd, posBow, mothership)) {
        N2kAISTarget* target = getAISTarget(mmsi);

        target->update(vesselType, length,
                       beam, 0,
                       callsign, nullptr, nullptr);
    }
}

// PGN 130306: AWS 12.2kts, AWA -31
void handlePgn130306Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    double val1, val2;
    tN2kWindReference ref2;

    if (ParseN2kWindSpeed(N2kMsg, sid, val1, val2, ref2)) {
        if (ref2 == N2kWind_Apparent && !N2kIsNA(val1) && !N2kIsNA(val2)) {
#ifdef TESTING
            static double fake_aws = 2.0;
            fake_aws += 0.5 - (double)rand() / RAND_MAX;
            fake_aws = std::min(6.0, fake_aws);
            fake_aws = std::max(1.0, fake_aws);
            val1 += fake_aws;
#endif
            g_appWind.set(metersPerSec2Kts(val1), rad2Deg(val2));

            // Calculate true wind, relative to boat
            g_trueWind = calcTrueWind(&g_appWind, g_localVelocity.getMagnitude(), 0);
        }
    }
}

// PGN 127250: HDG 43, Deviation ?, Variation ?, magnetic.
void handlePgn127250Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    double heading = 0.0;
    double deviation = 0.0;
    double variation = 0.0;
    tN2kHeadingReference ref;

    if (ParseN2kHeading(N2kMsg, sid, heading, deviation, variation, ref)) {
        if (!N2kIsNA(heading)) {
            // What to do about conflicting sources of heading info?
            if (ref == N2khr_magnetic) {
                if (N2kIsNA(variation)) {
                    variation = g_variation;
                } else {
                    g_variation = variation;
                }
                if (N2kIsNA(deviation)) {
                    deviation = 0.0;
                }
                g_hdg = normalizeBearing(rad2Deg(heading + deviation + variation));
            }
            else {
                g_hdg = rad2Deg(heading);
            }
        }
    }
}

// PGN 127258: Variation -14.0
void handlePgn127258Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    tN2kMagneticVariation src;
    uint16_t age = 0;
    double variation = 0.0;

    if (ParseN2kMagneticVariation(N2kMsg, sid, src, age, variation)) {
        if (!N2kIsNA(variation)) {
            g_variation = variation;
        }
    }
}

// PGN 130312: Sea Temp 67.1F
void handlePgn130312Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    unsigned char instance = 0;
    tN2kTempSource tempSrc = N2kts_SeaTemperature;
    double actualTemp = 0.0;
    double setTemp = 0.0;

    if (ParseN2kTemperature(N2kMsg, sid, instance, tempSrc, actualTemp, setTemp)) {
        if (!N2kIsNA(actualTemp)) {
            switch (tempSrc) {
                case N2kts_SeaTemperature:
#ifdef USE_METRIC
                    g_envSeaTemp = KelvinToC(actualTemp);
#else
                    g_envSeaTemp = KelvinToF(actualTemp);
#endif
                    break;

                case N2kts_OutsideTemperature:
                case N2kts_InsideTemperature:
#ifdef USE_METRIC
                    g_envAtmTemp = KelvinToC(actualTemp);
#else
                    g_envAtmTemp = KelvinToF(actualTemp);
#endif
                    break;

                default:
                    break;
            }
        }
    }
}

void handlePgn130314Msg(const tN2kMsg &N2kMsg) {
    unsigned char sid = 0;
    unsigned char instance = 0;
    tN2kPressureSource pressureSrc = N2kps_Atmospheric;
    double pressure = 0.0;

    if (ParseN2kPressure(N2kMsg, sid, instance, pressureSrc, pressure)) {
        if (!N2kIsNA(pressure)) {
            switch (pressureSrc) {
                case N2kps_Atmospheric:
#ifdef USE_METRIC_PRESSURE
                    g_envAtmPressure = PascalTomBar(pressure);
#else
                    g_envAtmPressure = millibars2inHg(PascalTomBar(pressure));
#endif
                    break;

                default:
                    break;
            }
        }
    }
}





// Process incoming N2K messages to update our data model.
void handleNMEA2000Msg(const tN2kMsg &N2kMsg) {

  switch (N2kMsg.PGN) {
    // Handle PGNs we are interested in
    case 127250:
        handlePgn127250Msg(N2kMsg);
        break;

    case 127258:
        handlePgn127258Msg(N2kMsg);
        break;

    case 128267:
        handlePgn128267Msg(N2kMsg);
        break;
      
    case 129025:
        handlePgn129025Msg(N2kMsg);
      break;

    case 129026:
        handlePgn129026Msg(N2kMsg);
      break;
      
    case 129033:
        handlePgn129033Msg(N2kMsg);
        break;

    case 129038:
        handlePgn129038Msg(N2kMsg);
        break;

    case 129039:
        handlePgn129039Msg(N2kMsg);
        break;

    case 129794:
        handlePgn129794Msg(N2kMsg);
        break;

    case 129809:
        handlePgn129809Msg(N2kMsg);
        break;

    case 129810:
        handlePgn129810Msg(N2kMsg);
        break;

    case 130312:
        handlePgn130312Msg(N2kMsg);
        break;

    case 130314:
        handlePgn130314Msg(N2kMsg);
        break;

    case 130306:
        handlePgn130306Msg(N2kMsg);
        break;

    default:
        // Not interested. Do nothing.
        return;
  }

  displayUpdate(false);
}
