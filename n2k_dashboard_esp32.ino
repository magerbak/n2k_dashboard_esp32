/**************************************************************************
  Monitors NMEA2000 bus and displays a simple dashboard of useful info.

  Works with the Adafruit ESP32-S3 Reverse TFT Feather
    ----> https://www.adafruit.com/products/5691

  Uses the Adafruit GFX library and the ST7789 display driver.
  Uses the NEMA2000 library for parsing N2K communications using the ESP32
  internal CAN controller (external transceiver required).

 **************************************************************************/
//#define TESTING

#include <limits>
#include <list>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "N2kMsg.h"
#include "NMEA2000.h"
#define ESP32_CAN_TX_PIN GPIO_NUM_6
#define ESP32_CAN_RX_PIN GPIO_NUM_5
#include <NMEA2000_esp32.h> // Custom NMEA2000_esp32 support for S3 (https://github.com/offspring/NMEA2000_esp32)
#include <N2kMessages.h>

#include "debounced_button.h"
#include "simple_timer.h"
#include "data_history.h"
#include "n2kvector.h"
#include "n2kpos.h"
#include "n2kunits.h"
#include "n2kaistarget.h"

#define UPDATE_INTERVAL  1
#define AIS_TIMEOUT      60

// Interval that min/max data is sampled.
#define HISTORY_SAMPLE_INTERVAL_MS          1000
// Interval over which sampled min/max data is aggregated per data point in history.
#ifdef TESTING
  #define HISTORY_AGGREGATION_INTERVAL_MS   (10 * 1000)
#else
  #define HISTORY_AGGREGATION_INTERVAL_MS   (60 * 1000)
#endif
// Default number of data points in history - can be customized in setup().
// Total duration of history = num data points * HISTORY_AGGREGATION_INTERVAL_MS
#define HISTORY_DEF_NUM_DATA_POINTS         60

tNMEA2000 &g_NMEA2000=*(new tNMEA2000_esp32());

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  135

// Use dedicated hardware SPI pins
Adafruit_ST7789 g_tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Display page
enum Page {
    PAGE_WIND,
    PAGE_POSITION,
    PAGE_AIS_12NM,
    PAGE_AIS_4NM,
    PAGE_AIS_1NM,
    NUM_PAGES
} g_page = PAGE_WIND;

enum Subpage {
    SUBPAGE_NONE,

    SUBPAGE_HIST_TWS,
    SUBPAGE_HIST_AWS,
    SUBPAGE_HIST_SOG,
    SUBPAGE_HIST_DEPTH,
    SUBPAGE_HIST_SEPARATOR,

    SUBPAGE_AIS_INFO,
} g_subpage = SUBPAGE_NONE;

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

struct HistWindowContext {
    uint16_t color;
    // Position of top-left of history window.
    int16_t x;
    int16_t y;
    // Width and height of history window.
    int16_t w;
    int16_t h;

    unsigned int range_x;   // Range of x axis
    unsigned int range_y;   // Range of y axis
};

struct HistStatsContext {
    bool bFirst = true;

    size_t len = 0;     // Num samples of history

    // Stats for current history samples.
    double min = 0.0;
    double max = 1.0;
    double avg = 0.0;
};


bool g_bPosValid = false;
N2kPos g_localPos;          // Latitude and Longitude
N2kVector g_localVelocity;  // Boat COG (in degrees) and SOG (kts) as a vector
double g_hdg = 0.0;         // degrees

double g_depth = 0.0;       // ft
N2kVector g_appWind;        // Degrees, knots
N2kVector g_trueWind;       // Degrees, knots

// AIS state
std::list<N2kAISTarget *> g_targets;
uint32_t g_selTarget = 0;   // MMSI of selected AIS target

// UI buttons. On the Adafruit ESP32-S3 reverse TFT feather, these are on pins 0, 1 and 2.
DebouncedButton g_buttonD0(0, LOW);
DebouncedButton g_buttonD1(1);
DebouncedButton g_buttonD2(2);

// 1s timer for refreshing data model (ageing out AIS targets, sampling data history).
SimpleTimer g_updateTimer;

// 60s timer interval for recording aggregate min/max history data.
SimpleTimer g_historyTimer;

MinMaxDataHistory<double> g_awsHistory;
MinMaxDataHistory<double> g_twsHistory;
MinMaxDataHistory<double> g_sogHistory;
MinMaxDataHistory<double> g_depthHistory;
//MinMaxDataHistory<double> g_voltageHistory;

// Time since last display update in millis.
uint16_t g_lastUpdate = 0;

// String just containing the degree symbol from the codepage 437 character set.
static const char g_degStr[] = { 0xf8, '\0' };

void handleButtonEvents(Event e);
void handlePageButtonEvents(Event e);
void handleSubpageButtonEvents(Event e);

void displaySplashScreen();
void drawJustifiedText(const char* str, int x, int y, TextLayout fmt);
void drawJustifiedVal(double val, int precision, const char* suffix, int x, int y, TextLayout fmt);

void displayPageWind();
void drawRadial(int x0,  int y0,  int r, int bearing, int len, uint16_t color);
void statsHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                          size_t len, size_t offset);
void drawHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                         size_t len, size_t offset);
void displaySubpageHistory(const char* title, const MinMaxDataHistory<double>* hist, uint16_t color);

void displayPagePosition();

void displayPageAis(Page pg, time_t now);
N2kAISTarget* getAISTarget(uint32_t mmsi);
void setTarget(const N2kAISTarget* p);
bool isVisibleTarget(const N2kAISTarget* t, Page pg);
bool isDangerousTarget(double d, double t);
void displaySubpageAisInfo();

void displayUpdate(bool bForce);

void handlePgn128267Msg(const tN2kMsg &N2kMsg);
void handlePgn129025Msg(const tN2kMsg &N2kMsg);
void handlePgn129026Msg(const tN2kMsg &N2kMsg);
void handlePgn129038Msg(const tN2kMsg &N2kMsg);
void handlePgn129039Msg(const tN2kMsg &N2kMsg);
void handlePgn129794Msg(const tN2kMsg &N2kMsg);
void handlePgn129809Msg(const tN2kMsg &N2kMsg);
void handlePgn129810Msg(const tN2kMsg &N2kMsg);
void handlePgn130306Msg(const tN2kMsg &N2kMsg);
void handlePgn127250Msg(const tN2kMsg &N2kMsg);

void handleNMEA2000Msg(const tN2kMsg &N2kMsg);


void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Starting N2K dashboard"));

  g_buttonD0.begin();
  g_buttonD1.begin();
  g_buttonD2.begin();

  g_updateTimer.begin(nullptr, HISTORY_SAMPLE_INTERVAL_MS, updateCallback);
  g_historyTimer.begin(nullptr, HISTORY_AGGREGATION_INTERVAL_MS, historyCallback);

  // Record history of AWS, TWS and SOG for last 60mins.
  g_awsHistory.begin(60);
  g_twsHistory.begin(60);
  g_sogHistory.begin(60);
  g_depthHistory.begin(60);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  g_tft.init(DISPLAY_HEIGHT, DISPLAY_WIDTH);
  g_tft.cp437(true);    // Use correct code page 437 indices
  g_tft.setRotation(3);

  displaySplashScreen();
  delay(1000);
  Serial.println(F("TFT Initialized"));

  //NMEA2000.SetN2kCANMsgBufSize(8);
  //NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  g_NMEA2000.EnableForward(false);  // Disable all msg forwarding to USB (=Serial)
  g_NMEA2000.SetMsgHandler(handleNMEA2000Msg);
  g_NMEA2000.Open();
  Serial.println(F("N2k Initialized"));
}

void loop() {
    uint32_t t = millis();

    // Drive our timers
    g_updateTimer.tick(t);
    g_historyTimer.tick(t);

    // Poll buttons and drive UI
    if (g_buttonD0.updateState()) {
        handleButtonEvents(g_buttonD0.isPressed() ? EVT_D0_PRESS : EVT_D0_RELEASE);
    }
    if (g_buttonD1.updateState()) {
        handleButtonEvents(g_buttonD1.isPressed() ? EVT_D1_PRESS : EVT_D1_RELEASE);
    }
    if (g_buttonD2.updateState()) {
        handleButtonEvents(g_buttonD2.isPressed() ? EVT_D2_PRESS : EVT_D2_RELEASE);
    }

    // Handle incoming N2K messages.
    g_NMEA2000.ParseMessages();
}

void handleButtonEvents(Event e) {
    // A top level page is displayed when no subpage is active.
    if (g_subpage == SUBPAGE_NONE) {
        handlePageButtonEvents(e);
    }
    else {
        handleSubpageButtonEvents(e);
    }
}

// Handle button events according to which top level page is active.
void handlePageButtonEvents(Event e) {
    const N2kAISTarget* last = nullptr;
    bool bNeedUpdate = false;
    int p;

    // For top-level pages, D2 advances to the next page.
    if (e == EVT_D2_PRESS) {
        p = (int)g_page + 1;
        if (p == NUM_PAGES) {
            p = 0;
        }
        g_page = (Page)p;
        bNeedUpdate = true;
    }
    else {
        switch (g_page) {
            case PAGE_WIND:
                switch (e) {
                    case EVT_D1_PRESS:
                        // Switch to first history subpage
                        g_subpage = SUBPAGE_HIST_TWS;
                        bNeedUpdate = true;
                        break;

                    default:
                        break;
                }
                break;

            case PAGE_AIS_12NM:
            case PAGE_AIS_4NM:
            case PAGE_AIS_1NM:
                if (g_targets.size() > 0) {
                    switch (e) {
                        case EVT_D0_PRESS:
                            // Cycle forward through targets
                            for (auto t : g_targets) {
                                if (!isVisibleTarget(t, g_page)) {
                                    continue;
                                }
                                if (g_selTarget == 0 || (last && last->getMmsi() == g_selTarget)) {
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
                            if (g_selTarget) {
                                g_subpage = SUBPAGE_AIS_INFO;
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

done:
    if (bNeedUpdate) {
        displayUpdate(true);
    }
}

// Handle button events according to which subpage is active.
void handleSubpageButtonEvents(Event e) {
    const N2kAISTarget* last = nullptr;
    bool bNeedUpdate = false;
    int p;

    // For all subpages, D1 exits back to parent top-level page.
    if (e == EVT_D1_PRESS) {
        g_subpage = SUBPAGE_NONE;
        bNeedUpdate = true;
    }
    else {
        switch (g_subpage) {
            case SUBPAGE_HIST_TWS:
            case SUBPAGE_HIST_AWS:
            case SUBPAGE_HIST_SOG:
            case SUBPAGE_HIST_DEPTH:
                switch (e) {
                    // Cycle through history subpages.
                    case EVT_D0_PRESS:
                        p = (int)g_subpage + 1;
                        if (p == SUBPAGE_HIST_SEPARATOR) {
                            p = SUBPAGE_HIST_TWS;
                        }
                        g_subpage = (Subpage)p;
                        bNeedUpdate = true;
                        break;

                    default:
                        break;
                }
                break;

            case SUBPAGE_AIS_INFO:
                switch (e) {
                    // Show info for the next visible target on the parent AIS page.
                    case EVT_D0_PRESS:
                        for (auto t : g_targets) {
                            if (!isVisibleTarget(t, g_page)) {
                                continue;
                            }
                            if (g_selTarget == 0 || (last && last->getMmsi() == g_selTarget)) {
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
                break;


            default:
                break;
        }
    }

done:
    if (bNeedUpdate) {
        displayUpdate(true);
    }
}

// Time-based refresh of our data model. This may update the display based on
// what changes.
bool updateCallback(void* user) {
    bool bChanged = false;
    time_t now = time(nullptr);
    time_t diff = now - g_lastUpdate;

    (void)user;

    // Age out stale AIS targets
    for (auto t = g_targets.begin(); t != g_targets.end();) {
        if ((*t)->getTimestamp() && now - (*t)->getTimestamp() > AIS_TIMEOUT) {
            if ((*t)->getMmsi() == g_selTarget) {
                setTarget(nullptr);
            }
            free(*t);
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

    // Continue running
    return true;
}

void displaySplashScreen() {
  g_tft.setTextWrap(false);
  g_tft.fillScreen(ST77XX_BLACK);
  g_tft.setCursor(0, 60);

  g_tft.setTextColor(ST77XX_GREEN);
  g_tft.setTextSize(3);
  g_tft.println("Hello Michael");
}

// Helper function to print text using a calculated starting position based on
// its length and a layout option.
void drawJustifiedText(const char* str, int x, int y, TextLayout fmt) {
    int16_t minx;
    int16_t miny;
    uint16_t w;
    uint16_t h;

    g_tft.getTextBounds(str, 0, 0, &minx, &miny, &w, &h);
    switch (fmt) {
        // x and y are either 0 or the rightmost/bottommost extent of the justified
        // text respectively. This allows text to be placed flush with the edge of
        // the display in all 4 corners.
        case TXT_JUSTIFIED:
            if (x) {
                x -= w + 1;
            }
            if (y) {
                y -= h + 1;
            }
            break;

        case TXT_CENTERED:
            // Note y is not adjusted, to allow manual vertical placement.
            x -= (w + 1) / 2;
            break;
    }
    g_tft.setCursor(x, y);
    g_tft.print(str);
}

// Helper function to print a floating point value using a calculated starting
// position based on its displayed length and a layout option.
void drawJustifiedVal(double val, int precision, const char* suffix, int x, int y, TextLayout fmt) {
    char str[64];

    if (!suffix) {
        suffix = "";
    }

    int rc = snprintf(str, sizeof(str), "%.*f%s", precision, val, suffix);
    drawJustifiedText(str, x, y, fmt);
}


//
// Wind info top-level page.
//
// D1 switches to history subpages.
// D2 cycles to next top-level page.
void displayPageWind() {
    struct HistWindowContext hist_window;

    int x0 = DISPLAY_WIDTH / 2;
    int y0 = DISPLAY_HEIGHT / 2;

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    // TWS history top right
    hist_window.color = ST77XX_BLUE;
    hist_window.x = DISPLAY_WIDTH - g_twsHistory.getLength();
    hist_window.y = 30;
    hist_window.w = 60;
    hist_window.h = 30;
    hist_window.range_x = g_twsHistory.getSize();
    hist_window.range_y = hist_window.h;
    g_twsHistory.forEachData(drawHistoryCallback, &hist_window);

    // AWS history bottom right
    hist_window.color = ST77XX_RED;
    hist_window.x = DISPLAY_WIDTH - g_twsHistory.getLength();
    hist_window.y = 70;
    hist_window.w = 60;
    hist_window.h = 30;
    hist_window.range_x = g_awsHistory.getSize();
    hist_window.range_y = hist_window.h;
    g_awsHistory.forEachData(drawHistoryCallback, &hist_window);

    g_tft.startWrite();
    // Draw port quadrants in red
    g_tft.drawCircleHelper(x0, y0, 60, 0x1, ST77XX_RED);
    g_tft.drawCircleHelper(x0, y0, 60, 0x8, ST77XX_RED);
    g_tft.drawCircleHelper(x0, y0, 59, 0x1, ST77XX_RED);
    g_tft.drawCircleHelper(x0, y0, 59, 0x8, ST77XX_RED);
    // Draw starboard quadrants in green
    g_tft.drawCircleHelper(x0, y0, 60, 0x2, ST77XX_GREEN);
    g_tft.drawCircleHelper(x0, y0, 60, 0x4, ST77XX_GREEN);
    g_tft.drawCircleHelper(x0, y0, 59, 0x2, ST77XX_GREEN);
    g_tft.drawCircleHelper(x0, y0, 59, 0x4, ST77XX_GREEN);
    g_tft.endWrite();

    for (int d = 0; d < 360; d += 10) {
        drawRadial(x0,  y0,  58,  d,  2,  ST77XX_WHITE);
    }
    for (int d = 0; d < 360; d += 30) {
        drawRadial(x0,  y0,  58,  d,  6,  ST77XX_WHITE);
    }

    drawRadial(x0, y0, 50, g_appWind.getBearing(), 20, ST77XX_WHITE);

    // True wind angle and speed at top
    g_tft.setTextColor(ST77XX_BLUE);
    g_tft.setTextSize(3);
    g_tft.setCursor(0, 0);
    g_tft.print(g_trueWind.getSignedBearing(), 0);
    g_tft.println((char)0xf8);
    g_tft.setTextSize(1);
    g_tft.print("TWA");

    g_tft.setTextSize(3);
    drawJustifiedVal(g_trueWind.getMagnitude(), 1, nullptr, DISPLAY_WIDTH, 0, TXT_JUSTIFIED);

    // Apparent wind angle and speed at bottom
    g_tft.setTextColor(ST77XX_RED);
    g_tft.setTextSize(1);
    g_tft.setCursor(0, 102);
    g_tft.print("AWA");
    g_tft.setTextSize(3);
    g_tft.setCursor(0, 112);
    g_tft.print(g_appWind.getSignedBearing(), 0);
    g_tft.print((char)0xf8);

    g_tft.setTextSize(3);
    drawJustifiedVal(g_appWind.getMagnitude(), 1, nullptr, DISPLAY_WIDTH, DISPLAY_HEIGHT, TXT_JUSTIFIED);

    // Local SOG and COG in center of dial
    g_tft.setTextColor(ST77XX_YELLOW);
    g_tft.setTextSize(2);
    drawJustifiedVal(g_localVelocity.getMagnitude(), 1, nullptr, x0, y0 - 15, TXT_CENTERED);
    drawJustifiedVal(g_localVelocity.getBearing(), 0, g_degStr, x0, y0 + 5, TXT_CENTERED);

    // Depth to left of dial
    g_tft.setTextColor(ST77XX_GREEN);
    g_tft.setCursor(0, 60);
    g_tft.println(g_depth, 1);
    g_tft.setTextSize(1);
    g_tft.print("ft");
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

    if (ctx->len && offset + len == ctx->len) {
        // We're finished.
        ctx->avg = ctx->avg / (2 * ctx->len);
    }
}

// Callback to draw history data scaled to the display window defined in the
// context structure. Note that if data points exceed range_x (in number) or
// range_y (in value) then drawing will exceed the defined window.
void drawHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                         size_t len, size_t offset) {
    HistWindowContext* ctx = (HistWindowContext *)user;

    double scale_x = ctx->w / ctx->range_x;
    double scale_y = ctx->h / ctx->range_y;

    for (unsigned int i = 0; i < len; i++) {
        int16_t dmax = round(dataMax[i] * scale_y);
        int16_t dmin = round(dataMin[i] * scale_y);

        g_tft.drawFastVLine(ctx->x + round((offset + i) * scale_x),
                            ctx->y + ctx->h - dmax,
                            dmax - dmin + 1, ctx->color);
    }
}


//
// Wind subpage to display data history detail.
//
// D0 to cycle through each history subpage.
// D1 to exit back to top level AIS page.
void displaySubpageHistory(const char* title, const MinMaxDataHistory<double>* hist, uint16_t color) {
    struct HistStatsContext hist_stats;
    struct HistWindowContext hist_window;

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    // Calculate history statistics.
    hist_stats.len = hist->getLength();
    hist->forEachData(statsHistoryCallback, &hist_stats);

    // Vertical scale: a series of horizontal lines every 10 units.
    int t = ((int)hist_stats.max / 10) * 10;
    while (t > 0) {
        g_tft.drawFastHLine(0, DISPLAY_HEIGHT - 3 - round(t * (DISPLAY_HEIGHT - 3) / hist_stats.max),
                          DISPLAY_WIDTH, ST77XX_WHITE);
        t -= 10;
    }

    // Horizontal scale: a series of ticks every 10 units.
    g_tft.drawFastHLine(0, DISPLAY_HEIGHT - 3, DISPLAY_WIDTH, ST77XX_WHITE);
    for (t = 0; t < hist->getSize(); t += 10) {
        g_tft.drawFastVLine(DISPLAY_WIDTH - round(t * DISPLAY_WIDTH / hist->getSize()),
                          DISPLAY_HEIGHT - 2, 2, ST77XX_WHITE);
    }

    // Title at top-left
    g_tft.setTextColor(color);
    g_tft.setTextSize(3);
    g_tft.setCursor(0, 0);
    g_tft.print(title);

    // Stats at bottom-left
    g_tft.setTextSize(2);
    g_tft.setCursor(0, DISPLAY_HEIGHT - 54);
    g_tft.print("Max ");
    g_tft.println(hist_stats.max, 0);

    g_tft.print("Avg ");
    g_tft.println(hist_stats.avg, 0);

    g_tft.print("Min ");
    g_tft.println(hist_stats.min, 0);

    // Draw history data
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

//
// Position info top-level page.
//
// This page is intended to provide essential textyak info for making
// logbook entries.
//
void displayPagePosition() {
    char buffer[128];

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);
    g_tft.setCursor(0, 0);

    // Local boat info in yellow
    g_tft.setTextColor(ST77XX_YELLOW);
    g_tft.setTextSize(3);

    // Position first in large font.
    if (g_bPosValid) {
        buffer[0] = '\0';
        g_localPos.toString(buffer, sizeof(buffer), N2kPos::FMT_LAT_ONLY);
        g_tft.println(buffer);
        buffer[0] = '\0';
        g_localPos.toString(buffer, sizeof(buffer), N2kPos::FMT_LON_ONLY);
        g_tft.println(buffer);
    }

    g_tft.setCursor(0, 55);
    g_tft.setTextSize(2);

    // Heading
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "HDG %.0f%s",
             g_hdg, g_degStr);
    g_tft.println(buffer);

    // COG and SOG
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "COG %.0f%s %.1fkts",
             g_localVelocity.getBearing(), g_degStr,
             g_localVelocity.getMagnitude());
    g_tft.println(buffer);

    // True wind direction and speed in blue
    g_tft.setTextColor(ST77XX_BLUE);
    double twd = normalizeBearing(g_hdg + g_trueWind.getSignedBearing());
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "TWD %.0f%s %.0fkts",
             twd, g_degStr, g_trueWind.getMagnitude());
    g_tft.println(buffer);

    // Apparent wind angle and speed in red
    g_tft.setTextColor(ST77XX_RED);
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "AWA %.0f%s %.0fkts",
             g_appWind.getSignedBearing(), g_degStr, g_appWind.getMagnitude());
    g_tft.println(buffer);

    // Depth in green
    g_tft.setTextColor(ST77XX_GREEN);
    buffer[0] = '\0';
    snprintf(buffer, sizeof(buffer), "Depth %.1fft", g_depth);
    g_tft.println(buffer);
}

//
// AIS top-level page.
//
// North-up plot of AIS targets at various ranges defined by page ID.
// Vectors for each target show projected position in 5min.
// Dangerous targets are displayed in red.
//
// D0 to cycle through targets displaying vessel name, range and bearing in
// bottom left. CPA distance and time (mins) is shown in top-right if vessel is
// approaching our position.
//
// D1 to switch to details AIS info subpage for the selected target.
//
// D2 cycles to next top-level page.
void displayPageAis(Page pg, time_t now) {
    const double radius = DISPLAY_HEIGHT / 2.0 - 7.0;
    double range = pg == PAGE_AIS_12NM ? 12.0 : pg == PAGE_AIS_4NM ? 4.0 : 1.0;
    double range_scale = radius / range;
    double vector_scale = range_scale / 12.0;;
    int x0 = DISPLAY_WIDTH / 2;
    int y0 = DISPLAY_HEIGHT / 2;

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    // Draw range circles. Possibly we should have fewer.
    g_tft.drawCircle(x0, y0, round(range * range_scale), ST77XX_WHITE);
    g_tft.drawCircle(x0, y0, round(range * 0.75 * range_scale), ST77XX_WHITE);
    g_tft.drawCircle(x0, y0, round(range * 0.5 * range_scale), ST77XX_WHITE);
    g_tft.drawCircle(x0, y0, round(range * 0.25 * range_scale), ST77XX_WHITE);

    // Display page max range top-left.
    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(2);
    g_tft.setCursor(0, 0);
    g_tft.print(range, 0);
    g_tft.print("nm");

    // Local COG vector at center.
    g_tft.drawLine(x0, y0, x0 + round(g_localVelocity.getX() * vector_scale),
                 y0 - round(g_localVelocity.getY() * vector_scale), ST77XX_YELLOW);

    // Skip targets until we have local position
    if (g_bPosValid) {
        const N2kAISTarget* target = nullptr;

        for (auto t : g_targets) {
            if (!isVisibleTarget(t, g_page)) {
                continue;
            }

            // Keep a pointer to the selected target and handle it later. We
            // draw it last so it's on top and stays visible.
            if (t->getMmsi() == g_selTarget) {
                target = t;
                continue;
            }

            const N2kVector &p = t->getRelDistance();
            const N2kVector &v = t->getVelocity();
            double cpad = t->getCpa()->getDistance();
            double cpat = t->getCpa()->getRelTime(now) / 60;
            bool bDangerous = isDangerousTarget(cpad, cpat);

            g_tft.fillCircle(x0 + round(p.getX() * range_scale),
                           y0 - round(p.getY() * range_scale),
                           3, bDangerous ? ST77XX_RED : ST77XX_GREEN);
            g_tft.drawLine(x0 + round(p.getX() * range_scale),
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

            // Draw vessel info bottom-left
            g_tft.setTextColor(ST77XX_GREEN);
            g_tft.setCursor(0, DISPLAY_HEIGHT - 47);
            g_tft.print(p.getMagnitude(), 2);
            g_tft.println("nm");
            g_tft.print(p.getBearing(), 0);
            g_tft.println(g_degStr);
            g_tft.print(target->getName());

            // Draw CPA info top-right
            if (cpat >= 0.0 && !std::isnan(cpad)) {
                g_tft.setTextColor(ST77XX_RED);
                drawJustifiedVal(cpad, 2, "nm", DISPLAY_WIDTH, 0, TXT_JUSTIFIED);
                drawJustifiedVal(cpab, 0, g_degStr, DISPLAY_WIDTH, 32, TXT_JUSTIFIED);
                drawJustifiedVal(cpat, 1, "m", DISPLAY_WIDTH, 48, TXT_JUSTIFIED);
            }

            // Draw selected vessel in white
            g_tft.fillCircle(x0 + round(p.getX()* range_scale),
                           y0 - round(p.getY()* range_scale),
                           3, ST77XX_WHITE);
            g_tft.drawLine(x0 + round(p.getX()* range_scale),
                         y0 - round(p.getY()* range_scale),
                         x0 + round(p.getX()* range_scale + v.getX()* vector_scale),
                         y0 - round(p.getY()* range_scale + v.getY()* vector_scale),
                         ST77XX_YELLOW);
        }
    }
}

// Fetch the existing N2kAISTarget object for the specified MMSI, or allocate a
// new object.
N2kAISTarget* getAISTarget(uint32_t mmsi) {
    for (auto t : g_targets) {
        if (t->getMmsi() == mmsi) {
            return t;
        }
    }

    N2kAISTarget* t = new N2kAISTarget(mmsi);
    g_targets.push_back(t);
    return t;
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

// Should the AIS target's closest point of approach (CPA) be considered dangerous?
bool isDangerousTarget(double d, double t) {
    if (!std::isnan(d) && !std::isnan(t)) {
        // Yes, if under 1nm within the next 60mins.
        return d<1.0 &&t> 0 && t < 60;
    }
    return false;
}


//
// AIS subpage to display detailed vessel info.
//
// D0 to cycle selected vessel.
// D1 to exit back to top level AIS page.
void displaySubpageAisInfo() {
    char buffer[128];

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    if (g_bPosValid) {
        for (auto t : g_targets) {
            if (!isVisibleTarget(t, g_page)) {
                continue;
            }

            if (t->getMmsi() == g_selTarget) {
                static const int txtSize = 13;
                g_tft.setTextColor(ST77XX_GREEN);
                g_tft.setTextSize(2);

                g_tft.setCursor(0, 0);
                g_tft.println(t->getName());
                g_tft.print("MMSI ");
                g_tft.println(t->getMmsi());
                g_tft.println("");

                const N2kVector& relPos = t->getRelDistance();
                g_tft.print("Rng ");
                g_tft.print(relPos.getMagnitude(), 2);
                g_tft.print("nm ");
                g_tft.print("Brg ");
                g_tft.print(relPos.getBearing(), 0);
                g_tft.println((char)0xf8);

                const N2kVector& v = t->getVelocity();
                g_tft.print("SOG ");
                g_tft.print(v.getMagnitude(), 1);
                g_tft.print("kts ");
                g_tft.print("COG ");
                g_tft.print(v.getBearing(), 0);
                g_tft.println((char)0xf8);

                g_tft.print("LOA ");
                g_tft.print(t->getLength(), 1);
                g_tft.print("ft ");
                g_tft.print("Bm ");
                g_tft.print(t->getBeam(), 1);
                g_tft.println("ft ");

                g_tft.print("Dft ");
                g_tft.print(t->getDraft(), 1);
                g_tft.println("ft ");
            }
        }
    }
}


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
                    case PAGE_POSITION:
                        displayPagePosition();
                        break;

                    case PAGE_WIND:
                        displayPageWind();
                        break;

                    case PAGE_AIS_12NM:
                    case PAGE_AIS_4NM:
                    case PAGE_AIS_1NM:
                        displayPageAis(g_page, now);
                        break;
                }
                break;

            case SUBPAGE_HIST_TWS:
                displaySubpageHistory("TWS", &g_twsHistory, ST77XX_BLUE);
                break;

            case SUBPAGE_HIST_AWS:
                displaySubpageHistory("AWS", &g_awsHistory, ST77XX_RED);
                break;

            case SUBPAGE_HIST_SOG:
                displaySubpageHistory("SOG", &g_sogHistory, ST77XX_YELLOW);
                break;

            case SUBPAGE_HIST_DEPTH:
                displaySubpageHistory("Depth", &g_depthHistory, ST77XX_GREEN);
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
        if (val1 != N2kDoubleNA && val2 != N2kDoubleNA) {
            g_depth = meters2Ft(val1 + val2);
        }
    }
}

// PGN 129025: Position 36°51.054'S 174°45.870'W
void handlePgn129025Msg(const tN2kMsg &N2kMsg) {
    double val1, val2;

    if (ParseN2kPositionRapid(N2kMsg, val1, val2)) {
        if (val1 != N2kDoubleNA && val2 != N2kDoubleNA) {
            g_localPos.set(val1, val2);

            bool bIsFirstPos = !g_bPosValid;
            g_bPosValid = true;

            // If this is the first time we've received our position, update the CPA
            // of all AIS targets.
            if (bIsFirstPos) {
                for (auto t : g_targets) {
                    t->calcCpa(g_localPos, g_localVelocity);
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
        if (ref == 0 && val1 != N2kDoubleNA && val2 != N2kDoubleNA) {
            g_localVelocity.set(metersPerSec2Kts(val2), rad2Deg(val1));
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

        N2kPos p(latitude, longitude);
        N2kVector v(metersPerSec2Kts(aisSog), rad2Deg(aisCog));
        target->update(p, v);

        if (g_bPosValid) {
            target->calcCpa(g_localPos, g_localVelocity);
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

        N2kPos p(latitude, longitude);
        N2kVector v(metersPerSec2Kts(aisSog), rad2Deg(aisCog));
        target->update(p, v);

        if (g_bPosValid) {
            target->calcCpa(g_localPos, g_localVelocity);
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
        if (ref2 == N2kWind_Apparent && val1 != N2kDoubleNA && val2 != N2kDoubleNA) {
            g_appWind.set(val1, rad2Deg(val2));

            // Calculate true wind, relative to boat
            N2kVector boat(g_localVelocity.getMagnitude(), 0);
            g_trueWind.setXY(g_appWind.getX() - boat.getX(), g_appWind.getY() - boat.getY());
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
        if (heading != N2kDoubleNA) {
            if (ref == N2khr_magnetic) {
                if (variation == N2kDoubleNA) {
                    // If variation is unknown use variation for Boston, MA in 2025.
                    variation = deg2Rad(-14.0);
                }
                if (deviation == N2kDoubleNA) {
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

// Process incoming N2K messages to update our data model.
void handleNMEA2000Msg(const tN2kMsg &N2kMsg) {

  switch (N2kMsg.PGN) {
    // Handle PGNs we are interested in
    case 127250:
        handlePgn127250Msg(N2kMsg);
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

    case 130306:
        handlePgn130306Msg(N2kMsg);
        break;

    default:
        // Not interested. Do nothing.
        return;
  }

  displayUpdate(false);
}
