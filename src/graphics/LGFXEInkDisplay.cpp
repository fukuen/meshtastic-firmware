#include "LGFXEInkDisplay.h"

#if defined(USE_EINK) && defined(USE_EINK_LGFX)

#include "UptimeClock.h"
#include "configuration.h"
#include "graphics/Backlight.h"
#include "mesh/NodeDB.h"

#include <LovyanGFX.hpp>
#include <esp_heap_caps.h>
#include <lgfx/v1/panel/Panel_SSD1677.hpp>

namespace
{
// LovyanGFX device for the M5Stack PaperMono SSD1677 4-gray E-Paper panel.
// Panel geometry mirrors M5GFX: native landscape 800x480, offset_rotation 3
// exposes the panel as the 480x800 portrait BaseUI draws into.
class LGFX : public lgfx::LGFX_Device
{
    lgfx::Bus_SPI _bus;
    lgfx::Panel_SSD1677_4Gray _panel;

  public:
    LGFX()
    {
        {
            auto cfg = _bus.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read = 10000000;
            cfg.pin_sclk = PIN_EINK_SCLK;
            cfg.pin_mosi = PIN_EINK_MOSI;
            cfg.pin_miso = -1;
            cfg.pin_dc = PIN_EINK_DC;
            cfg.spi_3wire = true;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.pin_cs = PIN_EINK_CS;
            cfg.pin_rst = PIN_EINK_RES;
            cfg.pin_busy = PIN_EINK_BUSY;
            cfg.panel_width = 800;
            cfg.panel_height = 480;
            cfg.memory_width = 800;
            cfg.memory_height = 480;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 3;
            cfg.readable = false;
            cfg.invert = false;
            cfg.bus_shared = false;
            _panel.config(cfg);
        }
        setPanel(&_panel);
    }
};

LGFX lcd;
} // namespace

LGFXEInkDisplay::LGFXEInkDisplay(uint16_t width, uint16_t height)
{
    this->geometry = GEOMETRY_RAWMODE;
    this->displayWidth = width;
    this->displayHeight = height;
    this->displayBufferSize = displayWidth * ((displayHeight + 7) / 8);
    lastBuffer = new uint8_t[displayBufferSize];
    memset(lastBuffer, 0xFF, displayBufferSize); // start all-white
}

LGFXEInkDisplay::~LGFXEInkDisplay()
{
    delete[] lastBuffer;
    if (pixels) {
        heap_caps_free(pixels);
    }
}

bool LGFXEInkDisplay::connect()
{
    LOG_INFO("Init LovyanGFX SSD1677 4-gray E-Ink (%u x %u)", displayWidth, displayHeight);

    if (!lcd.init()) {
        LOG_ERROR("LovyanGFX SSD1677 init failed");
        return false;
    }

    lcd.setRotation(0);
    lcd.setAutoDisplay(false); // refresh only when we explicitly call display()
    lcd.setEpdMode(lgfx::epd_mode_t::epd_fast);

    graphics::backlightInit();
    // A GPIO (on/off) backlight is left off when screen_brightness is 0.
    // The PaperMono frontlight is expected to light; drive it on.
    if (uiconfig.screen_brightness == 0)
        graphics::backlightSet(GPIO_BACKLIGHT_ON_LEVEL);
    return true;
}

void LGFXEInkDisplay::addFrameFlag(frameFlagTypes flag)
{
    frameFlags = (frameFlagTypes)(frameFlags | flag);
}

void LGFXEInkDisplay::setDisplayResilience(uint8_t fastPerFullValue, float stressMultiplierValue)
{
    fastPerFull = (fastPerFullValue == 0) ? 1 : fastPerFullValue;
    stressMultiplier = stressMultiplierValue;
}

void LGFXEInkDisplay::joinAsyncRefresh()
{
    if (lcd.displayBusy()) {
        lcd.waitDisplay();
    }
}

// OLEDDisplayUi tick path. Honours the rate-limit unless flags demand otherwise.
void LGFXEInkDisplay::display()
{
    const bool demandFast = frameFlags & DEMAND_FAST;
    const bool cosmetic = frameFlags & COSMETIC;
    const bool unlimitedFast = frameFlags & UNLIMITED_FAST;

    if (!demandFast && !cosmetic && !unlimitedFast) {
        if (!forceDisplay(lastDrawMsec == 0 ? 0 : 1000)) {
            return;
        }
        return;
    }

    forceDisplay(0);
}

// Keyframe path. Returns true if a frame was pushed.
bool LGFXEInkDisplay::forceDisplay(uint32_t msecLimit)
{
    const uint32_t now = Time::stampMillis();
    if (lastDrawMsec != 0 && (now - lastDrawMsec) < msecLimit) {
        return false;
    }

    const bool blocking = frameFlags & BLOCKING;

    const bool pushed = commit(blocking);
    if (pushed) {
        lastDrawMsec = now;
    }

    // Reset flags for next frame
    frameFlags = BACKGROUND;
    return pushed;
}

bool LGFXEInkDisplay::commit(bool blocking)
{
    int32_t dx = 0, dy = 0, dw = displayWidth, dh = displayHeight;
    const bool dirty = computeDirtyRect(dx, dy, dw, dh);
    const bool full = decide(Time::stampMillis(), dirty);

    if (!full && !dirty) {
        return false; // nothing changed
    }
    if (full) {
        dx = 0;
        dy = 0;
        dw = displayWidth;
        dh = displayHeight;
    }

    if (!ensurePixels((size_t)dw * dh * 2)) {
        LOG_ERROR("LGFXEInkDisplay scratch alloc failed (%ux%u)", dw, dh);
        return false;
    }

    // OLEDDisplay buffer: byte = buffer[x + (y/8) * displayWidth]; bit 1<<(y&7) = black.
    // Convert the dirty region into an RGB565 white/black image.
    for (int32_t py = 0; py < dh; py++) {
        const int32_t oy = dy + py;
        uint16_t *dst = &pixels[(size_t)py * dw];
        for (int32_t px = 0; px < dw; px++) {
            const int32_t ox = dx + px;
            const uint8_t b = buffer[ox + (oy >> 3) * displayWidth];
            dst[px] = (b & (1 << (oy & 7))) ? 0x0000 : 0xFFFF;
        }
    }

    lcd.startWrite();
    lcd.setEpdMode(full ? lgfx::epd_mode_t::epd_quality : lgfx::epd_mode_t::epd_fast);
    lcd.pushImage(dx, dy, dw, dh, pixels, 16);
    lcd.display(dx, dy, dw, dh);
    lcd.endWrite();

    if (blocking || full) {
        lcd.waitDisplay();
    }

    memcpy(lastBuffer, buffer, displayBufferSize);

    const uint32_t now = Time::stampMillis();
    if (full) {
        fullRefreshDebt = 0.0f;
        lastFullRefreshMsec = now;
    } else {
        // Debt accrues only on refreshes that actually hit the panel.
        fullRefreshDebt += 1.0f / fastPerFull;
    }
    return true;
}

// Decide FULL vs FAST. A full refresh runs on COSMETIC frames, and otherwise
// only on the first changed frame once either the refresh debt or the
// time-since-last-full threshold has been reached. Unchanged frames and
// user-interaction frames stay fast.
bool LGFXEInkDisplay::decide(uint32_t now, bool dirty)
{
    if (frameFlags & COSMETIC) {
        return true; // intentional full refresh (boot, screensaver, sleep)
    }
    if (frameFlags & (DEMAND_FAST | RESPONSIVE | UNLIMITED_FAST)) {
        return false; // defer the full refresh during interaction / typing
    }
    if (!dirty) {
        return false; // static screen: nothing to refresh
    }

    const bool timeDue = lastFullRefreshMsec != 0 && (now - lastFullRefreshMsec) >= fullRefreshIntervalMs;
    if (timeDue || fullRefreshDebt >= 1.0f) {
        fullRefreshDebt = 0.0f;
        return true;
    }
    return false;
}

bool LGFXEInkDisplay::computeDirtyRect(int32_t &x, int32_t &y, int32_t &w, int32_t &h)
{
    int32_t minX = displayWidth, minYBlock = displayHeight / 8, maxX = -1, maxYBlock = -1;
    for (int32_t i = 0; i < (int32_t)displayBufferSize; i++) {
        if (buffer[i] == lastBuffer[i]) {
            continue;
        }
        const int32_t xb = i % displayWidth;
        const int32_t yb = i / displayWidth;
        if (xb < minX) {
            minX = xb;
        }
        if (xb > maxX) {
            maxX = xb;
        }
        if (yb < minYBlock) {
            minYBlock = yb;
        }
        if (yb > maxYBlock) {
            maxYBlock = yb;
        }
    }

    if (maxX < 0) {
        return false;
    }

    x = minX;
    y = minYBlock * 8;
    w = maxX - minX + 1;
    h = (maxYBlock - minYBlock) * 8 + 8;
    if (y + h > (int32_t)displayHeight) {
        h = displayHeight - y;
    }
    return true;
}

bool LGFXEInkDisplay::ensurePixels(size_t bytes)
{
    if (pixels && pixelsSize >= bytes) {
        return true;
    }
    heap_caps_free(pixels);
    pixels = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pixels) {
        pixels = (uint16_t *)malloc(bytes);
    }
    if (pixels) {
        pixelsSize = bytes;
    }
    return pixels != nullptr;
}

#endif // USE_EINK && USE_EINK_LGFX