#pragma once

#include <OLEDDisplay.h>

/*
    OLEDDisplay adapter that renders BaseUI's monochrome framebuffer to a
    LovyanGFX E-Paper panel (SSD1677 4-gray on the M5Stack PaperMono).

    Routine frames refresh with the panel's fast four-gray waveform. A
    full-quality refresh runs on COSMETIC frames and otherwise on the first
    changed frame once either ~100 fast refreshes have accumulated or 5 minutes
    have passed since the last full refresh. Unchanged (static) frames are
    skipped entirely, so an idle screen never triggers a refresh. Only the
    changed bounding region is pushed into the LovyanGFX framebuffer before the
    refresh, keeping SPI traffic (and ghosting stress) low.
*/

#if defined(USE_EINK) && defined(USE_EINK_LGFX)

class LGFXEInkDisplay : public OLEDDisplay
{
  public:
    // Frame flags Screen.cpp sets via EINK_ADD_FRAMEFLAG before a draw.
    enum frameFlagTypes : uint8_t {
        BACKGROUND = (1 << 0),
        RESPONSIVE = (1 << 1),
        COSMETIC = (1 << 2),
        DEMAND_FAST = (1 << 3),
        BLOCKING = (1 << 4),
        UNLIMITED_FAST = (1 << 5),
    };

    LGFXEInkDisplay(uint16_t width, uint16_t height);
    ~LGFXEInkDisplay() override;

    // OLEDDisplay overrides
    bool connect() override;
    void display() override;
    void sendCommand(uint8_t com) override { (void)com; }
    int getBufferOffset(void) override { return 0; }

    // BaseUI public API (same shape as EInkDynamicDisplay)
    bool forceDisplay(uint32_t msecLimit = 1000);
    void addFrameFlag(frameFlagTypes flag);
    void joinAsyncRefresh();
    void enableUnlimitedFastMode() { addFrameFlag(UNLIMITED_FAST); }
    void disableUnlimitedFastMode() { frameFlags = (frameFlagTypes)(frameFlags & ~UNLIMITED_FAST); }

    void setDisplayResilience(uint8_t fastPerFull, float stressMultiplier = 2.0f);
    void setFullRefreshIntervalMs(uint32_t intervalMs) { fullRefreshIntervalMs = intervalMs; }

  private:
    bool commit(bool blocking);
    bool decide(uint32_t now, bool dirty);
    bool computeDirtyRect(int32_t &x, int32_t &y, int32_t &w, int32_t &h);
    bool ensurePixels(size_t bytes);

    uint8_t *lastBuffer = nullptr; // copy of the OLEDDisplay buffer, for dirty-region diffing
    uint16_t *pixels = nullptr;    // RGB565 scratch buffer for the dirty region
    size_t pixelsSize = 0;

    frameFlagTypes frameFlags = BACKGROUND;
    uint32_t lastDrawMsec = 0;

    // One full-quality refresh per fastPerFull pushed fast refreshes, and at
    // most once per fullRefreshIntervalMs. Debt only accrues on pushed frames.
    float fullRefreshDebt = 0.0f;
    uint8_t fastPerFull = 100;
    float stressMultiplier = 2.0f;
    uint32_t lastFullRefreshMsec = 0;
    uint32_t fullRefreshIntervalMs = 5 * 60 * 1000;
};

// Route the Screen.cpp compat macros to this class.
#undef EINK_ADD_FRAMEFLAG
#undef EINK_JOIN_ASYNCREFRESH
#define EINK_ADD_FRAMEFLAG(display, flag) static_cast<LGFXEInkDisplay *>(display)->addFrameFlag(LGFXEInkDisplay::flag)
#define EINK_JOIN_ASYNCREFRESH(display) static_cast<LGFXEInkDisplay *>(display)->joinAsyncRefresh()

#endif // USE_EINK && USE_EINK_LGFX