#include "Debug.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "FileUtils.h"

static uint32_t log_counter = 0;

bool Debug::log_enabled = false;

void Debug::led_blink()
{
#if DEBUG && defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    for (int i = 0; i < DEFAULT_BLINK_COUNT; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
#endif
}

void Debug::led_on()
{
#if DEBUG && defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, true);
#endif
}

void Debug::led_off()
{
#if DEBUG && defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, false);
#endif
}

// Push one byte to the console UART only if the TX FIFO has room; returns false
// (caller stops) when it's full so we DROP the rest instead of blocking. The
// stock printf path uses uart_write_blocking, which stalls the moment the 32-byte
// FIFO fills — a per-packet log flood in the ZiFi net pump then freezes the main
// loop and breaks WiFi scan / FTP / transfers (timing-sensitive). Best-effort,
// lossy-under-flood logging keeps the diagnostics without ever stalling.
//
// Gate on DBG_UART_ENABLED (set by CMake only when <BOARD>_DBG_UART is ON), NOT on
// PICO_DEFAULT_UART: the board header always defines a default UART (uart0/GP0-1 on
// pico2), which on PICO_DV is the ZiFi UART. Writing logs there corrupts the ESP AT
// stream and the FTP server / WiFi fail to start. With the console UART off, logging
// falls through to printf (no stdio driver linked → dropped, harmless).
#if defined(DBG_UART_ENABLED) && defined(PICO_DEFAULT_UART)
static inline bool dbg_uart_put(char c)
{
    // Bounded wait for FIFO space. At boot the 115200 UART keeps up if we wait a
    // few microseconds, so whole lines (incl. their CRLF) get out intact instead
    // of being truncated mid-line — which previously merged adjacent log lines.
    // The spin cap means a pathological flood (e.g. ZIFI_TRACE per-packet logging)
    // still drops bytes after the bound rather than ever freezing the main loop.
    for (uint32_t spin = 0; !uart_is_writable(uart_default); ++spin)
        if (spin >= 200000u) return false;   // ~1 ms ceiling per byte → drop, never hang
    uart_get_hw(uart_default)->dr = (uint8_t)c;
    return true;
}
#endif

void Debug::log(const char* fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;

#if defined(DBG_UART_ENABLED) && defined(PICO_DEFAULT_UART)
    for (int i = 0; i < n; i++) {
        if (buf[i] == '\n' && !dbg_uart_put('\r')) return; // CRLF for terminals
        if (!dbg_uart_put(buf[i])) return;                 // FIFO full → drop remainder
    }
    if (!dbg_uart_put('\r')) return;
    dbg_uart_put('\n');
#else
    printf("%s\n", buf);
#endif
}

void Debug::log2SD_impl(const string& data)
{
    if (!FileUtils::fsMount) return;
    static const char* nvs = STORAGE_LOG;

    // Заголовок сессии при первой записи
    bool first = (log_counter == 0);

    char prefix[32];
    snprintf(prefix, sizeof(prefix), "%08u %7u ", (unsigned)log_counter++,
             (unsigned)(to_ms_since_boot(get_absolute_time())));

    std::string logEntry;
    if (first) {
        const char* reason = watchdog_caused_reboot() ? "WATCHDOG" : "POWER-ON";
        char hdr[64];
        snprintf(hdr, sizeof(hdr), "--- BOOT (%s) ---\n", reason);
        logEntry = std::string(hdr);
    }
    logEntry += std::string(prefix) + data + "\n";

    // Лимит 200KB — перезаписываем с начала при переполнении
    FIL* handle = fopen2(nvs, FA_WRITE | FA_OPEN_APPEND);
    if (!handle) {
        FileUtils::mkdirParents(CONFIG_DIR);
        handle = fopen2(nvs, FA_WRITE | FA_OPEN_APPEND);
    }
    if (handle) {
        if (f_size(handle) >= 204800) {
            fclose2(handle);
            handle = fopen2(nvs, FA_WRITE | FA_CREATE_ALWAYS);
            if (!handle) return;
        }
        UINT btw;
        f_write(handle, logEntry.c_str(), logEntry.size(), &btw);
        fclose2(handle);
    }
}

void Debug::log2SD_impl(const char* fmt, ...)
{
    if (!FileUtils::fsMount) return;

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    log2SD_impl(std::string(buf));
}

extern "C" void debug_log2sd(const char* fmt, ...)
{
    if (!Debug::log_enabled) return;

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Debug::log2SD_impl(std::string(buf));
}