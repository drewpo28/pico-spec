#ifndef Debug_h
#define Debug_h

#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <cstring>

using namespace std;

#define DEFAULT_BLINK_COUNT 5

// Mirror of DEBUG_LOG_PATH from FileUtils.h. Kept as a literal here so Debug.h
// stays a leaf header (FileUtils.h pulls MemESP.h which depends on Config).
#define STORAGE_LOG "/.config/pico-spec/debug.log"

class Debug
{
public:

    // Runtime toggle for log2SD writes. Persisted via Config save/load.
    // Owned by Debug to keep this a leaf header (Config.h includes Debug.h).
    static bool log_enabled;

    static void led_blink();
    static void led_on();
    static void led_off();

    static void log(const char* fmt, ...);

    // Runtime-gated. The flag check is inlined here so when logging is
    // disabled the call collapses to a single branch on a global bool.
    static void log2SD_impl(const string& data);
    static void log2SD_impl(const char* fmt, ...);

    static inline void log2SD(const string& data) {
        if (__builtin_expect(log_enabled, 0)) log2SD_impl(data);
    }
    template <typename... Args>
    static inline void log2SD(const char* fmt, Args... args) {
        if (__builtin_expect(log_enabled, 0)) log2SD_impl(fmt, args...);
    }
};

#ifdef __cplusplus
extern "C" {
#endif
void debug_log2sd(const char* fmt, ...);
#ifdef __cplusplus
}
#endif

#endif