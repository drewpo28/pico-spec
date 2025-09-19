#include "Debug.h"
#include "pico/stdlib.h"
#include "FileUtils.h"

void Debug::led_blink()
{
    for (int i = 0; i < DEFAULT_BLINK_COUNT; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
}

void Debug::led_on()
{
    gpio_put(PICO_DEFAULT_LED_PIN, true);
}

void Debug::led_off()
{
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}

void Debug::log(string data)
{
    if (!FileUtils::fsMount) return;
    string nvs = MOUNT_POINT_SD STORAGE_LOG;
    FIL* handle = fopen2(nvs.c_str(), FA_WRITE | FA_OPEN_APPEND);
    if (handle) {
        UINT btw;
        const char* log = data.c_str();
        f_write(handle, log, strlen(log), &btw);
        f_write(handle, "\n", 1, &btw);
        fclose2(handle);
    }
}

void Debug::log(const char* fmt, ...)
{
    if (!FileUtils::fsMount) return;

    char buf[256]; // можно увеличить при необходимости
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    log(std::string(buf)); // вызов основной версии
}