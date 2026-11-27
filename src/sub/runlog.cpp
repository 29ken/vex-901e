#include "runlog.hpp"
#include "main.h"
#include "rw-config.h"
#include "claw.hpp"
#include "lift.hpp"
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

pros::Mutex fileMutex;          // one writer at a time: the sampler and whoever is printing
std::FILE* file = nullptr;
std::atomic<bool> open{false};
std::uint32_t startMs = 0;
bool samplerStarted = false;

// A line being built up by say(): the watch prints one line in several pieces.
char pending[320];
std::size_t pendingLen = 0;

double sinceStart() {
    return (pros::millis() - startMs) / 1000.0;
}

double inches(pros::Distance& sensor) {
    if (!sensor.is_installed()) return -1;
    std::int32_t mm = sensor.get();
    return (mm == PROS_ERR || mm <= 0 || mm >= 9999) ? -1 : mm / 25.4;
}

// Call with fileMutex held.
void closeFile() {
    if (!file) return;
    std::fclose(file);
    file = nullptr;
    open = false;
}

// Where the files go: the SD card on the robot. In the simulator nothing is
// written unless RUN_LOG_DIR names a folder (used to check this file's output).
const char* logDir() {
#ifdef RW_SIM_PROS
    return std::getenv("RUN_LOG_DIR");
#else
    return pros::usd::is_installed() ? "/usd" : nullptr;
#endif
}

// The next run number, kept on the card so files are never overwritten.
int nextRunNumber(const char* dir) {
    char counter[200];
    std::snprintf(counter, sizeof(counter), "%s/run_count.txt", dir);
    int n = 0;
    if (std::FILE* f = std::fopen(counter, "r")) {
        if (std::fscanf(f, "%d", &n) != 1) n = 0;
        std::fclose(f);
    }
    n = n >= 999 ? 1 : n + 1;
    if (std::FILE* f = std::fopen(counter, "w")) {
        std::fprintf(f, "%d\n", n);
        std::fclose(f);
    }
    return n;
}

void sampler(void*) {
    std::uint32_t lastFlush = 0;
    while (true) {
        if (open) {
            std::uint32_t now = pros::millis();
            bool over = now - startMs > static_cast<std::uint32_t>(RUN_LOG_MAX_S) * 1000;
            // A run cut short by the field controller never reaches runLogStop().
            bool disabled = pros::competition::is_disabled() && now - startMs > 500;
            LiftStatus lift = liftStatus();
            fileMutex.take();
            if (file) {
                std::fprintf(file, "P,%.2f,%.2f,%.2f,%.1f,%.0f,%d,%.1f,%.1f,%.1f,%.1f\n", sinceStart(), x_pos, y_pos,
                             getInertialHeading(), lift.sensorOk ? lift.height : -1.0, clawClosed() ? 1 : 0,
                             inches(front_sensor), inches(right_sensor), inches(back_sensor), inches(left_sensor));
                if (over || disabled) {
                    std::fprintf(file, "E,%.2f,%s\n", sinceStart(), over ? "log: time limit, closed" : "log: robot disabled, closed");
                    closeFile();
                } else if (now - lastFlush >= static_cast<std::uint32_t>(RUN_LOG_FLUSH_MS)) {
                    std::fflush(file);
                    lastFlush = now;
                }
            }
            fileMutex.give();
        }
        pros::delay(RUN_LOG_SAMPLE_MS);
    }
}

}  // namespace

bool runLogging() { return open; }

void runLogStart(const char* autonName) {
    const char* dir = logDir();
#ifdef RW_SIM_PROS
    if (!dir) return;
#endif
    fileMutex.take();
    closeFile();
    pendingLen = 0;
    if (dir) {
        char path[220];
        int n = nextRunNumber(dir);
        std::snprintf(path, sizeof(path), "%s/run_%03d.csv", dir, n);
        file = std::fopen(path, "w");
        if (file) {
            startMs = pros::millis();
            std::fprintf(file, "# 901E run log %d: %s. Battery %.0f%%. Brain clock %u ms at the start.\n", n, autonName,
                         pros::battery::get_capacity(), static_cast<unsigned>(startMs));
            std::fprintf(file, "# P,t,x,y,heading,lift,claw,front,right,back,left   E,t,text\n");
            open = true;
            std::printf("Run log: %s\n", path);
        } else {
            std::printf("Run log: could not open %s\n", path);
        }
    } else {
        std::printf("Run log: no SD card, nothing saved\n");
    }
    fileMutex.give();
    if (!samplerStarted) {
        samplerStarted = true;
        pros::Task task(sampler, nullptr, "Run log");
    }
}

void runLogStop() {
    fileMutex.take();
    if (file) std::fprintf(file, "E,%.2f,log: auton finished\n", sinceStart());
    closeFile();
    fileMutex.give();
}

void say(const char* format, ...) {
    char text[320];
    va_list args;
    va_start(args, format);
    int len = std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (len < 0) return;
    std::fputs(text, stdout);
    if (!open) return;
    fileMutex.take();
    if (file) {
        // Collect up to the end of the line, then write it as one event.
        for (const char* c = text; *c; c++) {
            if (*c == '\n') {
                pending[pendingLen] = 0;
                if (pendingLen) std::fprintf(file, "E,%.2f,%s\n", sinceStart(), pending);
                pendingLen = 0;
            } else if (pendingLen < sizeof(pending) - 1) {
                pending[pendingLen++] = *c;
            }
        }
    }
    fileMutex.give();
}
