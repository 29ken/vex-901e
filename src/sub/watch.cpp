#include "watch.hpp"
#include "main.h"
#include "rw-config.h"
#include "claw.hpp"
#include "resets.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#ifdef RW_SIM_PROS
#include "sim-hooks.h"
#endif

extern double field_half_size;  // RW's: the walls are this far from the centre

namespace {

// Every number here is a starting guess until a practice run's log says otherwise.
constexpr double MAX_OFF_SQUARE_DEG = 12;  // clear walls only echo near square
constexpr double MAX_RANGE_IN = 30;
constexpr double BACK_MAX_RANGE_IN = 12;   // the back sensor catches the toggle past this
constexpr double CONE_HALF_DEG = 14;       // the beam's half-width, with a little spare
constexpr double MAX_TURN_DPS = 60;
constexpr double AGREE_IN = 3;             // further off than this: something else is in the beam
constexpr double GAIN = 0.2;               // how much of each believed reading goes into the estimate
constexpr int LAG_TICKS = 4;               // a reading is about 40 ms old (10 ms ticks)
constexpr int READ_EVERY = 3;              // the sensors update about every 33 ms
constexpr int PRINT_MS = 250;
constexpr double TOGGLE_HALF_IN = 12.9;    // a toggle is 25.8 in long, centred on its wall
// The stop check (sensorCheckAtStop): how the public no-tracker teams use their
// sensors. They read from 20 to about 48 in, robot stopped or pressed on something.
constexpr double STOP_MAX_RANGE_IN = 48;
constexpr int STOP_SAMPLES = 3;
constexpr int STOP_SAMPLE_MS = 40;         // the sensors update about every 33 ms
constexpr double STOP_MAX_SPREAD_IN = 1.5; // the samples have to agree, or something is moving in the beam

struct Mount {
    const char* name;
    pros::Distance* sensor;
    double x, y;     // from the robot's centre: +x right, +y forward
    double facing;   // 0 front, 90 right, 180 back, 270 left
    double maxRange;
    double scale;
    bool front;
};

struct Obstacle {
    double x, y, r;
    bool gone;
};
// Field coordinates, both alliances' sides. Goals 2.81 (centre 4.4), stacks and
// loose Cup+Pins 1.58, loaders ~3.2.
Obstacle obstacles[] = {
    {0, 0, 4.4, false},
    {-47.09, -23.55, 2.81, false}, {-47.09, 23.55, 2.81, false}, {47.09, -23.55, 2.81, false}, {47.09, 23.55, 2.81, false},
    {-23.55, -47.09, 2.81, false}, {-23.55, 47.09, 2.81, false}, {23.55, -47.09, 2.81, false}, {23.55, 47.09, 2.81, false},
    {-68.7, -23.6, 1.58, false}, {-68.7, 23.6, 1.58, false}, {68.7, -23.6, 1.58, false}, {68.7, 23.6, 1.58, false},
    {-23.6, -68.7, 1.58, false}, {-23.6, 68.7, 1.58, false}, {23.6, -68.7, 1.58, false}, {23.6, 68.7, 1.58, false},
    {-45.9, -47.6, 1.58, false}, {-23.4, -25.3, 1.58, false}, {-22.8, -1.9, 1.58, false}, {-2.4, -24.2, 1.58, false},
    {45.9, 47.6, 1.58, false}, {23.4, 25.3, 1.58, false}, {22.8, 1.9, 1.58, false}, {2.4, 24.2, 1.58, false},
    {-68.5, -58.7, 3.2, false}, {-68.5, 58.7, 3.2, false}, {68.5, -58.7, 3.2, false}, {68.5, 58.7, 3.2, false},
};

std::atomic<bool> enabled{false};
std::atomic<bool> applying{false};

struct Pose {
    double x, y, heading;
};

// What one axis looks like to the sensors: a running estimate of how far the
// robot's position is off along it (+ means the robot is really further +).
struct Axis {
    double off = 0;   // inches
    int believed = 0; // readings used since the last print
    int total = 0;    // since the auton started
};

double readInches(const Mount& m) {
    std::int32_t mm = m.sensor->get();
    return (mm == PROS_ERR || mm <= 0 || mm >= 9999) ? -1 : mm / 25.4 * m.scale * distance_sensor_scale;
}

// One sensor, one reading, against the pose the robot had when it was taken.
// Returns 'x' or 'y' and sets `error` (how far off the position looks along
// that axis), or 0 with `why` when the reading isn't believed.
char judge(const Mount& m, const Pose& p, double reading, double turnDps, double& error, const char*& why, double agreeIn = AGREE_IN) {
    if (!m.sensor->is_installed()) { why = "unplugged"; return 0; }
    if (m.front && clawClosed()) { why = "claw closed"; return 0; }
    if (std::fabs(turnDps) > MAX_TURN_DPS) { why = "turning"; return 0; }
    double h = p.heading * M_PI / 180;
    // The sensor's place and where its beam points, in field coordinates.
    double sx = p.x + m.x * std::cos(h) + m.y * std::sin(h);
    double sy = p.y - m.x * std::sin(h) + m.y * std::cos(h);
    double b = (p.heading + m.facing) * M_PI / 180;
    double bx = std::sin(b), by = std::cos(b);
    // Which wall the beam meets first.
    double tx = std::fabs(bx) > 1e-6 ? ((bx > 0 ? field_half_size : -field_half_size) - sx) / bx : 1e9;
    double ty = std::fabs(by) > 1e-6 ? ((by > 0 ? field_half_size : -field_half_size) - sy) / by : 1e9;
    bool xWall = tx < ty;
    double predicted = xWall ? tx : ty;
    double offSquare = std::acos(std::fmin(1.0, std::fabs(xWall ? bx : by))) * 180 / M_PI;
    if (offSquare > MAX_OFF_SQUARE_DEG) { why = "off square"; return 0; }
    if (predicted > m.maxRange) { why = "too far"; return 0; }
    // The back sensor's lens is 7.95 in up and the toggles 10.7 in: from further
    // out than BACK_MAX_RANGE_IN its beam catches a toggle, if it lands on one.
    if (!m.front && m.facing == 180 && predicted > BACK_MAX_RANGE_IN) {
        double along = xWall ? sy + by * predicted : sx + bx * predicted;
        if (std::fabs(along) < TOGGLE_HALF_IN + predicted * std::tan(CONE_HALF_DEG * M_PI / 180)) { why = "toggle in the beam"; return 0; }
    }
    // Anything that should still be standing inside the beam's cone, nearer than the wall.
    for (const Obstacle& o : obstacles) {
        if (o.gone) continue;
        double dx = o.x - sx, dy = o.y - sy;
        double along = dx * bx + dy * by;
        if (along <= 0 || along - o.r > predicted) continue;
        double across = std::fabs(dx * by - dy * bx);
        if (across - o.r <= along * std::tan(CONE_HALF_DEG * M_PI / 180)) { why = "something in the beam"; return 0; }
    }
    if (reading <= 0) { why = "sees nothing"; return 0; }
    if (std::fabs(reading - predicted) > agreeIn) { why = "doesn't agree"; return 0; }
    // A short reading means the robot is nearer that wall than it thinks.
    double towardWall = (predicted - reading) * std::cos(offSquare * M_PI / 180);
    error = towardWall * ((xWall ? bx : by) > 0 ? 1 : -1);
    return xWall ? 'x' : 'y';
}

void loop() {
    Mount mounts[] = {
        {"front", &front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0, MAX_RANGE_IN, SCALE_FRONT, true},
        {"right", &right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90, MAX_RANGE_IN, SCALE_RIGHT, false},
        {"back", &back_sensor, back_sensor_offsetX, back_sensor_offsetY, 180, BACK_MAX_RANGE_IN, SCALE_BACK, false},
        {"left", &left_sensor, left_sensor_offsetX, left_sensor_offsetY, 270, MAX_RANGE_IN, SCALE_LEFT, false},
    };
    Pose history[LAG_TICKS + 1] = {};
    Axis ax, ay;
    int tick = 0;
    bool wasOn = false;
    std::uint32_t lastPrint = 0;
    while (true) {
        bool on = enabled;
        if (on && !wasOn) {  // a new auton
            ax = Axis();
            ay = Axis();
            tick = 0;
            lastPrint = pros::millis();
            for (Pose& p : history) p = {x_pos, y_pos, getInertialHeading()};
        }
        if (!on && wasOn) {
            std::printf("Watch: end. x off by %+.2f in (%d readings), y off by %+.2f in (%d readings)\n", ax.off, ax.total, ay.off, ay.total);
        }
        wasOn = on;
        if (on) {
            for (int i = LAG_TICKS; i > 0; i--) history[i] = history[i - 1];
            history[0] = {x_pos, y_pos, getInertialHeading()};
            if (++tick % READ_EVERY == 0) {
                for (const Mount& m : mounts) {
                    double error = 0;
                    const char* why = "";
                    char axis = judge(m, history[LAG_TICKS], readInches(m), odom_turn_rate_dps, error, why);
                    if (!axis) continue;
                    Axis& a = axis == 'x' ? ax : ay;
                    if (applying) {
                        // Move the position a part of the way; what's left shows up in the next reading.
                        (axis == 'x' ? x_pos : y_pos) += GAIN * error;
                        a.off = error * (1 - GAIN);
                    } else {
                        a.off += GAIN * (error - a.off);
                    }
                    a.believed++;
                    a.total++;
                }
            }
            std::uint32_t now = pros::millis();
            if (now - lastPrint >= static_cast<std::uint32_t>(PRINT_MS)) {
                if (ax.believed || ay.believed) {
                    std::printf("Watch %6.2f s: at (%.1f, %.1f)", now / 1000.0, x_pos, y_pos);
                    if (ax.believed) std::printf("  x looks off by %+.2f in (%d readings)", ax.off, ax.believed);
                    if (ay.believed) std::printf("  y looks off by %+.2f in (%d readings)", ay.off, ay.believed);
                    std::printf("\n");
                }
                ax.believed = ay.believed = 0;
                lastPrint = now;
            }
        }
        pros::delay(10);
    }
}

}  // namespace

StopReadings sensorSampleAtStop() {
    StopReadings r{};
    pros::Distance* sensors[4] = {&front_sensor, &right_sensor, &back_sensor, &left_sensor};
    double scale[4] = {SCALE_FRONT, SCALE_RIGHT, SCALE_BACK, SCALE_LEFT};
    double got[4][STOP_SAMPLES];
    for (int i = 0; i < STOP_SAMPLES; i++) {
        if (i) pros::delay(STOP_SAMPLE_MS);
        for (int k = 0; k < 4; k++) {
            std::int32_t mm = sensors[k]->is_installed() ? sensors[k]->get() : PROS_ERR;
            got[k][i] = (mm == PROS_ERR || mm <= 0 || mm >= 9999) ? -1 : mm / 25.4 * scale[k] * distance_sensor_scale;
        }
    }
    for (int k = 0; k < 4; k++) {
        double lo = 1e9, hi = -1e9;
        bool blind = false;
        for (double v : got[k]) {
            if (v < 0) blind = true;
            lo = std::fmin(lo, v);
            hi = std::fmax(hi, v);
        }
        std::sort(got[k], got[k] + STOP_SAMPLES);
        r.inches[k] = (blind || hi - lo > STOP_MAX_SPREAD_IN) ? -1 : got[k][STOP_SAMPLES / 2];
    }
    return r;
}

int sensorCheckAtStop(const StopReadings& r, const char* where, double keepWithin, double maxJump) {
    Mount mounts[] = {
        {"front", &front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0, STOP_MAX_RANGE_IN, SCALE_FRONT, true},
        {"right", &right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90, STOP_MAX_RANGE_IN, SCALE_RIGHT, false},
        {"back", &back_sensor, back_sensor_offsetX, back_sensor_offsetY, 180, STOP_MAX_RANGE_IN, SCALE_BACK, false},
        {"left", &left_sensor, left_sensor_offsetX, left_sensor_offsetY, 270, STOP_MAX_RANGE_IN, SCALE_LEFT, false},
    };
    int moved = 0;
    for (int k = 0; k < 4; k++) {
        const Mount& m = mounts[k];
        if (!m.sensor->is_installed()) continue;
        Pose p{x_pos, y_pos, getInertialHeading()};  // each axis is judged after any earlier correction
        double error = 0;
        const char* why = "";
        char axis = judge(m, p, r.inches[k], 0, error, why, maxJump);
        if (!axis) {
            std::printf("Dist %s, %s: not used (%s)\n", where, m.name, why);
            continue;
        }
        if (std::fabs(error) <= keepWithin) {
            std::printf("Dist %s, %s %.1f in: %c agrees to %+.2f in\n", where, m.name, r.inches[k], axis, error);
            continue;
        }
        double nx = x_pos + (axis == 'x' ? error : 0), ny = y_pos + (axis == 'y' ? error : 0);
        std::printf("Dist %s, %s %.1f in: %c %.2f -> %.2f\n", where, m.name, r.inches[k], axis,
                    axis == 'x' ? x_pos : y_pos, axis == 'x' ? nx : ny);
        setPose(nx, ny, p.heading);
        moved++;
    }
    return moved;
}

void initWatch() {
#ifdef RW_SIM_PROS
    if (std::getenv("WATCH_APPLY")) applying = true;  // for comparing runs in the simulator
#endif
    pros::Task task(loop, "Watch");
}

void watchApply(bool on) { applying = on; }

void watchEnable(bool on) {
    if (on) {
        for (Obstacle& o : obstacles) o.gone = false;
    }
    enabled = on;
}

void watchTaken(double x, double y) {
#ifdef RW_SIM_PROS
    rwsim::fieldPieceTaken(x, y);  // the simulator's sensors stop seeing it too
#endif
    for (Obstacle& o : obstacles) {
        if (std::hypot(o.x - x, o.y - y) < 3) o.gone = true;
    }
}
