#include "resets.hpp"
#include "rw-config.h"
#include "claw.hpp"
#include "lift.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

extern double field_half_size;  // RW's (its motor-control.cpp): the walls are this far from the centre

namespace {

struct Mount {
    ResetSensor which;
    const char* name;
    pros::Distance* sensor;
    double x, y;      // offsets from rw-config.cpp, +x right, +y forward
    double facing;    // RW's sensor angle: 0 front, 90 right, 180 back, 270 left
    double maxRange;  // inches
};

Mount mountOf(ResetSensor which) {
    switch (which) {
        case ResetSensor::Front:
            return {which, "front", &front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0, RESET_MAX_RANGE_IN};
        case ResetSensor::Back:
            return {which, "back", &back_sensor, back_sensor_offsetX, back_sensor_offsetY, 180, RESET_BACK_MAX_RANGE_IN};
        case ResetSensor::Left:
            return {which, "left", &left_sensor, left_sensor_offsetX, left_sensor_offsetY, 270, RESET_MAX_RANGE_IN};
        case ResetSensor::Right:
        default:
            return {which, "right", &right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90, RESET_MAX_RANGE_IN};
    }
}

// why can hold one %.1f for value.
bool skip(const Mount& m, const char* why, double value = 0) {
    char text[96];
    std::snprintf(text, sizeof(text), why, value);
    std::printf("Reset %-5s skipped: %s\n", m.name, text);
    return false;
}

bool still() {
    return std::fabs(odom_speed_ips) < RESET_STILL_IPS && std::fabs(odom_turn_rate_dps) < RESET_STILL_DPS;
}

// Everything but the reading itself that can rule a sensor out.
bool usable(const Mount& m) {
    if (!m.sensor->is_installed()) return skip(m, "sensor not plugged in");
    if (m.which == ResetSensor::Front && clawClosed()) {
        LiftStatus lift = liftStatus();
        if (!lift.sensorOk || !lift.zeroed || lift.height < RESET_FRONT_CLEAR_LIFT) {
            return skip(m, "claw closed, so a piece may be in the beam");
        }
    }
    return true;
}

// Checks one sensor's readings (inches, any order) and resets X or Y from them.
bool tryReset(const Mount& m, std::vector<double> readings) {
    if (readings.empty()) return skip(m, "sees nothing");
    std::sort(readings.begin(), readings.end());
    double spread = readings.back() - readings.front();
    double reading = readings[readings.size() / 2];
    if (spread > RESET_MAX_SPREAD_IN) return skip(m, "readings disagree by %.1f in", spread);
    if (reading > m.maxRange) return skip(m, "too far, %.1f in", reading);

    // RW's own checks, run here first so the terminal says which one refused.
    double newPos = 0;
    int axis = distanceResetPosition(x_pos, y_pos, getInertialHeading(), m.x, m.y, m.facing,
                                     reading * distance_sensor_scale, field_half_size,
                                     distance_reset_max_angle, distance_reset_corner_margin, newPos);
    if (axis == 0) return skip(m, "near a corner or loader, or can't tell which wall");
    double oldPos = axis == 1 ? x_pos : y_pos;
    if (distance_reset_max_jump > 0 && std::fabs(newPos - oldPos) > distance_reset_max_jump) {
        return skip(m, "would move it %.1f in, so something may be in front of the wall", newPos - oldPos);
    }

    if (!resetPositionWithReading(reading, m.x, m.y, m.facing, field_half_size)) {
        return skip(m, "RW refused it");
    }
    std::printf("Reset %-5s %5.1f in: %s %.2f -> %.2f\n", m.name, reading, axis == 1 ? "x" : "y", oldPos,
                axis == 1 ? x_pos : y_pos);
    return true;
}

}  // namespace

int resetFrom(std::initializer_list<ResetSensor> sensors) {
    std::vector<Mount> mounts;
    for (ResetSensor s : sensors) {
        Mount m = mountOf(s);
        if (usable(m)) mounts.push_back(m);
    }
    if (mounts.empty()) return 0;

    double heading = getInertialHeading();
    double offSquare = std::fabs(std::remainder(heading, 90.0));
    if (offSquare > RESET_MAX_OFF_SQUARE_DEG) {
        for (const Mount& m : mounts) skip(m, "%.0f degrees off square with the walls", offSquare);
        return 0;
    }

    std::uint32_t start = pros::millis();
    while (!still() && pros::millis() - start < static_cast<std::uint32_t>(RESET_SETTLE_MS)) pros::delay(10);
    if (!still()) {
        for (const Mount& m : mounts) skip(m, "robot still moving");
        return 0;
    }

    // Read every sensor at the same moments, so two sensors cost no more time than one.
    std::vector<std::vector<double>> readings(mounts.size());
    std::vector<bool> blind(mounts.size(), false);
    for (int i = 0; i < RESET_SAMPLES; i++) {
        if (i) pros::delay(RESET_SAMPLE_MS);
        for (std::size_t k = 0; k < mounts.size(); k++) {
            std::int32_t mm = mounts[k].sensor->get();
            if (mm == PROS_ERR || mm <= 0 || mm >= 9999) {
                blind[k] = true;  // no wall in the beam on at least one reading
            } else {
                readings[k].push_back(mm / 25.4);
            }
        }
    }

    int used = 0;
    for (std::size_t k = 0; k < mounts.size(); k++) {
        if (blind[k]) {
            skip(mounts[k], "sees nothing on some readings");
        } else if (tryReset(mounts[k], readings[k])) {
            used++;
        }
    }
    return used;
}

bool resetFrom(ResetSensor sensor) {
    return resetFrom({sensor}) == 1;
}
