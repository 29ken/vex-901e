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
    double scale;
};

Mount mountOf(ResetSensor which) {
    switch (which) {
        case ResetSensor::Front:
            return {which, "front", &front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0, RESET_MAX_RANGE_IN,
                    SCALE_FRONT};
        case ResetSensor::Back:
            return {which, "back", &back_sensor, back_sensor_offsetX, back_sensor_offsetY, 180,
                    RESET_BACK_MAX_RANGE_IN, SCALE_BACK};
        case ResetSensor::Left:
            return {which, "left", &left_sensor, left_sensor_offsetX, left_sensor_offsetY, 270, RESET_MAX_RANGE_IN,
                    SCALE_LEFT};
        case ResetSensor::Right:
        default:
            return {which, "right", &right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90, RESET_MAX_RANGE_IN,
                    SCALE_RIGHT};
    }
}

// The sensor's reading in inches, or a negative number when it sees nothing.
double readInches(const Mount& m) {
    std::int32_t mm = m.sensor->get();
    return (mm == PROS_ERR || mm <= 0 || mm >= 9999) ? -1 : mm / 25.4;
}

bool still() {
    return std::fabs(odom_speed_ips) < RESET_STILL_IPS && std::fabs(odom_turn_rate_dps) < RESET_STILL_DPS;
}

double offSquare() {
    return std::fabs(std::remainder(getInertialHeading(), 90.0));
}

const char* const OFF_SQUARE = "off square";
const char* const TOO_FAR = "too far";
const char* const JUMP = "jump too big";

// The rules that don't depend on the reading.
const char* blocked(const Mount& m) {
    if (!m.sensor->is_installed()) return "unplugged";
    if (m.which == ResetSensor::Front && clawClosed()) {
        LiftStatus lift = liftStatus();
        if (!lift.sensorOk || !lift.zeroed || lift.height < RESET_FRONT_CLEAR_LIFT) return "claw closed";
    }
    if (offSquare() > RESET_MAX_OFF_SQUARE_DEG) return OFF_SQUARE;
    return nullptr;
}

// The rules that do, for one reading (inches, before the sensor's scale).
// Fills in axis/newPos when the geometry works, even if the jump check refuses.
ResetPreview check(const Mount& m, double rawIn) {
    ResetPreview p{true, rawIn > 0, rawIn, rawIn * m.scale, false, '-', 0, ""};
    if (const char* why = blocked(m)) {
        p.why = why;
        return p;
    }
    if (!p.seesWall) {
        p.why = "sees nothing";
        return p;
    }
    if (p.readingIn > m.maxRange) {
        p.why = TOO_FAR;
        return p;
    }
    // RW's own checks, so the reason can be named (RW's reset just says no).
    int axis = distanceResetPosition(x_pos, y_pos, getInertialHeading(), m.x, m.y, m.facing,
                                     p.readingIn * distance_sensor_scale, field_half_size,
                                     distance_reset_max_angle, distance_reset_corner_margin, p.newPos);
    if (axis == 0) {
        p.why = "corner/loader";
        return p;
    }
    p.axis = axis == 1 ? 'x' : 'y';
    double oldPos = axis == 1 ? x_pos : y_pos;
    if (distance_reset_max_jump > 0 && std::fabs(p.newPos - oldPos) > distance_reset_max_jump) {
        p.why = JUMP;
        return p;
    }
    p.ok = true;
    return p;
}

void report(const Mount& m, const char* why, double value = NAN, const char* unit = "") {
    if (std::isnan(value)) {
        std::printf("Reset %-5s skipped: %s\n", m.name, why);
    } else {
        std::printf("Reset %-5s skipped: %s (%.1f %s)\n", m.name, why, value, unit);
    }
}

}  // namespace

const char* resetSensorName(ResetSensor sensor) { return mountOf(sensor).name; }

ResetPreview previewReset(ResetSensor sensor) {
    Mount m = mountOf(sensor);
    if (!m.sensor->is_installed()) return {false, false, -1, -1, false, '-', 0, "unplugged"};
    return check(m, readInches(m));
}

static int resetWith(const std::vector<ResetSensor>& sensors) {
    std::vector<Mount> mounts;
    for (ResetSensor s : sensors) {
        Mount m = mountOf(s);
        if (const char* why = blocked(m)) {
            report(m, why, why == OFF_SQUARE ? offSquare() : NAN, "deg");
        } else {
            mounts.push_back(m);
        }
    }
    if (mounts.empty()) return 0;

    std::uint32_t start = pros::millis();
    while (!still() && pros::millis() - start < static_cast<std::uint32_t>(RESET_SETTLE_MS)) pros::delay(10);
    if (!still()) {
        for (const Mount& m : mounts) report(m, "robot still moving");
        return 0;
    }

    // Read every sensor at the same moments, so two sensors cost no more time than one.
    std::vector<std::vector<double>> readings(mounts.size());
    std::vector<bool> blind(mounts.size(), false);
    for (int i = 0; i < RESET_SAMPLES; i++) {
        if (i) pros::delay(RESET_SAMPLE_MS);
        for (std::size_t k = 0; k < mounts.size(); k++) {
            double in = readInches(mounts[k]);
            if (in < 0) {
                blind[k] = true;  // no wall in the beam on at least one reading
            } else {
                readings[k].push_back(in);
            }
        }
    }

    int used = 0;
    for (std::size_t k = 0; k < mounts.size(); k++) {
        const Mount& m = mounts[k];
        if (blind[k]) {
            report(m, "sees nothing on some readings");
            continue;
        }
        std::vector<double>& r = readings[k];
        std::sort(r.begin(), r.end());
        double spread = (r.back() - r.front()) * m.scale;
        if (spread > RESET_MAX_SPREAD_IN) {
            report(m, "readings disagree", spread, "in");
            continue;
        }
        ResetPreview p = check(m, r[r.size() / 2]);
        if (!p.ok) {
            double value = p.why == TOO_FAR ? p.readingIn
                         : p.why == JUMP    ? p.newPos - (p.axis == 'x' ? x_pos : y_pos)
                                            : NAN;
            report(m, p.why, value, "in");
            continue;
        }
        double oldPos = p.axis == 'x' ? x_pos : y_pos;
        if (!resetPositionWithReading(p.readingIn, m.x, m.y, m.facing, field_half_size)) {
            report(m, "RW refused it");
            continue;
        }
        std::printf("Reset %-5s %5.1f in: %c %.2f -> %.2f\n", m.name, p.readingIn, p.axis, oldPos,
                    p.axis == 'x' ? x_pos : y_pos);
        used++;
    }
    return used;
}

int resetFrom(std::initializer_list<ResetSensor> sensors) {
    return resetWith(std::vector<ResetSensor>(sensors));
}

bool resetFrom(ResetSensor sensor) {
    return resetWith({sensor}) == 1;
}

int resetNearWalls() {
    double heading = getInertialHeading();
    std::vector<ResetSensor> close;
    for (ResetSensor s : {ResetSensor::Front, ResetSensor::Back, ResetSensor::Left, ResetSensor::Right}) {
        Mount m = mountOf(s);
        double ox = 0, oy = 0;
        localToGlobal(m.x, m.y, heading * M_PI / 180, ox, oy);
        double beam = (heading + m.facing) * M_PI / 180;
        bool hitsXWall = false;
        double otherGap = 0;
        double expected = beamToWall(x_pos + ox, y_pos + oy, std::sin(beam), std::cos(beam), field_half_size,
                                     hitsXWall, otherGap);
        if (expected < m.maxRange - 1) close.push_back(s);  // 1 in spare for drift
    }
    return close.empty() ? 0 : resetWith(close);
}
