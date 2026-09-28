// Runs the real relay() and refine() from tuner.cpp against a simulated
// drivetrain with velocity lag, dead time and static friction, driven by a
// copy of LemLib's PID and motion exit rules.
// Build and run with test/run_tuner_sim.sh.
#include "tuner.cpp"
#include <cassert>
#include <deque>

struct Plant { double K, tau, L, stiction; };
static Plant plant;
static double simTime = 0, x = 0, v = 0;
static int command = 0;
static std::deque<int> pipe;

// Simulated LemLib motion.
static lemlib::PID* activePid = nullptr;
static bool moving = false;
static bool strict = true;
static double worstFinal = 0;
static double moveTarget = 0, tolerance = 0;
static int smallMs = 0, largeMs = 0, elapsedMs = 0, timeoutMs = 0;

// Same math as LemLib's PID::update (checked against its disassembly).
struct PidState { float kP, kI, kD, windup; bool flip; float integral, prev; };
lemlib::PID::PID(float kP, float kI, float kD, float windup, bool flip)
    : kP(kP), kI(kI), kD(kD), windupRange(windup), signFlipReset(flip) {}
float lemlib::PID::update(float error) {
    integral += error;
    if ((error < 0) != (prevError < 0) && signFlipReset) integral = 0;
    if (std::fabs(error) > windupRange && windupRange != 0) integral = 0;
    float out = error * kP + integral * kI + (error - prevError) * kD;
    prevError = error;
    return out;
}
void lemlib::PID::reset() { integral = 0; prevError = 0; }
bool lemlib::Chassis::isInMotion() const { return moving; }
void lemlib::Chassis::cancelAllMotions() { moving = false; command = 0; }

static void startMove(double delta, int timeout) {
    moveTarget = x + delta;
    activePid->reset();
    smallMs = largeMs = elapsedMs = 0;
    timeoutMs = timeout;
    moving = true;
}

extern "C" uint32_t millis(void) { return static_cast<uint32_t>(simTime * 1000 + 0.5); }
extern "C" void delay(const uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        if (moving && millis() % 10 == 0) {
            float e = moveTarget - x;
            command = static_cast<int>(std::clamp(activePid->update(e), -127.0f, 127.0f));
            smallMs = std::fabs(e) < tolerance ? smallMs + 10 : 0;
            largeMs = std::fabs(e) < tolerance * 2 ? largeMs + 10 : 0;
            elapsedMs += 10;
            if (smallMs >= 100 || largeMs >= 300 || elapsedMs >= timeoutMs) {
                moving = false;
                command = 0;
            }
        }
        pipe.push_back(command);
        int u = pipe.size() > plant.L * 1000 ? (pipe.pop_front(), pipe.front()) : 0;
        // Friction: stuck until the motors push past it, and drag while moving.
        double push;
        if (std::fabs(v) < 1e-3) {
            push = std::abs(u) < plant.stiction ? 0 : u - std::copysign(plant.stiction, u);
        } else {
            push = u - std::copysign(plant.stiction, v);
        }
        double before = v;
        if (u == 0) push = -v / (plant.K * 0.02);  // hold: power 0 brakes to a stop in ~20 ms
        v += 0.001 * (plant.K * push - v) / plant.tau;
        if (before != 0 && (before > 0) != (v > 0)) v = 0;  // friction stops it, never reverses it
        x += 0.001 * v;
        simTime += 0.001;
    }
}
pros::v5::Controller::Controller(pros::controller_id_e_t id) : _id(id) {}
std::int32_t pros::v5::Controller::get_analog(pros::controller_analog_e_t) { return 0; }
pros::Controller controller(pros::E_CONTROLLER_MASTER);

bool driverBrakeHold = false;
void applyDriverBrakes() {}
void lemlib::Chassis::setBrakeMode(pros::motor_brake_mode_e) {}

static void summary(const char* name, const TuneResult& r) {
    std::printf("SUMMARY %-26s %s  kP %6.2f kI %6.3f kD %6.1f  worst landing %.3f\n", name,
                r.exact ? "ON TARGET" : "retry    ", r.kP, r.kI, r.kD, worstFinal);
}

template <std::size_t N>
static void run(const char* name, Plant p, int power, float hysteresis, const float (&targets)[N], float tol, int timeout) {
    plant = p;
    tolerance = tol / 2;  // same exit rule as drivetrain.cpp
    simTime = x = v = 0;
    command = 0;
    pipe.clear();

    TuneResult r = relay([] { return static_cast<float>(x); }, [](int u) { command = u; }, power, hysteresis);
    assert(r.ok);
    std::printf("%s relay:   kP %.3f  kD %.2f\n", name, r.kP, r.kD);

    lemlib::PID pid(r.kP, 0, r.kD, r.windup, true);
    activePid = &pid;
    auto move = [](float delta, int t) { startMove(delta, t); };
    for (int attempt = 1; attempt <= 3 && !r.exact; attempt++) {
        refine(r, pid, [] { return static_cast<float>(x); }, move, targets, tol, timeout);
        std::printf("%s refine %d: kP %.3f  kI %.4f  kD %.2f  windup %.1f  worst miss %.3f  %s\n", name, attempt,
                    r.kP, r.kI, r.kD, r.windup, r.miss, r.exact ? "ON TARGET" : "retry");
    }
    if (strict) assert(r.exact);
    worstFinal = 0;

    // Fresh moves with the final gains, including distances it wasn't tuned on.
    applyGains(pid, r);
    float lo = targets[0], hi = targets[N - 1];
    for (float d : {lo, -hi, (lo + targets[1]) / 2, -(targets[1] + hi) / 2, hi * 1.33f, -lo,
                    (lo + hi) / 2, -targets[1]}) {
        double start = x;
        startMove(d, timeout);
        while (moving) delay(10);
        delay(250);
        double err = (x - start) - d;
        std::printf("   %s %+6.1f -> moved %+8.3f  (off by %.3f)\n", name, d, x - start, std::fabs(err));
        worstFinal = std::max(worstFinal, std::fabs(err));
        if (strict) assert(std::fabs(err) <= tol);  // untuned distances included
    }
    summary(name, r);
}

int main() {
    // adjust(): each kind of miss moves the gains the right way.
    Search k1, k2, k3, k4;
    TuneResult g; g.kP = 2; g.kD = 10;
    assert(!adjust(g, k1, 2, 0, 1, false, 0.5f) && g.kP < 2 && g.kD > 10);          // overshoot -> softer
    TuneResult s; s.kP = 2; s.kD = 10;
    assert(!adjust(s, k2, 0, 1, 0, false, 0.5f) && s.kI > 0 && s.kP == 2 && s.windup == 3);  // short -> integral
    TuneResult far; far.kP = 2;
    assert(!adjust(far, k3, 0, 5, 0, false, 0.5f) && far.windup >= 5);              // stalls far out -> windup reaches it
    float before = s.kI;
    assert(!adjust(s, k2, 2, 0, 0, false, 0.5f) && s.kI < before && s.kD > 10);    // overshoot with integral -> less kI, more damping
    TuneResult slow; slow.kP = 2; Search k5;
    assert(!adjust(slow, k5, 0, 0.1f, 0, true, 0.5f) && slow.kP > 2);         // landed but timed out -> firmer
    TuneResult stuck; stuck.kP = 2; Search k6;
    assert(!adjust(stuck, k6, 0, 2, 0, true, 0.5f) && stuck.kI > 0 && stuck.kP == 2);  // stalled (and timed out) -> integral
    TuneResult ok; ok.kP = 2;
    assert(adjust(ok, k4, 0.2f, 0.1f, 0.3f, false, 0.5f));                          // on target -> done

    // Realistic drivetrains, light to heavy and slick to sticky: must land.
    int n = 0;
    for (double friction : {0.0, 6.0, 12.0, 20.0}) {
        for (double lag : {0.05, 0.10, 0.15}) {
            char name[32];
            std::snprintf(name, sizeof(name), "turn f%.0f lag%.2f", friction, lag);
            run(name, {3.0, lag, 0.04, friction}, TURN_POWER, TURN_HYSTERESIS, TEST_DEGREES, TURN_EXACT_DEG, TURN_TIMEOUT_MS);
            std::snprintf(name, sizeof(name), "move f%.0f lag%.2f", friction, lag);
            run(name, {0.51, lag, 0.04, friction}, MOVE_POWER, MOVE_HYSTERESIS, TEST_INCHES, DRIVE_EXACT_IN, DRIVE_TIMEOUT_MS);
            n += 2;
        }
    }
    std::printf("%d realistic drivetrains all land on target\n", n);

    // Beyond what a real drive does: reported, not required.
    strict = false;
    for (double friction : {6.0, 30.0}) {
        for (double lag : {0.25}) {
            char name[32];
            std::snprintf(name, sizeof(name), "EXTREME turn f%.0f lag%.2f", friction, lag);
            run(name, {3.0, lag, 0.04, friction}, TURN_POWER, TURN_HYSTERESIS, TEST_DEGREES, TURN_EXACT_DEG, TURN_TIMEOUT_MS);
            std::snprintf(name, sizeof(name), "EXTREME move f%.0f lag%.2f", friction, lag);
            run(name, {0.51, lag, 0.04, friction}, MOVE_POWER, MOVE_HYSTERESIS, TEST_INCHES, DRIVE_EXACT_IN, DRIVE_TIMEOUT_MS);
        }
    }
    for (double friction : {30.0}) {
        for (double lag : {0.05, 0.10}) {
            char name[32];
            std::snprintf(name, sizeof(name), "EXTREME turn f%.0f lag%.2f", friction, lag);
            run(name, {3.0, lag, 0.04, friction}, TURN_POWER, TURN_HYSTERESIS, TEST_DEGREES, TURN_EXACT_DEG, TURN_TIMEOUT_MS);
            std::snprintf(name, sizeof(name), "EXTREME move f%.0f lag%.2f", friction, lag);
            run(name, {0.51, lag, 0.04, friction}, MOVE_POWER, MOVE_HYSTERESIS, TEST_INCHES, DRIVE_EXACT_IN, DRIVE_TIMEOUT_MS);
        }
    }
    std::puts("tuner sim passes");
}
