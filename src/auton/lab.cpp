// Simulator only: runs a route written as a text script (the route lab). See auton.hpp.
#ifdef RW_SIM_PROS
#include "auton.hpp"
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace auton {

// ---- Route lab (simulator only) -----------------------------------------------
// Runs a route written as a text script, so route shapes can be tried without
// rebuilding: ROUTE_SCRIPT=file.txt sim/build/rw-sim "Lab" --start X,Y,H ...
// One command per line, everything in red coordinates; each maps 1:1 onto the
// helpers above, so a script that works is copied into a route function as is.
//   start X Y HEADING [LIFT]            startAtToggle
//   score GOAL LIFT [PT ...]            scoreOn through the points
//   back INCHES [PT ...]                backAway
//   take PIECE LIFT [PT ...]            take through the points
//   wall STACK FACE SIDE LIFT [flow] [PT ...]   grabWallStack (flow: a curved approach through the points)
//   aim X                               shifts the next wall stack's aim X in along the wall (wallAimNext)
//   curve [rev] [stop] PT ...           a plain curve
//   gapback GOAL                        backOffToTurn: the slow back-off for between a goal and a wall
//   face PT / turn HEADING              turn on the spot
//   oldscore GOAL LIFT / oldtake PIECE LIFT   the stop-and-go helpers
//   toggle X Y HEADING [PT ...]         flipToggleAt: a second toggle (bottom: toggle -6 -62.61 0)
//   sidetoggle X Y [ram] PT ...         sideToggle (ram: slides along the wall on purpose)
//   (old form) sidetoggle X Y PT ...    sideToggle: flips side-on, starting when the centre is within 3 in of X,Y
//   wait MS / lift LIFT / settle
// PT is x,y or x,y,speed. GOAL: RG RGB BG or x,y. PIECE: YF YN ME or x,y.
// STACK: W1 W2. LIFT: GRAB CARRY A0 A1 A2 N0 N1 or a number.
bool labPoint(const std::string& t, Via& v) {
    static const std::map<std::string, Pt> named = {
        {"RG", RED_GOAL_LEFT}, {"RGB", RED_GOAL_BOTTOM}, {"BG", BLACK_GOAL_LEFT},
        {"YF", DIAG_FAR}, {"YN", DIAG_NEAR}, {"ME", MID_EDGE}, {"M2", MID_BOTTOM}, {"BK2", BLACK_GOAL_BOTTOM},
        {"W1", STACK_BY_RED_GOAL}, {"W2", STACK_BY_BLACK_GOAL}, {"W3", STACK_BY_RED_GOAL_BOTTOM}, {"W4", STACK_BY_BLACK_GOAL_BOTTOM}};
    auto it = named.find(t);
    if (it != named.end()) {
        v = {it->second.x, it->second.y, 1};
        return true;
    }
    double x = 0, y = 0, s = 1;
    int n = std::sscanf(t.c_str(), "%lf,%lf,%lf", &x, &y, &s);
    if (n < 2) return false;
    v = {x, y, n == 3 ? s : 1};
    return true;
}

double labLift(const std::string& t) {
    static const std::map<std::string, double> named = {
        {"GRAB", LIFT_GRAB}, {"CARRY", LIFT_CARRY},
        {"A0", LIFT_ALLIANCE_GOAL}, {"A1", LIFT_ALLIANCE_GOAL + LIFT_PER_PIECE}, {"A2", LIFT_ALLIANCE_GOAL + 2 * LIFT_PER_PIECE},
        {"N0", LIFT_NEUTRAL_GOAL}, {"N1", LIFT_NEUTRAL_GOAL + LIFT_PER_PIECE}};
    auto it = named.find(t);
    return it != named.end() ? it->second : std::atof(t.c_str());
}

const char* labName(const std::string& t) {
    static std::map<std::string, std::string> names = {
        {"RG", "alliance goal"}, {"RGB", "bottom alliance goal"}, {"BG", "black goal"}, {"BK2", "bottom black goal"},
        {"M2", "Cup+Pin at the Midfield's bottom edge"}, {"W3", "wall stack by the bottom alliance goal"},
        {"YF", "yellow Cup+Pin by the corner"}, {"YN", "yellow Cup+Pin by the Midfield"}, {"ME", "Cup+Pin at the Midfield's edge"},
        {"W1", "wall stack by the alliance goal"}, {"W2", "wall stack by the black goal"}, {"W4", "wall stack by the bottom black goal"}};
    auto it = names.find(t);
    if (it == names.end()) it = names.emplace(t, "thing at " + t).first;
    return it->second.c_str();
}

void lab() {
    const char* file = std::getenv("ROUTE_SCRIPT");
    std::ifstream in(file ? file : "");
    if (!in) {
        say("Lab: set ROUTE_SCRIPT to a route script\n");
        return;
    }
    std::string line;
    int number = 0;
    while (std::getline(in, line)) {
        number++;
        std::istringstream words(line.substr(0, line.find('#')));
        std::vector<std::string> t;
        for (std::string word; words >> word;) t.push_back(word);
        if (t.empty()) continue;
        auto points = [&](size_t from) {
            std::vector<Via> pts;
            for (size_t i = from; i < t.size(); i++) {
                Via v;
                if (labPoint(t[i], v)) pts.push_back(v);
                else say("Lab line %d: '%s' is not a point\n", number, t[i].c_str());
            }
            return pts;
        };
        Via target;
        const std::string& c = t[0];
        if (c == "start" && t.size() >= 4) {
            startAtToggle({std::atof(t[1].c_str()), std::atof(t[2].c_str())}, std::atof(t[3].c_str()), t.size() > 4 ? labLift(t[4]) : LIFT_ALLIANCE_GOAL, false);
        } else if (c == "score" && t.size() >= 3 && labPoint(t[1], target)) {
            scoreOn({target.x, target.y}, labLift(t[2]), labName(t[1]), points(3));
        } else if (c == "oldscore" && t.size() >= 3 && labPoint(t[1], target)) {
            score({target.x, target.y}, labLift(t[2]), labName(t[1]));
        } else if (c == "take" && t.size() >= 3 && labPoint(t[1], target)) {
            take({target.x, target.y}, labName(t[1]), labLift(t[2]), points(3));
        } else if (c == "oldtake" && t.size() >= 3 && labPoint(t[1], target)) {
            grab({target.x, target.y}, labName(t[1]), labLift(t[2]));
        } else if (c == "back" && t.size() >= 2) {
            backAway(std::atof(t[1].c_str()), points(2));
        } else if (c == "wall" && t.size() >= 5 && labPoint(t[1], target)) {
            bool flow = t.size() > 5 && t[5] == "flow";
            bool backOut = t.back() != "noback";
            if (!backOut) t.pop_back();
            bool pivot = t.size() > 5 && t[5] == "pivot";
            grabWallStack({target.x, target.y}, std::atof(t[2].c_str()), std::atoi(t[3].c_str()), labName(t[1]), labLift(t[4]), flow, points(flow || pivot ? 6 : 5), backOut, !pivot);
        } else if (c == "wallin") {
            // A reverse curve that ends alongside a wall and may touch it on the way in (planned).
            curve(points(1), true, true, TOP_IPS);
            step("onto the wall");
        } else if (c == "swing" && t.size() >= 3) {
            swing(turned(std::atof(t[1].c_str())), std::atof(t[2].c_str()), 900, t.size() < 4 || t[3] != "roll", knob("SWING_VOLTS", 12));
            step("swing");
        } else if (c == "curve") {
            size_t i = 1;
            bool rev = false, stop = false;
            for (; i < t.size() && (t[i] == "rev" || t[i] == "stop"); i++) (t[i] == "rev" ? rev : stop) = true;
            curve(points(i), rev, stop, TOP_IPS);
            step("curve");
        } else if (c == "gapback" && t.size() >= 2 && labPoint(t[1], target)) {
            gapBack({target.x, target.y});
        } else if (c == "face" && t.size() >= 2 && labPoint(t[1], target)) {
            faceTo({target.x, target.y});
        } else if (c == "turn" && t.size() >= 2) {
            turnToAngle(turned(std::atof(t[1].c_str())), 900, knob("TURN_EXIT", 1) > 0);
            step("turn");
        } else if (c == "wait" && t.size() >= 2) {
            pros::delay(std::atoi(t[1].c_str()));
        } else if (c == "lift" && t.size() >= 2) {
            liftTo(labLift(t[1]));
        } else if (c == "toggle" && t.size() >= 4) {
            bool rev = t.size() > 4 && t[4] == "rev";
            bool roll = t.back() != "noroll";
            if (!roll) t.pop_back();
            flipToggleAt({std::atof(t[1].c_str()), std::atof(t[2].c_str())}, std::atof(t[3].c_str()), points(rev ? 5 : 4), rev, roll);
        } else if (c == "sidetoggle" && t.size() >= 4) {
            bool ram = t.size() > 3 && t[3] == "ram";
            sideToggle({std::atof(t[1].c_str()), std::atof(t[2].c_str())}, points(ram ? 4 : 3), ram);
        } else if (c == "skipnear" && t.size() >= 2) {
            wallLineupSkipNext = std::atof(t[1].c_str());
        } else if (c == "approach" && t.size() >= 2) {
            wallApproachNext = std::atof(t[1].c_str());
        } else if (c == "aim" && t.size() >= 2) {
            wallAimNext = std::atof(t[1].c_str());
        } else if (c == "testturn") {
            testTurn();
        } else if (c == "testdrive") {
            testDrive();
        } else if (c == "settle") {
            settle();
        } else {
            say("Lab line %d: don't understand '%s'\n", number, line.c_str());
        }
    }
}

}  // namespace auton

void labRoute() {
    using namespace auton;
    profile_max_accel = knob("ACCEL", profile_max_accel);  // lab only: try other drive limits
    path_max_lateral_accel = knob("LAT", path_max_lateral_accel);
    if (knob("MCL", 0) > 0) startMcl();  // lab only: RW's particle filter, to compare
    lab();
}
#endif
