#include "settings.hpp"

#include <cstdio>
#include <string>

#include "autonSelector.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "underglow.hpp"

namespace settings {
namespace {

constexpr uint32_t SETTLE_MS = 1000; // wait for the picks to stop changing before writing
constexpr uint32_t RETRY_MS = 5000;  // after a failed write

std::string displayName(int index) {
    if (index >= 0 && index < (int)selector::matchScreens.size()) return selector::matchScreens[index].name;
    return "OFF";
}

std::string animationName(int index) {
    if (index >= 0 && index < (int)underglow::animations.size()) return underglow::animations[index].name;
    return "";
}

// everything the file holds, in the order it's written
std::string snapshot() {
    std::string out = "# Override 1940D brain screen picks. Saved by the robot, delete this file to reset.\n";
    out += "auton=" + selector::selectedName() + "\n";
    out += std::string("alliance=") + (selector::isRed() ? "RED" : "BLUE") + "\n";
    out += "display=" + displayName(selector::matchScreenIndex()) + "\n";
    out += "leds_idle=" + animationName(underglow::idleIndex()) + "\n";
    out += "leds_match=" + animationName(underglow::matchIndex()) + "\n";
    return out;
}

void applySetting(const std::string& key, const std::string& value) {
    if (key == "auton") {
        selector::selectByName(value);
    } else if (key == "alliance") {
        if (value == "RED") selector::setAlliance(selector::Alliance::RED);
        if (value == "BLUE") selector::setAlliance(selector::Alliance::BLUE);
    } else if (key == "display") {
        for (int i = 0; i < (int)selector::matchScreens.size(); i++)
            if (value == selector::matchScreens[i].name) selector::setMatchScreen(i);
        if (value == "OFF") selector::setMatchScreen((int)selector::matchScreens.size());
    } else if (key == "leds_idle" || key == "leds_match") {
        for (int i = 0; i < (int)underglow::animations.size(); i++) {
            if (value != underglow::animations[i].name) continue;
            if (key == "leds_idle") underglow::setIdle(i);
            else underglow::setMatch(i);
        }
    }
}

// applies the file's picks and hands back what it held. false if there's no file yet
bool load(std::string& contents) {
    FILE* file = fopen(SETTINGS_FILE, "r");
    if (!file) return false;
    char line[160];
    while (fgets(line, sizeof(line), file)) {
        contents += line;
        std::string text = line;
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
        if (text.empty() || text[0] == '#') continue;
        size_t eq = text.find('=');
        if (eq == std::string::npos) continue;
        applySetting(text.substr(0, eq), text.substr(eq + 1));
    }
    fclose(file);
    return true;
}

bool save(const std::string& contents) {
    FILE* file = fopen(SETTINGS_FILE, "w");
    if (!file) return false;
    bool ok = fputs(contents.c_str(), file) >= 0;
    ok = (fclose(file) == 0) && ok;
    return ok;
}

std::string saved;   // what the file holds now
std::string waiting; // changed picks, written once they've settled
uint32_t waitingSince = 0;
uint32_t lastFailure = 0;
bool failed = false;

// checks for changed picks and writes them. Runs every 250 ms on the settings task
void saveIfChanged() {
    std::string now = snapshot();
    if (now == saved) return;
    uint32_t t = pros::millis();
    if (now != waiting) {
        waiting = now;
        waitingSince = t;
        return;
    }
    if (t - waitingSince < SETTLE_MS) return;
    if (failed && t - lastFailure < RETRY_MS) return;

    if (save(now)) {
        saved = now;
        if (failed) selector::notify("Saving picks to the SD card again");
        failed = false;
    } else {
        if (!failed) selector::notify("Couldn't save picks to the SD card");
        failed = true;
        lastFailure = t;
    }
}

} // namespace

bool init() {
    static bool started = false;
    if (started) return true;
    started = true;

    if (!pros::usd::is_installed()) {
        printf("settings: no SD card, picks won't be remembered\n");
        selector::notify("No SD card: picks won't be remembered", 6000);
        return false;
    }
    // `saved` is what's in the file, so if it's missing something (a renamed auton, a
    // setting added since) the up to date picks get written back a second later
    if (load(saved)) {
        printf("settings: loaded %s\n", SETTINGS_FILE);
    } else {
        printf("settings: no %s yet, it'll be made now\n", SETTINGS_FILE);
    }

    pros::Task(
        [] {
            while (true) {
                saveIfChanged();
                pros::delay(250);
            }
        },
        "settings");
    return true;
}

} // namespace settings
