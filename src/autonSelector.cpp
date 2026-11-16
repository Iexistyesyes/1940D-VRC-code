#include "autonSelector.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "underglow.hpp"

// generated assets in src/assets/
extern "C" {
LV_IMAGE_DECLARE(logo_icon);
LV_FONT_DECLARE(override_font_12);
LV_FONT_DECLARE(override_font_16);
LV_FONT_DECLARE(override_font_22);
}

namespace selector {
namespace {

// ---------------------------------------------------------------- layout ----
constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 240; // usable height under the V5 status bar
constexpr int BAR_H = 34;     // tab bar
constexpr int CONTENT_Y = BAR_H + 2;
constexpr int CONTENT_H = SCREEN_H - CONTENT_Y;
constexpr int PAD = 6;

// AUTON and STATS both use a 4 x 3 grid
constexpr int GRID_COLS = 4;
constexpr int GRID_ROWS = 3;
constexpr int PER_PAGE = GRID_COLS * GRID_ROWS;
constexpr int CELL_W = (SCREEN_W - (GRID_COLS + 1) * PAD) / GRID_COLS;

// the preview on the DISPLAY tab is the whole screen at half size
constexpr int PREVIEW_W = SCREEN_W / 2;
constexpr int PREVIEW_H = SCREEN_H / 2;

enum Tab { TAB_AUTON, TAB_STATS, TAB_ALLIANCE, TAB_DISPLAY, TAB_LEDS, TAB_COUNT };

// ----------------------------------------------------------------- state ----
std::vector<Auton> autonList;
std::vector<Stat> statList;

std::atomic<int> selected{0};
std::atomic<Alliance> allianceColour{Alliance::RED};
std::atomic<int> matchChoice{0}; // index into matchScreens, matchScreens.size() means OFF

// asked for by other tasks, picked up by tick() on the LVGL task
std::atomic<int> pendingSelect{-1};
std::atomic<int> pendingAlliance{-1};
std::atomic<int> pendingScreen{-1}; // 1 = match screen, 0 = menu
std::atomic<int> pendingMatchChoice{-1};

// notify(): text handed over from any task, shown by tick()
pros::Mutex noticeMutex;
std::string noticeText;
uint32_t noticeMs = 0; // 0 = nothing waiting
lv_obj_t* noticeLabel = nullptr;
lv_timer_t* noticeTimer = nullptr;

lv_obj_t* menuScreen = nullptr;
lv_obj_t* matchScreen = nullptr;
bool showingMatch = false;
bool wasInMatch = false;

lv_obj_t* tabButtons[TAB_COUNT] = {};
lv_obj_t* pages[TAB_COUNT] = {};
int currentTab = TAB_AUTON;

lv_obj_t* autonButtons[PER_PAGE] = {};
lv_obj_t* autonName = nullptr;
lv_obj_t* autonDesc = nullptr;
lv_obj_t* pagerLabel = nullptr;
int autonPage = 0;

lv_obj_t* statTiles[PER_PAGE] = {};
lv_obj_t* statValues[PER_PAGE] = {};
uint32_t statColours[PER_PAGE] = {};
int statValueWidth = 0;
int statsTick = 0;

lv_obj_t* allianceButtons[2] = {};
lv_obj_t* allianceHints[2] = {};
lv_obj_t* allianceMark = nullptr;

lv_obj_t* previewBox = nullptr;
lv_obj_t* displayName = nullptr;
lv_obj_t* displayInfo = nullptr;
lv_obj_t* fullScreenButton = nullptr;

// LEDS tab: row 0 is during the match, row 1 before / after
enum LedRow { ROW_MATCH, ROW_IDLE, ROW_COUNT };
constexpr int MAX_PREVIEW_LEDS = 64;
struct LedRowUi {
    lv_obj_t* name = nullptr;
    lv_obj_t* tag = nullptr; // "ON NOW" / "TRYING"
    lv_obj_t* cells[MAX_PREVIEW_LEDS] = {};
    uint32_t shown[MAX_PREVIEW_LEDS] = {};
    uint32_t startMs = 0; // when the preview started, so it plays from the beginning
};
LedRowUi ledRows[ROW_COUNT];

// ---------------------------------------------------------------- styles ----
lv_style_t styleButton;
lv_style_t styleButtonChecked;
lv_style_t styleButtonPressed;
lv_style_t styleButtonDisabled;
lv_style_t styleTab;
lv_style_t stylePanel;

lv_color_t col(uint32_t hex) { return lv_color_hex(hex); }

void initStyles() {
    lv_style_init(&styleButton);
    lv_style_set_bg_color(&styleButton, col(theme::SURFACE));
    lv_style_set_bg_opa(&styleButton, LV_OPA_COVER);
    lv_style_set_border_color(&styleButton, col(theme::BORDER));
    lv_style_set_border_width(&styleButton, 2);
    lv_style_set_radius(&styleButton, theme::RADIUS);
    lv_style_set_text_color(&styleButton, col(theme::WHITE));
    lv_style_set_text_font(&styleButton, &override_font_12);
    lv_style_set_pad_all(&styleButton, 3);

    lv_style_init(&styleButtonChecked);
    lv_style_set_bg_color(&styleButtonChecked, col(theme::YELLOW));
    lv_style_set_bg_opa(&styleButtonChecked, LV_OPA_COVER);
    lv_style_set_border_color(&styleButtonChecked, col(theme::YELLOW));
    lv_style_set_text_color(&styleButtonChecked, col(theme::BLACK));

    lv_style_init(&styleButtonPressed);
    lv_style_set_bg_color(&styleButtonPressed, col(theme::YELLOW_PRESSED));
    lv_style_set_bg_opa(&styleButtonPressed, LV_OPA_COVER);
    lv_style_set_border_color(&styleButtonPressed, col(theme::YELLOW));
    lv_style_set_text_color(&styleButtonPressed, col(theme::BLACK));

    lv_style_init(&styleButtonDisabled);
    lv_style_set_opa(&styleButtonDisabled, LV_OPA_40);

    // tabs look like plain text until they're selected
    lv_style_init(&styleTab);
    lv_style_set_bg_opa(&styleTab, LV_OPA_TRANSP);
    lv_style_set_radius(&styleTab, theme::RADIUS);
    lv_style_set_text_color(&styleTab, col(theme::WHITE));
    lv_style_set_text_font(&styleTab, &override_font_12);

    lv_style_init(&stylePanel);
    lv_style_set_bg_color(&stylePanel, col(theme::SURFACE));
    lv_style_set_bg_opa(&stylePanel, LV_OPA_COVER);
    lv_style_set_border_color(&stylePanel, col(theme::YELLOW));
    lv_style_set_border_width(&stylePanel, 2);
    lv_style_set_border_side(&stylePanel, LV_BORDER_SIDE_LEFT);
    lv_style_set_radius(&stylePanel, theme::RADIUS);
    lv_style_set_pad_hor(&stylePanel, 6);
    lv_style_set_pad_ver(&stylePanel, 4);
}

// --------------------------------------------------------------- helpers ----
lv_obj_t* makeLabel(lv_obj_t* parent, const char* text, const lv_font_t* font, uint32_t colour) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, col(colour), 0);
    lv_label_set_text(label, text);
    return label;
}

// themed button with centred text, `id` comes back in the click callback
lv_obj_t* makeButton(lv_obj_t* parent, const char* text, lv_event_cb_t onClick, intptr_t id) {
    lv_obj_t* button = lv_button_create(parent);
    lv_obj_remove_style_all(button);
    lv_obj_add_style(button, &styleButton, 0);
    lv_obj_add_style(button, &styleButtonChecked, LV_STATE_CHECKED);
    lv_obj_add_style(button, &styleButtonPressed, LV_STATE_PRESSED);
    lv_obj_add_style(button, &styleButtonDisabled, LV_STATE_DISABLED);
    lv_obj_add_event_cb(button, onClick, LV_EVENT_CLICKED, (void*)id);

    lv_obj_t* label = lv_label_create(button); // colour and font follow the button
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, lv_pct(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);
    return button;
}

lv_obj_t* makePanel(lv_obj_t* parent) {
    lv_obj_t* panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    lv_obj_add_style(panel, &stylePanel, 0);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    return panel;
}

intptr_t eventId(lv_event_t* e) { return (intptr_t)lv_event_get_user_data(e); }

// ------------------------------------------------------------- AUTON tab ----
int autonPageCount() { return std::max(1, ((int)autonList.size() + PER_PAGE - 1) / PER_PAGE); }

// fills the 12 buttons with the autons on the current page
void refreshAutonPage() {
    autonPage = std::clamp(autonPage, 0, autonPageCount() - 1);
    for (int slot = 0; slot < PER_PAGE; slot++) {
        int index = autonPage * PER_PAGE + slot;
        bool used = index < (int)autonList.size();
        lv_obj_update_flag(autonButtons[slot], LV_OBJ_FLAG_HIDDEN, !used);
        if (!used) continue;
        lv_label_set_text(lv_obj_get_child(autonButtons[slot], 0), autonList[index].name.c_str());
        lv_obj_set_state(autonButtons[slot], LV_STATE_CHECKED, index == selected);
    }
    if (pagerLabel) lv_label_set_text_fmt(pagerLabel, "%d/%d", autonPage + 1, autonPageCount());
}

void applySelection(int index) {
    if (autonList.empty()) {
        lv_label_set_text(autonName, "NO AUTONS");
        lv_label_set_text(autonDesc, "Add some to selector::init() in main.cpp");
        refreshAutonPage();
        return;
    }
    if (index < 0 || index >= (int)autonList.size()) return;
    selected = index;
    autonPage = index / PER_PAGE;
    refreshAutonPage();
    lv_label_set_text(autonName, autonList[index].name.c_str());
    lv_label_set_text(autonDesc, autonList[index].description.c_str());
}

void autonClicked(lv_event_t* e) { applySelection(autonPage * PER_PAGE + (int)eventId(e)); }

void pagerClicked(lv_event_t* e) {
    autonPage += (int)eventId(e); // -1 or +1
    refreshAutonPage();
}

void buildAutonPage(lv_obj_t* page) {
    const int buttonH = 38;
    for (int slot = 0; slot < PER_PAGE; slot++) {
        lv_obj_t* button = makeButton(page, "", autonClicked, slot);
        lv_obj_set_size(button, CELL_W, buttonH);
        lv_obj_set_pos(button, PAD + (slot % GRID_COLS) * (CELL_W + PAD), PAD + (slot / GRID_COLS) * (buttonH + PAD));
        autonButtons[slot] = button;
    }

    // strip along the bottom: what's selected, and the page arrows if there's more than one page
    const int infoY = PAD + GRID_ROWS * (buttonH + PAD);
    lv_obj_t* info = makePanel(page);
    lv_obj_set_pos(info, PAD, infoY);
    lv_obj_set_size(info, SCREEN_W - 2 * PAD, CONTENT_H - infoY - PAD);

    const bool paged = autonPageCount() > 1;
    const int pagerW = paged ? 136 : 0;

    autonName = makeLabel(info, "", &override_font_16, theme::WHITE);
    lv_obj_set_pos(autonName, 0, 0);

    autonDesc = makeLabel(info, "", &lv_font_montserrat_12, theme::GREY);
    lv_label_set_long_mode(autonDesc, LV_LABEL_LONG_DOT);
    lv_obj_set_pos(autonDesc, 0, 20);
    lv_obj_update_layout(info); // so the panel knows its size before we measure it
    lv_obj_set_size(autonDesc, lv_obj_get_content_width(info) - pagerW, 2 * lv_font_get_line_height(&lv_font_montserrat_12));

    if (paged) {
        lv_obj_t* next = makeButton(info, LV_SYMBOL_RIGHT, pagerClicked, +1);
        lv_obj_set_style_text_font(next, &lv_font_montserrat_16, 0);
        lv_obj_set_size(next, 42, 40);
        lv_obj_align(next, LV_ALIGN_RIGHT_MID, 0, 0);

        lv_obj_t* prev = makeButton(info, LV_SYMBOL_LEFT, pagerClicked, -1);
        lv_obj_set_style_text_font(prev, &lv_font_montserrat_16, 0);
        lv_obj_set_size(prev, 42, 40);
        lv_obj_align(prev, LV_ALIGN_RIGHT_MID, -86, 0);

        pagerLabel = makeLabel(info, "", &override_font_12, theme::WHITE);
        lv_obj_set_width(pagerLabel, 44);
        lv_obj_set_style_text_align(pagerLabel, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(pagerLabel, LV_ALIGN_RIGHT_MID, -42, 0);
    }
}

// ------------------------------------------------------------- STATS tab ----
int statCount() { return std::min((int)statList.size(), PER_PAGE); }

void refreshStats() {
    for (int i = 0; i < statCount(); i++) {
        const Stat& stat = statList[i];
        double value = stat.read ? stat.read() : NAN;

        char text[24];
        uint32_t colour = theme::WHITE;
        if (!std::isfinite(value)) {
            // PROS hands back infinity when the device isn't plugged in
            snprintf(text, sizeof(text), "--");
            colour = theme::RED;
        } else {
            snprintf(text, sizeof(text), stat.format.c_str(), value);
            bool lowIsBad = stat.danger < stat.warn;
            if (lowIsBad ? value <= stat.danger : value >= stat.danger) colour = theme::RED;
            else if (lowIsBad ? value <= stat.warn : value >= stat.warn) colour = theme::YELLOW;
        }

        if (strcmp(lv_label_get_text(statValues[i]), text) != 0) {
            // big numbers drop down a font size so they still fit in the tile
            bool fits = lv_text_get_width(text, strlen(text), &override_font_22, 0) <= statValueWidth;
            lv_obj_set_style_text_font(statValues[i], fits ? &override_font_22 : &override_font_16, 0);
            lv_label_set_text(statValues[i], text);
        }
        if (colour != statColours[i]) {
            statColours[i] = colour;
            lv_obj_set_style_text_color(statValues[i], col(colour), 0);
            lv_obj_set_style_border_color(statTiles[i], col(colour == theme::WHITE ? theme::BORDER : colour), 0);
        }
    }
}

void buildStatsPage(lv_obj_t* page) {
    if (statList.empty()) {
        lv_obj_t* empty = makeLabel(page, "NO STATS ADDED", &override_font_12, theme::GREY);
        lv_obj_center(empty);
        return;
    }
    const int tileH = (CONTENT_H - (GRID_ROWS + 1) * PAD) / GRID_ROWS;
    for (int i = 0; i < statCount(); i++) {
        lv_obj_t* tile = makePanel(page);
        lv_obj_set_size(tile, CELL_W, tileH);
        lv_obj_set_pos(tile, PAD + (i % GRID_COLS) * (CELL_W + PAD), PAD + (i / GRID_COLS) * (tileH + PAD));

        lv_obj_t* name = makeLabel(tile, statList[i].name.c_str(), &override_font_12, theme::GREY);
        lv_label_set_long_mode(name, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(name, lv_pct(100));
        lv_obj_align(name, LV_ALIGN_TOP_LEFT, 0, 2);

        statValues[i] = makeLabel(tile, "--", &override_font_22, theme::WHITE);
        lv_obj_align(statValues[i], LV_ALIGN_BOTTOM_LEFT, 0, 0);
        statTiles[i] = tile;
        lv_obj_update_layout(tile);
        statValueWidth = lv_obj_get_content_width(tile);
        statColours[i] = 0; // not a real colour, so the first refresh always paints it
    }
}

// ---------------------------------------------------------- ALLIANCE tab ----
void applyAlliance(Alliance colour) {
    allianceColour = colour;
    for (int i = 0; i < 2; i++) {
        bool on = i == (int)colour;
        lv_obj_set_state(allianceButtons[i], LV_STATE_CHECKED, on);
        lv_label_set_text(allianceHints[i], on ? "SELECTED" : "TAP TO SELECT");
    }
    // little bar on the ALLIANCE tab so the colour shows from every tab
    lv_obj_set_style_bg_color(allianceMark, col(colour == Alliance::RED ? theme::RED : theme::BLUE), 0);
}

void allianceClicked(lv_event_t* e) { applyAlliance((Alliance)eventId(e)); }

void buildAlliancePage(lv_obj_t* page) {
    lv_obj_t* caption = makeLabel(page, "ALLIANCE COLOUR", &override_font_12, theme::GREY);
    lv_obj_set_pos(caption, PAD + 2, 10);

    static const char* names[2] = {"RED", "BLUE"};
    static const uint32_t colours[2] = {theme::RED, theme::BLUE};
    const int top = 30;
    const int gap = 8;
    const int buttonW = (SCREEN_W - 3 * gap) / 2;
    const int buttonH = CONTENT_H - top - 30;

    for (int i = 0; i < 2; i++) {
        lv_obj_t* button = lv_button_create(page);
        lv_obj_remove_style_all(button);
        lv_obj_add_style(button, &styleButton, 0);
        lv_obj_set_style_border_color(button, col(colours[i]), 0);
        lv_obj_set_style_text_color(button, col(colours[i]), 0);
        lv_obj_set_style_bg_color(button, col(colours[i]), LV_STATE_CHECKED);
        lv_obj_set_style_text_color(button, col(theme::WHITE), LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(button, lv_color_darken(col(colours[i]), LV_OPA_30), LV_STATE_PRESSED);
        lv_obj_set_style_text_color(button, col(theme::WHITE), LV_STATE_PRESSED);
        lv_obj_set_size(button, buttonW, buttonH);
        lv_obj_set_pos(button, gap + i * (buttonW + gap), top);
        lv_obj_add_event_cb(button, allianceClicked, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t* name = lv_label_create(button);
        lv_obj_set_style_text_font(name, &override_font_22, 0);
        lv_label_set_text(name, names[i]);
        lv_obj_align(name, LV_ALIGN_CENTER, 0, -10);

        allianceHints[i] = lv_label_create(button);
        lv_label_set_text(allianceHints[i], "");
        lv_obj_align(allianceHints[i], LV_ALIGN_CENTER, 0, 16);
        allianceButtons[i] = button;
    }

    lv_obj_t* footer =
        makeLabel(page, "Used by the auto toggle.  In code: selector::alliance()", &lv_font_montserrat_12, theme::GREY);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_LEFT, PAD + 2, -8);
}

// ----------------------------------------------------------- DISPLAY tab ----
int screenCount() { return (int)matchScreens.size(); }

// puts a match picture / animation in the middle of `parent`
lv_obj_t* makeMatchImage(lv_obj_t* parent, const MatchScreen& screen) {
    lv_obj_t* image;
    if (screen.animated) {
        image = lv_gif_create(parent);
        lv_gif_set_src(image, screen.image);
    } else {
        image = lv_image_create(parent);
        lv_image_set_src(image, screen.image);
    }
    lv_obj_center(image);
    return image;
}

void rebuildPreview() {
    lv_obj_clean(previewBox);
    int choice = matchChoice;
    if (choice >= screenCount()) {
        lv_obj_set_style_bg_color(previewBox, col(theme::BLACK), 0);
        lv_obj_center(makeLabel(previewBox, "MENU STAYS ON", &override_font_12, theme::GREY));
        return;
    }
    const MatchScreen& screen = matchScreens[choice];
    lv_obj_set_style_bg_color(previewBox, col(screen.background), 0);
    lv_obj_t* image = makeMatchImage(previewBox, screen);
    lv_image_set_scale(image, 256 * PREVIEW_W / SCREEN_W); // 256 = full size
}

void applyMatchChoice(int choice) {
    const int options = screenCount() + 1; // the extra one is OFF
    choice = ((choice % options) + options) % options; // wraps round at both ends
    matchChoice = choice;

    if (choice == screenCount()) {
        lv_label_set_text(displayName, "OFF");
        lv_label_set_text_fmt(displayInfo, "Menu stays on  -  %d / %d", choice + 1, options);
    } else {
        lv_label_set_text(displayName, matchScreens[choice].name);
        lv_label_set_text_fmt(displayInfo, "%s  -  %d / %d", matchScreens[choice].animated ? "Animation" : "Photo",
                              choice + 1, options);
    }
    lv_obj_set_state(fullScreenButton, LV_STATE_DISABLED, choice == screenCount());
    if (currentTab == TAB_DISPLAY && !showingMatch) rebuildPreview();
}

void openMatchScreen() {
    int choice = matchChoice;
    if (showingMatch || choice >= screenCount()) return;
    const MatchScreen& screen = matchScreens[choice];
    lv_obj_clean(previewBox); // so a GIF isn't playing twice
    lv_obj_clean(matchScreen);
    lv_obj_set_style_bg_color(matchScreen, col(screen.background), 0);
    makeMatchImage(matchScreen, screen);
    lv_screen_load(matchScreen);
    showingMatch = true;
}

void closeMatchScreen() {
    if (!showingMatch) return;
    lv_screen_load(menuScreen);
    lv_obj_clean(matchScreen);
    showingMatch = false;
    if (currentTab == TAB_DISPLAY) rebuildPreview();
}

void displayArrowClicked(lv_event_t* e) { applyMatchChoice(matchChoice + (int)eventId(e)); }

void buildDisplayPage(lv_obj_t* page) {
    // left: half size copy of what the brain will show
    previewBox = lv_obj_create(page);
    lv_obj_remove_style_all(previewBox);
    lv_obj_set_style_bg_opa(previewBox, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(previewBox, col(theme::BORDER), 0);
    lv_obj_set_style_border_width(previewBox, 1, 0);
    lv_obj_set_pos(previewBox, PAD + 2, 8);
    lv_obj_set_size(previewBox, PREVIEW_W + 2, PREVIEW_H + 2);
    lv_obj_remove_flag(previewBox, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* hint = makeLabel(page, "Comes up by itself when the match starts. Tap the brain to get the menu back.",
                               &lv_font_montserrat_12, theme::GREY);
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hint, PREVIEW_W + 2);
    lv_obj_set_pos(hint, PAD + 2, 8 + PREVIEW_H + 2 + 8);

    // right: which one, and the buttons
    const int left = PAD + 2 + PREVIEW_W + 2 + 10;
    const int width = SCREEN_W - left - (PAD + 2);

    lv_obj_t* caption = makeLabel(page, "DURING MATCH", &override_font_12, theme::YELLOW);
    lv_obj_set_pos(caption, left, 10);

    displayName = makeLabel(page, "", &override_font_16, theme::WHITE);
    lv_label_set_long_mode(displayName, LV_LABEL_LONG_DOT); // two lines, then dots
    lv_obj_set_size(displayName, width, 2 * lv_font_get_line_height(&override_font_16));
    lv_obj_set_pos(displayName, left, 28);

    displayInfo = makeLabel(page, "", &lv_font_montserrat_12, theme::GREY);
    lv_obj_set_pos(displayInfo, left, 70);

    const int arrowW = (width - PAD) / 2;
    lv_obj_t* prev = makeButton(page, LV_SYMBOL_LEFT, displayArrowClicked, -1);
    lv_obj_set_style_text_font(prev, &lv_font_montserrat_16, 0);
    lv_obj_set_size(prev, arrowW, 44);
    lv_obj_set_pos(prev, left, 96);

    lv_obj_t* next = makeButton(page, LV_SYMBOL_RIGHT, displayArrowClicked, +1);
    lv_obj_set_style_text_font(next, &lv_font_montserrat_16, 0);
    lv_obj_set_size(next, arrowW, 44);
    lv_obj_set_pos(next, left + arrowW + PAD, 96);

    fullScreenButton = makeButton(page, "FULL SCREEN", [](lv_event_t*) { openMatchScreen(); }, 0);
    lv_obj_set_size(fullScreenButton, width, 44);
    lv_obj_set_pos(fullScreenButton, left, 96 + 44 + PAD);
}

// -------------------------------------------------------------- LEDS tab ----
int ledCount() { return std::min<int>(underglow::LEDS_PER_SIDE, MAX_PREVIEW_LEDS); }
int ledChoice(int row) { return row == ROW_MATCH ? underglow::matchIndex() : underglow::idleIndex(); }

void refreshLedRow(int row) {
    LedRowUi& ui = ledRows[row];
    int choice = ledChoice(row);
    if (underglow::animations.empty()) {
        lv_label_set_text(ui.name, "NO ANIMATIONS");
        return;
    }
    lv_label_set_text(ui.name, underglow::animations[choice].name);
    ui.startMs = lv_tick_get();
    for (uint32_t& colour : ui.shown) colour = 0xFFFFFFFF; // not a real colour, so every cell repaints
}

// draws the strip previews and the ON NOW tags. Only runs while the LEDS tab is open
void refreshLedPreview() {
    underglow::Showing showing = underglow::showing();
    for (int row = 0; row < ROW_COUNT; row++) {
        LedRowUi& ui = ledRows[row];
        bool on = row == ROW_MATCH ? showing != underglow::Showing::IDLE : showing == underglow::Showing::IDLE;
        lv_obj_update_flag(ui.tag, LV_OBJ_FLAG_HIDDEN, !on);
        if (on) lv_label_set_text(ui.tag, showing == underglow::Showing::TRYING ? "TRYING" : "ON NOW");

        if (underglow::animations.empty()) continue;
        const underglow::Animation& anim = underglow::animations[ledChoice(row)];
        uint32_t ms = lv_tick_elaps(ui.startMs);
        for (int i = 0; i < ledCount(); i++) {
            uint32_t colour = anim.look ? underglow::toScreen(anim.look(i, ms)) : 0;
            if (colour == ui.shown[i]) continue;
            ui.shown[i] = colour;
            lv_obj_set_style_bg_color(ui.cells[i], col(colour), 0);
        }
    }
}

void ledArrowClicked(lv_event_t* e) {
    intptr_t id = eventId(e); // row * 2 + (0 = back, 1 = forward)
    int row = (int)(id / 2);
    int count = (int)underglow::animations.size();
    if (count == 0) return;
    int choice = (ledChoice(row) + (id % 2 ? 1 : -1) + count) % count;
    if (row == ROW_MATCH) {
        underglow::setMatch(choice);
        underglow::tryOut(choice); // so you can see it on the robot
    } else {
        underglow::setIdle(choice);
        underglow::stopTryOut(); // it's on the strips already outside a match
    }
    refreshLedRow(row);
    refreshLedPreview();
}

void buildLedsPage(lv_obj_t* page) {
    static const char* captions[ROW_COUNT] = {"DURING MATCH", "BEFORE + AFTER"};
    const int rowH = (CONTENT_H - 3 * PAD) / 2;
    const int arrowW = 56;

    for (int row = 0; row < ROW_COUNT; row++) {
        LedRowUi& ui = ledRows[row];
        lv_obj_t* panel = makePanel(page);
        lv_obj_set_pos(panel, PAD, PAD + row * (rowH + PAD));
        lv_obj_set_size(panel, SCREEN_W - 2 * PAD, rowH);
        lv_obj_update_layout(panel);
        const int innerW = lv_obj_get_content_width(panel);
        const int leftW = innerW - 2 * arrowW - 2 * PAD - 4;

        lv_obj_t* caption = makeLabel(panel, captions[row], &override_font_12, theme::YELLOW);
        lv_obj_set_pos(caption, 0, 2);

        ui.tag = makeLabel(panel, "ON NOW", &override_font_12, theme::BLACK);
        lv_obj_set_style_bg_color(ui.tag, col(theme::YELLOW), 0);
        lv_obj_set_style_bg_opa(ui.tag, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(ui.tag, 4, 0);
        lv_obj_set_style_pad_hor(ui.tag, 5, 0);
        lv_obj_set_style_pad_ver(ui.tag, 1, 0);
        lv_obj_align_to(ui.tag, caption, LV_ALIGN_OUT_RIGHT_MID, 8, 0);

        ui.name = makeLabel(panel, "", &override_font_16, theme::WHITE);
        lv_label_set_long_mode(ui.name, LV_LABEL_LONG_DOT);
        lv_obj_set_width(ui.name, leftW);
        lv_obj_set_pos(ui.name, 0, 20);

        // the strip preview: one cell per LED on a side
        const int n = ledCount();
        const int gap = n <= 40 ? 2 : 1;
        const int cellW = std::max(1, (leftW - (n - 1) * gap) / n);
        for (int i = 0; i < n; i++) {
            lv_obj_t* cell = lv_obj_create(panel);
            lv_obj_remove_style_all(cell);
            lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(cell, col(theme::BLACK), 0);
            lv_obj_set_style_radius(cell, cellW >= 6 ? 2 : 0, 0);
            lv_obj_set_size(cell, cellW, 22);
            lv_obj_set_pos(cell, i * (cellW + gap), rowH - 8 - 22 - 6);
            lv_obj_remove_flag(cell, LV_OBJ_FLAG_CLICKABLE);
            ui.cells[i] = cell;
        }

        for (int dir = 0; dir < 2; dir++) {
            lv_obj_t* arrow = makeButton(panel, dir ? LV_SYMBOL_RIGHT : LV_SYMBOL_LEFT, ledArrowClicked, row * 2 + dir);
            lv_obj_set_style_text_font(arrow, &lv_font_montserrat_16, 0);
            lv_obj_set_size(arrow, arrowW, rowH - 20);
            lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, dir ? 0 : -(arrowW + PAD), 0);
        }
        refreshLedRow(row);
    }
}

// ------------------------------------------------------------------ tabs ----
void showTab(int tab) {
    for (int i = 0; i < TAB_COUNT; i++) {
        lv_obj_set_state(tabButtons[i], LV_STATE_CHECKED, i == tab);
        lv_obj_update_flag(pages[i], LV_OBJ_FLAG_HIDDEN, i != tab);
    }
    currentTab = tab;
    // the preview only exists while its tab is open, so an animation isn't running in the background
    if (tab == TAB_DISPLAY) rebuildPreview();
    else lv_obj_clean(previewBox);
    if (tab == TAB_STATS) refreshStats();
    if (tab == TAB_LEDS) {
        for (int row = 0; row < ROW_COUNT; row++) refreshLedRow(row);
        refreshLedPreview();
    }
}

void tabClicked(lv_event_t* e) { showTab((int)eventId(e)); }

void buildTabBar(lv_obj_t* screen) {
    lv_obj_t* icon = lv_image_create(screen);
    lv_image_set_src(icon, &logo_icon);
    lv_obj_set_pos(icon, 8, (BAR_H - logo_icon.header.h) / 2);

    static const char* names[TAB_COUNT] = {"AUTON", "STATS", "ALLIANCE", "DISPLAY", "LEDS"};
    const int left = 38;
    const int gap = 3;

    // each tab is as wide as its name plus an equal share of the space left over
    int textW[TAB_COUNT];
    int totalText = 0;
    for (int i = 0; i < TAB_COUNT; i++) {
        textW[i] = lv_text_get_width(names[i], strlen(names[i]), &override_font_12, 0);
        totalText += textW[i];
    }
    const int extra = (SCREEN_W - left - 4 - gap * (TAB_COUNT - 1) - totalText) / TAB_COUNT;
    int x = left;
    for (int i = 0; i < TAB_COUNT; i++) {
        lv_obj_t* tab = makeButton(screen, names[i], tabClicked, i);
        lv_obj_remove_style(tab, &styleButton, 0);
        lv_obj_add_style(tab, &styleTab, 0);
        lv_obj_set_style_pad_hor(tab, 0, 0);
        lv_obj_set_size(tab, textW[i] + extra, BAR_H - 8);
        lv_obj_set_pos(tab, x, 4);
        x += textW[i] + extra + gap;
        tabButtons[i] = tab;
    }

    allianceMark = lv_obj_create(tabButtons[TAB_ALLIANCE]);
    lv_obj_remove_style_all(allianceMark);
    lv_obj_set_style_bg_opa(allianceMark, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(allianceMark, 2, 0);
    lv_obj_set_size(allianceMark, textW[TAB_ALLIANCE], 3);
    lv_obj_align(allianceMark, LV_ALIGN_BOTTOM_MID, 0, -1);
    lv_obj_remove_flag(allianceMark, LV_OBJ_FLAG_CLICKABLE);

    // yellow line under the tab bar
    lv_obj_t* line = lv_obj_create(screen);
    lv_obj_remove_style_all(line);
    lv_obj_set_style_bg_color(line, col(theme::YELLOW), 0);
    lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_size(line, SCREEN_W, 2);
    lv_obj_set_pos(line, 0, BAR_H);
}

// --------------------------------------------------------------- notices ----
void hideNotice(lv_timer_t*) {
    lv_obj_add_flag(noticeLabel, LV_OBJ_FLAG_HIDDEN);
    noticeTimer = nullptr;
}

void showWaitingNotice() {
    std::string text;
    uint32_t ms = 0;
    if (!noticeMutex.try_lock()) return; // busy, try again next tick
    if (noticeMs) {
        text = noticeText;
        ms = noticeMs;
        noticeMs = 0;
    }
    noticeMutex.unlock();
    if (!ms) return;

    if (!noticeLabel) {
        // on the top layer, so it shows over the menu and the match picture
        noticeLabel = makeLabel(lv_layer_top(), "", &override_font_12, theme::BLACK);
        lv_obj_set_style_bg_color(noticeLabel, col(theme::YELLOW), 0);
        lv_obj_set_style_bg_opa(noticeLabel, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(noticeLabel, theme::RADIUS, 0);
        lv_obj_set_style_pad_hor(noticeLabel, 10, 0);
        lv_obj_set_style_pad_ver(noticeLabel, 6, 0);
        lv_label_set_long_mode(noticeLabel, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_max_width(noticeLabel, SCREEN_W - 4 * PAD, 0);
        lv_obj_set_style_text_align(noticeLabel, LV_TEXT_ALIGN_CENTER, 0);
    }
    lv_label_set_text(noticeLabel, text.c_str());
    lv_obj_remove_flag(noticeLabel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(noticeLabel, LV_ALIGN_BOTTOM_MID, 0, -PAD);
    if (noticeTimer) lv_timer_delete(noticeTimer);
    noticeTimer = lv_timer_create(hideNotice, ms, nullptr);
    lv_timer_set_repeat_count(noticeTimer, 1);
}

// ------------------------------------------------------------------ core ----
lv_obj_t* makeScreen() {
    lv_obj_t* screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen, col(theme::BLACK), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    return screen;
}

// runs 20 times a second on the LVGL task, so it's safe to touch the screen from here
void tick(lv_timer_t*) {
    int request = pendingSelect.exchange(-1);
    if (request >= 0) applySelection(request);
    request = pendingAlliance.exchange(-1);
    if (request >= 0) applyAlliance((Alliance)request);
    request = pendingScreen.exchange(-1);
    if (request == 1) openMatchScreen();
    if (request == 0) closeMatchScreen();
    request = pendingMatchChoice.exchange(-1);
    if (request >= 0) applyMatchChoice(request);
    showWaitingNotice();

    // match picture goes up when the field enables the robot, menu comes back when it's disabled
    bool inMatch = pros::competition::is_connected() && !pros::competition::is_disabled();
    if (inMatch != wasInMatch) {
        wasInMatch = inMatch;
        if (inMatch) openMatchScreen();
        else closeMatchScreen();
    }

    // stats update twice a second, and only while someone can see them
    if (!showingMatch && currentTab == TAB_STATS && ++statsTick >= 10) {
        statsTick = 0;
        refreshStats();
    }
    if (!showingMatch && currentTab == TAB_LEDS) refreshLedPreview();
}

// builds everything. Runs on the LVGL task (see init) so it can't clash with a screen redraw
void build(void*) {
    initStyles();
    menuScreen = makeScreen();
    matchScreen = makeScreen();
    lv_obj_add_event_cb(matchScreen, [](lv_event_t*) { closeMatchScreen(); }, LV_EVENT_CLICKED, nullptr);

    buildTabBar(menuScreen);
    for (int i = 0; i < TAB_COUNT; i++) {
        lv_obj_t* page = lv_obj_create(menuScreen);
        lv_obj_remove_style_all(page);
        lv_obj_set_pos(page, 0, CONTENT_Y);
        lv_obj_set_size(page, SCREEN_W, CONTENT_H);
        lv_obj_remove_flag(page, LV_OBJ_FLAG_SCROLLABLE);
        pages[i] = page;
    }
    buildAutonPage(pages[TAB_AUTON]);
    buildStatsPage(pages[TAB_STATS]);
    buildAlliancePage(pages[TAB_ALLIANCE]);
    buildDisplayPage(pages[TAB_DISPLAY]);
    buildLedsPage(pages[TAB_LEDS]);

    applySelection(selected);
    applyAlliance(allianceColour);
    applyMatchChoice(matchChoice);
    showTab(TAB_AUTON);

    lv_screen_load(menuScreen);
    lv_timer_create(tick, 50, nullptr);
}

} // namespace

// ------------------------------------------------------------------ API ----
void init(const std::vector<Auton>& autons, const std::vector<Stat>& stats, int defaultAuton) {
    static bool started = false;
    if (started) return;
    started = true;

    autonList = autons;
    statList = stats;
    if (defaultAuton >= 0 && defaultAuton < (int)autonList.size()) selected = defaultAuton;
    lv_async_call(build, nullptr);
}

int selectedIndex() { return selected; }

std::string selectedName() {
    int i = selected;
    if (i < 0 || i >= (int)autonList.size()) return "";
    return autonList[i].name;
}

void runSelected() {
    int i = selected;
    if (i < 0 || i >= (int)autonList.size()) return;
    if (autonList[i].run) autonList[i].run();
}

Alliance alliance() { return allianceColour; }

void select(int index) {
    if (index < 0 || index >= (int)autonList.size()) return;
    selected = index;
    pendingSelect = index;
}

void setAlliance(Alliance colour) {
    allianceColour = colour;
    pendingAlliance = (int)colour;
}

void showMatchScreen() { pendingScreen = 1; }
void showMenu() { pendingScreen = 0; }

bool selectByName(const std::string& name) {
    for (int i = 0; i < (int)autonList.size(); i++) {
        if (autonList[i].name == name) {
            select(i);
            return true;
        }
    }
    return false;
}

int matchScreenIndex() { return matchChoice; }

void setMatchScreen(int index) {
    if (index < 0 || index > (int)matchScreens.size()) return;
    matchChoice = index;
    pendingMatchChoice = index;
}

void notify(const std::string& text, uint32_t ms) {
    noticeMutex.lock();
    noticeText = text;
    noticeMs = ms ? ms : 1;
    noticeMutex.unlock();
}

Stat motorTemp(const std::string& name, pros::Motor& motor) {
    return {name, [&motor] { return motor.get_temperature(); }, "%.0f°C", 45, 55};
}

Stat motorTemp(const std::string& name, pros::MotorGroup& motors) {
    return {name,
            [&motors] {
                double hottest = 0;
                for (double temperature : motors.get_temperature_all()) hottest = std::max(hottest, temperature);
                return hottest;
            },
            "%.0f°C", 45, 55};
}

Stat battery() {
    return {"BATTERY", [] { return pros::battery::get_capacity(); }, "%.0f%%", 50, 25};
}

} // namespace selector
