#include "autonSelector.hpp"

#include <atomic>

#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"

// generated assets in src/assets/
extern "C" {
LV_IMAGE_DECLARE(logo_icon);
LV_IMAGE_DECLARE(logo_wordmark);
LV_FONT_DECLARE(override_font_12);
LV_FONT_DECLARE(override_font_16);
LV_FONT_DECLARE(override_font_22);
}

namespace selector {
namespace {

constexpr int MAX_AUTONS = 6;
constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 240; // usable height under the V5 status bar
constexpr int HEADER_H = 50;
constexpr int PAD = 8;

std::vector<Auton> autonList;
std::atomic<int> selected{0};
std::atomic<int> pendingSelect{-1}; // set from other tasks, applied on the LVGL task

pros::Mutex statusMutex;
std::string statusText;
bool statusDirty = false;

lv_obj_t* buttons[MAX_AUTONS] = {};
lv_obj_t* nameLabel = nullptr;
lv_obj_t* descLabel = nullptr;
lv_obj_t* indexLabel = nullptr;
lv_obj_t* statusLabel = nullptr;

lv_color_t col(uint32_t hex) { return lv_color_hex(hex); }

// ---------------------------------------------------------------- styles ----
lv_style_t styleButton;
lv_style_t styleButtonChecked;
lv_style_t styleButtonPressed;
lv_style_t stylePanel;

void initStyles() {
    lv_style_init(&styleButton);
    lv_style_set_bg_color(&styleButton, col(theme::SURFACE));
    lv_style_set_bg_opa(&styleButton, LV_OPA_COVER);
    lv_style_set_border_color(&styleButton, col(theme::BORDER));
    lv_style_set_border_width(&styleButton, 2);
    lv_style_set_radius(&styleButton, theme::RADIUS);
    lv_style_set_shadow_width(&styleButton, 0);
    lv_style_set_text_color(&styleButton, col(theme::WHITE));
    lv_style_set_text_font(&styleButton, &override_font_12);
    lv_style_set_pad_all(&styleButton, 4);

    lv_style_init(&styleButtonChecked);
    lv_style_set_bg_color(&styleButtonChecked, col(theme::YELLOW));
    lv_style_set_border_color(&styleButtonChecked, col(theme::YELLOW));
    lv_style_set_text_color(&styleButtonChecked, col(theme::BLACK));

    lv_style_init(&styleButtonPressed);
    lv_style_set_bg_color(&styleButtonPressed, col(theme::YELLOW_PRESSED));
    lv_style_set_border_color(&styleButtonPressed, col(theme::YELLOW));
    lv_style_set_text_color(&styleButtonPressed, col(theme::BLACK));

    lv_style_init(&stylePanel);
    lv_style_set_bg_color(&stylePanel, col(theme::SURFACE));
    lv_style_set_bg_opa(&stylePanel, LV_OPA_COVER);
    lv_style_set_border_color(&stylePanel, col(theme::YELLOW));
    lv_style_set_border_width(&stylePanel, 2);
    lv_style_set_border_side(&stylePanel, LV_BORDER_SIDE_LEFT);
    lv_style_set_radius(&stylePanel, theme::RADIUS);
    lv_style_set_pad_all(&stylePanel, 10);
}

// ------------------------------------------------------------ behaviour ----
void applySelection(int index) {
    if (autonList.empty()) return;
    if (index < 0 || index >= (int)autonList.size()) return;
    selected = index;

    for (int i = 0; i < MAX_AUTONS; i++) {
        if (!buttons[i]) continue;
        if (i == index) lv_obj_add_state(buttons[i], LV_STATE_CHECKED);
        else lv_obj_remove_state(buttons[i], LV_STATE_CHECKED);
    }
    lv_label_set_text(nameLabel, autonList[index].name.c_str());
    lv_label_set_text(descLabel, autonList[index].description.c_str());
    lv_label_set_text_fmt(indexLabel, "%d / %d", index + 1, (int)autonList.size());
}

void buttonClicked(lv_event_t* e) {
    applySelection((int)(intptr_t)lv_event_get_user_data(e));
}

// runs on the LVGL task, so it's safe to touch widgets here
void syncTimer(lv_timer_t*) {
    int pending = pendingSelect.exchange(-1);
    if (pending >= 0) applySelection(pending);

    if (statusMutex.take(0)) {
        if (statusDirty) {
            lv_label_set_text(statusLabel, statusText.c_str());
            statusDirty = false;
        }
        statusMutex.give();
    }
}

// --------------------------------------------------------------- layout ----
void buildHeader(lv_obj_t* screen) {
    lv_obj_t* icon = lv_image_create(screen);
    lv_image_set_src(icon, &logo_icon);
    lv_obj_set_pos(icon, PAD, (HEADER_H - logo_icon.header.h) / 2);

    lv_obj_t* wordmark = lv_image_create(screen);
    lv_image_set_src(wordmark, &logo_wordmark);
    lv_obj_set_pos(wordmark, PAD + logo_icon.header.w + 8, (HEADER_H - logo_wordmark.header.h) / 2);

    lv_obj_t* title = lv_label_create(screen);
    lv_obj_set_style_text_font(title, &override_font_16, 0);
    lv_obj_set_style_text_color(title, col(theme::YELLOW), 0);
    lv_label_set_text(title, "AUTON");
    lv_obj_align(title, LV_ALIGN_TOP_RIGHT, -PAD, 6);

    statusLabel = lv_label_create(screen);
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(statusLabel, col(theme::GREY), 0);
    lv_label_set_text(statusLabel, "");
    lv_obj_align(statusLabel, LV_ALIGN_TOP_RIGHT, -PAD, 28);

    // yellow stripe under the header, like the bolt cutting through the gear
    lv_obj_t* stripe = lv_obj_create(screen);
    lv_obj_remove_style_all(stripe);
    lv_obj_set_size(stripe, SCREEN_W - 2 * PAD, 3);
    lv_obj_set_pos(stripe, PAD, HEADER_H);
    lv_obj_set_style_bg_color(stripe, col(theme::YELLOW), 0);
    lv_obj_set_style_bg_opa(stripe, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(stripe, 2, 0);
}

void buildButtons(lv_obj_t* screen) {
    const int top = HEADER_H + 3 + PAD;
    const int areaH = SCREEN_H - top - PAD;
    const int cols = 2;
    const int rows = 3;
    const int btnW = 142;
    const int btnH = (areaH - (rows - 1) * PAD) / rows;

    int count = autonList.size() < MAX_AUTONS ? autonList.size() : MAX_AUTONS;
    for (int i = 0; i < count; i++) {
        lv_obj_t* btn = lv_button_create(screen);
        lv_obj_remove_style_all(btn);
        lv_obj_add_style(btn, &styleButton, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_add_style(btn, &styleButtonChecked, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_add_style(btn, &styleButtonPressed, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_size(btn, btnW, btnH);
        lv_obj_set_pos(btn, PAD + (i % cols) * (btnW + PAD), top + (i / cols) * (btnH + PAD));
        lv_obj_add_event_cb(btn, buttonClicked, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text(label, autonList[i].name.c_str());
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label, btnW - 12);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(label);

        buttons[i] = btn;
    }
}

void buildInfoPanel(lv_obj_t* screen) {
    const int top = HEADER_H + 3 + PAD;
    const int left = PAD + 2 * (142 + PAD);

    lv_obj_t* panel = lv_obj_create(screen);
    lv_obj_remove_style_all(panel);
    lv_obj_add_style(panel, &stylePanel, 0);
    lv_obj_set_pos(panel, left, top);
    lv_obj_set_size(panel, SCREEN_W - left - PAD, SCREEN_H - top - PAD);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(panel, 6, 0);

    lv_obj_t* caption = lv_label_create(panel);
    lv_obj_set_style_text_font(caption, &override_font_12, 0);
    lv_obj_set_style_text_color(caption, col(theme::YELLOW), 0);
    lv_label_set_text(caption, "SELECTED");

    indexLabel = lv_label_create(panel);
    lv_obj_set_style_text_font(indexLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(indexLabel, col(theme::GREY), 0);
    lv_label_set_text(indexLabel, "");
    lv_obj_add_flag(indexLabel, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_align(indexLabel, LV_ALIGN_TOP_RIGHT, 0, 0);

    nameLabel = lv_label_create(panel);
    lv_obj_set_style_text_font(nameLabel, &override_font_22, 0);
    lv_obj_set_style_text_color(nameLabel, col(theme::WHITE), 0);
    lv_label_set_long_mode(nameLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(nameLabel, lv_pct(100));
    lv_label_set_text(nameLabel, "");

    descLabel = lv_label_create(panel);
    lv_obj_set_style_text_font(descLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(descLabel, col(theme::GREY), 0);
    lv_label_set_long_mode(descLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(descLabel, lv_pct(100));
    lv_label_set_text(descLabel, "");
}

} // namespace

// ------------------------------------------------------------------ API ----
void init(const std::vector<Auton>& autons, int defaultAuton) {
    autonList = autons;

    lv_obj_t* screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen, col(theme::BLACK), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    initStyles();
    buildHeader(screen);
    buildButtons(screen);
    buildInfoPanel(screen);

    lv_screen_load(screen);
    applySelection(defaultAuton);
    lv_timer_create(syncTimer, 50, nullptr);
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

void select(int index) { pendingSelect = index; }

void setStatus(const std::string& text) {
    statusMutex.take();
    statusText = text;
    statusDirty = true;
    statusMutex.give();
}

} // namespace selector
