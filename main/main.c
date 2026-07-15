/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_event.h"
#include "lvgl.h"
#include "bsp_extra.h"
#include "bsp_display.h"
#include "bsp_illuminate.h"
#include "bsp_i2c.h"
#include "esp_wifi_remote.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "gt7parser/gt7wrapper.h"
#include "freertos/semphr.h"
#include "fonts/fonts.h"
#include "images/images.h"

// Die Event-Gruppe und das "Verbunden"-Bit
static EventGroupHandle_t wifi_event_group;
static SemaphoreHandle_t data_mutex = NULL;

#define WIFI_CONNECTED_BIT BIT0

#define LOCAL_PORT      33740
#define BUFFER_SIZE     1024       // Maximale Größe der empfangenen Daten

#define SERVER_PORT 1
#define SERVER_IP "192.168.1.1"
#define TARGET_IP   "192.168.178.24"  // Die IP-Adresse deines Zielgeräts (z.B. PlayStation)
#define TARGET_PORT 33739             // Der Ziel-Port

static const char *TAG = "UDP_RECEIVER";
#define UI_UPDATE_INTERVAL_MS   10
#define UI_SHIFT_ANIMATION_INTERVAL_MS 100

#define MAIN_TAG "MAIN"
#define MAIN_INFO(fmt, ...) ESP_LOGI(MAIN_TAG, fmt, ##__VA_ARGS__)
#define MAIN_DEBUG(fmt, ...) ESP_LOGD(MAIN_TAG, fmt, ##__VA_ARGS__)
#define MAIN_ERROR(fmt, ...) ESP_LOGE(MAIN_TAG, fmt, ##__VA_ARGS__)



static esp_ldo_channel_handle_t ldo3 = NULL;
static esp_ldo_channel_handle_t ldo4 = NULL;

static lv_subject_t speed_binding;
static lv_subject_t gear_binding;
static lv_subject_t rev_binding;
static lv_subject_t laptime_binding;
static lv_subject_t throttle_binding;
static lv_subject_t throttle_filtered_binding;
static lv_subject_t brake_binding;
static lv_subject_t brake_filtered_binding;
static lv_subject_t lapcount_binding;

static lv_style_t label_style;

lv_obj_t * ui_Screen1 = NULL;
lv_obj_t * ui_Speed = NULL;
lv_obj_t * ui_Gear = NULL;
lv_obj_t * ui_Rev = NULL;
lv_obj_t * ui_Brake = NULL;
lv_obj_t * ui_BrakeFiltered = NULL;
lv_obj_t * ui_Throttle = NULL;
lv_obj_t * ui_ThrottleFiltered = NULL;
lv_obj_t * ui_laptime = NULL;
lv_obj_t * ui_laptimelast = NULL;
lv_obj_t * ui_Container1 = NULL;
lv_obj_t * ui_currentLap = NULL;
lv_obj_t * ui_Lap = NULL;
lv_obj_t * ui_OilTempLabel = NULL;
lv_obj_t * ui_OilPressLabel = NULL;
lv_obj_t * ui_CoolantTempLabel = NULL;
lv_obj_t * ui_FuelLabel = NULL;
lv_obj_t * ui_BoostLabel = NULL;
lv_obj_t * ui_OilTempValueLabel = NULL;
lv_obj_t * ui_OilPressValueLabel = NULL;
lv_obj_t * ui_CoolantTempValueLabel = NULL;
lv_obj_t * ui_FuelValueLabel = NULL;
lv_obj_t * ui_BoostValueLabel = NULL;
lv_obj_t * ui_tyreTemp0 = NULL;
lv_obj_t * ui_tyreTemp1 = NULL;
lv_obj_t * ui_tyreTemp2 = NULL;
lv_obj_t * ui_tyreTemp3 = NULL;
lv_obj_t * ui_tyreTempContainer = NULL;
lv_obj_t * ui_tyreTempLabel = NULL;
lv_obj_t * ui_gridContainer = NULL;
lv_obj_t * ui_laptimeContainer = NULL;
lv_obj_t * ui_laptimeLabel = NULL;
lv_obj_t * ui_laptimelastLabel = NULL;
lv_obj_t * ui_difftolastLabel = NULL;
lv_obj_t * ui_difftolast = NULL;
lv_obj_t * ui_bestLapLabel = NULL;
lv_obj_t * ui_laptimeBest = NULL;

int speed_value = 0;
int rev_value = 0;
int gear = 0;
int laptime = 0;
int laptime_last = 0;
int laptime_best = 0;
int throttle_value = 0;
int throttle_filtered_value = 0;
int brake_value = 0;
int brake_filtered_value = 0;
float tyreTemperatures[4]; // 0 = FL, 1 = FR, 2 = RL, 3 = RR
int min_rev = 0;
int max_rev = 0;
float oil_temperature = 0;
float oil_pressure = 0;
float coolant_temperature = 0;
float boost = 0;
float fuel = 0;
int current_lap = 0;

int local_gear = 0;
int local_speed = 0;
int local_brake = 0;
int local_throttle = 0;
int local_laptime = 0;
int local_laptime_last = 0;
int local_rev = 0;
int local_min_rev = 0;
int local_max_rev = 0;
int local_brake_filtered_value = 0;
int local_throttle_filtered_value = 0;
float local_tyre_temps[4];
int rev_bar_start_value = 0;
int rev_bar_value = 0;
int time_diff_shift_animation = 0;
float local_oil_temperature = 0;
float local_oil_pressure = 0;
float local_coolant_temperature = 0;
float local_boost = 0;
float local_fuel = 0;
int local_lap = 0;
int local_diff_to_last = 0;
int local_laptime_last_save = 0;
int local_laptime_best = 0;


bool shift_inidicator_changed = false;
bool shift_indicator_last = false;
bool blink_on = false;
uint shift_animation_last_switch = 0; 

bool reverse_gear = false;
uint32_t bg_color = 0x010101;
uint32_t widget_bg_color = 0x417386;

// LV_FONT_DECLARE(ui_font_Arial36);
// LV_FONT_DECLARE(ui_font_Arial100);
// LV_FONT_DECLARE(ui_font_Arial140); 
// LV_FONT_DECLARE(ui_font_Arial180);
// LV_FONT_DECLARE(ui_font_Arial50);
// LV_FONT_DECLARE(ui_font_Consolas98);

// LV_IMG_DECLARE(ui_img_rev_bar_inactive_png);    // assets/rev_bar_inactive.png
// LV_IMG_DECLARE(ui_img_rev_bar_indicator_png);    // assets/rev_bar_indicator.png
// LV_IMG_DECLARE(ui_img_laptimer_png);    // assets/laptimer.png


static void init_fail_handler(const char *module_name, esp_err_t err) {
    while (1) {  // Infinite loop to help debug which module failed to initialize
        MAIN_ERROR("[%s] init failed: %s", module_name, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) 
{
    // ... deine restlichen Event-Abfragen ...

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // IP-Adresse wurde erfolgreich vom Router zugewiesen!
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "IP erhalten: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // JETZT signalisieren wir dem UDP-Task: Du darfst starten!
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Falls die Verbindung abbricht, löschen wir das Bit wieder sicherheitshalber
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW("WIFI", "Verbindung verloren, versuche Reconnect...");
        esp_wifi_connect();
    }
}

void system_display_init(){
    esp_err_t err = ESP_OK;

    // 1. Initialize LDO (required for screen)
    esp_ldo_channel_config_t ldo3_cof = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    err = esp_ldo_acquire_channel(&ldo3_cof, &ldo3);
    if (err != ESP_OK) init_fail_handler("ldo3", err);

    esp_ldo_channel_config_t ldo4_cof = {
        .chan_id = 4,
        .voltage_mv = 3300,
    };
    err = esp_ldo_acquire_channel(&ldo4_cof, &ldo4);
    if (err != ESP_OK) init_fail_handler("ldo4", err);
    MAIN_INFO("LDO3 and LDO4 init success");

    // 2. Initialize I2C (required for touch chip)
    MAIN_INFO("Initializing I2C...");
    err = i2c_init();
    if (err != ESP_OK) init_fail_handler("I2C", err);
    MAIN_INFO("I2C init success");

    // 3. Initialize touch panel (low-level driver)
    MAIN_INFO("Initializing touch panel...");
    err = touch_init();
    if (err != ESP_OK) init_fail_handler("Touch", err);
    MAIN_INFO("Touch panel init success");

    // 4. Initialize LCD hardware and LVGL (must initialize before turning on backlight)
    err = display_init();
    if (err != ESP_OK) init_fail_handler("LCD", err);
    MAIN_INFO("LCD init success");

    set_lcd_blight(25);

}

void screen_init()
{
    ui_Screen1 = lv_scr_act();
    lv_obj_remove_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(bg_color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void create_label_style(){

    lv_style_init(&label_style);
    lv_style_set_bg_color(&label_style, lv_color_hex(bg_color));
}

void init_gear(void){
    ui_Gear = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Gear, 220);
    lv_obj_set_height(ui_Gear, 220);
    lv_obj_set_x(ui_Gear, 0);
    lv_obj_set_y(ui_Gear, -70);
    lv_obj_set_align(ui_Gear, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Gear, "4");
    lv_obj_remove_flag(ui_Gear, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_text_color(ui_Gear, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Gear, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Gear, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Gear, &ui_font_Arial180, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Gear, lv_color_hex(bg_color), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Gear, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Gear, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Gear, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Gear, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Gear, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Gear, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Gear, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Gear, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Gear, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_Gear,10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_subject_init_int(&gear_binding, gear);
    lv_label_bind_text(ui_Gear, &gear_binding, "%d");
}

void init_speed(void){
    ui_Speed = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Speed, 220);
    lv_obj_set_height(ui_Speed, 70);
    lv_obj_set_x(ui_Speed, 0);
    lv_obj_set_y(ui_Speed, -215);
    lv_obj_set_align(ui_Speed, LV_ALIGN_CENTER);
    lv_obj_set_style_pad_top(ui_Speed, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_Speed, "187");
    lv_obj_remove_flag(ui_Speed, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_text_color(ui_Speed, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_text_opa(ui_Speed, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Speed, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Speed, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Speed, lv_color_hex(bg_color), LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_bg_opa(ui_Speed, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Speed, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Speed, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Speed, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_Speed,10, LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_border_opa(ui_Speed, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_subject_init_int(&speed_binding, speed_value);
    lv_label_bind_text(ui_Speed, &speed_binding, "%d");
}

void init_rev(void){

    ui_Rev = lv_bar_create(ui_Screen1);
    lv_bar_set_mode(ui_Rev, LV_BAR_MODE_RANGE);
    lv_bar_set_range(ui_Rev, -20, 20);
    lv_bar_set_value(ui_Rev, 20, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_Rev, -20, LV_ANIM_OFF);
    lv_obj_set_width(ui_Rev, 800);
    lv_obj_set_height(ui_Rev, 20);
    lv_obj_set_x(ui_Rev, 0);
    lv_obj_set_y(ui_Rev, -275);
    lv_obj_set_align(ui_Rev, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Rev, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Rev, lv_color_hex(0x2A2A2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Rev, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_image_src(ui_Rev, &ui_img_rev_bar_indicator_symmetrical_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_image_src(ui_Rev, &ui_img_rev_bar_inactive_png, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_image_recolor(ui_Rev, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_image_recolor_opa(ui_Rev, 10, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    //Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
    if(lv_obj_get_style_pad_top(ui_Rev, LV_PART_MAIN) > 0) lv_obj_set_style_pad_right(ui_Rev,lv_obj_get_style_pad_right(ui_Rev, LV_PART_MAIN) + 1, LV_PART_MAIN);

    // lv_subject_init_int(&rev_binding,rev_value);
    // lv_bar_bind_value(ui_Rev,&rev_binding);


}

void init_brake(void){
    ui_Brake = lv_bar_create(ui_Screen1);
    
    lv_bar_set_value(ui_Brake, 50, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_Brake, 0, LV_ANIM_OFF);
    lv_bar_set_max_value(ui_Brake,255);
    lv_obj_set_width(ui_Brake, 200);
    lv_obj_set_height(ui_Brake, 40);
    lv_obj_set_x(ui_Brake, -67);
    lv_obj_set_y(ui_Brake, 41);
    lv_obj_set_align(ui_Brake, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Brake, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Brake, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Brake, lv_color_hex(0x2A2A2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Brake, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Brake, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Brake, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Brake, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Brake, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_rotation(ui_Brake, 2700, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_Brake, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Brake, lv_color_hex(0xCD0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Brake, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    //Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
    if(lv_obj_get_style_pad_top(ui_Brake, LV_PART_MAIN) > 0) lv_obj_set_style_pad_right(ui_Brake,lv_obj_get_style_pad_right(ui_Brake, LV_PART_MAIN) + 1, LV_PART_MAIN);

    lv_subject_init_int(&brake_binding,brake_value);
    lv_bar_bind_value(ui_Brake,&brake_binding);


    ui_BrakeFiltered = lv_bar_create(ui_Screen1);
    lv_bar_set_value(ui_BrakeFiltered, 50, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_BrakeFiltered, 0, LV_ANIM_OFF);
    lv_bar_set_max_value(ui_BrakeFiltered,255);
    lv_obj_set_width(ui_BrakeFiltered, 200);
    lv_obj_set_height(ui_BrakeFiltered, 40);
    lv_obj_set_x(ui_BrakeFiltered, -67);
    lv_obj_set_y(ui_BrakeFiltered, 41);
    lv_obj_set_align(ui_BrakeFiltered, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_BrakeFiltered, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_BrakeFiltered, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BrakeFiltered, lv_color_hex(0x2A2A2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BrakeFiltered, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_BrakeFiltered, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_BrakeFiltered, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_BrakeFiltered, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_BrakeFiltered, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_rotation(ui_BrakeFiltered, 2700, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_BrakeFiltered, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BrakeFiltered, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BrakeFiltered, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    //Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
    if(lv_obj_get_style_pad_top(ui_BrakeFiltered, LV_PART_MAIN) > 0) lv_obj_set_style_pad_right(ui_Brake,lv_obj_get_style_pad_right(ui_Brake, LV_PART_MAIN) + 1, LV_PART_MAIN);

    lv_subject_init_int(&brake_filtered_binding,brake_filtered_value);
    lv_bar_bind_value(ui_BrakeFiltered,&brake_filtered_binding);

}

void init_throttle(void){

    ui_ThrottleFiltered = lv_bar_create(ui_Screen1);
    lv_bar_set_value(ui_ThrottleFiltered, 50, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_ThrottleFiltered, 0, LV_ANIM_OFF);
    lv_bar_set_max_value(ui_ThrottleFiltered,255);
    lv_obj_set_width(ui_ThrottleFiltered, 200);
    lv_obj_set_height(ui_ThrottleFiltered, 40);
    lv_obj_set_x(ui_ThrottleFiltered, 230);
    lv_obj_set_y(ui_ThrottleFiltered, 41);
    lv_obj_set_align(ui_ThrottleFiltered, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_ThrottleFiltered, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_ThrottleFiltered, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ThrottleFiltered, lv_color_hex(0x2A2A2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ThrottleFiltered, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_ThrottleFiltered, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_ThrottleFiltered, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_ThrottleFiltered, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_ThrottleFiltered, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_rotation(ui_ThrottleFiltered, 2700, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_ThrottleFiltered, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ThrottleFiltered, lv_color_hex(0xCD0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ThrottleFiltered, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    //Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
    if(lv_obj_get_style_pad_top(ui_ThrottleFiltered, LV_PART_MAIN) > 0) lv_obj_set_style_pad_right(ui_Throttle,lv_obj_get_style_pad_right(ui_Throttle, LV_PART_MAIN) + 1, LV_PART_MAIN);

    lv_subject_init_int(&throttle_filtered_binding,throttle_filtered_value);
    lv_bar_bind_value(ui_ThrottleFiltered,&throttle_filtered_binding);

    ui_Throttle = lv_bar_create(ui_Screen1);
    lv_bar_set_value(ui_Throttle, 50, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_Throttle, 0, LV_ANIM_OFF);
    lv_bar_set_max_value(ui_Throttle,255);
    lv_obj_set_width(ui_Throttle, 200);
    lv_obj_set_height(ui_Throttle, 40);
    lv_obj_set_x(ui_Throttle, 230);
    lv_obj_set_y(ui_Throttle, 41);
    lv_obj_set_align(ui_Throttle, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Throttle, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Throttle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Throttle, lv_color_hex(0x2A2A2A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Throttle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Throttle, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Throttle, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Throttle, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui_Throttle, LV_BORDER_SIDE_FULL, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_rotation(ui_Throttle, 2700, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_Throttle, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Throttle, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Throttle, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    //Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
    if(lv_obj_get_style_pad_top(ui_Throttle, LV_PART_MAIN) > 0) lv_obj_set_style_pad_right(ui_Throttle,lv_obj_get_style_pad_right(ui_Throttle, LV_PART_MAIN) + 1, LV_PART_MAIN);

    lv_subject_init_int(&throttle_binding,throttle_value);
    lv_bar_bind_value(ui_Throttle,&throttle_binding);

}

void init_laptimer(void){

    ui_laptimeContainer = lv_obj_create(ui_Screen1);
    lv_obj_remove_style_all(ui_laptimeContainer);
    lv_obj_set_width(ui_laptimeContainer, 300);
    lv_obj_set_height(ui_laptimeContainer, 420);
    lv_obj_set_x(ui_laptimeContainer, 340);
    lv_obj_set_y(ui_laptimeContainer, 30);
    lv_obj_set_align(ui_laptimeContainer, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_laptimeContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_laptimeContainer, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_laptimeContainer, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_laptimeContainer, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_laptimeContainer, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_laptimeLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_laptimeLabel, 300);
    lv_obj_set_height(ui_laptimeLabel, 40);
    lv_obj_set_x(ui_laptimeLabel, 340);
    lv_obj_set_y(ui_laptimeLabel, -159);
    lv_obj_set_align(ui_laptimeLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_laptimeLabel, "Laptime");
    lv_obj_set_style_text_color(ui_laptimeLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_laptimeLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_laptimeLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_laptimeLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_laptimeLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_laptimeLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_laptimeLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_laptimelastLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_laptimelastLabel, 300);
    lv_obj_set_height(ui_laptimelastLabel, 40);
    lv_obj_set_x(ui_laptimelastLabel, 340);
    lv_obj_set_y(ui_laptimelastLabel, -54);
    lv_obj_set_align(ui_laptimelastLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_laptimelastLabel, "Last");
    lv_obj_set_style_text_color(ui_laptimelastLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_laptimelastLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_laptimelastLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_laptimelastLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_laptimelastLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_laptimelastLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_laptimelastLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_laptime = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_laptime, 300);
    lv_obj_set_height(ui_laptime, 60);
    lv_obj_set_x(ui_laptime, 343);
    lv_obj_set_y(ui_laptime, -106);
    lv_obj_set_align(ui_laptime, LV_ALIGN_CENTER);
    lv_label_set_text(ui_laptime, "--:--.---");
    lv_obj_set_style_text_color(ui_laptime, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_laptime, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_laptime, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_laptime, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_laptime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_laptime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_laptime, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_laptime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_laptimelast = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_laptimelast, 300);
    lv_obj_set_height(ui_laptimelast, 60);
    lv_obj_set_x(ui_laptimelast, 343);
    lv_obj_set_y(ui_laptimelast, -1);
    lv_obj_set_align(ui_laptimelast, LV_ALIGN_CENTER);
    lv_label_set_text(ui_laptimelast, "--:--.---");
    lv_obj_set_style_text_color(ui_laptimelast, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_laptimelast, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_laptimelast, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_laptimelast, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_laptimelast, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_laptimelast, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_laptimelast, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_laptimelast, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_difftolastLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_difftolastLabel, 300);
    lv_obj_set_height(ui_difftolastLabel, 40);
    lv_obj_set_x(ui_difftolastLabel, 340);
    lv_obj_set_y(ui_difftolastLabel, 51);
    lv_obj_set_align(ui_difftolastLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_difftolastLabel, "Diff to Last");
    lv_obj_set_style_text_color(ui_difftolastLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_difftolastLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_difftolastLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_difftolastLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_difftolastLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_difftolastLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_difftolastLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_difftolast = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_difftolast, 300);
    lv_obj_set_height(ui_difftolast, 60);
    lv_obj_set_x(ui_difftolast, 343);
    lv_obj_set_y(ui_difftolast, 104);
    lv_obj_set_align(ui_difftolast, LV_ALIGN_CENTER);
    lv_label_set_text(ui_difftolast, "+0.000");
    lv_obj_set_style_text_color(ui_difftolast, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_difftolast, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_difftolast, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_difftolast, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_difftolast, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_difftolast, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_difftolast, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_difftolast, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_bestLapLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_bestLapLabel, 300);
    lv_obj_set_height(ui_bestLapLabel, 40);
    lv_obj_set_x(ui_bestLapLabel, 340);
    lv_obj_set_y(ui_bestLapLabel, 156);
    lv_obj_set_align(ui_bestLapLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_bestLapLabel, "Best\n");
    lv_obj_set_style_text_color(ui_bestLapLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_bestLapLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_bestLapLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_bestLapLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_bestLapLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_bestLapLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_bestLapLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_laptimeBest = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_laptimeBest, 300);
    lv_obj_set_height(ui_laptimeBest, 60);
    lv_obj_set_x(ui_laptimeBest, 343);
    lv_obj_set_y(ui_laptimeBest, 209);
    lv_obj_set_align(ui_laptimeBest, LV_ALIGN_CENTER);
    lv_label_set_text(ui_laptimeBest, "--:--.---");
    lv_obj_set_style_text_color(ui_laptimeBest, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_laptimeBest, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_laptimeBest, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_laptimeBest, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_laptimeBest, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_laptimeBest, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_laptimeBest, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_laptimeBest, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

}

void init_lap_display(void){
    ui_Container1 = lv_obj_create(ui_Screen1);
    lv_obj_remove_style_all(ui_Container1);
    lv_obj_set_width(ui_Container1, 224);
    lv_obj_set_height(ui_Container1, 50);
    lv_obj_set_x(ui_Container1, 318);
    lv_obj_set_y(ui_Container1, -218);
    lv_obj_set_align(ui_Container1, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_Container1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Container1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Container1, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Container1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Container1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_currentLap = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_currentLap, 60);
    lv_obj_set_height(ui_currentLap, 50);
    lv_obj_set_x(ui_currentLap, 395);
    lv_obj_set_y(ui_currentLap, -218);
    lv_obj_set_align(ui_currentLap, LV_ALIGN_CENTER);
    lv_label_set_text(ui_currentLap, "123");
    lv_obj_set_style_text_color(ui_currentLap, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_currentLap, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_currentLap, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_currentLap, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_currentLap, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_currentLap, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_currentLap, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_currentLap, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_subject_init_int(&lapcount_binding,current_lap);
    lv_label_bind_text(ui_currentLap,&lapcount_binding,"%d");

    ui_Lap = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Lap, 100);
    lv_obj_set_height(ui_Lap, 50);
    lv_obj_set_x(ui_Lap, 257);
    lv_obj_set_y(ui_Lap, -218);
    lv_obj_set_align(ui_Lap, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Lap, "Lap");
    lv_obj_set_style_text_color(ui_Lap, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Lap, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Lap, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Lap, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_Lap, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Lap, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Lap, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Lap, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Lap, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Lap, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Lap, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void init_values_grid(void){

    ui_gridContainer = lv_obj_create(ui_Screen1);
    lv_obj_remove_style_all(ui_gridContainer);
    lv_obj_set_width(ui_gridContainer, 320);
    lv_obj_set_height(ui_gridContainer, 290);
    lv_obj_set_x(ui_gridContainer, -344);
    lv_obj_set_y(ui_gridContainer, -106);
    lv_obj_set_align(ui_gridContainer, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_gridContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_gridContainer, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_gridContainer, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_gridContainer, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_gridContainer, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_OilTempLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_OilTempLabel, 170);
    lv_obj_set_height(ui_OilTempLabel, 58);
    lv_obj_set_x(ui_OilTempLabel, -419);
    lv_obj_set_y(ui_OilTempLabel, -220);
    lv_obj_set_align(ui_OilTempLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_OilTempLabel, "Oil Temp.");
    lv_obj_set_style_text_color(ui_OilTempLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_OilTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_OilTempLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_OilTempLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_OilTempLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_OilTempLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_OilTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_OilTempLabel, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_OilTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_OilTempLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_OilTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_OilPressLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_OilPressLabel, 170);
    lv_obj_set_height(ui_OilPressLabel, 57);
    lv_obj_set_x(ui_OilPressLabel, -419);
    lv_obj_set_y(ui_OilPressLabel, -163);
    lv_obj_set_align(ui_OilPressLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_OilPressLabel, "Oil Press.");
    lv_obj_set_style_text_color(ui_OilPressLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_OilPressLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_OilPressLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_OilPressLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_OilPressLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_OilPressLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_OilPressLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_OilPressLabel, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_OilPressLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_OilPressLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_OilPressLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_CoolantTempLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_CoolantTempLabel, 170);
    lv_obj_set_height(ui_CoolantTempLabel, 57);
    lv_obj_set_x(ui_CoolantTempLabel, -419);
    lv_obj_set_y(ui_CoolantTempLabel, -106);
    lv_obj_set_align(ui_CoolantTempLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_CoolantTempLabel, "Coolant");
    lv_obj_set_style_text_color(ui_CoolantTempLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_CoolantTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_CoolantTempLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_CoolantTempLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_CoolantTempLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_CoolantTempLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_CoolantTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_CoolantTempLabel, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_CoolantTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_CoolantTempLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_CoolantTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FuelLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_FuelLabel, 170);
    lv_obj_set_height(ui_FuelLabel, 57);
    lv_obj_set_x(ui_FuelLabel, -419);
    lv_obj_set_y(ui_FuelLabel, -49);
    lv_obj_set_align(ui_FuelLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_FuelLabel, "Fuel");
    lv_obj_set_style_text_color(ui_FuelLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_FuelLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_FuelLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_FuelLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_FuelLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_FuelLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_FuelLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_FuelLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_FuelLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_FuelLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_FuelLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BoostLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_BoostLabel, 170);
    lv_obj_set_height(ui_BoostLabel, 57);
    lv_obj_set_x(ui_BoostLabel, -419);
    lv_obj_set_y(ui_BoostLabel, 8);
    lv_obj_set_align(ui_BoostLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_BoostLabel, "Boost");
    lv_obj_set_style_text_color(ui_BoostLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_BoostLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_BoostLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_BoostLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_BoostLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BoostLabel, lv_color_hex(0x417386), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BoostLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_BoostLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_BoostLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_BoostLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_BoostLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_OilTempValueLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_OilTempValueLabel, 170);
    lv_obj_set_height(ui_OilTempValueLabel, 57);
    lv_obj_set_x(ui_OilTempValueLabel, -276);
    lv_obj_set_y(ui_OilTempValueLabel, -221);
    lv_obj_set_align(ui_OilTempValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_OilTempValueLabel, "110 °C");
    lv_obj_set_style_text_color(ui_OilTempValueLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_OilTempValueLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_OilTempValueLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_OilTempValueLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_OilTempValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_OilTempValueLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_OilTempValueLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_OilTempValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_OilPressValueLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_OilPressValueLabel, 170);
    lv_obj_set_height(ui_OilPressValueLabel, 57);
    lv_obj_set_x(ui_OilPressValueLabel, -274);
    lv_obj_set_y(ui_OilPressValueLabel, -163);
    lv_obj_set_align(ui_OilPressValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_OilPressValueLabel, "5 bar");
    lv_obj_set_style_text_color(ui_OilPressValueLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_OilPressValueLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_OilPressValueLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_OilPressValueLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_OilPressValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_OilPressValueLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_OilPressValueLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_OilPressValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_CoolantTempValueLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_CoolantTempValueLabel, 170);
    lv_obj_set_height(ui_CoolantTempValueLabel, 57);
    lv_obj_set_x(ui_CoolantTempValueLabel, -274);
    lv_obj_set_y(ui_CoolantTempValueLabel, -106);
    lv_obj_set_align(ui_CoolantTempValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_CoolantTempValueLabel, "85 °C");
    lv_obj_set_style_text_color(ui_CoolantTempValueLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_CoolantTempValueLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_CoolantTempValueLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_CoolantTempValueLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_CoolantTempValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_CoolantTempValueLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_CoolantTempValueLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_CoolantTempValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FuelValueLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_FuelValueLabel, 170);
    lv_obj_set_height(ui_FuelValueLabel, 57);
    lv_obj_set_x(ui_FuelValueLabel, -274);
    lv_obj_set_y(ui_FuelValueLabel, -49);
    lv_obj_set_align(ui_FuelValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_FuelValueLabel, "46 l");
    lv_obj_set_style_text_color(ui_FuelValueLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_FuelValueLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_FuelValueLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_FuelValueLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_FuelValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_FuelValueLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_FuelValueLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_FuelValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_BoostValueLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_BoostValueLabel, 170);
    lv_obj_set_height(ui_BoostValueLabel, 57);
    lv_obj_set_x(ui_BoostValueLabel, -274);
    lv_obj_set_y(ui_BoostValueLabel, 8);
    lv_obj_set_align(ui_BoostValueLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_BoostValueLabel, "1.2 bar");
    lv_obj_set_style_text_color(ui_BoostValueLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_BoostValueLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_BoostValueLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_BoostValueLabel, &ui_font_Arial36, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_BoostValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_BoostValueLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_BoostValueLabel, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_BoostValueLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void init_tyre_temps(void){

    // ui_Image3 = lv_image_create(ui_Screen1);
    // lv_image_set_src(ui_Image3, &ui_img_tyretemp_grid_png);
    // lv_obj_set_width(ui_Image3, LV_SIZE_CONTENT);   /// 1
    // lv_obj_set_height(ui_Image3, LV_SIZE_CONTENT);    /// 1
    // lv_obj_set_x(ui_Image3, 0);
    // lv_obj_set_y(ui_Image3, 168);
    // lv_obj_set_align(ui_Image3, LV_ALIGN_CENTER);
    // lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_CLICKABLE);     /// Flags
    // lv_obj_remove_flag(ui_Image3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_tyreTemp0 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_tyreTemp0, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tyreTemp0, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tyreTemp0, -105);
    lv_obj_set_y(ui_tyreTemp0, 149);
    lv_obj_set_align(ui_tyreTemp0, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tyreTemp0, "68.9");
    lv_obj_set_style_text_color(ui_tyreTemp0, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tyreTemp0, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tyreTemp0, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tyreTemp0, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_tyreTemp0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_tyreTemp0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_tyreTemp0, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_tyreTemp0, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tyreTemp1 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_tyreTemp1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tyreTemp1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tyreTemp1, 105);
    lv_obj_set_y(ui_tyreTemp1, 149);
    lv_obj_set_align(ui_tyreTemp1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tyreTemp1, "68.9");
    lv_obj_set_style_text_color(ui_tyreTemp1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tyreTemp1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tyreTemp1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tyreTemp1, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_tyreTemp1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_tyreTemp1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_tyreTemp1, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_tyreTemp1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tyreTemp2 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_tyreTemp2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tyreTemp2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tyreTemp2, -105);
    lv_obj_set_y(ui_tyreTemp2, 249);
    lv_obj_set_align(ui_tyreTemp2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tyreTemp2, "68.9");
    lv_obj_set_style_text_color(ui_tyreTemp2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tyreTemp2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tyreTemp2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tyreTemp2, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_tyreTemp2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_tyreTemp2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_tyreTemp2, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_tyreTemp2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tyreTemp3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_tyreTemp3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tyreTemp3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tyreTemp3, 105);
    lv_obj_set_y(ui_tyreTemp3, 249);
    lv_obj_set_align(ui_tyreTemp3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tyreTemp3, "68.9");
    lv_obj_set_style_text_color(ui_tyreTemp3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tyreTemp3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tyreTemp3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tyreTemp3, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_tyreTemp3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_tyreTemp3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_tyreTemp3, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_tyreTemp3, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tyreTempContainer = lv_obj_create(ui_Screen1);
    lv_obj_remove_style_all(ui_tyreTempContainer);
    lv_obj_set_width(ui_tyreTempContainer, 365);
    lv_obj_set_height(ui_tyreTempContainer, 250);
    lv_obj_set_x(ui_tyreTempContainer, 0);
    lv_obj_set_y(ui_tyreTempContainer, 168);
    lv_obj_set_align(ui_tyreTempContainer, LV_ALIGN_CENTER);
    lv_obj_remove_flag(ui_tyreTempContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_tyreTempContainer, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_tyreTempContainer, lv_color_hex(0xD58918), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_tyreTempContainer, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_tyreTempContainer, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tyreTempLabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_tyreTempLabel, 365);
    lv_obj_set_height(ui_tyreTempLabel, 55);
    lv_obj_set_x(ui_tyreTempLabel, 0);
    lv_obj_set_y(ui_tyreTempLabel, 72);
    lv_obj_set_align(ui_tyreTempLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tyreTempLabel, "TYRE TEMP");
    lv_obj_set_style_text_color(ui_tyreTempLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_tyreTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_tyreTempLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tyreTempLabel, &ui_font_Arial50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_tyreTempLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_tyreTempLabel, lv_color_hex(0xD58918), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_tyreTempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_tyreTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_tyreTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_tyreTempLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_tyreTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

}

void format_ms_to_string(int32_t total_ms, char *out_str, size_t max_len) {
    // 1. Ungültige Werte abfangen (GT7 gibt oft -1 aus, wenn keine Zeit vorhanden ist)
    if (total_ms < 0) {
        snprintf(out_str, max_len, "--.--.--");
        return;
    }

    // 2. Mathematisch zerlegen
    int32_t total_seconds = total_ms / 1000;
    int minutes = total_seconds / 60;
    int seconds = total_seconds % 60;
    int milliseconds = total_ms % (total_seconds*1000);

    // 3. Als String formatieren ("%02d" sorgt für die führende Null)
    // mm.ss -> z.B. 01.05
    snprintf(out_str, max_len, "%02d:%02d.%03d", minutes, seconds, milliseconds);
}

void format_ms_to_string_diff(int32_t total_ms, char *out_str, size_t max_len) {
    // 1. Ungültige Werte abfangen (GT7 gibt oft -1 aus, wenn keine Zeit vorhanden ist)
    // 2. Mathematisch zerlegen

    if(total_ms == 0){
        snprintf(out_str, max_len, "+00.000");
        return;
    }    

    int32_t total_seconds = total_ms / 1000;
    int seconds = total_seconds % 60;
    int milliseconds = total_ms % (total_seconds*1000);

    // 3. Als String formatieren ("%02d" sorgt für die führende Null)
    // mm.ss -> z.B. 01.05

    if(total_ms > 0)
        snprintf(out_str, max_len, "+%02d.%03d", seconds, milliseconds);
    if(total_ms < 0)
        snprintf(out_str, max_len, "-%02d.%03d", seconds*-1, milliseconds*-1);
}

void format_float_to_tyretemp(float val, char *out_str, size_t max_len) {
    // 1. Ungültige Werte abfangen (GT7 gibt oft -1 aus, wenn keine Zeit vorhanden ist)
    if (val < 0.0 || val > 1000.0) {
        snprintf(out_str, max_len, "-.-");
        return;
    }

    snprintf(out_str, max_len, "%.1f", val);
}

void format_float_temperature(float val, char *out_str, size_t max_len) {
    snprintf(out_str, max_len, "%.0f °C", val);
}

void format_float_pressure(float val, char *out_str, size_t max_len) {
    snprintf(out_str, max_len, "%.2f", val);
}

void format_float_fuel(float val, char *out_str, size_t max_len) {
    snprintf(out_str, max_len, "%.1f l", val);
}

void get_revbar_values(int16_t min_limit_rev, int16_t max_limit_rev, int current_rev, int* out_bar_start_value, int* out_bar_value, bool* shift)
{
    int stepping = 200;
    int distance_to_max = 0;
    int steps = 0;
    int shift_rev = 0;
    div_t rev_div;

    if(current_rev == 0)
    {
        *shift = false;
        *out_bar_start_value = -20;
        *out_bar_value = 20;
        return;
    }

    // Schaltblitz wenn in der Mitte vom Limiter
    shift_rev = ((max_limit_rev - min_limit_rev) / 2) + min_limit_rev;
    //MAIN_INFO("Shiftrev at %d | Currentrev %d | Min REV %d | Max REV %d", shift_rev, current_rev, min_limit_rev, max_limit_rev);
    *shift = false;

    if(current_rev >= min_limit_rev){
        //MAIN_INFO("Rev over Max | Current Rev: %d | Limit: %d", current_rev, min_limit_rev);
        *out_bar_start_value = 0;
        *out_bar_value = 0;

        if(current_rev >= min_limit_rev) *shift = true;

        return;
    }

    distance_to_max = min_limit_rev - current_rev;
    rev_div = div(distance_to_max, stepping);
    steps = rev_div.quot;
    
    if(steps >= 20){
        //MAIN_INFO("Rev too low | Current Rev: %d | Limit: %d", current_rev, min_limit_rev);
        *out_bar_start_value = -20;
        *out_bar_value = 20;
        return;
    }

    //MAIN_INFO("Rev in range | Current Rev: %d | Limit: %d", current_rev, min_limit_rev);

    *out_bar_start_value = -steps;
    *out_bar_value = steps;
    return;

}

bool update_value_if_changed_int(int* target_value, int* source_value){
    if(*target_value != *source_value){
        *target_value = *source_value;
        return true;
    }
    return false;
}

bool update_value_if_changed_int_save(int* target_value, int* source_value, int* old_value){
    if(*target_value != *source_value){
        *old_value = *target_value;
        *target_value = *source_value;
        return true;
    }
    return false;
}

bool update_value_if_changed_float(float* target_value, float* source_value){
    if(*target_value != *source_value){
        *target_value = *source_value;
        return true;
    }
    return false;
}

void update_ui(){

    char tyretemp0_str[10];
    char tyretemp1_str[10];
    char tyretemp2_str[10];
    char tyretemp3_str[10];

    char oiltemp_str[10];
    char oilPress_str[15];
    char coolantTemp_str[15];
    char fuel_str[10];
    char boost_str[15];

    bool gear_changed = false;
    bool speed_changed = false;
    bool brake_changed = false;
    bool throttle_changed = false;
    bool laptime_changed = false;
    bool rev_changed = false;
    bool laptimelast_changed = false;
    bool min_rev_changed = false;
    bool max_rev_changed = false;
    bool brake_filtered_changed = false;
    bool throttle_filtered_changed = false;
    bool lap_changed = false;
    bool tyretemp0_changed = false;
    bool tyretemp1_changed = false;
    bool tyretemp2_changed = false;
    bool tyretemp3_changed = false;
    bool boost_changed = false;
    bool oil_pressure_changed = false;
    bool oil_temperature_changed = false;
    bool coolant_temp_changed = false;
    bool fuel_changed = false;
    bool best_lap_changed = false;
    bool laptime_diff_changed = false;

    bool shift_indicator = false;

    if (data_mutex && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        gear_changed = update_value_if_changed_int(&local_gear,&gear);
        //local_gear = gear;

        speed_changed = update_value_if_changed_int(&local_speed,&speed_value);
        //local_speed = speed_value;

        brake_changed = update_value_if_changed_int(&local_brake,&brake_value);
        //local_brake = brake_value;

        throttle_changed = update_value_if_changed_int(&local_throttle,&throttle_value);
        //local_throttle = throttle_value;

        laptime_changed = update_value_if_changed_int(&local_laptime,&laptime);
        //local_laptime = laptime;

        best_lap_changed = update_value_if_changed_int(&local_laptime_best,&laptime_best);

        rev_changed = update_value_if_changed_int(&local_rev,&rev_value);
        //local_rev = rev_value;

        laptimelast_changed = update_value_if_changed_int_save(&local_laptime_last,&laptime_last,&local_laptime_last_save);            

        min_rev_changed = update_value_if_changed_int(&local_min_rev,&min_rev);
        //local_min_rev = min_rev;

        max_rev_changed = update_value_if_changed_int(&local_max_rev,&max_rev);
        //local_max_rev = max_rev;

        brake_filtered_changed = update_value_if_changed_int(&local_brake_filtered_value,&brake_filtered_value);
        // local_brake_filtered_value = brake_filtered_value;

        throttle_filtered_changed = update_value_if_changed_int(&local_throttle_filtered_value,&throttle_filtered_value);
        // local_throttle_filtered_value = throttle_filtered_value;

        lap_changed = update_value_if_changed_int(&local_lap,&current_lap);
        // local_lap = current_lap;

        tyretemp0_changed = update_value_if_changed_float(&local_tyre_temps[0],&tyreTemperatures[0]);
        // local_tyre_temps[0] = tyreTemperatures[0];

        tyretemp1_changed = update_value_if_changed_float(&local_tyre_temps[1],&tyreTemperatures[1]);
        //local_tyre_temps[1] = tyreTemperatures[1];

        tyretemp2_changed = update_value_if_changed_float(&local_tyre_temps[2],&tyreTemperatures[2]);
        // local_tyre_temps[2] = tyreTemperatures[2];

        tyretemp3_changed = update_value_if_changed_float(&local_tyre_temps[3],&tyreTemperatures[3]);
        // local_tyre_temps[3] = tyreTemperatures[3];

        boost_changed = update_value_if_changed_float(&local_boost,&boost);
        // local_boost = boost;

        oil_pressure_changed = update_value_if_changed_float(&local_oil_pressure,&oil_pressure);
        // local_oil_pressure = oil_pressure;

        oil_temperature_changed = update_value_if_changed_float(&local_oil_temperature,&oil_temperature);
        // local_oil_temperature = oil_temperature;

        coolant_temp_changed = update_value_if_changed_float(&local_coolant_temperature,&coolant_temperature);
        // local_coolant_temperature = coolant_temperature;

        fuel_changed = update_value_if_changed_float(&local_fuel,&fuel);
        // local_fuel = fuel;

        xSemaphoreGive(data_mutex);
    }

    get_revbar_values(local_min_rev, local_max_rev ,local_rev,&rev_bar_start_value,&rev_bar_value, &shift_indicator);

    shift_inidicator_changed = false;
    if(shift_indicator != shift_indicator_last)
        shift_inidicator_changed = true;
    shift_indicator_last = shift_indicator;

    char laptimer_str[10];
    format_ms_to_string(local_laptime, laptimer_str, sizeof(laptimer_str));

    char laptimer_last_str[10];
    format_ms_to_string(local_laptime_last, laptimer_last_str, sizeof(laptimer_last_str));

    char laptimer_diff_str[10];
    char laptimer_best_str[10];
    format_ms_to_string(local_laptime_best, laptimer_best_str, sizeof(laptimer_best_str));

    format_float_to_tyretemp(local_tyre_temps[0],tyretemp0_str,sizeof(tyretemp0_str));
    format_float_to_tyretemp(local_tyre_temps[1],tyretemp1_str,sizeof(tyretemp1_str));
    format_float_to_tyretemp(local_tyre_temps[2],tyretemp2_str,sizeof(tyretemp2_str));
    format_float_to_tyretemp(local_tyre_temps[3],tyretemp3_str,sizeof(tyretemp3_str));

    format_float_fuel(local_fuel,fuel_str,sizeof(fuel_str));

    format_float_pressure(local_oil_pressure,oilPress_str,sizeof(oilPress_str));
    format_float_pressure(local_boost,boost_str,sizeof(boost_str));

    format_float_temperature(local_oil_temperature, oiltemp_str, sizeof(oiltemp_str));
    format_float_temperature(local_coolant_temperature,coolantTemp_str,sizeof(coolantTemp_str));

    if(laptimelast_changed && local_laptime_last_save > 0 && local_laptime_last > 0){
        local_diff_to_last = local_laptime_last - local_laptime_last_save;
        format_ms_to_string_diff(local_diff_to_last, laptimer_diff_str, sizeof(laptimer_diff_str));
        laptime_diff_changed = true;
    }
    // if(laptimelast_changed && (local_laptime_best = 0 || local_laptime_last < local_laptime_best) && local_laptime_last > 0) {
    //     local_laptime_best = local_laptime_last;
    //     best_lap_changed = true;
    // }

    if(lvgl_port_lock(0)){
        if(gear_changed)
            lv_subject_set_int(&gear_binding,local_gear);

        if(speed_changed)
            lv_subject_set_int(&speed_binding,local_speed);
        
        if(brake_changed)
            lv_subject_set_int(&brake_binding, local_brake);

        if(brake_filtered_changed)
            lv_subject_set_int(&brake_filtered_binding, local_brake_filtered_value);

        if(throttle_changed)
            lv_subject_set_int(&throttle_binding, local_throttle);

        if(throttle_filtered_changed)
            lv_subject_set_int(&throttle_filtered_binding, local_throttle_filtered_value);

        
        if(rev_changed || min_rev_changed || max_rev_changed){
            lv_bar_set_start_value(ui_Rev,rev_bar_start_value, LV_ANIM_OFF);
            lv_bar_set_value(ui_Rev,rev_bar_value, LV_ANIM_OFF);
        }
        
        if(boost_changed)
            lv_label_set_text(ui_BoostValueLabel,boost_str);
        
        if(oil_temperature_changed)
            lv_label_set_text(ui_OilTempValueLabel,oiltemp_str);

        if(oil_pressure_changed)
            lv_label_set_text(ui_OilPressValueLabel,oilPress_str);

        if(coolant_temp_changed)
            lv_label_set_text(ui_CoolantTempValueLabel,coolantTemp_str);

        if(fuel_changed)
            lv_label_set_text(ui_FuelValueLabel,fuel_str);

        if(lap_changed)
            lv_subject_set_int(&lapcount_binding,local_lap);
            

        if(tyretemp0_changed)
            lv_label_set_text(ui_tyreTemp0, tyretemp0_str);

        if(tyretemp1_changed)
            lv_label_set_text(ui_tyreTemp1, tyretemp1_str);

        if(tyretemp2_changed)
            lv_label_set_text(ui_tyreTemp2, tyretemp2_str);

        if(tyretemp3_changed)
            lv_label_set_text(ui_tyreTemp3, tyretemp3_str);


        // Hintergrund der Bar umschalten
        if(!shift_indicator && shift_inidicator_changed){
            lv_obj_set_style_bg_image_src(ui_Rev, &ui_img_rev_bar_indicator_symmetrical_png, LV_PART_MAIN | LV_STATE_DEFAULT);
            shift_animation_last_switch = 0;
            blink_on = false;
        }
            

        if(shift_indicator && shift_inidicator_changed){
            lv_obj_set_style_bg_image_src(ui_Rev, &ui_img_rev_bar_shift_png, LV_PART_MAIN | LV_STATE_DEFAULT);
            blink_on = true;
            shift_animation_last_switch = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
            
        if(shift_indicator && !shift_inidicator_changed){
            time_diff_shift_animation = xTaskGetTickCount() * portTICK_PERIOD_MS;
            time_diff_shift_animation = time_diff_shift_animation - shift_animation_last_switch;
            int shiftanmiation_switch = UI_SHIFT_ANIMATION_INTERVAL_MS;
            if(time_diff_shift_animation >= shiftanmiation_switch){
                shift_animation_last_switch = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if(blink_on){
                    lv_obj_set_style_bg_image_src(ui_Rev, &ui_img_rev_bar_inactive_png, LV_PART_MAIN | LV_STATE_DEFAULT);
                    blink_on = false;
                }
                else{
                    lv_obj_set_style_bg_image_src(ui_Rev, &ui_img_rev_bar_shift_png, LV_PART_MAIN | LV_STATE_DEFAULT);
                    blink_on = true;
                }
                    
            }
        }

        if(laptime_changed)
            lv_label_set_text(ui_laptime, laptimer_str);

        if(best_lap_changed)
            lv_label_set_text(ui_laptimeBest,laptimer_best_str);

        if(laptimelast_changed){
            lv_label_set_text(ui_laptimelast, laptimer_last_str);            
        }

        if(laptime_diff_changed){
            lv_label_set_text(ui_difftolast,laptimer_diff_str);
            if(local_diff_to_last > 0){
                lv_obj_set_style_text_color(ui_difftolast, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else if (local_diff_to_last < 0)
            {
                lv_obj_set_style_text_color(ui_difftolast, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
            else{
                lv_obj_set_style_text_color(ui_difftolast, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
            }
        }

            

        
        lvgl_port_unlock();
    }
    
}

void wifi_init(){

    wifi_event_group = xEventGroupCreate();

    // 2. Netzwerk-Infrastruktur vorbereiten
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();

    // 3. Initialisierung über die Remote-Bibliothek
    // Das Makro greift nun dank menuconfig auf die gehosteten Buffer zu!
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); 
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 4. Zugangsdaten setzen
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Aldebaran 2,4 GHz",
            .password = "runtervonmeinemrasen111",
        },
    };

    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    // 5. WLAN starten -> esp_wifi_remote schickt das nun live an den C6!
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    MAIN_INFO("Wifi init success...");

}

void udp_send_heartbeat_task(void *pvParameters){
    while(1){
        xEventGroupWaitBits(wifi_event_group,
                    WIFI_CONNECTED_BIT,
                    pdFALSE,        // Bit nach dem Lesen NICHT automatisch löschen
                    pdTRUE,         // Warte, bis das Bit gesetzt ist
                    portMAX_DELAY);

            // 1. Einen temporären UDP Protocol Control Block (PCB) erstellen
            struct udp_pcb *pcb = udp_new();
            if (pcb == NULL) {
                ESP_LOGE(TAG, "Konnte UDP PCB zum Senden nicht erstellen!");
                return;
            }

            // 2. Ziel-IP-Adresse aus dem String parsen
            ip_addr_t dest_addr;
            if (!ipaddr_aton(TARGET_IP, &dest_addr)) {
                ESP_LOGE(TAG, "Ungültige IP-Adresse: %s", TARGET_IP);
                udp_remove(pcb);
                return;
            }

            // 3. Einen lwIP Packet-Buffer (pbuf) für exakt 1 Byte erstellen
            // PBUF_TRANSPORT reserviert automatisch Platz für die UDP-Header im Speicher
            // PBUF_RAM legt den Puffer im normalen Arbeitsspeicher an
            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 1, PBUF_RAM);
            if (p == NULL) {
                ESP_LOGE(TAG, "Konnte pbuf nicht allozieren!");
                udp_remove(pcb);
                return;
            }

            // 4. Das Zeichen 'C' in den Nutzdaten-Bereich (payload) des pbuf schreiben
            char *payload = (char *)p->payload;
            *payload = 'C';

            // 5. Das Paket absenden
            err_t err = udp_sendto(pcb, p, &dest_addr, TARGET_PORT);
            if (err != ERR_OK) {
                ESP_LOGE(TAG, "Senden fehlgeschlagen! Fehler-Code: %d", err);
            } else {
                ESP_LOGI(TAG, "'C' erfolgreich an %s:%d gesendet.", TARGET_IP, TARGET_PORT);
            }

            // 6. Aufräumen (Lebenswichtig, um Speicherlecks zu verhindern!)
            pbuf_free(p);      // Gibt den Packet-Buffer wieder frei
            udp_remove(pcb);   // Löscht den temporären PCB aus dem Speiche

            vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                              const ip_addr_t *addr, u16_t port)
{

    if (p != NULL) {
        // IP-Adresse des Absenders lesbar machen
        char ip_str[IPADDR_STRLEN_MAX];
        ipaddr_ntoa_r(addr, ip_str, sizeof(ip_str));
        
        //ESP_LOGI(TAG, "Paket erhalten von %s:%d, Länge: %d Bytes", ip_str, port, p->tot_len);

        //return;
        // Die Daten liegen im lwIP "pbuf" (Packet Buffer). 
        // WICHTIG: Da die Daten im pbuf nicht zwingend an einem Stück im RAM liegen müssen, 
        // kopiert man sie am sichersten in ein flaches Array, bevor man sie parst.
        if (p->tot_len <= 1500) { // Typische maximale MTU-Größe
            uint8_t rx_buffer[p->tot_len];
            
            // pbuf_copy_partial kopiert die Daten sicher zusammenhängend in dein Array
            pbuf_copy_partial(p, rx_buffer, p->tot_len, 0);

            // HIER kannst du die Daten jetzt an deine GT7-Struktur übergeben!
            // struct Packet my_packet = gt7_parse_and_get_packet(rx_buffer, p->tot_len);
            //MAIN_INFO("Packet size: %d",sizeof(rx_buffer));
            gt7_process_raw_udp(rx_buffer,sizeof(rx_buffer));

            if (data_mutex && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                gear = gt7_get_current_Gear();
                speed_value = gt7_get_Speed();
                throttle_value = gt7_get_Throttle();
                brake_value = gt7_get_Brake();
                laptime = gt7_get_current_Laptime();
                laptime_last = gt7_get_last_Laptime();
                laptime_best = gt7_get_best_Laptime();
                min_rev = gt7_getminAlertRPM();
                max_rev = gt7_getmaxAlertRPM();
                rev_value = gt7_get_current_RPM();
                throttle_filtered_value = gt7_getcurrentThrottleFiltered();
                brake_filtered_value = gt7_getcurrentBrakeFiltered();
                tyreTemperatures[0] = gt7_getcurrenttyreTemp(0);
                tyreTemperatures[1] = gt7_getcurrenttyreTemp(1);
                tyreTemperatures[2] = gt7_getcurrenttyreTemp(2);
                tyreTemperatures[3] = gt7_getcurrenttyreTemp(3);
                fuel = gt7_getcurrentFuelLevel();
                oil_temperature = gt7_getcurrentOilTemp();
                oil_pressure = gt7_getcurrentOilPressure();
                coolant_temperature = gt7_getcurrentCoolantTemp();
                boost = gt7_getcurrentBoost();
                current_lap = gt7_getcurrentLapCount();
                xSemaphoreGive(data_mutex); // Sofort wieder freigeben!
            }
        }

        // LEBENSWICHTIG: Du musst den pbuf freigeben, sonst hast du sofort ein Speicherleck!
        pbuf_free(p);
    }
}

void start_lwip_udp_listener(void)
{
    // 1. Einen neuen UDP Protocol Control Block (PCB) erstellen
    struct udp_pcb *pcb = udp_new();
    if (pcb == NULL) {
        ESP_LOGE(TAG, "Konnte UDP PCB nicht erstellen!");
        return;
    }

    // 2. An alle lokalen IP-Adressen (IP_ANY_TYPE) und deinen Wunsch-Port binden
    // Ersetze 12345 durch den Port, auf dem Gran Turismo 7 oder dein Gerät sendet
    err_t err = udp_bind(pcb, IP_ANY_TYPE, 33740);
    if (err != ERR_OK) {
        ESP_LOGE(TAG, "Bind fehlgeschlagen! Fehler-Code: %d", err);
        udp_remove(pcb); // PCB wieder löschen bei Fehler
        return;
    }

    // 3. Die Callback-Funktion mit dem PCB verknüpfen
    udp_recv(pcb, udp_recv_callback, NULL);

    ESP_LOGI(TAG, "Nativer lwIP UDP-Listener auf Port 12345 erfolgreich gestartet.");
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    data_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(ret);

    wifi_init();

    system_display_init();
    MAIN_INFO("System init success...");
    screen_init();
    MAIN_INFO("Screen init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_gear();
    MAIN_INFO("Gear init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_speed();
    MAIN_INFO("Speed init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_brake();
    MAIN_INFO("Brake init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_throttle();
    MAIN_INFO("Throttle init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_rev();
    MAIN_INFO("Rev init success...");

    vTaskDelay(pdMS_TO_TICKS(50));
    
    init_laptimer();
    MAIN_INFO("Laptimer init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_lap_display();
    MAIN_INFO("Lapdisplay init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_values_grid();
    MAIN_INFO("Overviewgrid init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    init_tyre_temps();
    MAIN_INFO("Tyretemps init success...");

    vTaskDelay(pdMS_TO_TICKS(50));

    start_lwip_udp_listener();
    xTaskCreate(
        udp_send_heartbeat_task,    // Name der C-Funktion deines Tasks
        "udp_heartbeat",       // Text-Name des Tasks (wichtig für Debugging)
        4096,               // Stack-Größe in Bytes (4KB ist sicher für Sockets)
        NULL,               // Parameter, die an den Task übergeben werden (hier keine)
        1,                  // Priorität des Tasks (5 ist ein solider Standardwert)
        NULL                // Task-Handle (wird nur zum Löschen/Pausieren benötigt)
    );



    lv_display_t* ui_display = lv_disp_get_default();


    vTaskDelay(pdMS_TO_TICKS(50));

    int time1 = 0;
    int time2 = 0;
    int time3 = 0;
    while(1){
        time1 = xTaskGetTickCount() * portTICK_PERIOD_MS;
        update_ui();
        time2 = xTaskGetTickCount() * portTICK_PERIOD_MS;
        time3 = time2-time1;
        //MAIN_INFO("Refresh of UI tooke %d ms",time3);
        vTaskDelay(pdMS_TO_TICKS(UI_UPDATE_INTERVAL_MS));
    }
}
