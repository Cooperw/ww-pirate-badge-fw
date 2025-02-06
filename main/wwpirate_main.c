/*
 * a. @shiloh
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "esp_lcd_panel_vendor.h"

// ---------------------
// Change Me
// ---------------------
#define HANDLE "shiloh"
LV_IMG_DECLARE(badge_pic);
#define RFID_CODE "11:38:d9:40"

// ---------------------
// Button/LED Mapping
// ---------------------
//#define BUTTON_LEFT  35       // wwhf2024deadwood
//#define BUTTON_RIGHT 36       // wwhf2024deadwood
#define BUTTON_LEFT  35         // wwhf2025denver
#define BUTTON_RIGHT 36         // wwhf2025denver


// ---------------------
// Useful Links for OLED
// ---------------------
// User a 45x45 image if you want space for text beneath
// 64x64 if you want the entire square
// 64x128 if you want the entire screen
//
// https://docs.lvgl.io/7.11/widgets/obj.html
// https://www.simpleimageresizer.com
// https://lvgl.io/tools/imageconverter

// UFO/Circle group: 8 LEDs in order.
const int ufo_gpios[] = {48, 17, 18, 21, 40, 47, 1, 2};
#define UFO_COUNT (sizeof(ufo_gpios) / sizeof(ufo_gpios[0]))

// Eyes
#define EYES_LEFT_GPIO  42
#define EYES_RIGHT_GPIO 41

// Status (RGB)
#define STATUS_RED_GPIO    3
#define STATUS_GREEN_GPIO  5
#define STATUS_BLUE_GPIO   6

//------------------------------------------------------------------------------

static const char *TAG = "wwpirate-wwhf2024";
#define I2C_BUS_PORT  0

//SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           8
#define EXAMPLE_PIN_NUM_SCL           9
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

//------------------------------------------------------------------------------
// New: Status LED Colors
// Only use a fixed set of colors (primary, secondary, and tertiary)
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

const rgb_color_t status_colors[] = {
    // Primary colors
    {255, 0, 0},    // Red
    {0, 255, 0},    // Green
    {0, 0, 255},    // Blue
    // Secondary colors
    {255, 255, 0},  // Yellow
    {0, 255, 255},  // Cyan
    {255, 0, 255},  // Magenta
    // Tertiary colors
    {255, 127, 0},  // Orange
    {127, 0, 255},  // Violet
    {127, 255, 0}   // Chartreuse
};
#define NUM_STATUS_COLORS (sizeof(status_colors)/sizeof(status_colors[0]))

// Handle for display
lv_disp_t * disp;
char *mac_address;

// ---------------------
// UFO Effect Task
// ---------------------
// Effect: All UFO LEDs on except one "off" LED that moves left-to-right and then back.
void ufo_effect_task(void *arg)
{
    for (int i = 0; i < UFO_COUNT; i++) {
        gpio_reset_pin(ufo_gpios[i]);
        gpio_set_direction(ufo_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(ufo_gpios[i], 1);
    }
    int index = UFO_COUNT - 1; // start at LED8 (if that’s what you call index 7)
    int direction = -1;
    while (1) {
        for (int i = 0; i < UFO_COUNT; i++) {
            gpio_set_level(ufo_gpios[i], (i == index) ? 0 : 1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        index += direction;
        if (index < 0) {
            index = 1;
            direction = 1;
        } else if (index >= UFO_COUNT) {
            index = UFO_COUNT - 2;
            direction = -1;
        }
    }
}

// ---------------------
// Eyes Pulse Task
// ---------------------
// Effect: Both eyes pulse in brightness (using LEDC PWM).
void eyes_pulse_task(void *arg)
{
    // Configure LEDC timer for eyes (timer 0, 8-bit resolution, 5000 Hz)
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);
    
    // Configure LEDC channels for the left and right eyes.
    ledc_channel_config_t channel_conf = {0};
    channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_conf.timer_sel = LEDC_TIMER_0;
    channel_conf.duty = 0;
    channel_conf.hpoint = 0;
    
    // Left eye on GPIO 42, channel 0.
    channel_conf.channel = LEDC_CHANNEL_0;
    channel_conf.gpio_num = EYES_LEFT_GPIO;
    ledc_channel_config(&channel_conf);
    
    // Right eye on GPIO 41, channel 1.
    channel_conf.channel = LEDC_CHANNEL_1;
    channel_conf.gpio_num = EYES_RIGHT_GPIO;
    ledc_channel_config(&channel_conf);
    
    uint32_t duty;
    while (1) {
        // Fade up from 0 to 255
        for (duty = 0; duty <= 255; duty += 5) {
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 20);
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty, 20);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        // Fade down from 255 to 0
        for (duty = 255; duty > 0; duty -= 5) {
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 20);
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty, 20);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}


//------------------------------------------------------------------------------
// Status Effect Task
// Instead of completely random colors, choose a random color from the preset list.
void status_effect_task(void *arg)
{
    ledc_timer_config_t status_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&status_timer);
    
    ledc_channel_config_t status_channel = {0};
    status_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    status_channel.timer_sel = LEDC_TIMER_1;
    status_channel.duty = 0;
    status_channel.hpoint = 0;
    
    // Red LED on GPIO 3, channel 2.
    status_channel.channel = LEDC_CHANNEL_2;
    status_channel.gpio_num = STATUS_RED_GPIO;
    ledc_channel_config(&status_channel);
    
    // Green LED on GPIO 5, channel 3.
    status_channel.channel = LEDC_CHANNEL_3;
    status_channel.gpio_num = STATUS_GREEN_GPIO;
    ledc_channel_config(&status_channel);
    
    // Blue LED on GPIO 6, channel 4.
    status_channel.channel = LEDC_CHANNEL_4;
    status_channel.gpio_num = STATUS_BLUE_GPIO;
    ledc_channel_config(&status_channel);
    
    while (1) {
        int idx = esp_random() % NUM_STATUS_COLORS;
        uint32_t r = status_colors[idx].r;
        uint32_t g = status_colors[idx].g;
        uint32_t b = status_colors[idx].b;
        
        // Fade to the chosen preset color over 1000 ms.
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, r, 1000);
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, g, 1000);
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, b, 1000);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
        
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void build_oled_main() {
    lv_obj_t * scr = lv_disp_get_scr_act(disp);
    lv_obj_clean(scr);

    // Create an image object on the screen
    lv_obj_t * img = lv_img_create(scr);
    lv_img_set_src(img, &badge_pic);
    lv_obj_align(img, LV_ALIGN_TOP_MID, 0, 0);
    //lv_obj_align(img, LV_ALIGN_LEFT_MID, 0, 0);

    // Create a label object
    lv_obj_t * label = lv_label_create(scr);
    lv_label_set_text(label, "shiloh");
    //lv_obj_align(label, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 0);
}

void build_oled_details() {
    lv_obj_t * scr = lv_disp_get_scr_act(disp);
    lv_obj_clean(scr);

    lv_obj_t * label_handle = lv_label_create(scr);
    lv_label_set_text(label_handle, HANDLE);
    lv_obj_align(label_handle, LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_obj_t * label_mac = lv_label_create(scr);
    lv_label_set_text(label_mac, "00:00:00:00:00:00");
    lv_obj_align(label_mac, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * label_title_rfid = lv_label_create(scr);
    lv_label_set_text(label_title_rfid, "RFID");
    lv_obj_align(label_title_rfid, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    lv_obj_t * label_rfid = lv_label_create(scr);
    lv_label_set_text(label_rfid, RFID_CODE);
    lv_obj_align(label_rfid, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
}

void set_screen_main(){
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {
        build_oled_main();
        // Release the mutex
        lvgl_port_unlock();
    }
}

void set_screen_details(){
    if (lvgl_port_lock(0)) {
        build_oled_details();
        lvgl_port_unlock();
    }
}

//------------------------------------------------------------------------------
// Button Input Handling
// This section configures a set of candidate GPIO pins as inputs and attaches interrupts,
// so you can press buttons and see which GPIO triggers an event.
typedef struct {
    uint32_t pin;
    int level;
} button_event_t;

static QueueHandle_t button_evt_queue = NULL;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    button_event_t evt = { .pin = gpio_num, .level = gpio_get_level(gpio_num) };
    xQueueSendFromISR(button_evt_queue, &evt, NULL);
}

void button_event_task(void *arg)
{
    button_event_t evt;
    while (1) {
        if (xQueueReceive(button_evt_queue, &evt, portMAX_DELAY)) {
            if (evt.pin == BUTTON_LEFT && evt.level == 1){
                set_screen_main();
            }else if (evt.pin == BUTTON_LEFT && evt.level == 0){
                set_screen_details();
            } else {
                printf("Button event: GPIO %lu is now %d\n", evt.pin, evt.level);
            }
        }
    }
}

void init_button_inputs(void)
{
    const int button_pins[] = {BUTTON_LEFT,BUTTON_RIGHT};
    int num_buttons = sizeof(button_pins) / sizeof(button_pins[0]);

    button_evt_queue = xQueueCreate(10, sizeof(button_event_t));
    gpio_install_isr_service(0);

    for (int i = 0; i < num_buttons; i++) {
        int pin = button_pins[i];
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(pin, button_isr_handler, (void *) pin);
    }

    xTaskCreate(button_event_task, "button_event_task", 2048, NULL, 10, NULL);
}

// Function that retrieves the MAC address and returns a pointer to a string
void get_mac_address_str(void)
{
    uint8_t mac[6];
    // Read the MAC address for the Wi-Fi station interface.
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MAC address");
    }
    // Allocate enough space for the MAC string: 6 pairs + 5 colons + terminating null = 18 bytes.
    char *mac_address = malloc(18);
    if (mac_address == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for MAC string");
    }
    // Format the MAC address into the string
    snprintf(mac_address, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return;
}

void app_main(void)
{
    get_mac_address_str();

    // Install LEDC fade functionality once.
    ledc_fade_func_install(0);
    
    // Create the UFO effect task.
    xTaskCreate(ufo_effect_task, "ufo_effect_task", 4096, NULL, 5, NULL);
    xTaskCreate(eyes_pulse_task, "eyes_pulse_task", 4096, NULL, 5, NULL);
    xTaskCreate(status_effect_task, "status_effect_task", 4096, NULL, 5, NULL);

    // Initialize button inputs
    init_button_inputs();

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    disp = lvgl_port_add_disp(&disp_cfg);
    lv_disp_set_rotation(disp, LV_DISP_ROT_180);
    set_screen_main();
}
