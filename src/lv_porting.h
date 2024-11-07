// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
 // #include <demos/lv_demos.h>
 // #include <examples/lv_examples.h>

 /* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

ESP_Panel* panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex

/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
    }

/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t* indev, lv_indev_data_t* data) {
    panel->getLcdTouch()->readData();
    bool touched = panel->getLcdTouch()->getTouchState();
    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
        } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();
        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;
        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
        }
    }

void lvgl_port_lock(int timeout_ms) {
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
    }

void lvgl_port_unlock(void) {
    xSemaphoreGiveRecursive(lvgl_mux);
    }

void lvgl_port_task(void* arg) {
    // Serial.println("Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();
        delay(5);
        // if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
        //     task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        //     } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
        //         task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        //         }
        //     vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        }
    }