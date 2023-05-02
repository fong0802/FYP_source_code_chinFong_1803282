// The operation speed of esp32 set to 80MHz (save energy)

// CDC stands for the capacitance to digital converter which refer to AD7147

#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "ad7147_config.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_sens.h"

#define CDC_SDA_PIN 21
#define CDC_SCL_PIN 22
#define CDC_INT_PIN 23
#define AD7147_ADDR 0x2C  // 010 1100

void set_nominal_sensor_val();
void init_cdc_bank1();
void init_cdc_bank2();
void i2c_cdc_init();
void gpio_init();
void gpio_interrupt_init();
void start_ble_transaction();
void print_addr(const void *addr);
void blePressure_host_task(void *param);
static void blePressure_advertise();
static void blePressure_tx_psure_stop();
static void blePressure_tx_psure_reset();
static void blePressure_on_sync();
static void blePressure_on_reset(int reason);
static void blePressure_tx_psure(TimerHandle_t ev);
static int blePressure_gap_event(struct ble_gap_event *event, void *arg);

uint16_t cdc_16bit_arr[10];
uint8_t cdc_16bit_arr_start_flag = 0;
uint8_t cdc_16bit_arr_full_flag = 0;
uint16_t cdc_nominal_val;
esp_err_t errChk;
uint16_t cdc_16bit;
uint8_t stage0_conv_data[2] = {0, 0};
uint16_t capacitance_change = 0;

static QueueHandle_t cdc_int_evt_queue = NULL;
static const char *TAG = "CDC";
static const char *TAG2 = "BLE_Pressure";
static TimerHandle_t blePressure_tx_timer;
static bool notify_state;
static uint16_t conn_handle;
static const char *device_name = "NCF_pressure_sensor";
static uint8_t blePressure_addr_type;

i2c_config_t configi2c0 = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = CDC_SDA_PIN,
    .scl_io_num = CDC_SCL_PIN,
    .sda_pullup_en = 0,
    .scl_pullup_en = 0,
    .master.clk_speed = 100000, // 100kHz
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

static void IRAM_ATTR cdc_isr_handler(void* arg)
{
    uint32_t cdc_int_pin_num = (uint32_t) arg;
    xQueueSendFromISR(cdc_int_evt_queue, &cdc_int_pin_num, NULL);  // queue for cpu to handle
}

// isr to handle cdc conversion data by reading it
static void cdc_data_ready_isr(void* arg)
{
    uint8_t count = 0;
    uint32_t cdc_int_pin_num;
    for(;;) {
        if(xQueueReceive(cdc_int_evt_queue, &cdc_int_pin_num, portMAX_DELAY)) {  // if cpu start to handler gpio interrupt

            // check why interrupt raise from ad7147
            uint8_t stage_complete_int_status_reg_addr[] = {0x00, 0x0A};
            uint8_t stage_complete_int_status[2] = {0, 0};
            errChk = i2c_master_write_read_device(I2C_NUM_0, AD7147_ADDR, stage_complete_int_status_reg_addr, 
                                                    sizeof(stage_complete_int_status_reg_addr),
                                                    stage_complete_int_status, sizeof(stage_complete_int_status), 80000);
            ESP_ERROR_CHECK(errChk);

            // if ad7147 STAGE0 conversion complete
            if(stage_complete_int_status[1] == 1)
            {
                // read cdc conversion data from ad7147
                uint8_t stage0_conv_data_reg_addr[] = {0x00, 0xE0};
                errChk = i2c_master_write_read_device(I2C_NUM_0, AD7147_ADDR, stage0_conv_data_reg_addr, 
                                                        sizeof(stage0_conv_data_reg_addr),
                                                        stage0_conv_data, sizeof(stage0_conv_data), 80000);
                ESP_ERROR_CHECK(errChk);

                // process conversion data and capacitance change
                cdc_16bit = 0;
                cdc_16bit = cdc_16bit + stage0_conv_data[0];
                cdc_16bit = cdc_16bit << 8;
                cdc_16bit = cdc_16bit | stage0_conv_data[1];
                capacitance_change = cdc_16bit - cdc_nominal_val;

                // if capacitance reduce then is due to nominal capacitance drift
                if(cdc_16bit < cdc_nominal_val)
                {
                    capacitance_change = 0;
                }

                // store 10 capacitance digital data to cdc_16bit_arr to compute average capacitance digital value
                // for setting up the nominal capacitance digital value
                if(cdc_16bit_arr_start_flag == 1)
                {
                    cdc_16bit_arr[count] = cdc_16bit;
                    ESP_LOGI(TAG, "cdc_16bit_arr[count] = %d %d", cdc_16bit_arr[count], count);
                    if(count == 9)
                    {
                        count = 0;
                        cdc_16bit_arr_start_flag = 0;
                        cdc_16bit_arr_full_flag = 1;
                    }else
                    {
                        count ++;
                    }
                }
            }
        }
    }
}

void app_main(void)
{
    gpio_init();
    gpio_interrupt_init();
    i2c_cdc_init();
    set_nominal_sensor_val();
    start_ble_transaction();

    while(1)
    {
        ESP_LOGI(TAG, "--------------------------------------------");
        ESP_LOGI(TAG, "cdc value: %d", cdc_16bit);
        ESP_LOGI(TAG, "nominal val: %d", cdc_nominal_val);
        ESP_LOGI(TAG, "capacitance change: %d", capacitance_change);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void start_ble_transaction(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blePressure_on_sync;
    ble_hs_cfg.reset_cb = blePressure_on_reset;

    // can at fastest speed is 30ms interval, but set to 100ms to conserve energy
    /* name, period/time,  auto reload, timer ID, callback */
    blePressure_tx_timer = xTimerCreate("blePressure_tx_timer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, blePressure_tx_psure);

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blePressure_host_task);
}

void gpio_init(void)
{
    // initialize the GPIO connected to AD7147 interrupt pin
    errChk = gpio_set_direction(CDC_INT_PIN, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(errChk);
    errChk = gpio_set_pull_mode(CDC_INT_PIN, GPIO_FLOATING);
    ESP_ERROR_CHECK(errChk);
    errChk = gpio_set_intr_type(CDC_INT_PIN, GPIO_INTR_NEGEDGE);
    ESP_ERROR_CHECK(errChk);
}

void gpio_interrupt_init(void)
{
    //create a queue to handle cdc interrupt event from isr
    cdc_int_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //start the task for cdc_data_ready_isr
    xTaskCreate(cdc_data_ready_isr, "cdc_data_ready_isr", 2048, NULL, 10, NULL);

    // create gpio interrupt from cdc interrupt pin
    errChk = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(errChk);
    errChk = gpio_isr_handler_add(CDC_INT_PIN, cdc_isr_handler, (void*)CDC_INT_PIN);
    ESP_ERROR_CHECK(errChk);
}

void i2c_cdc_init(void)
{
    errChk = i2c_param_config(I2C_NUM_0, &configi2c0);
    ESP_ERROR_CHECK(errChk);

    errChk = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(errChk);

    init_cdc_bank2();  // write initial data to Bank 2
    init_cdc_bank1(); // write initial data to Bank 1

    // start capacitance conversion by writting to 0x3001 register
    uint8_t stage_cal_en_addr_msb = 0x30;
    uint8_t stage_cal_en_addr_lsb = 0x01;
    uint8_t msb_stage_cal_en = 0x0F;
    uint8_t lsb_stage_cal_en = 0xFF;
    uint8_t dataFor001Reg[] = {stage_cal_en_addr_msb, stage_cal_en_addr_lsb, msb_stage_cal_en, lsb_stage_cal_en};
    errChk = i2c_master_write_to_device(I2C_NUM_0, AD7147_ADDR, dataFor001Reg, sizeof(dataFor001Reg), 80000);
    ESP_ERROR_CHECK(errChk);
}

// bank 1 register address to be initialize => 0x000 to 0x007
// ad7147 will increase the register address by one automatically after that register filled up, so just send an start address enough
void init_cdc_bank1(void)
{
    // 10 bit cdc register addr 6 MSB bit in high byte is ignored
    uint8_t bank1StartAddrHigh = 0x00;
    uint8_t bank1StartAddrLow = 0x00;
    uint8_t dataToBank1[] = {
        bank1StartAddrHigh, bank1StartAddrLow, 
        MSB_PWR_CONTROL, LSB_PWR_CONTROL, MSB_STAGE_CAL_EN, LSB_STAGE_CAL_EN,
        MSB_AMB_COMP_CTRL0, LSB_AMB_COMP_CTRL0, MSB_AMB_COMP_CTRL1, LSB_AMB_COMP_CTRL1, 
        MSB_AMB_COMP_CTRL2, LSB_AMB_COMP_CTRL2, MSB_STAGE_LOW_INT_ENABLE, LSB_STAGE_LOW_INT_ENABLE,
        MSB_STAGE_HIGH_INT_ENABLE, LSB_STAGE_HIGH_INT_ENABLE, MSB_STAGE_COMPLETE_INT_ENABLE, LSB_STAGE_COMPLETE_INT_ENABLE
        };

    errChk = i2c_master_write_to_device(I2C_NUM_0, AD7147_ADDR, dataToBank1, sizeof(dataToBank1), 80000);
    ESP_ERROR_CHECK(errChk);
}

// bank 2 register address to be initialize => 0x080 to 0x0DF
// ad7147 will increase the register address by one automatically after that register filled up, so just send an start address enough
void init_cdc_bank2(void)
{
    // 10 bit cdc register addr 6 MSB bit in high byte is ignored
    uint8_t bank2StartAddrHigh = 0x00;
    uint8_t bank2StartAddrLow = 0x80;
    uint8_t dataToBank2[] = {
        bank2StartAddrHigh, bank2StartAddrLow, 
        MSB_STAGE0_CONNECTION_6_to_0, LSB_STAGE0_CONNECTION_6_to_0, MSB_STAGE0_CONNECTION_12_to_7, LSB_STAGE0_CONNECTION_12_to_7, 
        MSB_STAGE0_AFE_OFFSET, LSB_STAGE0_AFE_OFFSET, MSB_STAGE0_SENSITIVITY, LSB_STAGE0_SENSITIVITY, 
        MSB_STAGE0_OFFSET_LOW, LSB_STAGE0_OFFSET_LOW, MSB_STAGE0_OFFSET_HIGH, LSB_STAGE0_OFFSET_HIGH,
        MSB_STAGE0_OFFSET_HIGH_CLAMP , LSB_STAGE0_OFFSET_HIGH_CLAMP, MSB_STAGE0_OFFSET_LOW_CLAMP, LSB_STAGE0_OFFSET_LOW_CLAMP,
        MSB_STAGE1_CONNECTION_6_to_0, LSB_STAGE1_CONNECTION_6_to_0, MSB_STAGE1_CONNECTION_12_to_7, LSB_STAGE1_CONNECTION_12_to_7, 
        MSB_STAGE1_AFE_OFFSET, LSB_STAGE1_AFE_OFFSET, MSB_STAGE1_SENSITIVITY, LSB_STAGE1_SENSITIVITY,
        MSB_STAGE1_OFFSET_LOW, LSB_STAGE1_OFFSET_LOW, MSB_STAGE1_OFFSET_HIGH, LSB_STAGE1_OFFSET_HIGH,
        MSB_STAGE1_OFFSET_HIGH_CLAMP, LSB_STAGE1_OFFSET_HIGH_CLAMP, MSB_STAGE1_OFFSET_LOW_CLAMP, LSB_STAGE1_OFFSET_LOW_CLAMP,
        MSB_STAGE2_CONNECTION_6_to_0, LSB_STAGE2_CONNECTION_6_to_0, MSB_STAGE2_CONNECTION_12_to_7, LSB_STAGE2_CONNECTION_12_to_7, 
        MSB_STAGE2_AFE_OFFSET, LSB_STAGE2_AFE_OFFSET, MSB_STAGE2_SENSITIVITY, LSB_STAGE2_SENSITIVITY, 
        MSB_STAGE2_OFFSET_LOW, LSB_STAGE2_OFFSET_LOW, MSB_STAGE2_OFFSET_HIGH, LSB_STAGE2_OFFSET_HIGH, 
        MSB_STAGE2_OFFSET_HIGH_CLAMP, LSB_STAGE2_OFFSET_HIGH_CLAMP, MSB_STAGE2_OFFSET_LOW_CLAMP, LSB_STAGE2_OFFSET_LOW_CLAMP,
        MSB_STAGE3_CONNECTION_6_to_0, LSB_STAGE3_CONNECTION_6_to_0, MSB_STAGE3_CONNECTION_12_to_7, LSB_STAGE3_CONNECTION_12_to_7, 
        MSB_STAGE3_AFE_OFFSET, LSB_STAGE3_AFE_OFFSET, MSB_STAGE3_SENSITIVITY, LSB_STAGE3_SENSITIVITY, 
        MSB_STAGE3_OFFSET_LOW, LSB_STAGE3_OFFSET_LOW, MSB_STAGE3_OFFSET_HIGH, LSB_STAGE3_OFFSET_HIGH, 
        MSB_STAGE3_OFFSET_HIGH_CLAMP, LSB_STAGE3_OFFSET_HIGH_CLAMP, MSB_STAGE3_OFFSET_LOW_CLAMP, LSB_STAGE3_OFFSET_LOW_CLAMP,
        MSB_STAGE4_CONNECTION_6_to_0, LSB_STAGE4_CONNECTION_6_to_0, MSB_STAGE4_CONNECTION_12_to_7, LSB_STAGE4_CONNECTION_12_to_7, 
        MSB_STAGE4_AFE_OFFSET, LSB_STAGE4_AFE_OFFSET, MSB_STAGE4_SENSITIVITY, LSB_STAGE4_SENSITIVITY, 
        MSB_STAGE4_OFFSET_LOW, LSB_STAGE4_OFFSET_LOW, MSB_STAGE4_OFFSET_HIGH, LSB_STAGE4_OFFSET_HIGH, 
        MSB_STAGE4_OFFSET_HIGH_CLAMP, LSB_STAGE4_OFFSET_HIGH_CLAMP, MSB_STAGE4_OFFSET_LOW_CLAMP, LSB_STAGE4_OFFSET_LOW_CLAMP,
        MSB_STAGE5_CONNECTION_6_to_0, LSB_STAGE5_CONNECTION_6_to_0, MSB_STAGE5_CONNECTION_12_to_7, LSB_STAGE5_CONNECTION_12_to_7, 
        MSB_STAGE5_AFE_OFFSET, LSB_STAGE5_AFE_OFFSET, MSB_STAGE5_SENSITIVITY, LSB_STAGE5_SENSITIVITY, 
        MSB_STAGE5_OFFSET_LOW, LSB_STAGE5_OFFSET_LOW, MSB_STAGE5_OFFSET_HIGH, LSB_STAGE5_OFFSET_HIGH, 
        MSB_STAGE5_OFFSET_HIGH_CLAMP, LSB_STAGE5_OFFSET_HIGH_CLAMP, MSB_STAGE5_OFFSET_LOW_CLAMP, LSB_STAGE5_OFFSET_LOW_CLAMP,
        MSB_STAGE6_CONNECTION_6_to_0, LSB_STAGE6_CONNECTION_6_to_0, MSB_STAGE6_CONNECTION_12_to_7, LSB_STAGE6_CONNECTION_12_to_7, 
        MSB_STAGE6_AFE_OFFSET, LSB_STAGE6_AFE_OFFSET, MSB_STAGE6_SENSITIVITY, LSB_STAGE6_SENSITIVITY, 
        MSB_STAGE6_OFFSET_LOW, LSB_STAGE6_OFFSET_LOW, MSB_STAGE6_OFFSET_HIGH, LSB_STAGE6_OFFSET_HIGH,
        MSB_STAGE6_OFFSET_HIGH_CLAMP, LSB_STAGE6_OFFSET_HIGH_CLAMP, MSB_STAGE6_OFFSET_LOW_CLAMP, LSB_STAGE6_OFFSET_LOW_CLAMP,
        MSB_STAGE7_CONNECTION_6_to_0, LSB_STAGE7_CONNECTION_6_to_0, MSB_STAGE7_CONNECTION_12_to_7, LSB_STAGE7_CONNECTION_12_to_7, 
        MSB_STAGE7_AFE_OFFSET, LSB_STAGE7_AFE_OFFSET, MSB_STAGE7_SENSITIVITY, LSB_STAGE7_SENSITIVITY, 
        MSB_STAGE7_OFFSET_LOW, LSB_STAGE7_OFFSET_LOW, MSB_STAGE7_OFFSET_HIGH, LSB_STAGE7_OFFSET_HIGH, 
        MSB_STAGE7_OFFSET_HIGH_CLAMP, LSB_STAGE7_OFFSET_HIGH_CLAMP, MSB_STAGE7_OFFSET_LOW_CLAMP, LSB_STAGE7_OFFSET_LOW_CLAMP,
        MSB_STAGE8_CONNECTION_6_to_0, LSB_STAGE8_CONNECTION_6_to_0, MSB_STAGE8_CONNECTION_12_to_7, LSB_STAGE8_CONNECTION_12_to_7, 
        MSB_STAGE8_AFE_OFFSET, LSB_STAGE8_AFE_OFFSET, MSB_STAGE8_SENSITIVITY, LSB_STAGE8_SENSITIVITY, 
        MSB_STAGE8_OFFSET_LOW, LSB_STAGE8_OFFSET_LOW, MSB_STAGE8_OFFSET_HIGH, LSB_STAGE8_OFFSET_HIGH, 
        MSB_STAGE8_OFFSET_HIGH_CLAMP, LSB_STAGE8_OFFSET_HIGH_CLAMP, MSB_STAGE8_OFFSET_LOW_CLAMP, LSB_STAGE8_OFFSET_LOW_CLAMP,
        MSB_STAGE9_CONNECTION_6_to_0, LSB_STAGE9_CONNECTION_6_to_0, MSB_STAGE9_CONNECTION_12_to_7, LSB_STAGE9_CONNECTION_12_to_7, 
        MSB_STAGE9_AFE_OFFSET, LSB_STAGE9_AFE_OFFSET, MSB_STAGE9_SENSITIVITY, LSB_STAGE9_SENSITIVITY,
        MSB_STAGE9_OFFSET_LOW, LSB_STAGE9_OFFSET_LOW, MSB_STAGE9_OFFSET_HIGH, LSB_STAGE9_OFFSET_HIGH, 
        MSB_STAGE9_OFFSET_HIGH_CLAMP, LSB_STAGE9_OFFSET_HIGH_CLAMP, MSB_STAGE9_OFFSET_LOW_CLAMP, LSB_STAGE9_OFFSET_LOW_CLAMP,
        MSB_STAGE10_CONNECTION_6_to_0, LSB_STAGE10_CONNECTION_6_to_0, MSB_STAGE10_CONNECTION_12_to_7, LSB_STAGE10_CONNECTION_12_to_7, 
        MSB_STAGE10_AFE_OFFSET, LSB_STAGE10_AFE_OFFSET, MSB_STAGE10_SENSITIVITY, LSB_STAGE10_SENSITIVITY, 
        MSB_STAGE10_OFFSET_LOW, LSB_STAGE10_OFFSET_LOW, MSB_STAGE10_OFFSET_HIGH, LSB_STAGE10_OFFSET_HIGH, 
        MSB_STAGE10_OFFSET_HIGH_CLAMP, LSB_STAGE10_OFFSET_HIGH_CLAMP, MSB_STAGE10_OFFSET_LOW_CLAMP, LSB_STAGE10_OFFSET_LOW_CLAMP,
        MSB_STAGE11_CONNECTION_6_to_0, LSB_STAGE11_CONNECTION_6_to_0, MSB_STAGE11_CONNECTION_12_to_7, LSB_STAGE11_CONNECTION_12_to_7, 
        MSB_STAGE11_AFE_OFFSET, LSB_STAGE11_AFE_OFFSET, MSB_STAGE11_SENSITIVITY, LSB_STAGE11_SENSITIVITY, 
        MSB_STAGE11_OFFSET_LOW, LSB_STAGE11_OFFSET_LOW, MSB_STAGE11_OFFSET_HIGH, LSB_STAGE11_OFFSET_HIGH, 
        MSB_STAGE11_OFFSET_HIGH_CLAMP, LSB_STAGE11_OFFSET_HIGH_CLAMP, MSB_STAGE11_OFFSET_LOW_CLAMP, LSB_STAGE11_OFFSET_LOW_CLAMP
        };

    errChk = i2c_master_write_to_device(I2C_NUM_0, AD7147_ADDR, dataToBank2, sizeof(dataToBank2), 80000);
    ESP_ERROR_CHECK(errChk);
}

/**
 * print bluetooth address
 */
void print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void blePressure_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blePressure_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blePressure_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void blePressure_tx_psure_stop(void)
{
    xTimerStop( blePressure_tx_timer, 1000 / portTICK_PERIOD_MS );
}

static void blePressure_tx_psure_reset(void)
{
    int rc;

    if (xTimerReset(blePressure_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
        rc = 0;
    } else {
        rc = 1;
    }

    assert(rc == 0);

}

/* This function get capacitance change and notifies it to the client */
static void blePressure_tx_psure(TimerHandle_t ev)
{
    static uint8_t cap_chg[2];
    int rc;
    struct os_mbuf *om;

    // if the connected device unsubscribe the notification of capacitance change
    if (!notify_state) {
        blePressure_tx_psure_stop();
        return;
    }

    // split the 16 bits capacitance change to two 8 bits
    cap_chg[0] = ((capacitance_change & 0xFF00) >> 8); 
    cap_chg[1] = capacitance_change & 0x00FF; 
    ESP_LOGI(TAG, "cap chg: %d", cap_chg[0]);
    ESP_LOGI(TAG, "cap chg: %d", cap_chg[1]);

    // send out the capacitance change via bluetooth
    om = ble_hs_mbuf_from_flat(cap_chg, sizeof(cap_chg));
    rc = ble_gatts_notify_custom(conn_handle, cdc_value_handle, om);

    assert(rc == 0);

    // reset timer
    blePressure_tx_psure_reset();
}

static int blePressure_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blePressure_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blePressure_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blePressure_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, cdc_value_handle);
        if (event->subscribe.attr_handle == cdc_value_handle) {
            notify_state = event->subscribe.cur_notify;
            blePressure_tx_psure_reset();
        } else if (event->subscribe.attr_handle != cdc_value_handle) {
            notify_state = event->subscribe.cur_notify;
            blePressure_tx_psure_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void blePressure_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blePressure_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blePressure_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
    blePressure_advertise();
}

static void blePressure_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void blePressure_host_task(void *param)
{
    ESP_LOGI(TAG2, "BLE Host Task Started");

    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void set_nominal_sensor_val(void)
{
    uint32_t cdc_sum;
    uint16_t avg_cdc_16bit;
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // collect 10 cdc data to get the average cdc value
    cdc_16bit_arr_start_flag = 1;
    while(cdc_16bit_arr_full_flag == 0);
    cdc_16bit_arr_full_flag = 0;
    cdc_sum = 0;
    for(int i = 0; i < 10; i ++)
    { 
        cdc_sum += cdc_16bit_arr[i];
    }
    avg_cdc_16bit = cdc_sum / 10;

    cdc_nominal_val = avg_cdc_16bit;
    ESP_LOGI(TAG, "%d", avg_cdc_16bit);
}