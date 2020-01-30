/**
  ******************************************************************************
  * @file   main.c
  * @author Ji Chen
  * @brief  Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "bma2x2.h"
#include "bmg160.h"
#include "bmm150.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#if defined(HORIZON_MINI_L4)
#include "al_stm32l4xx.h"
#include "al_stm32l4xx_i2c.h"
#include "al_stm32l4xx_uart.h"
#include "nucleo_l432kc_bsp.h"
#elif defined(HORIZON_GS_STD_L4)
#include "al_stm32l4xx.h"
#include "nucleo_l476rg_bsp.h"
#else
#error please specify a target board
#endif

/* Definitions ---------------------------------------------------------------*/
#define BMX055_I2C_FD       (0)
#define BMP280_I2C_FD       (0)

#define BMX055_ACC_DEV_ADDR (BMA2x2_I2C_ADDR1 << 1)
#define BMX055_GYR_DEV_ADDR (BMG160_I2C_ADDR1 << 1)
#define BMX055_MAG_DEV_ADDR (BMM150_DEFAULT_I2C_ADDRESS << 1)
#define BMP280_DEV_ADDR     (0x76 << 1)

#define BIT_BMA_XFER_CPLT   (1)
#define BIT_BMA_ARMED       (1 << 1)
#define BIT_BMA_ERROR       (1 << 2)
#define BIT_BMG_XFER_CPLT   (1 << 3)
#define BIT_BMG_ARMED       (1 << 4)
#define BIT_BMG_ERROR       (1 << 5)
#define BIT_BMM_XFER_CPLT   (1 << 6)
#define BIT_BMM_DRDY        (1 << 7)
#define BIT_BMM_ERROR       (1 << 8)

#define BMX_POWERUP_MS      (50)
#define BMX_ACC_TIMEOUT_MS  (10)
#define BMX_GYR_TIMEOUT_MS  (3)
#define BMX_MAG_PERIOD_MS   (64)
#define BMX_DATA_STABLE_MS  (2 * BMX_ACC_TIMEOUT_MS)

/* Typedef -------------------------------------------------------------------*/
struct bmx_busStatus {
    const EventBits_t event;
    volatile int ec;
};

/* Global variables ----------------------------------------------------------*/
SemaphoreHandle_t gSem_bmaDataReady;
SemaphoreHandle_t gSem_bmgNewData;
SemaphoreHandle_t gSem_patternRecv;
EventGroupHandle_t gEvt_globalEvents;

/* bma2x2 */
struct bma2x2_t g_bma2x2;
struct bmx_busStatus g_bmaBusStatus = { BIT_BMA_XFER_CPLT };
volatile u8 g_bmaRawData[BMA2x2_ACCEL_XYZ_DATA_SIZE];
volatile int g_bmaArmed;
volatile int g_bmaIntrCnt;

/* bmg160 */
struct bmg160_t g_bmg160;
struct bmx_busStatus g_bmgBusStatus = { BIT_BMG_XFER_CPLT };
volatile u8 g_bmgRawData[BMG160_XYZ_DATA_SIZE];
volatile int g_bmgArmed;
volatile int g_bmgIntrCnt;
volatile int g_bmgIntrResponse;

/* bmm150 */
struct bmm150_dev g_bmm150;
struct bmx_busStatus g_bmmBusStatus = { BIT_BMM_XFER_CPLT };

/* test */
volatile int g_seq;
const char g_strBlob[] =
" ___________\r\n"
"< what's up >\r\n"
" -----------\r\n"
"        \\   ^__^\r\n"
"         \\  (oo)\\_______\r\n"
"            (__)\\       )\\/\\\r\n"
"                ||----w |\r\n"
"                ||     ||\r\n";

/* Function prototypes -------------------------------------------------------*/
s8 bma_i2c_write(u8, u8, u8*, u8);
s8 bma_i2c_read(u8, u8, u8*, u8);
s8 bmg_i2c_write(u8, u8, u8*, u8);
s8 bmg_i2c_read(u8, u8, u8*, u8);
int8_t bmm_i2c_read(uint8_t, uint8_t, uint8_t*, uint16_t);
int8_t bmm_i2c_write(uint8_t, uint8_t, uint8_t*, uint16_t);
void bmx_delay_msec(u32);
int bmx_busXferCplt(union sigval, int);

/* Functions -----------------------------------------------------------------*/
/* BMX055 i2c API */
s8 bmx_i2c_xfer(struct al_i2c_aiocb *aiocb, int (*f)(struct al_i2c_aiocb*)) {
    if ((*f)(aiocb) < 0) {
        return 1;
    }
    xEventGroupWaitBits(gEvt_globalEvents,
                        ((struct bmx_busStatus*) aiocb->aio_sigevent.sigev_value.sival_ptr)->event,
                        pdTRUE,
                        pdTRUE,
                        portMAX_DELAY);
    return (((struct bmx_busStatus*) aiocb->aio_sigevent.sigev_value.sival_ptr)->ec == 0) ?
        0 : 1;
}

/* BMA2x2 */
inline s8 bma_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    struct al_i2c_aiocb aiocb = {
        BMX055_I2C_FD,
        dev_addr,
        reg_addr,
        reg_data,
        cnt,
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmaBusStatus;
    aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
    return bmx_i2c_xfer(&aiocb, al_i2c_aio_write);
}

inline s8 bma_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    struct al_i2c_aiocb aiocb = {
        BMX055_I2C_FD,
        dev_addr,
        reg_addr,
        reg_data,
        cnt,
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmaBusStatus;
    aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
    return bmx_i2c_xfer(&aiocb, al_i2c_aio_read);
}

/* BMG160 */
inline s8 bmg_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    struct al_i2c_aiocb aiocb = {
        BMX055_I2C_FD,
        dev_addr,
        reg_addr,
        reg_data,
        cnt,
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmgBusStatus;
    aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
    return bmx_i2c_xfer(&aiocb, al_i2c_aio_write);
}

inline s8 bmg_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    struct al_i2c_aiocb aiocb = {
        BMX055_I2C_FD,
        dev_addr,
        reg_addr,
        reg_data,
        cnt,
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmgBusStatus;
    aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
    return bmx_i2c_xfer(&aiocb, al_i2c_aio_read);
}

inline int8_t bmm_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len) {
    struct al_i2c_aiocb aiocb = {
        BMX055_I2C_FD,
        dev_id,
        reg_addr,
        read_data,
        len,
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmmBusStatus;
    aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
    return bmx_i2c_xfer(&aiocb, al_i2c_aio_read);
}

inline int8_t bmm_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *written_data, uint16_t len) {
    struct al_i2c_aiocb aiocb = {
        BMX055_I2C_FD,
        dev_id,
        reg_addr,
        written_data,
        len,
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmmBusStatus;
    aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
    return bmx_i2c_xfer(&aiocb, al_i2c_aio_write);
}

inline void bmx_delay_msec(u32 msec) { vTaskDelay(pdMS_TO_TICKS(msec)); }

void bma_rawDataNormalize(uint8_t *rawData, int16_t *normalData) {
    for (int i = 0; i < 3; i++) {
        normalData[i] = (int16_t) ((rawData[2 * i] & BMA2x2_12_BIT_SHIFT) | (rawData[2 * i + 1] << BMA2x2_SHIFT_EIGHT_BITS));
        normalData[i] >>= BMA2x2_SHIFT_FOUR_BITS;
    }
}

void bmg_rawDataNormalize(uint8_t *rawData, int16_t *normalData) {
    for (int i = 0; i < 3; i++) {
        normalData[i] = (int16_t) (rawData[2 * i] | rawData[2 * i + 1] << 8);
    }
}

/* ISR */
/* uart (fd = 1) */
int uart1_dataRecv(unsigned short data, int rc) {
    static unsigned short lastData = 0;
    BaseType_t higherPriorityTaskWoken;

    if (rc > 0) {
        if (lastData == 'a' && data == 'Z') {
            higherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(gSem_patternRecv, &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
        lastData = data;
    } else {
        lastData = 0;
    }

    return 0;
}

int uart1_msgSent(union sigval sigev_value, int rc) {
    if (rc <= 0) {
        BSP_SYSLED_Set_Fault();
    } else {
        if (sigev_value.sival_int) {
            BSP_SYSLED_Set_Normal();
        } else if ((g_seq % 10) == 0) {
            BSP_SYSLED_Set_UnderInit();
        }
    }

    return 0;
}

/* i2c (fd = 0) */
int bmx_busXferCplt(union sigval sigev_value, int ec) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(gEvt_globalEvents,
                              ((struct bmx_busStatus*) sigev_value.sival_ptr)->event,
                              &higherPriorityTaskWoken);
    ((struct bmx_busStatus*) sigev_value.sival_ptr)->ec = ec;
    return (int) higherPriorityTaskWoken;
}

/* external interrupts (INT0) */
void al_exti_0_callback(void) {
    BaseType_t xHigherPriorityTaskWoken;

    if (g_bmaArmed) {
        ++g_bmaIntrCnt;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(gSem_bmaDataReady, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* external interrupts (INT1) */
void al_exti_1_callback(void) {
    BaseType_t xHigherPriorityTaskWoken;

    if (g_bmgArmed) {
        ++g_bmgIntrCnt;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(gSem_bmgNewData, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Threads */
void thr_printBlob(void *pvParameters) {
    struct al_uart_aiocb aiocb = {
        1,
        (void*) g_strBlob,
        sizeof(g_strBlob),
        AIO_FLAGNONE,
        { }
    };
    aiocb.aio_sigevent.sigev_value.sival_int = 1;
    aiocb.aio_sigevent.sigev_notify_function = uart1_msgSent;

    while (1) {
        xSemaphoreTake(gSem_patternRecv, portMAX_DELAY);
        al_uart_aio_write(&aiocb);
    }
}

void thr_accelCollector(void *pvParameters) {
    s8 bma_rc = 0;

    /*================*/
    /* configure bma2x2 */
    g_bma2x2.dev_addr = BMX055_ACC_DEV_ADDR;
    g_bma2x2.bus_write = bma_i2c_write;
    g_bma2x2.bus_read = bma_i2c_read;
    g_bma2x2.delay_msec = bmx_delay_msec;

    /* wait to power up */
    vTaskDelay(pdMS_TO_TICKS(BMX_POWERUP_MS));

    /* init */
    bma_rc = bma2x2_init(&g_bma2x2);

    /* soft reset */
    //bma_rc |= bma2x2_soft_rst();

    /* switch to normal mode (default) */
    //bma_rc |= bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

    /* set range */
    bma_rc |= bma2x2_set_range(BMA2x2_RANGE_8G); // 0x0F

    /* set bandwidth */
    bma_rc |= bma2x2_set_bw(BMA2x2_BW_62_50HZ); // 0x10

    /* configure interrupt(s) */
    /* set INT1 output type as push pull */
    bma_rc |= bma2x2_set_intr_output_type(BMA2x2_INTR1_OUTPUT, PUSS_PULL); // 0x20
    /* set INT1 active level as active high */
    bma_rc |= bma2x2_set_intr_level(BMA2x2_INTR1_LEVEL, ACTIVE_HIGH); // 0x20
    /* map data ready interrupt to INT1 pin */
    bma_rc |= bma2x2_set_new_data(BMA2x2_INTR1_NEWDATA, INTR_ENABLE); // 0x1A

    // as per datasheet, it should wait for at least 10ms, then (re)enable intrrupt
    vTaskDelay(pdMS_TO_TICKS(BMX_DATA_STABLE_MS));

    /* enable data ready interrupt */
    bma_rc |= bma2x2_set_intr_enable(BMA2x2_DATA_ENABLE, INTR_ENABLE); // 0x17

    /*================*/
    /* notify that the accelerometer has been armed */
    /* the main thread will start polling data once both gyro and accel are armed */
    xEventGroupSetBits(gEvt_globalEvents, (bma_rc == 0) ? BIT_BMA_ARMED : BIT_BMA_ERROR);

    /*================*/
    while (1) {
        if (xSemaphoreTake(gSem_bmaDataReady, pdMS_TO_TICKS(BMX_ACC_TIMEOUT_MS)) == pdTRUE) {
            struct al_i2c_aiocb aiocb = {
                BMX055_I2C_FD,
                BMX055_ACC_DEV_ADDR,
                BMA2x2_ACCEL_X12_LSB_REG,
                g_bmaRawData,
                BMA2x2_SHIFT_SIX_BITS,
                AIO_FLAGNONE,
                { }
            };
            aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmaBusStatus;
            aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
            al_i2c_aio_read(&aiocb);
        } else {
            xEventGroupSetBits(gEvt_globalEvents, BIT_BMA_ERROR);
        }
    }
}

void thr_gyroCollector(void *pvParameters) {
    s8 bmg_rc = 0;

    /*================*/
    /* configure bmg160 */
    g_bmg160.dev_addr = BMX055_GYR_DEV_ADDR;
    g_bmg160.bus_write = bmg_i2c_write;
    g_bmg160.bus_read = bmg_i2c_read;
    g_bmg160.delay_msec = bmx_delay_msec;

    /* wait to power up */
    vTaskDelay(pdMS_TO_TICKS(BMX_POWERUP_MS));

    /* init */
    bmg_rc = bmg160_init(&g_bmg160);

    /* soft reset (will cause I2C bus error) */
    //bmg_rc |= bmg160_set_soft_rst();

    /* switch to normal mode (default) */
    //bmg_rc |= bmg160_set_power_mode(BMG160_MODE_NORMAL);

    /* set full scale range */
    bmg_rc |= bmg160_set_range_reg(BMG160_RANGE_2000); // 0x0F

    /* set bandwidth */
    bmg_rc |= bmg160_set_bw(BMG160_BW_230_HZ); // 0x10

    /* configure interrupt(s) */
    /* set INT3 output type as push pull */
    bmg_rc |= bmg160_set_intr_output_type(BMG160_INTR1, 0); // 0x16
    /* set INT3 active level as active high */
    bmg_rc |= bmg160_set_intr_level(BMG160_INTR1, 1); // 0x16
    /* map new data interrupt to INT3 pin */
    bmg_rc |= bmg160_set_intr_data(BMG160_INTR1_DATA, BMG160_ENABLE); // 0x18

    // XXX delay?
    vTaskDelay(pdMS_TO_TICKS(BMX_DATA_STABLE_MS));

    /* enable new data interrupt */
    bmg_rc |= bmg160_set_data_enable(BMG160_ENABLE); // 0x15

    /*================*/
    /* notify that the accelerometer has been armed */
    xEventGroupSetBits(gEvt_globalEvents, (bmg_rc == 0) ? BIT_BMG_ARMED : BIT_BMG_ERROR);

    /*================*/
    while (1) {
        if (xSemaphoreTake(gSem_bmgNewData, pdMS_TO_TICKS(BMX_GYR_TIMEOUT_MS)) == pdTRUE) {
            struct al_i2c_aiocb aiocb = {
                BMX055_I2C_FD,
                BMX055_GYR_DEV_ADDR,
                BMG160_RATE_X_LSB_BIT__REG,
                g_bmgRawData,
                BMG160_ALL_DATA_FRAME_LENGTH,
                AIO_FLAGNONE,
                { }
            };
            aiocb.aio_sigevent.sigev_value.sival_ptr = &g_bmgBusStatus;
            aiocb.aio_sigevent.sigev_notify_function = bmx_busXferCplt;
            ++g_bmgIntrResponse;
            al_i2c_aio_read(&aiocb);
       } else {
           xEventGroupSetBits(gEvt_globalEvents, BIT_BMG_ERROR);
       }
    }
}

void thr_magCollector(void *pvParameters) {
    int8_t bmm_rc = 0;

    /*================*/
    /* configure bmm150 */
    g_bmm150.dev_id = BMX055_MAG_DEV_ADDR;
    g_bmm150.intf = BMM150_I2C_INTF;
    g_bmm150.read = bmm_i2c_read;
    g_bmm150.write = bmm_i2c_write;
    g_bmm150.delay_ms = bmx_delay_msec;

    /* wait for device to power up */
    vTaskDelay(pdMS_TO_TICKS(BMX_POWERUP_MS));

    /* init */
    bmm_rc = bmm150_init(&g_bmm150);

    /* soft reset */
    /* soft reset doesn't execute a full POR sequence, but all registers are
       reset except for the "trim" registers and the power controll register. */
    bmm_rc |= bmm150_soft_reset(&g_bmm150);

    /* Setting the preset mode as high accuracy mode */
    g_bmm150.settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
    bmm_rc |= bmm150_set_presetmode(&g_bmm150);

    if (bmm_rc != 0) {
        xEventGroupSetBits(gEvt_globalEvents, BIT_BMM_ERROR);
    }

    while (1) {
        /* Setting the power mode as forced (acquire data) */
        g_bmm150.settings.pwr_mode = BMM150_FORCED_MODE;
        bmm_rc = bmm150_set_op_mode(&g_bmm150);

        vTaskDelay(pdMS_TO_TICKS(BMX_MAG_PERIOD_MS)); // ~ 15.625Hz

        /* Mag data for X,Y,Z axes are stored inside the
           bmm150_dev structure in float format (unit in uT) */
        bmm_rc |= bmm150_read_mag_data(&g_bmm150);

        xEventGroupSetBits(gEvt_globalEvents, (bmm_rc == 0) ? BIT_BMM_DRDY : BIT_BMM_ERROR);
    }
}

void thr_main(void *pvParameters) {
    static char msgBuf[128];
    int16_t accData[3];
    int16_t gyrData[3];
    unsigned int seq;
    TickType_t lastWakeTime;
    struct al_uart_aiocb uart1_aiocb = {
        1,
        msgBuf,
        0,
        AIO_NONBLOCK,
        { }
    };
    uart1_aiocb.aio_sigevent.sigev_value.sival_int = 0;
    uart1_aiocb.aio_sigevent.sigev_notify_function = uart1_msgSent;

    // start UART reading
    al_uart_async_read_one(1, uart1_dataRecv);

    // print welcome message
    uart1_aiocb.aio_nbytes = snprintf(msgBuf, sizeof(msgBuf), "initializing...\r\n");
    al_uart_aio_write(&uart1_aiocb);

    // wait to init...
    xEventGroupWaitBits(gEvt_globalEvents,
                        BIT_BMA_ARMED | BIT_BMG_ARMED,
                        pdTRUE,
                        pdTRUE,
                        portMAX_DELAY);
    g_bmaArmed = 1;
    g_bmgArmed = 1;

    // start the main loop
    lastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
        seq = ++g_seq;

        xEventGroupClearBits(gEvt_globalEvents, BIT_BMA_XFER_CPLT);
        xEventGroupWaitBits(gEvt_globalEvents,
                            BIT_BMA_XFER_CPLT,
                            pdTRUE,
                            pdTRUE,
                            portMAX_DELAY);
        bma_rawDataNormalize((uint8_t*) g_bmaRawData, accData);

        xEventGroupClearBits(gEvt_globalEvents, BIT_BMG_XFER_CPLT);
        xEventGroupWaitBits(gEvt_globalEvents,
                            BIT_BMG_XFER_CPLT,
                            pdTRUE,
                            pdTRUE,
                            portMAX_DELAY);
        bmg_rawDataNormalize((uint8_t*) g_bmgRawData, gyrData);

        xEventGroupClearBits(gEvt_globalEvents, BIT_BMM_DRDY);
        xEventGroupWaitBits(gEvt_globalEvents,
                            BIT_BMM_DRDY,
                            pdTRUE,
                            pdTRUE,
                            portMAX_DELAY);

        uart1_aiocb.aio_nbytes = snprintf(msgBuf, sizeof(msgBuf),
            "[%d] (%d)(%d,%d) [{%hd,%hd,%hd},{%hd,%hd,%hd},{%f,%f,%f}]\r\n",
            seq, g_bmaIntrCnt, g_bmgIntrCnt, g_bmgIntrResponse,
            accData[0], accData[1], accData[2],
            gyrData[0], gyrData[1], gyrData[2],
            g_bmm150.data.x, g_bmm150.data.y, g_bmm150.data.z);
        al_uart_aio_write(&uart1_aiocb);
    }
}

int main(void) {
    /* BSP Init */
    BSP_MCU_Init();

    /* AL Init */
    al_uart_init();
    al_i2c_init();
    al_tim_dshot_init();

    /* Create semaphores */
    gSem_bmaDataReady = xSemaphoreCreateBinary();
    gSem_bmgNewData = xSemaphoreCreateBinary();
    gSem_patternRecv = xSemaphoreCreateBinary();

    /* Create events */
    gEvt_globalEvents = xEventGroupCreate();

    /* Create tasks */
    xTaskCreate(thr_main,
                "the main thread",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(thr_accelCollector,
                "thread to collect Accelerometer's data",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(thr_gyroCollector,
                "thread to collect Gyroscope's data",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(thr_magCollector,
                "thread to collect Magnetometer's data",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(thr_printBlob,
                "thread to print strBlob",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
       will never be reached.
       If the following line does execute, then there was insufficient FreeRTOS
       heap memory available for the idle and/or timer tasks to be created.
       See the memory management section on the FreeRTOS web site for more
       details. */
    while(1);
}

#if defined(HORIZON_GS_STD_L4)
void al_exti_2_callback(void) {
    static uint32_t period = 1000;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        period = (period == 1000) ? 100 : 1000;
        xTimerChangePeriodFromISR(xTimer_blinker, pdMS_TO_TICKS(period), NULL);
    }
}
#endif

/******************************** END OF FILE *********************************/
