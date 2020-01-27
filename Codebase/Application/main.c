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
#define BMX055_GYR_DEV_ADDR (0X68 << 1)
#define BMX055_MAG_DEV_ADDR (0x10 << 1)
#define BMP280_DEV_ADDR     (0x76 << 1)

#define BIT_BMA_XFER_CPLT   (1 << 0)
#define BIT_BMA_ARMED       (1 << 1)
#define BIT_BMA_ERROR       (1 << 2)

/* Typedef -------------------------------------------------------------------*/
struct bmx_busStatus {
    const EventBits_t event;
    volatile int ec;
};

/* Global variables ----------------------------------------------------------*/
EventGroupHandle_t gEvt_globalEvents;
SemaphoreHandle_t gSem_bmaNewData;
SemaphoreHandle_t gSem_patternRecv;

/* bma2x2 */
struct bma2x2_t g_bma2x2;
struct bmx_busStatus g_bmaBusStatus = { BIT_BMA_XFER_CPLT };
volatile uint8_t g_bmaRawData[BMA2x2_ACCEL_XYZ_DATA_SIZE];
volatile int g_bmaArmed;
volatile int g_bmaIntrCnt;

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
void bmx_delay_msec(u32);
int bmx_busXferCplt(union sigval, int);

/* Functions -----------------------------------------------------------------*/
/* BMX055 i2c API */
s8 bmx_i2c_xfer(struct al_i2c_aiocb *aiocb, int (*f)(struct al_i2c_aiocb*)) {
    if ((*f)(aiocb) < 0) {
        return ERROR;
    }
    xEventGroupWaitBits(gEvt_globalEvents,
                        ((struct bmx_busStatus*)aiocb->aio_sigevent.sigev_value.sival_ptr)->event,
                        pdTRUE,
                        pdTRUE,
                        portMAX_DELAY);
    return (s8) ((struct bmx_busStatus*)aiocb->aio_sigevent.sigev_value.sival_ptr)->ec;
}

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

inline void bmx_delay_msec(u32 msec) { vTaskDelay(pdMS_TO_TICKS(msec)); }

void bma_rawDataNormalize(uint8_t *rawData, int16_t *normalData) {
    for (int i = 0; i < 3; i++) {
        normalData[i] = (int16_t) ((rawData[2 *i] & BMA2x2_12_BIT_SHIFT) | (rawData[2 * i + 1] << BMA2x2_SHIFT_EIGHT_BITS));
        normalData[i] >>= BMA2x2_SHIFT_FOUR_BITS;
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

/* external interrupts (fd = 0) */
void al_exti_0_callback(void) {
    BaseType_t xHigherPriorityTaskWoken;

    if (g_bmaArmed) {
        ++g_bmaIntrCnt;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(gSem_bmaNewData, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Threads */
void thr_accelCollector(void *pvParameters) {
    s8 bma_rc = 0;

    /*================*/
    /* configure bma2x2 */
    g_bma2x2.dev_addr = BMX055_ACC_DEV_ADDR;
    g_bma2x2.bus_write = bma_i2c_write;
    g_bma2x2.bus_read = bma_i2c_read;
    g_bma2x2.delay_msec = bmx_delay_msec;

    /* wait to power up (10ms) */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* init */
    bma_rc |= bma2x2_init(&g_bma2x2);

    /* soft reset */
    bma_rc |= bma2x2_soft_rst();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* set power mode */
    //bma_rc |= bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

    /* set range */
    bma_rc |= bma2x2_set_range(BMA2x2_RANGE_16G); // 0x0F

    /* set bandwidth */
    bma_rc |= bma2x2_set_bw(BMA2x2_BW_62_50HZ); // 0x10

    /* configure interrupt(s) */
    //bma_rc |= bma2x2_set_intr_output_type(BMA2x2_INTR1_OUTPUT, PUSS_PULL);
    //bma_rc |= bma2x2_set_intr_level(BMA2x2_INTR1_LEVEL, ACTIVE_HIGH);
    bma_rc |= bma2x2_set_new_data(BMA2x2_INTR1_NEWDATA, INTR_ENABLE); // 0x1A

    /* as per datasheet, wait for at least 10ms, then (re)enable intr */
    vTaskDelay(pdMS_TO_TICKS(25));

    /* enable interrupts */
    bma_rc |= bma2x2_set_intr_enable(BMA2x2_DATA_ENABLE, INTR_ENABLE); // 0x17

    /*================*/
    /* notify the main thread that the accelerometer has been armed */
    /* it will start polling data once both gyro and accel are armed */
    xEventGroupSetBits(gEvt_globalEvents,
                       ((bma_rc == 0) ? BIT_BMA_ARMED : BIT_BMA_ERROR));

    /*================*/
    while (1) {
        if (xSemaphoreTake(gSem_bmaNewData, pdMS_TO_TICKS(10)) == pdTRUE) {
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

void thr_main(void *pvParameters) {
    static char msgBuf[81];
    int16_t accData[3];
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

    /* get ready */
    xEventGroupWaitBits(gEvt_globalEvents,
                        BIT_BMA_ARMED,
                        pdTRUE,
                        pdTRUE,
                        portMAX_DELAY);
    g_bmaArmed = 1;

    /* start loop */
    lastWakeTime = xTaskGetTickCount();
    while (1) {
        seq = g_seq++;
        xEventGroupClearBits(gEvt_globalEvents, BIT_BMA_XFER_CPLT);
        xEventGroupWaitBits(gEvt_globalEvents,
                            BIT_BMA_XFER_CPLT,
                            pdTRUE,
                            pdTRUE,
                            portMAX_DELAY);
        bma_rawDataNormalize((uint8_t*) g_bmaRawData, accData);
        uart1_aiocb.aio_nbytes = snprintf(msgBuf, sizeof(msgBuf),
            "[%d](%d) { %hd, %hd, %hd }\r\n",
            seq, g_bmaIntrCnt, accData[0], accData[1], accData[2]);
        al_uart_aio_write(&uart1_aiocb);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
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
    gSem_patternRecv = xSemaphoreCreateBinary();
    gSem_bmaNewData = xSemaphoreCreateBinary();

    /* Create events */
    gEvt_globalEvents = xEventGroupCreate();

    /* Create tasks */
    xTaskCreate(thr_main,
                "the main thread",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(thr_printBlob,
                "thread to print strBlob",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(thr_accelCollector,
                "thread to collect Accelerometer's data",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
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
