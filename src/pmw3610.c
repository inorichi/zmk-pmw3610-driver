/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

/* Timings (in us) used in SPI communication. Since MCU should not do other tasks during wait,
 * k_busy_wait is used instead of k_sleep */
// - sub-us time is rounded to us, due to the limitation of k_busy_wait, see :
// https://github.com/zephyrproject-rtos/zephyr/issues/6498
#define T_NCS_SCLK 1     /* 120 ns (rounded to 1us) */
#define T_SCLK_NCS_WR 10 /* 10 us */
#define T_SRAD 4         /* 4 us */
#define T_SRAD_MOTBR 4   /* same as T_SRAD */
#define T_SRX 1          /* 250 ns (rounded to 1 us) */
#define T_SWX 30         /* SWW: 30 us, SWR: 20 us */
#define T_BEXIT 1        /* 250 ns (rounded to 1us)*/

/* Sensor registers (addresses) */
#define PMW3610_REG_PRODUCT_ID 0x00
#define PMW3610_REG_REVISION_ID 0x01
#define PMW3610_REG_MOTION 0x02
#define PMW3610_REG_DELTA_X_L 0x03
#define PMW3610_REG_DELTA_Y_L 0x04
#define PMW3610_REG_DELTA_XY_H 0x05
#define PMW3610_REG_SQUAL 0x06
#define PMW3610_REG_SHUTTER_HIGHER 0x07
#define PMW3610_REG_SHUTTER_LOWER 0x08
#define PMW3610_REG_PIX_MAX 0x09
#define PMW3610_REG_PIX_AVG 0x0A
#define PMW3610_REG_PIX_MIN 0x0B

#define PMW3610_REG_CRC0 0x0C
#define PMW3610_REG_CRC1 0x0D
#define PMW3610_REG_CRC2 0x0E
#define PMW3610_REG_CRC3 0x0F
#define PMW3610_REG_SELF_TEST 0x10

#define PMW3610_REG_PERFORMANCE 0x11
#define PMW3610_REG_MOTION_BURST 0x12

#define PMW3610_REG_RUN_DOWNSHIFT 0x1B
#define PMW3610_REG_REST1_PERIOD 0x1C
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_REST2_PERIOD 0x1E
#define PMW3610_REG_REST2_DOWNSHIFT 0x1F
#define PMW3610_REG_REST3_PERIOD 0x20
#define PMW3610_REG_OBSERVATION 0x2D

#define PMW3610_REG_PIXEL_GRAB 0x35
#define PMW3610_REG_FRAME_GRAB 0x36

#define PMW3610_REG_POWER_UP_RESET 0x3A
#define PMW3610_REG_SHUTDOWN 0x3B

#define PMW3610_REG_SPI_CLK_ON_REQ 0x41
#define PMW3610_REG_RES_STEP 0x85

#define PMW3610_REG_NOT_REV_ID 0x3E
#define PMW3610_REG_NOT_PROD_ID 0x3F

#define PMW3610_REG_PRBS_TEST_CTL 0x47
#define PMW3610_REG_SPI_PAGE0 0x7F
#define PMW3610_REG_VCSEL_CTL 0x9E
#define PMW3610_REG_LSR_CONTROL 0x9F
#define PMW3610_REG_SPI_PAGE1 0xFF

/* Sensor identification values */
#define PMW3610_PRODUCT_ID 0x3E

/* Power-up register commands */
#define PMW3610_POWERUP_CMD_RESET 0x5A
#define PMW3610_POWERUP_CMD_WAKEUP 0x96

/* spi clock enable/disable commands */
#define PMW3610_SPI_CLOCK_CMD_ENABLE 0xBA
#define PMW3610_SPI_CLOCK_CMD_DISABLE 0xB5

/* Max register count readable in a single motion burst */
#define PMW3610_MAX_BURST_SIZE 10

/* Register count used for reading a single motion burst */
#define PMW3610_BURST_SIZE 7

/* Position in the motion registers */
#define PMW3610_X_L_POS 1
#define PMW3610_Y_L_POS 2
#define PMW3610_XY_H_POS 3
#define PMW3610_SHUTTER_H_POS 5
#define PMW3610_SHUTTER_L_POS 6

/* cpi/resolution range */
#define PMW3610_MAX_CPI 3200
#define PMW3610_MIN_CPI 200

/* write command bit position */
#define SPI_WRITE_BIT BIT(7)

/* Helper macros used to convert sensor values. */
#define PMW3610_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define PMW3610_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10, // test shows > 5ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] =
        200,                          // 150 us required, test shows too short,
                                      // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50, // 10 ms required in spec,
                                      // test shows too short,
                                      // especially when integrated with display,
                                      // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

// checked and keep
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

// checked and keep
static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

// primitive write without enable/disable spi clock on the sensor
static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    // enable spi clock
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    // write the target register
    err = _reg_write(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }

    // disable spi clock to save power
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Send motion burst address */
    uint8_t reg_buf[] = {PMW3610_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {
        .buf = buf,
        .len = burst_size,
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    /* Terminate burst */
    k_busy_wait(T_BEXIT);

    return 0;
}

/** Writing an array of registers in sequence, used in power-up register initialization and running
 * mode switching */
static int burst_write(const struct device *dev, const uint8_t *addr, const uint8_t *buf,
                       size_t size) {
    int err;

    // enable spi clock
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    /* Write data */
    for (size_t i = 0; i < size; i++) {
        err = _reg_write(dev, addr[i], buf[i]);

        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            return err;
        }
    }

    // disable spi clock to save power
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int check_product_id(const struct device *dev) {
    uint8_t product_id = 0x01;
    int err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    // Convert CPI to register value
    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};
    int err = burst_write(dev, addr, data, 3);
    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range", time);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static void set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    LOG_INF("async_init_power_up");

    /* Reset spi port */
    spi_cs_ctrl(dev, false);
    spi_cs_ctrl(dev, true);

    /* not required in datashet, but added any way to have a clear state */
    return reg_write(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    LOG_INF("async_init_clear_ob1");

    return reg_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    LOG_INF("async_init_check_ob1");

    uint8_t value;
    int err = reg_read(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    err = check_product_id(dev);
    if (err) {
        LOG_ERR("Failed checking product id");
        return err;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    LOG_INF("async_init_configure");

    int err = 0;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = reg_read(dev, reg, buf);
    }

    // cpi
    if (!err) {
        err = set_cpi(dev, CONFIG_PMW3610_CPI);
    }

    // set performace register: run mode, vel_rate, poshi_rate, poslo_rate
    if (!err) {
        // use the recommended value in datasheet: normal, 4ms, 4ms, 4ms
        err = reg_write(dev, PMW3610_REG_PERFORMANCE, 0x0D);
    }

    // sample period, which affects scaling of rest1 downshift time
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST1_PERIOD, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST2_PERIOD, CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
    }
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST3_PERIOD, CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
    }

    // downshift time for each rest mode
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                 CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                 CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                 CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
    }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

// checked and keep
static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed");
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("PMW3610 initialized");
            set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

static bool is_scroll_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < config->scroll_layers_size; i++) {
        if (config->scroll_layers[i] == curr_layer) {
            return true;
        }
    }
    return false;
}

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t dividor;

    bool is_scroll = is_scroll_layer(dev);
    if (is_scroll && data->curr_mode != 1) {
        set_cpi(dev, CONFIG_PMW3610_SCROLL_CPI);
        data->curr_mode = 1;
    } else if (!is_scroll && data->curr_mode != 0) {
        set_cpi(dev, CONFIG_PMW3610_CPI);
        data->curr_mode = 0;
    }
    switch (data->curr_mode) {
    case 1:
        dividor = CONFIG_PMW3610_SCROLL_CPI_DIVIDOR;
        break;
    default:
        dividor = CONFIG_PMW3610_CPI_DIVIDOR;
        break;
    }

    int err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        return err;
    }

    int16_t x =
        TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12) / dividor;
    int16_t y =
        TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12) / dividor;

    if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_0)) {
        data->x = -x;
        data->y = y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_90)) {
        data->x = y;
        data->y = -x;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_180)) {
        data->x = x;
        data->y = -y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_270)) {
        data->x = -y;
        data->y = x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_X)) {
        data->x = -data->x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_Y)) {
        data->y = -data->y;
    }

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    int16_t shutter =
        ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        reg_write(dev, 0x32, 0x00);

        data->sw_smart_flag = false;
    }

    if (!data->sw_smart_flag && shutter > 45) {
        reg_write(dev, 0x32, 0x80);

        data->sw_smart_flag = true;
    }
#endif

    if (data->x != 0 || data->y != 0) {
        if (!is_scroll) {
            input_report_rel(dev, INPUT_REL_X, data->x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, data->y, true, K_FOREVER);
        } else {
            input_report_rel(dev, INPUT_REL_WHEEL, data->y, true, K_FOREVER);
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    // submit the real handler work
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3610_report_data(dev);
    set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    LOG_INF("Configure irq...");

    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    LOG_INF("Configure irq done");

    return err;
}

static int pmw3610_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    // init smart algorithm flag;
    data->sw_smart_flag = false;

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);

    // check readiness of cs gpio pin and init it to inactive
    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
                                                                                                   \
    static const struct pixart_config config##n = {                                                \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = DT_PROP(DT_DRV_INST(n), scroll_layers),                                   \
        .scroll_layers_size = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                          \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
