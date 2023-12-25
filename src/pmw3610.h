#pragma once

#include <zephyr/drivers/sensor.h>
#include "pixart.h"

#ifdef __cplusplus
extern "C" {
#endif
enum pmw3610_attribute {
    /** Sensor CPI for both X and Y axes. */
    PMW3610_ATTR_CPI = SENSOR_ATTR_PRIV_START,

    /** Entering time from Run mode to REST1 mode [ms]. */
    PMW3610_ATTR_RUN_DOWNSHIFT_TIME,

    /** Entering time from REST1 mode to REST2 mode [ms]. */
    PMW3610_ATTR_REST1_DOWNSHIFT_TIME,

    /** Entering time from REST2 mode to REST3 mode [ms]. */
    PMW3610_ATTR_REST2_DOWNSHIFT_TIME,

    /** Sampling frequency time during REST1 mode [ms]. */
    PMW3610_ATTR_REST1_SAMPLE_TIME,

    /** Sampling frequency time during REST2 mode [ms]. */
    PMW3610_ATTR_REST2_SAMPLE_TIME,

    /** Sampling frequency time during REST3 mode [ms]. */
    PMW3610_ATTR_REST3_SAMPLE_TIME,
};

#ifdef __cplusplus
}
#endif
