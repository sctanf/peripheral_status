#ifndef SLIMENRF_SENSOR_FUSIONS
#define SLIMENRF_SENSOR_FUSIONS

#include "motionsense/motionsense.h"
#include "xiofusion/xiofusion.h"
#include "vqf/vqf.h"

enum fusion {
    FUSION_NONE,
    FUSION_FUSION,
    FUSION_MOTIONSENSE,
	FUSION_VQF
};

const char *fusion_names[] = {
    "None",
    "x-io Technologies Fusion",
    "NXP SensorFusion",
    "VQF"
};
const sensor_fusion_t *sensor_fusions[] = {
    NULL,
    &sensor_fusion_fusion,
    &sensor_fusion_motionsense,
    &sensor_fusion_vqf
};

#endif