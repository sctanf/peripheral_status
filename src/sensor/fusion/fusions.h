/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_SENSOR_FUSIONS
#define SLIMENRF_SENSOR_FUSIONS

#include "motionsense/motionsense.h"
#include "vqf/vqf.h"
#include "xiofusion/xiofusion.h"

enum fusion { FUSION_NONE, FUSION_FUSION, FUSION_MOTIONSENSE, FUSION_VQF };

const char* fusion_names[]
	= {"None", "x-io Technologies Fusion", "NXP SensorFusion", "VQF"};
const sensor_fusion_t* sensor_fusions[]
	= {NULL, &sensor_fusion_fusion, &sensor_fusion_motionsense, &sensor_fusion_vqf};

#endif