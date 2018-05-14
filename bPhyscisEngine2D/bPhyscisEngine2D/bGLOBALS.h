#pragma once
#include "Dependencies\GL\freeglut.h"
#define CIRCLE_RADIUS 5
#define PI 3.1415927
#define CIRCLE_BOUNCE 0.9
#define SQUARE_BOUNCE 0
#define G_SQR_BUF_DATA_SIZE 18
#define G_SQR_UV_BUF_DATA_SIZE 8
#define WIDTH 512
#define HEIGHT 384
#define UNIT_SCALE 10

#define MAX_THREADS 1024

static const GLfloat g_square_buffer_data[] = {
	-1.000, -1.000, 0.000,
	-1.000, 1.000, 0.000,
	1.000, 1.000, 0.000,
	-1.000, -1.000, 0.000,
	1.000, 1.000, 0.000,
	1.000, -1.000, 0.000
};

static const GLfloat g_square_uv_buffer_data[] = {
	1.000000, 0.000000,
	1.000000, 1.000000,
	0.000000, 1.000000,
	0.000000, 1.000000
};

#define MEM_SIZE (128)
#define MAX_SOURCE_SIZE (0x100000)