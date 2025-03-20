/*
 * quality.c
 *
 *  Created on: Mar 7, 2025
 *      Author: NMAIL_NJH
 */

#include "main.h"

static int count=0;
static int spheredist[100];
static Point_t spheredata[100];
static Point_t sphereideal[100];

static int sphereideal_initialized=0;
static float magnitude[MAGBUFFSIZE];

static float quality_gaps_buffer;
static float quality_variance_buffer;
static float quality_wobble_buffer;

static int quality_gaps_computed=0;
static int quality_variance_computed=0;
static int quality_wobble_computed=0;

int gap_fuc_flag;
int gap_fuc_spheredist;

// How many surface gaps
float quality_surface_gap_error(void)
{
	float error=0.0f;
	int i, num;

	gap_fuc_flag = 1;
	if (quality_gaps_computed) return quality_gaps_buffer;
	for (i=0; i < 100; i++) {
		num = spheredist[i];
		gap_fuc_spheredist = num;
		if (num == 0) {
			error += 1.0f;
		} else if (num == 1) {
			error += 0.2f;
		} else if (num == 2) {
			error += 0.01f;
		}
	}
	quality_gaps_buffer = error;
//	quality_gaps_computed = 1;
	return quality_gaps_buffer;
}

// Variance in magnitude
float quality_magnitude_variance_error(void)
{
	float sum, mean, diff, variance;
	int i;

	if (quality_variance_computed) return quality_variance_buffer;
	sum = 0.0f;
	for (i=0; i < count; i++) {
		sum += magnitude[i];
	}
	mean = sum / (float)count;
	variance = 0.0f;
	for (i=0; i < count; i++) {
		diff = magnitude[i] - mean;
		variance += diff * diff;
	}
	variance /= (float)count;
	quality_variance_buffer = sqrtf(variance) / mean * 100.0f;
	quality_variance_computed = 1;
	return quality_variance_buffer;
}

// Offset of piecewise average data from ideal sphere surface
float quality_wobble_error(void)
{
	float sum, radius, x, y, z, xi, yi, zi;
	float xoff=0.0f, yoff=0.0f, zoff=0.0f;
	int i, n=0;

	if (quality_wobble_computed) return quality_wobble_buffer;
	sum = 0.0f;
	for (i=0; i < count; i++) {
		sum += magnitude[i];
	}
	radius = sum / (float)count;
	//if (pr) printf("  radius = %.2f\n", radius);
	for (i=0; i < 100; i++) {
		if (spheredist[i] > 0) {
			//if (pr) printf("  i=%3d", i);
			x = spheredata[i].x / (float)spheredist[i];
			y = spheredata[i].y / (float)spheredist[i];
			z = spheredata[i].z / (float)spheredist[i];
			//if (pr) printf("  at: %5.1f %5.1f %5.1f :", x, y, z);
			xi = sphereideal[i].x * radius;
			yi = sphereideal[i].y * radius;
			zi = sphereideal[i].z * radius;
			//if (pr) printf("   ideal: %5.1f %5.1f %5.1f :", xi, yi, zi);
			xoff += x - xi;
			yoff += y - yi;
			zoff += z - zi;
			//if (pr) printf("\n");
			n++;
		}
	}
	if (n == 0) return 100.0f;
	//if (pr) printf("  off = %.2f, %.2f, %.2f\n", xoff, yoff, zoff);
	xoff /= (float)n;
	yoff /= (float)n;
	zoff /= (float)n;
	//if (pr) printf("  off = %.2f, %.2f, %.2f\n", xoff, yoff, zoff);
	quality_wobble_buffer = sqrtf(xoff * xoff + yoff * yoff + zoff * zoff) / radius * 100.0f;
	quality_wobble_computed = 1;
	return quality_wobble_buffer;
}

// Freescale's algorithm fit error
float quality_spherical_fit_error(void)
{
	return magcal.FitError;
}
