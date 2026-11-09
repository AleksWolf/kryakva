// VL53L8CX check: ID, firmware load, 8×8 frames at 15 Hz.
// Driver: ST ULD from stm32duino/VL53L8CX (folder uld); here only reads and writes over /dev/i2c-N.
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "vl53l8cx_api.h"

#define ADDR 0x29
#define CHUNK 4096
#define HZ 15

static int fd;
static unsigned i2c_errors;
static uint8_t tx[CHUNK + 2];

static uint8_t bus_write(void *handle, uint16_t reg, uint8_t *data, uint32_t size)
{
	(void)handle;
	for (uint32_t done = 0; done < size;) {
		uint32_t n = size - done > CHUNK ? CHUNK : size - done;
		uint16_t a = reg + done;
		tx[0] = a >> 8;
		tx[1] = a & 0xff;
		memcpy(tx + 2, data + done, n);
		struct i2c_msg m = {ADDR, 0, (uint16_t)(n + 2), tx};
		struct i2c_rdwr_ioctl_data q = {&m, 1};
		if (ioctl(fd, I2C_RDWR, &q) != 1) {
			i2c_errors++;
			return 1;
		}
		done += n;
	}
	return 0;
}

static uint8_t bus_read(void *handle, uint16_t reg, uint8_t *data, uint32_t size)
{
	(void)handle;
	for (uint32_t done = 0; done < size;) {
		uint32_t n = size - done > CHUNK ? CHUNK : size - done;
		uint16_t a = reg + done;
		uint8_t r[2] = {a >> 8, a & 0xff};
		struct i2c_msg m[2] = {
			{ADDR, 0, 2, r},
			{ADDR, I2C_M_RD, (uint16_t)n, data + done},
		};
		struct i2c_rdwr_ioctl_data q = {m, 2};
		if (ioctl(fd, I2C_RDWR, &q) != 2) {
			i2c_errors++;
			return 1;
		}
		done += n;
	}
	return 0;
}

static uint8_t bus_wait(void *handle, uint32_t ms)
{
	(void)handle;
	usleep(ms * 1000);
	return 0;
}

static double now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

static int by_value(const void *a, const void *b)
{
	return *(const int *)a - *(const int *)b;
}

int main(int argc, char **argv)
{
	static VL53L8CX_Configuration dev;
	static VL53L8CX_ResultsData res;
	static const int centre[4] = {27, 28, 35, 36};
	int bus = argc > 1 ? atoi(argv[1]) : 4;
	int frames = argc > 2 ? atoi(argv[2]) : 150;
	int stream = frames == 0;    /* 0 frames = endless stream */
	if (stream)
		frames = 1;
	char path[32];
	uint8_t id = 0, rev = 0, ready = 0, status;

	snprintf(path, sizeof path, "/dev/i2c-%d", bus);
	fd = open(path, O_RDWR);
	if (fd < 0) {
		perror(path);
		return 1;
	}
	dev.platform.address = ADDR << 1;
	dev.platform.Write = bus_write;
	dev.platform.Read = bus_read;
	dev.platform.Wait = bus_wait;

	VL53L8CX_WrByte(&dev.platform, 0x7fff, 0x00);
	VL53L8CX_RdByte(&dev.platform, 0, &id);
	VL53L8CX_RdByte(&dev.platform, 1, &rev);
	VL53L8CX_WrByte(&dev.platform, 0x7fff, 0x02);
	printf("id 0x%02X rev 0x%02X (expected 0xF0 0x0C), I2C errors %u\n", id, rev, i2c_errors);
	if (id != 0xF0 || rev != 0x0C)
		return 2;

	double t0 = now();
	status = vl53l8cx_init(&dev);
	printf("firmware: status %u, %.1f s\n", status, now() - t0);
	if (status)
		return 3;
	status = vl53l8cx_set_resolution(&dev, VL53L8CX_RESOLUTION_8X8);
	status |= vl53l8cx_set_ranging_frequency_hz(&dev, HZ);
	status |= vl53l8cx_start_ranging(&dev);
	if (status) {
		printf("start ranging: status %u\n", status);
		return 4;
	}

	int got = 0, late = 0, valid_sum = 0;
	t0 = now();
	while (got < frames) {
		double t1 = now();
		ready = 0;
		while (!ready && now() - t1 < 1.0) {
			if (vl53l8cx_check_data_ready(&dev, &ready))
				break;
			usleep(2000);
		}
		if (!ready) {
			late++;
			if (late > 5)
				break;
			continue;
		}
		if (vl53l8cx_get_ranging_data(&dev, &res))
			break;
		got++;
		for (int z = 0; z < 64; z++)
			valid_sum += res.target_status[z] == 5 || res.target_status[z] == 9;
		if (stream) {    /* for the duck page: one line per frame, -1 = invalid zone */
			printf("F");
			for (int z = 0; z < 64; z++)
				printf(" %d", res.target_status[z] == 5 || res.target_status[z] == 9 ? res.distance_mm[z] : -1);
			printf("\n");
			fflush(stdout);
			got = 0;
		}
	}
	double took = now() - t0;
	vl53l8cx_stop_ranging(&dev);

	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 8; col++) {
			int z = row * 8 + col;
			int ok = res.target_status[z] == 5 || res.target_status[z] == 9;
			if (ok)
				printf("%6d", res.distance_mm[z]);
			else
				printf("%6s", "-");
		}
		printf("\n");
	}
	int mid[4];
	for (int i = 0; i < 4; i++)
		mid[i] = res.distance_mm[centre[i]];
	qsort(mid, 4, sizeof mid[0], by_value);
	printf("frames %d of %d, %.1f Hz, missed %d, I2C errors %u\n", got, frames,
	       took > 0 ? got / took : 0, late, i2c_errors);
	printf("valid zones on average %.1f of 64, center %d mm\n", got ? (double)valid_sum / got : 0,
	       (mid[1] + mid[2]) / 2);
	return got == frames && !i2c_errors ? 0 : 5;
}
