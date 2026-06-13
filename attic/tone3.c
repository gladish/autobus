#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define PCA9685_ADDR   0x40
#define I2C_BUS        "/dev/i2c-1"
#define OSC_FREQ       25000000.0
#define BUZZER_CHANNEL 15

#define MODE1     0x00
#define PRESCALE  0xFE
#define LED0_ON_L 0x06

int fd;

void pca_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    write(fd, buf, 2);
}

uint8_t pca_read(uint8_t reg) {
    write(fd, &reg, 1);
    uint8_t val;
    read(fd, &val, 1);
    return val;
}

void pca_set_pwm_freq(double freq_hz) {
    if (freq_hz < 40) freq_hz = 40;
    if (freq_hz > 1500) freq_hz = 1500;

    uint8_t prescale = (uint8_t)(round(OSC_FREQ / (4096.0 * freq_hz)) - 1);
    uint8_t old_mode = pca_read(MODE1);

    pca_write(MODE1, (old_mode & 0x7F) | 0x10);
    pca_write(PRESCALE, prescale);
    pca_write(MODE1, old_mode);

    usleep(500);
    pca_write(MODE1, old_mode | 0xA0);
}

void pca_set_channel(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t base = LED0_ON_L + 4 * channel;
    pca_write(base + 0, on & 0xFF);
    pca_write(base + 1, on >> 8);
    pca_write(base + 2, off & 0xFF);
    pca_write(base + 3, off >> 8);
}

void buzz(double freq, double dur) {
    if (freq <= 0) {
        usleep((useconds_t)(dur * 1e6));
        return;
    }

    pca_set_pwm_freq(freq);

    int steps = 20;
    for (int i = 0; i < steps; i++) {
        uint16_t duty = (uint16_t)(2048.0 * (i / (double)steps));
        pca_set_channel(BUZZER_CHANNEL, 0, duty);
        usleep((dur * 1e6) / (steps * 2));
    }

    for (int i = steps; i >= 0; i--) {
        uint16_t duty = (uint16_t)(2048.0 * (i / (double)steps));
        pca_set_channel(BUZZER_CHANNEL, 0, duty);
        usleep((dur * 1e6) / (steps * 2));
    }

    pca_set_channel(BUZZER_CHANNEL, 0, 0);
}

typedef struct {
    double f;
    double d;
} Tone;

void play(Tone *t, int n) {
    for (int i = 0; i < n; i++) {
        buzz(t[i].f, t[i].d);
        usleep(40000);
    }
}

/* ===== JINGLES ===== */

Tone startup[] = {
    {261.63, 0.10},
    {329.63, 0.10},
    {392.00, 0.10},
    {523.25, 0.20},
    {0,      0.08},
    {659.25, 0.08},
    {783.99, 0.20}
};

Tone success[] = {
    {523.25, 0.10},
    {659.25, 0.10},
    {783.99, 0.25},
    {0,      0.08},
    {783.99, 0.15}
};

Tone error[] = {
    {392.00, 0.20},
    {349.23, 0.20},
    {261.63, 0.45}
};

Tone game[] = {
    {440.00, 0.08},
    {0,      0.04},
    {440.00, 0.08},
    {523.25, 0.08},
    {659.25, 0.15},
    {523.25, 0.08},
    {659.25, 0.08},
    {880.00, 0.25}
};

int main() {
    fd = open(I2C_BUS, O_RDWR);
    if (fd < 0) {
        perror("i2c open");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("i2c ioctl");
        return 1;
    }

    pca_write(MODE1, 0x00);
    usleep(1000);

    printf("Startup\n");
    play(startup, sizeof(startup)/sizeof(startup[0]));
    sleep(2);

    printf("Success\n");
    play(success, sizeof(success)/sizeof(success[0]));
    sleep(2);

    printf("Error\n");
    play(error, sizeof(error)/sizeof(error[0]));
    sleep(2);

    printf("Game intro\n");
    play(game, sizeof(game)/sizeof(game[0]));
    sleep(2);

    pca_set_channel(BUZZER_CHANNEL, 0, 0);
    close(fd);

    return 0;
}
