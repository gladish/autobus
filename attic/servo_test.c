#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define PCA9685_ADDR   0x40

#define MODE1          0x00
#define PRESCALE       0xFE
#define LED0_ON_L      0x06

static int fd;

void write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    write(fd, buf, 2);
}

void set_pwm_freq(float freq_hz)
{
    uint8_t prescale =
        (uint8_t)(25000000.0 / (4096.0 * freq_hz) - 1.0);

    write_reg(MODE1, 0x10);      // sleep
    write_reg(PRESCALE, prescale);
    write_reg(MODE1, 0x20);      // auto-increment
    usleep(5000);
}

void set_pwm(int channel, uint16_t on, uint16_t off)
{
    uint8_t reg = LED0_ON_L + 4 * channel;

    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = on & 0xff;
    buf[2] = on >> 8;
    buf[3] = off & 0xff;
    buf[4] = off >> 8;

    write(fd, buf, 5);
}

void set_servo_us(int channel, int pulse_us)
{
    printf("set_servo_us(channel=%d, pulse_us=%d)\n",
		    channel, pulse_us);

    // 50Hz => 20ms period
    int counts = (pulse_us * 4096) / 20000;

    set_pwm(channel, 0, counts);
}

int main(void)
{
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        printf("open:%s\n", strerror(errno));
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0)
    {
        perror("ioctl");
        close(fd);
        return 1;
    }

    int channel = 8;
    set_pwm_freq(50.0);

    // 1. ESCs need to arm at neutral first
    printf("Arming ESC at neutral...\n");
    set_servo_us(channel, 1500);
    sleep(2);

    // 2. Trigger the Reverse Double-Tap
    printf("Engaging reverse (Tap 1: Brake/Register)...\n");
    set_servo_us(channel, 1300); // Push into reverse range
    usleep(500000);              // Hold for 0.5 seconds

    printf("Returning to neutral...\n");
    set_servo_us(channel, 1500); // Return to neutral
    usleep(500000);              // Hold for 0.5 seconds

    // 3. Spin up in reverse, then slow down
    printf("Spinning up in REVERSE...\n");
    set_servo_us(channel, 1200); // Faster reverse
    sleep(2);

    printf("Slowing down reverse...\n");
    set_servo_us(channel, 1400); // Slow reverse
    sleep(2);

    // 4. Return to neutral before forward (good practice)
    set_servo_us(channel, 1500);
    sleep(1);

    // 5. Spin up forward, then slow down
    printf("Spinning up FORWARD...\n");
    set_servo_us(channel, 1800); // Fast forward
    sleep(2);

    printf("Slowing down forward...\n");
    set_servo_us(channel, 1600); // Slow forward
    sleep(2);

    // 6. Stop
    printf("Stopping motor.\n");
    set_servo_us(channel, 1500);
    sleep(2);

    close(fd);
    return 0;
}
