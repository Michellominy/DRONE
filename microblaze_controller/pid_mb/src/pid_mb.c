#include <stdint.h>
#include <string.h>
#include "xio_switch.h"

#include <i2c.h>
#include "xparameters.h"

/* Motor layout and rotation directions:
M1 => Rotate CW
M2 => Rotate CCW
M3 => Rotate CCW
M4 => Rotate CW

if Positive Pitch ==> To stabilize, motor 1,3 need to increase
if Positive Yaw ==> To stabilize, motor 2,3 need to increase
if Positive Roll ==> To stabilize, motor 3,4 need to increase


FRONT
       3        1
     +---+    +---+
     | ↺ |    | ↻ |
     +---+    +---+
        \      /
         \    /
          \  /
           ||
       +-------+
       | DRONE |
       | BODY  |
       | FROM  |
       | ABOVE |
       +-------+
           ||
          /  \
         /    \
        /      \
     +---+    +---+
     | ↻ |    | ↺ |
     +---+    +---+
       4        2
BACK

        ^(y)
        |
        |
        |
        |
        |
        * ----------> (x)
        (z)

 */
#define MB_CLOCK_FREQ_HZ 100000000

/* --- AXI Timer Register Definitions (for Timer 1) --- */
#define TMR_BASEADDR XPAR_TMRCTR_0_BASEADDR
#define TMR_TCSR1 (*(volatile uint32_t *)(TMR_BASEADDR + 0x10))
#define TMR_TLR1 (*(volatile uint32_t *)(TMR_BASEADDR + 0x14))
#define TMR_TCR1 (*(volatile uint32_t *)(TMR_BASEADDR + 0x18))

/* --- AXI Timer Control/Status Register (TCSR) Bits --- */
#define TMR_CSR_LOAD_MASK 0x00000020 // Load Timer (set bit 5)
#define TMR_CSR_ARHT_MASK 0x00000010 // Auto-Reload (set bit 4)
#define TMR_CSR_ENT_MASK 0x00000080  // Enable Timer (set bit 7)
#define TMR_CSR_GENT_MASK 0x00000004 // Generate Event (set bit 2)
#define TMR_CSR_UDT_MASK 0x00000002  // Up/Down Timer (set bit 1)
#define MAILBOX_BASE 0xF000
#define MAILBOX_CMD_ADDR (*(volatile unsigned int *)(0xFFFC))
#define MAILBOX_DATA(i) (*(volatile unsigned int *)(MAILBOX_BASE + ((i) * 4)))
#define CMD_NOP 0x0
#define CMD_INIT 0x1
#define CMD_SET_GAINS 0x2
#define CMD_START_PID 0x3
#define CMD_STOP_PID 0x4

#define MAILBOX_BASE 0xF000
#define MAILBOX_CMD_ADDR (*(volatile unsigned int *)(0xFFFC))
#define MAILBOX_DATA(i) (*(volatile unsigned int *)(MAILBOX_BASE + ((i) * 4)))

#define CMD_NOP 0x0
#define CMD_INIT 0x1
#define CMD_SET_GAINS 0x2
#define CMD_START_PID 0x3
#define CMD_STOP_PID 0x4
/**
 * CMD_NOP: No operation
 * CMD_INIT: Initialize I2C, motors, MPU6050, and PID controllers. No MAILBOX_DATA used.
 * CMD_SET_GAINS: Set PID gains. Uses MAILBOX_DATA(0..8):
 *      MAILBOX_DATA(0..2): Roll Kp, Ki, Kd
 *      MAILBOX_DATA(3..5): Pitch Kp, Ki, Kd
 *      MAILBOX_DATA(6..8): Yaw Kp, Ki, Kd
 * CMD_START_PID: Start PID control loop. Uses MAILBOX_DATA(0..3):
 *      MAILBOX_DATA(0): Setpoint roll (Q16)
 *      MAILBOX_DATA(1): Setpoint pitch (Q16)
 *      MAILBOX_DATA(2): Setpoint yaw (Q16)
 *      MAILBOX_DATA(3): Adjusted throttle (Q16, 0.0 to 1.0)
 *      MAILBOX_DATA(4..7): (output) Motor outputs (Q16) for motors 1 to 4
 *      MAILBOX_DATA(8..10): (output) Gyro readings (Q16) for gx, gy, gz
 *      The PID loop continues until MAILBOX_CMD_ADDR is changed to a different command.
 * CMD_STOP_PID: Stop PID control loop and stop motors. No MAILBOX_DATA used.
 *
 */

/* Constants from constant.py */
#define TCA_DEVICE_ADDRESS 0x70
#define TCA_MOTOR_CHANNEL 0
#define TCA_MPU_CHANNEL 1

#define MPU_DEVICE_ADDRESS 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_INT_ENABLE 0x38
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_YOUT_H 0x3D
#define MPU_ACCEL_ZOUT_H 0x3F
#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_YOUT_H 0x45
#define MPU_GYRO_ZOUT_H 0x47

#define PCA9685_ADDRESS 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define PCA9685_SLEEP 0x10
#define PCA9685_ALLCALL 0x01
#define PCA9685_OUTDRV 0x04
#define PCA9685_AI 0x20  // Auto-Increment Bit (Bit 5 of MODE1)

#define PMOD_SDA_PIN 6
#define PMOD_SCL_PIN 7

#define MIN_MOTOR_US 1000
#define MAX_MOTOR_US 2000
#define MAX_ALLOWED_MOTOR_PERCENTAGE 0.4
#define MIN_ALLOWED_MOTOR_PERCENTAGE 0.05

/* Q16.16 helpers */
typedef int32_t q16;
#define Q16_ONE (1 << 16)
static inline q16 float_to_q16(float f) { return (q16)(f * (float)Q16_ONE); }
static inline float q16_to_float(q16 q) { return ((float)q) / (float)Q16_ONE; }
static inline q16 q16_mul(q16 a, q16 b) { return (q16)(((int64_t)a * (int64_t)b) >> 16); }
static inline q16 q16_div(q16 a, q16 b) { return (q16)((((int64_t)a << 16) / (int64_t)b)); }

typedef struct
{
    q16 kp, ki, kd;
    q16 integ;
    q16 last_err;
    q16 out_min, out_max;
} pid_type;

i2c i2c_dev;

pid_type pid_roll, pid_pitch, pid_yaw;

q16 gyro_bias_x = 0;
q16 gyro_bias_y = 0;
q16 gyro_bias_z = 0;

void timer_init_ms();
uint32_t get_timer_ms(void);
void mb_delay_ms(uint32_t ms);
void mb_i2c_open(void);
int i2c_write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, int len);
int i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buf, int len);

void tca_select_channel(uint8_t channel);

void pca_write8(uint8_t reg, uint8_t value);
uint8_t pca_read8(uint8_t reg);
void pca9685_init(void);
void pca9685_set_pwm(int motor, uint16_t on, uint16_t off);
uint16_t us_to_pca_value(int us);
void pca9685_set_pwm_us(int motor, int us);

void arm_motor(int ch);
void arm_all();
void motors_init();

void mpu_init(void);
int16_t mpu_read_raw16(uint8_t reg);
void mpu_read_gyro(q16 *gx, q16 *gy, q16 *gz);

void mb_write_motor_pwm_from_q16(int motor, q16 norm_q16, int min_us, int max_us);
q16 pid_step(pid_type *p, q16 setpoint, q16 measurement, q16 dt_q16);

void mb_delay_ms(uint32_t ms)
{
    uint32_t start = get_timer_ms();
    while ((get_timer_ms() - start) < ms)
        ;
}

void timer_init_ms()
{
    TMR_TLR1 = 0;
    TMR_TCSR1 = TMR_CSR_LOAD_MASK;
    TMR_TCSR1 = TMR_CSR_ENT_MASK | TMR_CSR_ARHT_MASK | TMR_CSR_GENT_MASK;
}

uint32_t get_timer_ms(void)
{
    uint32_t current_count = TMR_TCR1;
    uint32_t elapsed_ms = (uint32_t)((uint64_t)current_count * 1000 / MB_CLOCK_FREQ_HZ);
    return elapsed_ms;
}

void mb_i2c_open(void)
{
    init_io_switch();
    set_pin(PMOD_SCL_PIN, SCL0);
    set_pin(PMOD_SDA_PIN, SDA0);
    mb_delay_ms(10);
    i2c_dev = i2c_open(PMOD_SDA_PIN, PMOD_SCL_PIN);
    uint8_t mask = (1 << TCA_MPU_CHANNEL);
    i2c_write(i2c_dev, TCA_DEVICE_ADDRESS, &mask, 1);
}

int i2c_write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, int len)
{
    unsigned char buf[32];
    if (len + 1 > (int)sizeof(buf))
        return -1;
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_write(i2c_dev, dev_addr, buf, len + 1);
}

int i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buf, int len)
{
    uint8_t tx = reg;
    i2c_write(i2c_dev, dev_addr, (unsigned char *)&tx, 1);
    return i2c_read(i2c_dev, dev_addr, (unsigned char *)buf, len);
}

void tca_select_channel(uint8_t channel)
{
    if (channel > 7)
        return;
    uint8_t mask = (uint8_t)(1 << channel);
    uint8_t reg = 0x00;
    i2c_write_reg(TCA_DEVICE_ADDRESS, reg, &mask, 1);
}

void pca9685_set_pwm(int motor, uint16_t on, uint16_t off)
{
    int ch;
    switch (motor)
    {
    case 1: ch = 3; break;
    case 2: ch = 4; break;
    case 3: ch = 7; break;
    case 4: ch = 8; break;
    default: return;
    }

    uint8_t ON_L = LED0_ON_L + 4 * ch;
    
    // Create a buffer with all 4 bytes
    uint8_t payload[4];
    payload[0] = on & 0xFF;
    payload[1] = (on >> 8) & 0x0F;
    payload[2] = off & 0xFF;
    payload[3] = (off >> 8) & 0x0F;

    // Write all 4 bytes in ONE transaction
    // i2c_write_reg automatically sends the Register address (ON_L)
    // followed by the 4 bytes of data. The PCA9685 auto-increments the 
    // internal pointer to ON_H, OFF_L, OFF_H automatically.
    i2c_write_reg(PCA9685_ADDRESS, ON_L, payload, 4);
}
uint16_t us_to_pca_value(int us)
{
    uint32_t v = ((uint32_t)us * 4096U) / 20000U;
    if (v > 4095U)
        v = 4095U;
    return (uint16_t)v;
}

void pca9685_set_pwm_us(int motor, int us)
{
    uint16_t val = us_to_pca_value(us);
    pca9685_set_pwm(motor, 0, val);
}

void pca_write8(uint8_t reg, uint8_t value)
{
    i2c_write_reg(PCA9685_ADDRESS, reg, &value, 1);
}

uint8_t pca_read8(uint8_t reg)
{
    uint8_t v = 0;
    i2c_read_reg(PCA9685_ADDRESS, reg, &v, 1);
    return v;
}
void pca9685_init(void)
{
    tca_select_channel(TCA_MOTOR_CHANNEL);
    
    pca_write8(PCA9685_MODE1, PCA9685_OUTDRV); 
    pca_write8(PCA9685_MODE1, PCA9685_ALLCALL);
    mb_delay_ms(5);

    uint8_t mode1 = pca_read8(PCA9685_MODE1);
    mode1 &= ~PCA9685_SLEEP; // Wake up
    pca_write8(PCA9685_MODE1, mode1);
    mb_delay_ms(5);

    // Set Prescale
    float freq = 1000.0f;
    float prescaleval = 25000000.0f / (4096.0f * freq) - 1.0f;
    int prescale = (int)(prescaleval + 0.5f);

    uint8_t oldmode = pca_read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // Sleep to set prescale
    pca_write8(PCA9685_MODE1, newmode);
    pca_write8(PCA9685_PRESCALE, (uint8_t)prescale);
    
    pca_write8(PCA9685_MODE1, oldmode);
    mb_delay_ms(5);

    // IMPORTANT: Enable Auto-Increment (PCA9685_AI) here
    // This sets bit 5 HIGH, allowing us to write sequential registers in one go
    pca_write8(PCA9685_MODE1, oldmode | 0x80 | PCA9685_AI); 
}

void arm_motor(int ch)
{
    pca9685_set_pwm_us(ch, MIN_MOTOR_US);
    mb_delay_ms(1000);
    pca9685_set_pwm_us(ch, MAX_MOTOR_US);
    mb_delay_ms(1000);
    pca9685_set_pwm_us(ch, MIN_MOTOR_US);
    mb_delay_ms(1000);
}

void init_pid()
{
    pid_roll.kp = float_to_q16(1.0f);
    pid_roll.ki = float_to_q16(0.0f);
    pid_roll.kd = float_to_q16(0.0f);
    pid_roll.integ = pid_roll.last_err = 0;
    pid_roll.out_min = float_to_q16(-1.0f);
    pid_roll.out_max = float_to_q16(1.0f);
    pid_pitch = pid_roll;
    pid_yaw = pid_roll;
}

void arm_all()
{
    for (int i = 1; i <= 4; i++)
    {
        arm_motor(i);
    }
    mb_delay_ms(1000);
}

void motors_init()
{
    pca9685_init();
    arm_all();
}

void mpu_init(void)
{
    tca_select_channel(TCA_MPU_CHANNEL);
    uint8_t tmp;
    tmp = 7;
    i2c_write_reg(MPU_DEVICE_ADDRESS, MPU_SMPLRT_DIV, &tmp, 1);
    mb_delay_ms(100);
    tmp = 1;
    i2c_write_reg(MPU_DEVICE_ADDRESS, MPU_PWR_MGMT_1, &tmp, 1);
    mb_delay_ms(100);
    tmp = 0;
    i2c_write_reg(MPU_DEVICE_ADDRESS, MPU_CONFIG, &tmp, 1);
    mb_delay_ms(100);
    tmp = 24;
    i2c_write_reg(MPU_DEVICE_ADDRESS, MPU_GYRO_CONFIG, &tmp, 1);
    mb_delay_ms(100);
    tmp = 1;
    i2c_write_reg(MPU_DEVICE_ADDRESS, MPU_INT_ENABLE, &tmp, 1);
    mb_delay_ms(100);
}

void set_gyro_bias(void)
{
    uint32_t started_at_ms = get_timer_ms();
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int32_t count = 0;
    while ((get_timer_ms() - started_at_ms) < 2000)
    {
        q16 gx, gy, gz;
        mpu_read_gyro(&gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        count++;
        mb_delay_ms(5);
    }
    gyro_bias_x = sum_x / count;
    gyro_bias_y = sum_y / count;
    gyro_bias_z = sum_z / count;
}

int16_t mpu_read_raw16(uint8_t reg)
{
    uint8_t buf[2];
    i2c_read_reg(MPU_DEVICE_ADDRESS, reg, buf, 2);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

void mpu_read_gyro(q16 *gx, q16 *gy, q16 *gz)
{
    tca_select_channel(TCA_MPU_CHANNEL);
    int16_t rx = mpu_read_raw16(MPU_GYRO_XOUT_H);
    int16_t ry = mpu_read_raw16(MPU_GYRO_YOUT_H);
    int16_t rz = mpu_read_raw16(MPU_GYRO_ZOUT_H);
    const int32_t GYRO_SCALE_DIV = 131;
    float fx = ((float)rx) / GYRO_SCALE_DIV;
    float fy = ((float)ry) / GYRO_SCALE_DIV;
    float fz = ((float)rz) / GYRO_SCALE_DIV;
    fy = fy * -1; // Invert Y axis
    fz = fz * -1; // Invert Z axis
    *gx = float_to_q16(fx);
    *gy = float_to_q16(fy);
    *gz = float_to_q16(fz);
    *gx -= gyro_bias_x;
    *gy -= gyro_bias_y;
    *gz -= gyro_bias_z;
}

void mb_write_motor_pwm_from_q16(int motor, q16 norm_q16, int min_us, int max_us)
{
    float f = q16_to_float(norm_q16);
    if (f < MIN_ALLOWED_MOTOR_PERCENTAGE)
        f = MIN_ALLOWED_MOTOR_PERCENTAGE;
    if (f > MAX_ALLOWED_MOTOR_PERCENTAGE)
        f = MAX_ALLOWED_MOTOR_PERCENTAGE;
    int us = (int)(min_us + f * (max_us - min_us));
    pca9685_set_pwm_us(motor, us);
}

q16 pid_step(pid_type *p, q16 setpoint, q16 measurement, q16 dt_q16)
{
    q16 err = setpoint - measurement;
    q16 P = q16_mul(p->kp, err);
    q16 ki_err = q16_mul(p->ki, err);
    q16 delta_i = q16_mul(ki_err, dt_q16);
    p->integ += delta_i;
    q16 diff = err - p->last_err;
    q16 kd_diff = q16_mul(p->kd, diff);
    q16 D = 0;
    if (dt_q16 != 0)
        D = q16_div(kd_diff, dt_q16);
    p->last_err = err;
    q16 out = P + p->integ + D;

    if (out > p->out_max)
    {
        out = p->out_max;
        p->integ -= delta_i;
    }
    else if (out < p->out_min)
    {
        out = p->out_min;
        p->integ -= delta_i;
    }

    return out;
}

int main(void)
{
    volatile uint32_t cmd = 0;
    timer_init_ms();
    while (1)
    {
        cmd = MAILBOX_CMD_ADDR;

        switch (cmd)
        {
        case CMD_INIT:
            mb_i2c_open();
            motors_init();
            mpu_init();
            set_gyro_bias();
            init_pid();
            MAILBOX_CMD_ADDR = CMD_NOP;
            break;

        case CMD_SET_GAINS:
            pid_roll.kp = (q16)MAILBOX_DATA(0);
            pid_roll.ki = (q16)MAILBOX_DATA(1);
            pid_roll.kd = (q16)MAILBOX_DATA(2);
            pid_pitch.kp = (q16)MAILBOX_DATA(3);
            pid_pitch.ki = (q16)MAILBOX_DATA(4);
            pid_pitch.kd = (q16)MAILBOX_DATA(5);
            pid_yaw.kp = (q16)MAILBOX_DATA(6);
            pid_yaw.ki = (q16)MAILBOX_DATA(7);
            pid_yaw.kd = (q16)MAILBOX_DATA(8);
            MAILBOX_CMD_ADDR = CMD_NOP;
            break;

        case CMD_START_PID:
        {
            q16 set_roll = 0, set_pitch = 0, set_yaw = 0;
            uint32_t period_ms = 10;
            q16 adj_throttle_q16 = 0;
            const q16 dt_q16 = (period_ms * Q16_ONE) / 1000;

            while (MAILBOX_CMD_ADDR == CMD_START_PID)
            {
                uint32_t start_ms = get_timer_ms();

                set_roll = (q16)MAILBOX_DATA(0);
                set_pitch = (q16)MAILBOX_DATA(1);
                set_yaw = (q16)MAILBOX_DATA(2);
                adj_throttle_q16 = (q16)MAILBOX_DATA(3);

                q16 gx, gy, gz;
                mpu_read_gyro(&gx, &gy, &gz);

                q16 out_roll = pid_step(&pid_roll, set_roll, gy, dt_q16);
                q16 out_pitch = pid_step(&pid_pitch, set_pitch, gx, dt_q16);
                q16 out_yaw = pid_step(&pid_yaw, set_yaw, gz, dt_q16);

                q16 t1 = adj_throttle_q16 + out_pitch - out_roll - out_yaw;
                q16 t2 = adj_throttle_q16 - out_pitch - out_roll + out_yaw;
                q16 t3 = adj_throttle_q16 + out_pitch + out_roll + out_yaw;
                q16 t4 = adj_throttle_q16 - out_pitch + out_roll - out_yaw;

                tca_select_channel(TCA_MOTOR_CHANNEL);
                mb_write_motor_pwm_from_q16(1, t1, MIN_MOTOR_US, MAX_MOTOR_US);
// 		mb_delay_ms(5); // Small delay to ensure I2C bus stability
                mb_write_motor_pwm_from_q16(2, t2, MIN_MOTOR_US, MAX_MOTOR_US);
// 		mb_delay_ms(5); // Small delay to ensure I2C bus stability
                mb_write_motor_pwm_from_q16(3, t3, MIN_MOTOR_US, MAX_MOTOR_US);
// 		mb_delay_ms(5); // Small delay to ensure I2C bus stability
                mb_write_motor_pwm_from_q16(4, t4, MIN_MOTOR_US, MAX_MOTOR_US);

                MAILBOX_DATA(4) = t1;
                MAILBOX_DATA(5) = t2;
                MAILBOX_DATA(6) = t3;
                MAILBOX_DATA(7) = t4;
                MAILBOX_DATA(8) = gx;
                MAILBOX_DATA(9) = gy;
                MAILBOX_DATA(10) = gz;

                uint32_t elapsed_ms = get_timer_ms() - start_ms;
		MAILBOX_DATA(11) = elapsed_ms;
                if (elapsed_ms < period_ms)
                    mb_delay_ms(period_ms - elapsed_ms);
            }
            MAILBOX_CMD_ADDR = CMD_STOP_PID;
            break;
        }
        case CMD_STOP_PID:
            for (int i = 1; i <= 4; i++)
            {
                pca9685_set_pwm_us(i, 0);
            }
            MAILBOX_CMD_ADDR = CMD_NOP;
            break;
        default:
            /* NOP, do nothing */
            break;
        }
    }
    return 0;
}

