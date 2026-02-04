#include <stdint.h>
#include <string.h>
#include "xio_switch.h"
#include "xparameters.h" 
#include "xiic_l.h"
#include "xiic.h"

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
 *      MAILBOX_DATA(4): PID loop execution time in ms (output)
 *      MAILBOX_DATA(5..7): (output) Accel readings (Q16) for ax, ay, az
 *      MAILBOX_DATA(8..10): (output) Gyro readings (Q16) for gx, gy, gz
 *      The PID loop continues until MAILBOX_CMD_ADDR is changed to a different command.
 * CMD_STOP_PID: Stop PID control loop and stop motors. No MAILBOX_DATA used.
 *
 */

/* Constants from constant.py */
#define TCA_DEVICE_ADDRESS 0x70
#define TCA_MPU_CHANNEL 0

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
#define PCA9685_AI 0x20 // Auto-Increment Bit (Bit 5 of MODE1)

#define PMOD_TCA_SDA_PIN 3
#define PMOD_TCA_SCL_PIN 2
#define PMOD_MOTOR_SDA_PIN 1
#define PMOD_MOTOR_SCL_PIN 0

#define MIN_MOTOR_US 1000
#define MAX_MOTOR_US 2000
#define MAX_ALLOWED_MOTOR_PERCENTAGE 0.5
#define MIN_ALLOWED_MOTOR_PERCENTAGE 0.05
#define MOTOR_UPDATE_DEADBAND_Q16 (Q16_ONE / 200) // ≈ 0.5%

#define XPAR_IO_SWITCH_0_I2C0_BASEADDR 0x40800000 // From: /home/xilinx/pynq/lib/rpi/bsp_iop_rpi/iop_rpi_mb/include/xparameters.h
#define XPAR_IO_SWITCH_0_I2C1_BASEADDR 0x40810000

#define IIC_TIMEOUT_MAX  100000
#define MOTOR_IIC_BASE   XPAR_IO_SWITCH_0_I2C0_BASEADDR
#define TCA_IIC_BASE     XPAR_IO_SWITCH_0_I2C1_BASEADDR


/* Q16.16 helpers */
typedef int32_t q16;
#define Q16_ONE (1 << 16)
static inline q16 float_to_q16(float f) { return (q16)(f * (float)Q16_ONE); }
static inline float q16_to_float(q16 q) { return ((float)q) / (float)Q16_ONE; }
static inline q16 q16_mul(q16 a, q16 b) { return (q16)(((int64_t)a * (int64_t)b) >> 16); }
static inline q16 q16_div(q16 a, q16 b) { return (q16)((((int64_t)a << 16) / (int64_t)b)); }
static inline q16 q16_abs(q16 x) { return (x < 0) ? -x : x; }


typedef struct
{
    q16 kp, ki, kd;
    q16 integ;
    q16 last_err;
    q16 out_min, out_max;
} pid_type;

pid_type pid_roll, pid_pitch, pid_yaw;

q16 gyro_bias_x = 0;
q16 gyro_bias_y = 0;
q16 gyro_bias_z = 0;

static q16 last_motor_q16[4] = {0, 0, 0, 0};

void timer_init_ms();
uint32_t get_timer_ms(void);
void mb_delay_ms(uint32_t ms);
void setup_hardware(void);
int i2c_tca_write_reg(uint8_t dev, uint8_t reg, uint8_t val);
int i2c_tca_read_reg(uint8_t dev, uint8_t reg, uint8_t* val, int len);
int i2c_motor_write_reg(uint8_t reg, uint8_t* data, int len);
int i2c_motor_read_reg( uint8_t reg, uint8_t *buf, int len);

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
void mpu_read_gyro(q16 *gx, q16 *gy, q16 *gz);

void mb_write_motor_pwm_from_q16(int motor, q16 norm_q16, int min_us, int max_us);
q16 pid_step(pid_type *p, q16 setpoint, q16 measurement, q16 dt_q16);


/* =========================================================================
   2. NON-BLOCKING LOW-LEVEL DRIVERS (Integrated directly)
   ========================================================================= */

/* Helper: RecvData (Internal) */
static unsigned MyRecvData(UINTPTR BaseAddress, u8 *BufferPtr, unsigned ByteCount, u8 Option) {
    u32 CntlReg;
    u32 IntrStatusMask;
    volatile u32 IntrStatus;
    volatile int Timeout;

    while (ByteCount > 0) {
        if (ByteCount == 1) IntrStatusMask = XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_BNB_MASK;
        else IntrStatusMask = XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_BNB_MASK;

        /* Wait for FIFO with Timeout */
        Timeout = IIC_TIMEOUT_MAX;
        while (1) {
            IntrStatus = XIic_ReadIisr(BaseAddress);
            if (IntrStatus & XIIC_INTR_RX_FULL_MASK) break;
            if (IntrStatus & IntrStatusMask) return ByteCount;
            if (Timeout-- == 0) return ByteCount; // TIMEOUT
        }

        CntlReg = XIic_ReadReg(BaseAddress, XIIC_CR_REG_OFFSET);
        if (ByteCount == 1 && Option == XIIC_STOP) {
            XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK);
        }
        if (ByteCount == 2) {
            XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET, CntlReg | XIIC_CR_NO_ACK_MASK);
        }

        *BufferPtr++ = (u8) XIic_ReadReg(BaseAddress, XIIC_DRR_REG_OFFSET);

        if ((ByteCount == 1) && (Option == XIIC_REPEATED_START)) {
            XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET,
                 XIIC_CR_ENABLE_DEVICE_MASK | XIIC_CR_MSMS_MASK | XIIC_CR_REPEATED_START_MASK);
        }

        XIic_ClearIisr(BaseAddress, XIIC_INTR_RX_FULL_MASK | XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK);
        ByteCount--;
    }

    if (Option == XIIC_STOP) {
        Timeout = IIC_TIMEOUT_MAX;
        while (1) {
            if (XIic_ReadIisr(BaseAddress) & XIIC_INTR_BNB_MASK) break;
            if (Timeout-- == 0) break; 
        }
    }
    return ByteCount;
}

/* Helper: SendData (Internal) */
static unsigned MySendData(UINTPTR BaseAddress, u8 *BufferPtr, unsigned ByteCount, u8 Option) {
    volatile u32 IntrStatus;
    volatile int Timeout;

    while (ByteCount > 0) {
        Timeout = IIC_TIMEOUT_MAX;
        while (1) {
            IntrStatus = XIic_ReadIisr(BaseAddress);
            if (IntrStatus & (XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_BNB_MASK)) return ByteCount;
            if (IntrStatus & XIIC_INTR_TX_EMPTY_MASK) break;
            if (Timeout-- == 0) return ByteCount; // TIMEOUT
        }

        if (ByteCount > 1) {
            XIic_WriteReg(BaseAddress, XIIC_DTR_REG_OFFSET, *BufferPtr++);
        } else {
            if (Option == XIIC_STOP) {
                XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK | XIIC_CR_DIR_IS_TX_MASK);
            }
            XIic_WriteReg(BaseAddress, XIIC_DTR_REG_OFFSET, *BufferPtr++);
        }
        XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK);
        ByteCount--;
    }

    if (Option == XIIC_STOP) {
        Timeout = IIC_TIMEOUT_MAX;
        while (1) {
            if (XIic_ReadIisr(BaseAddress) & XIIC_INTR_BNB_MASK) break;
            if (Timeout-- == 0) break;
        }
    }
    return ByteCount;
}

/* Main Receive Function (Non-Blocking) */
int MyXIic_Recv(UINTPTR BaseAddress, u8 Address, u8 *BufferPtr, unsigned ByteCount) {
    u32 CntlReg;
    volatile u32 StatusReg;
    volatile int Timeout = IIC_TIMEOUT_MAX;

    // Reset FIFO
    XIic_WriteReg(BaseAddress, XIIC_RFD_REG_OFFSET, 0);
    
    // Clear Interrupts
    XIic_ClearIisr(BaseAddress, XIIC_INTR_RX_FULL_MASK | XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK);

    // Send Address
    XIic_Send7BitAddress(BaseAddress, Address, XIIC_READ_OPERATION);

    // Start
    CntlReg = XIIC_CR_MSMS_MASK | XIIC_CR_ENABLE_DEVICE_MASK;
    if (ByteCount == 1) CntlReg |= XIIC_CR_NO_ACK_MASK;
    XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET, CntlReg);

    // Wait for Bus Busy
    while ((XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET) & XIIC_SR_BUS_BUSY_MASK) == 0) {
        if (Timeout-- == 0) return 0;
    }
    XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);

    // Read Data
    if (MyRecvData(BaseAddress, BufferPtr, ByteCount, XIIC_STOP) != 0) return 0; // Error if remaining != 0

    // Cleanup
    XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET, 0);
    return ByteCount;
}

/* Main Send Function (Non-Blocking) */
int MyXIic_Send(UINTPTR BaseAddress, u8 Address, u8 *BufferPtr, unsigned ByteCount) {
    volatile u32 StatusReg;
    volatile int Timeout = IIC_TIMEOUT_MAX;

    // Wait Bus Free
    while (XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET) & XIIC_SR_BUS_BUSY_MASK) {
        if (Timeout-- == 0) return 0;
    }

    // Send Address
    XIic_Send7BitAddress(BaseAddress, Address, XIIC_WRITE_OPERATION);
    XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK | XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_ARB_LOST_MASK);

    // Start
    XIic_WriteReg(BaseAddress, XIIC_CR_REG_OFFSET, XIIC_CR_MSMS_MASK | XIIC_CR_DIR_IS_TX_MASK | XIIC_CR_ENABLE_DEVICE_MASK);

    // Wait Bus Busy
    Timeout = IIC_TIMEOUT_MAX;
    while ((XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET) & XIIC_SR_BUS_BUSY_MASK) == 0) {
        if (Timeout-- == 0) return 0;
    }
    XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);

    // Send Data
    if (MySendData(BaseAddress, BufferPtr, ByteCount, XIIC_STOP) != 0) return 0; // Error if remaining != 0

    // Cleanup
    StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
    while ((StatusReg & XIIC_SR_BUS_BUSY_MASK) != 0) {
        StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
        // No timeout here usually needed, but good practice to allow system to proceed
    }
    return ByteCount; 
}





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

void setup_hardware() {
    init_io_switch(); 
    
    set_pin(PMOD_MOTOR_SCL_PIN, SCL0);
    set_pin(PMOD_MOTOR_SDA_PIN, SDA0);

    set_pin(PMOD_TCA_SCL_PIN, SCL1);
    set_pin(PMOD_TCA_SDA_PIN, SDA1);
}

int i2c_tca_write_reg(uint8_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = val;
    return MyXIic_Send(TCA_IIC_BASE, dev, buf, 2); 
}

int i2c_tca_read_reg(uint8_t dev, uint8_t reg, uint8_t* val, int len) {
    // MPU is on I2C 1
    // Write Reg Address
    if(MyXIic_Send(TCA_IIC_BASE, dev, &reg, 1) == 0) return 0; 
    // Read Data
    return MyXIic_Recv(TCA_IIC_BASE, dev, val, len);
}

int i2c_motor_write_reg(uint8_t reg, uint8_t* data, int len) {
    uint8_t buf[32];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return MyXIic_Send(MOTOR_IIC_BASE, PCA9685_ADDRESS, buf, len+1);
}

int i2c_motor_read_reg(uint8_t reg, uint8_t* buffer, int length) {
    // 1. Send the register address we want to read (Write operation)
    // Motors are on MOTOR_IIC_BASE (I2C 0)
    if (MyXIic_Send(MOTOR_IIC_BASE, PCA9685_ADDRESS, &reg, 1) == 0) {
        return 0; // Timeout or NACK
    }
    
    // 2. Read the data from the device (Read operation)
    return MyXIic_Recv(MOTOR_IIC_BASE, PCA9685_ADDRESS, buffer, length);
}

void tca_select_channel(uint8_t channel) {
    if (channel > 7) return;
    uint8_t mask = (1 << channel);
    MyXIic_Send(TCA_IIC_BASE, TCA_DEVICE_ADDRESS, &mask, 1);
}


void pca9685_set_pwm(int motor, uint16_t on, uint16_t off)
{
    int ch;
    switch (motor)
    {
    case 1:
        ch = 3;
        break;
    case 2:
        ch = 4;
        break;
    case 3:
        ch = 7;
        break;
    case 4:
        ch = 8;
        break;
    default:
        return;
    }

    uint8_t ON_L = LED0_ON_L + 4 * ch;

    uint8_t payload[4];
    payload[0] = on & 0xFF;
    payload[1] = (on >> 8) & 0x0F;
    payload[2] = off & 0xFF;
    payload[3] = (off >> 8) & 0x0F;

    i2c_motor_write_reg( ON_L, payload, 4);
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
    i2c_motor_write_reg(reg, &value, 1);
}

uint8_t pca_read8(uint8_t reg)
{
    uint8_t v = 0;
    i2c_motor_read_reg( reg, &v, 1);
    return v;
}
void pca9685_init(void)
{
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
    i2c_tca_write_reg(MPU_DEVICE_ADDRESS, MPU_SMPLRT_DIV, tmp);
    mb_delay_ms(100);
    tmp = 1;
    i2c_tca_write_reg(MPU_DEVICE_ADDRESS, MPU_PWR_MGMT_1, tmp);
    mb_delay_ms(100);
    tmp = 4;
    i2c_tca_write_reg(MPU_DEVICE_ADDRESS, MPU_CONFIG, tmp);
    mb_delay_ms(100);
    tmp = 24;
    i2c_tca_write_reg(MPU_DEVICE_ADDRESS, MPU_GYRO_CONFIG,tmp);
    mb_delay_ms(100);
    tmp = 1;
    i2c_tca_write_reg(MPU_DEVICE_ADDRESS, MPU_INT_ENABLE, tmp);
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

void mpu_read_gyro(q16 *gx, q16 *gy, q16 *gz)
{
    uint8_t buf[6];

    if(i2c_tca_read_reg(MPU_DEVICE_ADDRESS, MPU_GYRO_XOUT_H, buf, 6) != 6) {
        // READ FAILED (TIMEOUT) - Keep old values or zero
        mb_delay_ms(5);
        return; 
    }

    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);

    const int32_t GYRO_SCALE_DIV = 131;

    *gx = float_to_q16((float)rx / GYRO_SCALE_DIV) - gyro_bias_x;
    *gy = float_to_q16(-(float)ry / GYRO_SCALE_DIV) - gyro_bias_y; // Invert Y axis
    *gz = float_to_q16(-(float)rz / GYRO_SCALE_DIV) - gyro_bias_z; // Invert Z axis
}

void mpu_read_accel(q16 *ax, q16 *ay, q16 *az)
{
    uint8_t buf[6];
    i2c_tca_read_reg(MPU_DEVICE_ADDRESS, MPU_ACCEL_XOUT_H, buf, 6);

    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);

    const int32_t ACCEL_SCALE_DIV = 16384;

    *ax = float_to_q16((float)rx / ACCEL_SCALE_DIV);
    *ay = float_to_q16((float)ry / ACCEL_SCALE_DIV);
    *az = float_to_q16((float)rz / ACCEL_SCALE_DIV);
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
            setup_hardware();
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

                q16 gx, gy, gz, ax, ay, az;
                mpu_read_gyro(&gx, &gy, &gz);
                mpu_read_accel(&ax, &ay, &az);

                q16 out_roll = pid_step(&pid_roll, set_roll, gy, dt_q16);
                q16 out_pitch = pid_step(&pid_pitch, set_pitch, gx, dt_q16);
                q16 out_yaw = pid_step(&pid_yaw, set_yaw, gz, dt_q16);

                q16 t1 = adj_throttle_q16 + out_pitch - out_roll + out_yaw;
                q16 t2 = adj_throttle_q16 - out_pitch - out_roll - out_yaw;
                q16 t3 = adj_throttle_q16 + out_pitch + out_roll - out_yaw;
                q16 t4 = adj_throttle_q16 - out_pitch + out_roll + out_yaw;

                mb_write_motor_pwm_from_q16(1, t1, MIN_MOTOR_US, MAX_MOTOR_US);
                mb_write_motor_pwm_from_q16(2, t2, MIN_MOTOR_US, MAX_MOTOR_US);
                mb_write_motor_pwm_from_q16(3, t3, MIN_MOTOR_US, MAX_MOTOR_US);
                mb_write_motor_pwm_from_q16(4, t4, MIN_MOTOR_US, MAX_MOTOR_US);

                MAILBOX_DATA(5) = ax;
                MAILBOX_DATA(6) = ay;
                MAILBOX_DATA(7) = az;
                MAILBOX_DATA(8) = gx;
                MAILBOX_DATA(9) = gy;
                MAILBOX_DATA(10) = gz;

                uint32_t elapsed_ms = get_timer_ms() - start_ms;
                MAILBOX_DATA(4) = elapsed_ms;
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

