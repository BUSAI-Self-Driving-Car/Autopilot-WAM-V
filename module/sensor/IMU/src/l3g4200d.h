
// Gyroscope
struct l3g4200d : public I2CDevice<l3g4200d> {

    l3g4200d(uint8_t device_address = 0x68)
    : I2CDevice<l3g4200d>(device_address) {
    }

    I2CVar<0x0F, uint8_t, LE> whoami;

    // Output datarate selection
    // Bandwidth selection
    // Power down mode enable
    // X, Y, Z axis enable
    I2CVar<0x20, uint8_t, LE> ctrl1;

    // High pass filter mode
    // High pass cutoff frequency
    I2CVar<0x21, uint8_t, LE> ctrl2;

    // Interupt enable on int1 pin
    // Boot status on int1
    // Interupt active on int1
    // Push/pull/open drain
    // Date ready
    // FIFO watermark interrupt
    // FIFO overrun interupt
    // Empty interrupt
    I2CVar<0x22, uint8_t, LE> ctrl3;

    // Block data update
    // Bit little endian data selection
    // Full scale selection
    // Self test enable
    // SPI mode selection
    I2CVar<0x23, uint8_t, LE> ctrl4;

    // Reboot memory content
    // FIFO enable
    // High pass filter enable
    // INT1 selection config
    // Out selection config
    I2CVar<0x24, uint8_t, LE> ctrl5;

    // Reference value for interrupt configuration
    I2CVar<0x25, uint8_t, LE> reference;

    // Temperature data
    I2CVar<0x26, uint8_t, LE> temperature;

    // x y z overrun
    I2CVar<0x27, uint8_t, LE> status;

    // X y and z as twos complement
    I2CVar<0x28, int16_t, LE> x;
    I2CVar<0x2A, int16_t, LE> y;
    I2CVar<0x2C, int16_t, LE> z;

    // FIFO mode selection and watermark settings
    I2CVar<0x2E, uint8_t, LE> fifo_ctrl;
    // FIFO watermark status, overrun bit status, FIFO stored data level
    I2CVar<0x2F, uint8_t, LE> fifo_src;

    I2CVar<0x30, uint8_t, LE> int1_cfg;
    I2CVar<0x31, uint8_t, LE> int1_src;

    // Interrupt threshold for x y and z
    I2CVar<0x32, uint8_t, LE> tsh_x;
    I2CVar<0x34, uint8_t, LE> tsh_y;
    I2CVar<0x36, uint8_t, LE> tsh_z;

    // Duration wait for interrupt to be recognized
    I2CVar<0x38, uint8_t, LE> duration;
};
