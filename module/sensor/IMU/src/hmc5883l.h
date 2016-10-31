
// Magnetometer
struct hmc5883l : public I2CDevice<hmc5883l> {

    hmc5883l(uint8_t device_address = 0x1E)
    : I2CDevice<hmc5883l>(device_address) {
    }

    enum class Gain : uint8_t {
        FULL_SCALE_880_MILLIGAUSS   = 0x00,
        FULL_SCALE_1300_MILLIGAUSS  = 0x01,
        FULL_SCALE_1900_MILLIGAUSS  = 0x02,
        FULL_SCALE_2500_MILLIGAUSS  = 0x03,
        FULL_SCALE_4000_MILLIGAUSS  = 0x04,
        FULL_SCALE_4700_MILLIGAUSS  = 0x05,
        FULL_SCALE_5600_MILLIGAUSS  = 0x06,
        FULL_SCALE_8100_MILLIGAUSS  = 0x07
    };

    enum class Mode : uint8_t {
        IDLE = 0x03,
        CONTINUOUS = 0x00;
        SINGLE_MEASUREMENT = 0x01;
    };

    // Data output rate and measurement configuration
    I2CVar<0x00, uint8_t, BE> config_a;

    // Setting for device gain
    I2CVar<0x01, uint8_t, BE> config_b;

    // Setting for device mode
    I2CVar<0x02, Mode, BE> mode;

    I2CVar<0x03, int16_t, BE> x;
    I2CVar<0x05, int16_t, BE> y;
    I2CVar<0x07, int16_t, BE> z;

    I2CVar<0x09, uint8_t, BE> status;
    I2CVar<0x0A, uint8_t, BE> id_a;
    I2CVar<0x0B, uint8_t, BE> id_b;
    I2CVar<0x0C, uint8_t, BE> id_c;
};
