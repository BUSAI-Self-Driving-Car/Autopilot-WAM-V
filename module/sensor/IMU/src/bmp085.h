struct bmp085 : public I2CDevice<bmp085> {

    bmp085(uint8_t device_address = 0x77)
    : I2CDevice<bmp085>(device_address) {
    }

    uint8_t device_address;

    I2CVar<0xAA, uint16_t, BE> ac1;
    I2CVar<0xAC, uint16_t, BE> ac2;
    I2CVar<0xAE, uint16_t, BE> ac3;
    I2CVar<0xB0, uint16_t, BE> ac4;
    I2CVar<0xB2, uint16_t, BE> ac5;
    I2CVar<0xB4, uint16_t, BE> ac6;


    I2CVar<0xB6, uint16_t, BE> b1;
    I2CVar<0xB8, uint16_t, BE> b2;


    I2CVar<0xBA, uint16_t, BE> mb;
    I2CVar<0xBC, uint16_t, BE> mc;
    I2CVar<0xBE, uint16_t, BE> md;

    I2CVar<0xF4, uint8_t, BE> control;



    #define BMP085_AC1_H                0xAA
    #define BMP085_AC1_L                0xAB
    #define BMP085_AC2_H                0xAC
    #define BMP085_AC2_L                0xAD
    #define BMP085_AC3_H                0xAE
    #define BMP085_AC3_L                0xAF
    #define BMP085_AC4_H                0xB0
    #define BMP085_AC4_L                0xB1
    #define BMP085_AC5_H                0xB2
    #define BMP085_AC5_L                0xB3
    #define BMP085_AC6_H                0xB4
    #define BMP085_AC6_L                0xB5

    #define BMP085_B1_H                 0xB6
    #define BMP085_B1_L                 0xB7
    #define BMP085_B2_H                 0xB8
    #define BMP085_B2_L                 0xB9

    #define BMP085_MB_H                 0xBA
    #define BMP085_MB_L                 0xBB
    #define BMP085_MC_H                 0xBC
    #define BMP085_MC_L                 0xBD
    #define BMP085_MD_H                 0xBE
    #define BMP085_MD_L                 0xBF

    #define BMP085_CONTROL_REG          0xF4

    #define BMP085_ADC_RAW_MSB          0xF6
    #define BMP085_ADC_RAW_LSB          0xF7
    #define BMP085_ADC_RAW_XLSB         0xF8

    /***** BMP085 Command List *****/
    #define BMP085_READ_TEMP            0x2E
    #define BMP085_READ_PRESSURE        0x34
    #define BMP085_OVERSAMPLING(x)      ((x) << 6)
};
