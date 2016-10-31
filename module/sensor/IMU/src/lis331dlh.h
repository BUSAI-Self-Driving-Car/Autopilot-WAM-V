
struct lis331dlh : public I2CDevice<hmc5883l> {

    lis331dlh(uint8_t device_address = 0x18)
    : I2CDevice<lis331dlh>(device_address) {
    }

    #define LIS331DLH_WHO_AM_I          0x0F
    #define LIS331DLH_CTRL_REG1         0x20
    #define LIS331DLH_CTRL_REG2         0x21
    #define LIS331DLH_CTRL_REG3         0x22
    #define LIS331DLH_CTRL_REG4         0x23
    #define LIS331DLH_CTRL_REG5         0x24
    #define LIS331DLH_HP_FILTER_RESET   0x25
    #define LIS331DLH_REFERENCE         0x26
    #define LIS331DLH_STATUS_REG        0x27
    #define LIS331DLH_OUT_X_L           0x28
    #define LIS331DLH_OUT_X_H           0x29
    #define LIS331DLH_OUT_Y_L           0x2A
    #define LIS331DLH_OUT_Y_H           0x2B
    #define LIS331DLH_OUT_Z_L           0x2C
    #define LIS331DLH_OUT_Z_H           0x2D
    #define LIS331DLH_INT1_CFG          0x30
    #define LIS331DLH_INT1_SOURCE       0x31
    #define LIS331DLH_INT1_THS          0x32
    #define LIS331DLH_INT1_DURATION     0x33
    #define LIS331DLH_INT2_CFG          0x34
    #define LIS331DLH_INT2_SOURCE       0x35
    #define LIS331DLH_INT2_THS          0x36
    #define LIS331DLH_INT2_DURATION     0x37
};
