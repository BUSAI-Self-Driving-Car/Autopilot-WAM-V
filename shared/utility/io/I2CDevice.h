#ifndef UTILITY_IO_I2CDEVICE
#define UTILITY_IO_I2CDEVICE

namespace utility {
    namespace io {

        template <typename Sub>
        struct I2CDevice {
            enum Endian {
                BE,
                LE
            };

        protected:
            I2CDevice(std::string device_path, uint8_t device_address) {

                // Open our file
                fd = open(device_path, O_RDWR);
                if(fd < 0) {
                    throw std::system_error(errno, std::system_category(), "Error connecting to I2C device");
                }

                // Set the slave device that we are connecting to
                if (ioctl(file, I2C_SLAVE, device_address) < 0) {
                    throw std::system_error(errno, std::system_category(), "Error setting the slave address");
                }
            }


            template <uint8_t addr, typename T, enum Endian E>
            struct I2CVar {
            public:
                I2CVar(I2CDevice& device)
                : device(device) {
                }

                operator= (const T&& data) {
                    // Write the data
                    i2c_smbus_write_word_data(device.fd, sizeof(data), &data);
                }

                operator T() const {
                    // Read the data
                    T data;
                    i2c_smbus_write_word_data(device.fd, sizeof(data), &data);
                    return data;
                }

            private:
                I2CDevice& device_address;
            };

        private:
            int fd;
        };
    }
}

#endif  // UTILITY_IO_I2CDEVICE
