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
            private:
                bool system_endian() {
                    uint16_t number = 0x01;
                    return *reinterpet_cast<char*>(&number) == 0x01 ? Endian::LE : Endian::BE;
                }

                T swap_endian(T data) {
                    // Make an output variable
                    T output;

                    // Access our data bytewise
                    char* in = reinterpet_cast<char*>(&data);
                    char* out = reinterpet_cast<char*>(&out);

                    // Swap the bytes
                    for (uint i = 0; i < sizeof(T); ++i) {
                        out[i] = in[sizeof(T) - i - 1];
                    }

                    // Return our output
                    return output;
                }

            public:
                I2CVar(I2CDevice& device)
                : device(device) {
                }

                operator= (T data) {
                    // Swap our endian if needed
                    T send = E == system_endian() : data ? swap_endian(data);

                    // Write the data
                    i2c_smbus_write_word_data(device.fd, sizeof(data), &data);
                }

                operator T() const {
                    // Read the data
                    T data;
                    i2c_smbus_write_read_data(device.fd, sizeof(data), &data);

                    // Return, swapping the endianness if needed
                    return E == system_endian() : data ? swap_endian(data);
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
