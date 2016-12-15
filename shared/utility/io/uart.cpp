#include "uart.h"
#include <stdexcept>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

namespace utility {
namespace io {

    uart::uart()
            : device(""), fd(-1) {
    }

    uart::uart(const std::string& device, const unsigned int& baud)
            : device(device), fd(-1) {

        open(device, baud);
    }

    void uart::open(const std::string& device, const unsigned int& baud) {

        // Open our file descriptor for read/write with no controlling TTY and nonblock mode
        fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd >= 0) {

            // We want exclusive access to this uart
            if (ioctl(fd, TIOCEXCL) == -1) {
                throw std::runtime_error("Failed to set exclusive access for " + device);
            }

            // Set our baud rate
            set_baud(baud);
        }
        else {
            throw std::runtime_error("Failed to connect to " + device);
        }
    }

    void uart::close() {
        if (fd >= 0) {
            ::close(fd);
        }
    }

    void uart::flush() {
        // Do nothing, a UART isn't paged so no need to flush
    }

    uart::~uart() {
        close();
    }

    int uart::native_handle() {
        return fd;
    }

// Setting the baud rate on a linux system
#if defined(__linux__)

// Close our namespace so we can do the include
}
}

#include <linux/serial.h>

namespace utility {
namespace io {

    void uart::set_rts(int value)
    {
        int flag = TIOCM_RTS;
        if (value) { ioctl(fd, TIOCMBIS, &flag); }
        else { ioctl(fd, TIOCMBIC, &flag); }

    }

    void uart::set_dtr(int value)
    {
        int flag = TIOCM_DTR;
        if (value) { ioctl(fd, TIOCMBIS, &flag); }
        else { ioctl(fd, TIOCMBIC, &flag); }
    }

    void uart::set_baud(const int& baud) {

        // Do our setup for the tio settings, you must set BS38400 in order to set custom baud using "baud rate aliasing"
        // http://stackoverflow.com/questions/4968529/how-to-set-baud-rate-to-307200-on-linux
        termios tio;
        memset(&tio, 0, sizeof(tio));
        // B38400 for aliasing, CS8 (8bit,no parity,1 stopbit), CLOCAL (local connection, no modem control), CREAD (enable receiving characters)
        if (baud == 9600) {
            tio.c_cflag = B9600|CS8|CLOCAL|CREAD;
        }
        else if (baud == 115200) {
            tio.c_cflag = B115200|CS8|CLOCAL|CREAD;
        }
        else {
            tio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
        }
        // IGNPAR (ignore incoming parity bits as we don't have parity)
        tio.c_iflag      = IGNPAR;
        // 0 means raw output
        tio.c_oflag      = 0;
        // No ICANON so we read immediately rather then line by line
        tio.c_lflag     &= ~ICANON;
        tio.c_cc[VTIME]  = 0;
        tio.c_cc[VMIN]   = 0;
        // Set the settings
        tcsetattr(fd, TCSANOW, &tio);
        if (baud == 9600 || baud == 115200) return;

        // Here we do the baud rate aliasing in order to set the custom baud rate
        serial_struct serinfo;

        // Get our serial_info from the system
        if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
            throw std::runtime_error("There was an error setting the baud rate for " + device);
        }

        // Set the speed flags to "Custom Speed" (clear the existing speed, and set the custom speed flags)
        serinfo.flags &= ~ASYNC_SPD_MASK;
        serinfo.flags |= ASYNC_SPD_CUST;

        // Set our serial port to use low latency mode (otherwise the USB driver buffers for 16ms before sending data)
        serinfo.flags |= ASYNC_LOW_LATENCY;

        // Set our custom divsor for our speed
        serinfo.custom_divisor = serinfo.baud_base / baud;

        // Set our custom speed in the system
        if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
            throw std::runtime_error("There was an error setting the baud rate for " + device);
        }

        // Flush our connection to remove all existing data
        tcflush(fd, TCIFLUSH);
    }

// Setting baud rate for apple (should also be any BSD system)
#elif defined(__APPLE__)

    void uart::set_baud(const int& /*baud*/) {
        // There is an example of how to do this here if ever this needs to be written
        // https://developer.apple.com/library/mac/samplecode/SerialPortSample/Introduction/Intro.html
    }

#endif

    bool uart::good() const {
        return !(fcntl(fd, F_GETFL) < 0 && errno == EBADF);
    }

    int uart::get() {
        int data = 0;
        auto res = read(&data, 1);
        if (res <= 0) return -1;
        else return data;
    }

    ssize_t uart::read(void* buf, size_t count) {
        return ::read(fd, buf, count);
    }

    ssize_t uart::write(const void* buf, size_t count) {
        return  ::write(fd, buf, count);
    }

    ssize_t uart::blocking_write(const void* buf, size_t count) {
        auto n =  ::write(fd, buf, count);
        tcdrain(fd);
        return n;
    }

}
}
