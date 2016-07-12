#ifndef UART_H
#define UART_H

#include <boost/asio.hpp>

namespace utility {
namespace io {

class uart {
public:
    using char_type = uint8_t;
    uart();
    void open(const std::string& device, unsigned int baud_rate);
    void close();
    uart(const std::string& device, unsigned int baud_rate);
    ~uart();
    size_t write(const uint8_t* buffer, size_t size);
    boost::asio::serial_port::native_handle_type get_native_handle() { return port.native_handle(); }
    int read();
    int get() { return read(); }
    const boost::system::error_code& error_code() const { return ec; }
    bool is_open() { return port.is_open(); }
	bool good() { return is_open(); }
	void flush() {}

private:
    boost::asio::io_service io_service;
    boost::asio::serial_port port;
    boost::system::error_code ec;
};

}
}

#endif // UART_H
