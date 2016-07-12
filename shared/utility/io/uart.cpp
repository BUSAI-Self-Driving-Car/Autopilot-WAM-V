#include "uart.h"

using namespace utility::io;

uart::uart()
    : port(io_service)
{ }

uart::uart(const std::string& device, unsigned int baud_rate)
    : uart()
{
   open(device, baud_rate);
}

uart::~uart()
{
    close();
}

void uart::close()
{
    if (port.is_open())
    {
        port.cancel();
        port.get_io_service().run();
        port.get_io_service().reset();
        port.close();
    }
}

void uart::open(const std::string &device, unsigned int baud_rate)
{
    port.open(device, ec);
    port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

    if (ec) {
        throw std::runtime_error(ec.message());
    }
}

size_t uart::write(const uint8_t* buffer, size_t size)
{
    if (!port.is_open()) return 0;

    boost::asio::write(port, boost::asio::buffer(buffer, size));
    return size;
}

int uart::read()
{
    if (!port.is_open()) return -1;

    ec.clear();
    uint8_t c;
    std::size_t n = boost::asio::read(port, boost::asio::buffer(&c,1));

    if (n > 0)
        return c;
    else
        return -1;
}
