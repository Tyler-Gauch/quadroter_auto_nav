#include <string>
#include "Serial.h"

Serial::Serial(const std::string& port, int baud_rate, boost::asio::io_service& io): 
	_port(port),
	_baud_rate(baud_rate),
	_serial(io, _port) 
{
	_serial.set_option(boost::asio::serial_port_base::baud_rate(_baud_rate));
}

int Serial::write(const std::string &buf)
{
	return write(buf.c_str(), buf.size());
}

int Serial::write(const char *buf, const int &size)
{
	boost::system::error_code ec;

	if (size == 0) return 0;

	return _serial.write_some(boost::asio::buffer(buf, size), ec);
}

int Serial::read(char * buffer, int size){
	return boost::asio::read(_serial,boost::asio::buffer(buffer,size));
}