#include "SerialHandler.h"

#include <string>

SerialHandler::SerialHandler(const std::string& port, int baud_rate, int timeout) {
	_serial.setPort(port);
	_serial.setBaudrate(baud_rate);
	serial::Timeout to = serial::Timeout::simpleTimeout(timeout);

	_serial.setTimeout(to);
	_serial.open();

}

SerialHandler::SerialHandler(const std::string& port, int baud_rate) {
	_serial.setPort(port);
	_serial.setBaudrate(baud_rate);

	_serial.open();
}

SerialHandler::~SerialHandler() {
	_serial.close();
}

int SerialHandler::write(const std::string &buf)
{
	_serial.write(buf);
}

int SerialHandler::write(const char *buf, const int &size)
{
//	boost::system::error_code ec;
//
//	if (size == 0) return 0;
//
//	return _serial.write_some(boost::asio::buffer(buf, size), ec);
}

int SerialHandler::read(char * buffer, int size){
//	return boost::asio::read(_serial,boost::asio::buffer(buffer,size));
}
