#include "SerialInterface.hpp"

int main(int argc, char* argv[])
{
    std::string readData;

	SerialInterface SerialInterface("/dev/ttyUSB0", SerialInterface::BaudRate::B_115200, SerialInterface::NumDataBits::EIGHT, 
                                                    SerialInterface::Parity::NONE, SerialInterface::NumStopBits::ONE);

	SerialInterface.SetTimeout(-1); // Block when reading until any data is received
	SerialInterface.Open();

	SerialInterface.Write("0");
	SerialInterface.Read(readData);

    std::cout << readData << std::endl;

	SerialInterface.Write("1");
	SerialInterface.Read(readData);

    std::cout << readData << std::endl;

	SerialInterface.Write("0");
	SerialInterface.Read(readData);

    std::cout << readData << std::endl;

	SerialInterface.Close();
}