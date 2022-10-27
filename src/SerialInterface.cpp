#include <iostream>
#include <sstream>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <system_error>	// For throwing std::system_error
#include <sys/ioctl.h>  // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

// User includes
#include "Exception.hpp"
#include "SerialInterface.hpp"

#define    BOTHER 0010000

SerialInterface::SerialInterface() {
        echo_ = false;
        timeout_ms_ = defaultTimeout_ms_;
        baudRate_ = defaultBaudRate_;
        readBufferSize_B_ = defaultReadBufferSize_B_;
        readBuffer_.reserve(readBufferSize_B_);
		state_ = State::CLOSED;
	}

SerialInterface::SerialInterface(const std::string& device, BaudRate baudRate) :
            SerialInterface() {
		device_ = device;
        baudRate_ = baudRate;
	}

SerialInterface::SerialInterface(const std::string& device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits) :
            SerialInterface() {
		device_ = device;
        baudRate_ = baudRate;
		numDataBits_ = numDataBits;
		parity_ = parity;
		numStopBits_ = numStopBits;
	}

SerialInterface::~SerialInterface() {
        try {
            Close();
        } catch(...) {
            // We can't do anything about this!
            // But we don't want to throw within destructor, so swallow
        }
	}

void SerialInterface::SetDevice(const std::string& device) {
		device_ = device;
        if(state_ == State::OPEN)
        	ConfigureTermios();
	}

void SerialInterface::SetBaudRate(BaudRate baudRate)	{		
		baudRate_ = baudRate;
        if(state_ == State::OPEN)
            ConfigureTermios();
	}

void SerialInterface::SetNumDataBits(NumDataBits numDataBits) {
		numDataBits_ = numDataBits;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

void SerialInterface::SetParity(Parity parity) {
		parity_ = parity;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

void SerialInterface::SetNumStopBits(NumStopBits numStopBits) {
		numStopBits_ = numStopBits;
		if(state_ == State::OPEN)
            ConfigureTermios();
	}

void SerialInterface::Open()
	{
		if(device_.empty()) {
			THROW_EXCEPT("Attempted to open file when file path has not been assigned to.");
		}

		// O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both read/write access
		fileDesc_ = open(device_.c_str(), O_RDWR);

		// Check status
		if(fileDesc_ == -1) {
		    THROW_EXCEPT("Could not open device " + device_ + ". Is the device name correct and do you have read/write permission?");
		}

        ConfigureTermios();

        state_ = State::OPEN;
	}

void SerialInterface::SetEcho(bool value) {
        echo_ = value;
        ConfigureTermios();
	}

void SerialInterface::ConfigureTermios()
	{
		//================== CONFIGURE ==================//

		termios2 tty = GetTermios2();

		//================= (.c_cflag) ===============//

		// Set num. data bits
		// See https://man7.org/linux/man-pages/man3/tcflush.3.html
		tty.c_cflag     &=  ~CSIZE;			// CSIZE is a mask for the number of bits per character
		switch(numDataBits_) {
			case NumDataBits::FIVE:
				tty.c_cflag     |=  CS5;
				break;
			case NumDataBits::SIX:
				tty.c_cflag     |=  CS6;
				break;
			case NumDataBits::SEVEN:
				tty.c_cflag     |=  CS7;
				break;
			case NumDataBits::EIGHT:
				tty.c_cflag     |=  CS8;
				break;
			default:
				THROW_EXCEPT("numDataBits_ value not supported!");
		}
		
		// Set parity
		// See https://man7.org/linux/man-pages/man3/tcflush.3.html
		switch(parity_) {
			case Parity::NONE:
				tty.c_cflag     &=  ~PARENB;
				break;
			case Parity::EVEN:	
				tty.c_cflag 	|=   PARENB;
				tty.c_cflag		&=	 ~PARODD; // Clearing PARODD makes the parity even
				break;
			case Parity::ODD:
				tty.c_cflag     |=   PARENB;
				tty.c_cflag		|=	 PARODD;
				break;
			default:
				THROW_EXCEPT("parity_ value not supported!");

		}

		// Set num. stop bits
		switch(numStopBits_) {
			case NumStopBits::ONE:
				tty.c_cflag     &=  ~CSTOPB;
				break;
			case NumStopBits::TWO:
				tty.c_cflag     |=  CSTOPB;
				break;
			default:
				THROW_EXCEPT("numStopBits_ value not supported!");
		}

		tty.c_cflag     &=  ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
		tty.c_cflag     |=  CREAD | CLOCAL;     				// Turn on READ & ignore ctrl lines (CLOCAL = 1)


        //===================== BAUD RATE =================//

		tty.c_cflag &= ~CBAUD;
		tty.c_cflag |= CBAUDEX;
		switch(baudRate_) 
        {
			case BaudRate::B_9600:
				tty.c_ispeed = 9600;
				tty.c_ospeed = 9600;
				break;
			case BaudRate::B_19200:
				tty.c_ispeed = 19200;
				tty.c_ospeed = 19200;
				break;
			case BaudRate::B_38400:
				tty.c_ispeed = 38400;
				tty.c_ospeed = 38400;
				break;
			case BaudRate::B_57600:
                tty.c_ispeed = 57600;
				tty.c_ospeed = 57600;
				break;
			case BaudRate::B_115200:
				tty.c_ispeed = 115200;
				tty.c_ospeed = 115200;
				break;
			case BaudRate::B_230400:
				tty.c_ispeed = 230400;
				tty.c_ospeed = 230400;
				break;
			case BaudRate::B_460800:
				tty.c_ispeed = 460800;
				tty.c_ospeed = 460800;
				break;
			default:
				throw std::runtime_error(std::string() + "baudRate passed to " + __PRETTY_FUNCTION__ + " unrecognized.");
		}

		//===================== (.c_oflag) =================//

		tty.c_oflag     =   0;              // No remapping, no delays
		tty.c_oflag     &=  ~OPOST;			// Make raw

		//================= CONTROL CHARACTERS (.c_cc[]) ==================//

		// c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
		// Only meaningful when port is set to non-canonical mode
        // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
        // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
        // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
        // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
        //                      after first character has elapsed
        // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
        // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

        if(timeout_ms_ == -1) {
            // Always wait for at least one byte, this could
            // block indefinitely
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 1;
        } else if(timeout_ms_ == 0) {
            // Setting both to 0 will give a non-blocking read
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 0;
        } else if(timeout_ms_ > 0) {
            tty.c_cc[VTIME] = (cc_t)(timeout_ms_/100);    // 0.5 seconds read timeout
            tty.c_cc[VMIN] = 0;
        }

		//======================== (.c_iflag) ====================//

		tty.c_iflag     &= ~(IXON | IXOFF | IXANY);			// Turn off s/w flow ctrl
		tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

		//=========================== LOCAL MODES (c_lflag) =======================//

		// Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
		// read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]

		tty.c_lflag		&= ~ICANON;								// Turn off canonical input, which is suitable for pass-through
		// Configure echo depending on echo_ boolean
        if(echo_) {
			tty.c_lflag |= ECHO;
		} else {
			tty.c_lflag &= ~(ECHO);
		}
		tty.c_lflag		&= ~ECHOE;								// Turn off echo erase (echo erase only relevant if canonical input is active)
		tty.c_lflag		&= ~ECHONL;								//
		tty.c_lflag		&= ~ISIG;								// Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

		this->SetTermios2(tty);

	}

void SerialInterface::Write(const std::string& data) {

        if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

		if(fileDesc_ < 0) {
			THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
		}

		int writeResult = write(fileDesc_, data.c_str(), data.size());

		// Check status
		if (writeResult == -1) {
			throw std::system_error(EFAULT, std::system_category());
		}
	}

void SerialInterface::Read(std::string& data)
	{
        data.clear();

		if(fileDesc_ == 0) 
        {
			THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
		}

		// Read from file
        // We provide the underlying raw array from the readBuffer_ vector to this C api.
        // This will work because we do not delete/resize the vector while this method
        // is called
		ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

		// Error Handling
		if(n < 0) 
        {
			throw std::system_error(EFAULT, std::system_category());
		}

		if(n > 0) {
            data = std::string(&readBuffer_[0], n);
		}

		// If code reaches here, read must of been successful
	}

termios2 SerialInterface::GetTermios2()
	{
		struct termios2 term2;

        ioctl(fileDesc_, TCGETS2, &term2);

		return term2;
	}

void SerialInterface::SetTermios2(termios2 tty)
{
	ioctl(fileDesc_, TCSETS2, &tty);
}

void SerialInterface::Close()
{
    if(fileDesc_ != -1) 
    {
            auto retVal = close(fileDesc_);
            if(retVal != 0)
                THROW_EXCEPT("Tried to close serial port " + device_ + ", but close() failed.");

            fileDesc_ = -1;
        }

    state_ = State::CLOSED;
}

void SerialInterface::SetTimeout(int32_t timeout_ms) {
        if(timeout_ms < -1)
            THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was < -1, which is invalid.");
        if(timeout_ms > 25500)
            THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was > 25500, which is invalid.");
        if(state_ == State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called while state == OPEN.");
        timeout_ms_ = timeout_ms;
    }
    
int32_t SerialInterface::Available() {
		if(state_ != State::OPEN)
            THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");
        int32_t ret = 0;
        ioctl(fileDesc_, FIONREAD, &ret);
        return ret;
        
    }

SerialInterface::State SerialInterface::GetState() {
      return state_;
    }