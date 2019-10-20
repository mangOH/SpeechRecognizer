#include "legato.h"
#include "interfaces.h"

#include <termios.h>
#include <unistd.h>

#define UNKNOWN_CMD_NUM		252

// Data Hub resource paths.
#define RES_PATH_WRITE_EINK     "/app/einkDhubIf/value"

static const char *voiceBuffer[] =
{
	"Turn on the light",
	"Turn off the light",
	"Play music",
	"Pause",
	"Next",
	"Previous",
	"Up",
	"Down",
	"Turn on the TV",
	"Turn off the TV",
	"Increase temperature",
	"Decrease temperature",
	"What's the time",
	"Open the door",
	"Close the door",
	"Left",
	"Right",
	"Stop",
	"Start",
	"Mode 1",
	"Mode 2",
	"Go",
};

int open_uart1(char *dev)
{ 

	int     fd;
	struct termios options;

	fd = open (dev, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1) {
		LE_ERROR("Failed open of serial port: %s", strerror(fd));
		exit(1);
	}

	// The old way. Let's not change baud settings
	fcntl (fd, F_SETFL, 0);
	if(fd == -1) {
		LE_ERROR("Failed F_SETFL: %s", strerror(fd));
		exit(1);
	}

	// get the parameters
	tcgetattr (fd, &options);
	if(fd == -1) {
		LE_ERROR("Failed tcgetattr: %s", strerror(fd));
		exit(1);
	}

	// Set the baud rates to 9600...
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);
	// No parity (8N1):
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
 	// enable hardware flow control (CNEW_RTCCTS)
	// options.c_cflag |= CRTSCTS;
	// if(hw_handshake)
	// Disable the hardware flow control for use with mangOH RED
	options.c_cflag &= ~CRTSCTS;

	// set raw input
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_iflag &= ~(INLCR | ICRNL | IGNCR);

	// set raw output
	options.c_oflag &= ~OPOST;
	options.c_oflag &= ~OLCUC;
	options.c_oflag &= ~ONLRET;
	options.c_oflag &= ~ONOCR;
	options.c_oflag &= ~OCRNL;

	// Set the new options for the port...
	tcsetattr (fd, TCSANOW, &options);
	if(fd == -1) {
		LE_ERROR("Failed tcsetattr: %s", strerror(fd));
		exit(1);
	}

	return fd;
}

void read_uart1(int fd)
{
	char read_buffer[32];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */
	int i = 0;
	uint CmdNum;

	bytes_read = read(fd,&read_buffer,10); /* Read the data                   */

	for(i=0 ; i < bytes_read; i++) {
		CmdNum = read_buffer[i] - 1;
		if(CmdNum == UNKNOWN_CMD_NUM)
			admin_PushString(RES_PATH_WRITE_EINK, 0, "TALK LOUDER or CLEARER");
		else if(CmdNum < sizeof(voiceBuffer)/sizeof(voiceBuffer[0]))
			admin_PushString(RES_PATH_WRITE_EINK, 0, voiceBuffer[CmdNum]);
	}
}

COMPONENT_INIT
{
	int serial_fd;
	
	serial_fd= open_uart1("/dev/ttyHS0");
        if(serial_fd == -1) {
                LE_ERROR("Failed open_uart1");
                exit(1);
        }

	LE_INFO("We have %d commands", sizeof(voiceBuffer)/sizeof(voiceBuffer[0]));
	// Let's loop forever reading from the recognizer and updating outputs
	while(1) 
		read_uart1(serial_fd);

	close(serial_fd);
}
