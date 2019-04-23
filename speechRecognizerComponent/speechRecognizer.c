#include "legato.h"
#include "interfaces.h"

#include <termios.h>
#include <unistd.h>

const char *voiceBuffer[] =
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

int open_uart1 (  char *dev){ 
	int     fd;
	fd = open (dev, O_RDWR | O_NOCTTY | O_NDELAY);
	struct termios options;
	// The old way. Let's not change baud settings
	fcntl (fd, F_SETFL, 0);
	// get the parameters
	tcgetattr (fd, &options);
	// Set the baud rates to 115200...
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

	return fd;
}

void read_uart1(int fd)
{
	char read_buffer[32];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */
	int i = 0;

	bytes_read = read(fd,&read_buffer,10); /* Read the data                   */

	for(i=0 ; i < bytes_read; i++)
	{
		printf(voiceBuffer[read_buffer[i] - 1]);
	}
	printf("\n----------------------------------\n");
}

void write_uart1 (int fd, char *cmd)
{
	int     wrote = 0;
	wrote = write (fd, cmd, strlen (cmd));
	LE_INFO("wrote  %d ",wrote);
} 


COMPONENT_INIT
{
	LE_INFO("Hello, world.");
	int serial_fd;
	
	serial_fd= open_uart1("/dev/ttyHS0");

	while(1)
	{
		read_uart1(serial_fd);
	}
	close(serial_fd);
}
