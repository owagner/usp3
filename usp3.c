/*
 *
 * USP3
 * ====
 *
 * Trivial utility to control an Chromoflex RGB LED controller
 * See http://www.xeroflex.com/Usp3_com.pdf for protocol specs
 *
 * Written by Oliver Wagner <owagner@tellerulam.com>
 * Free to use
 *
 */

#define VERSION "1.2"
 
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/sysinfo.h>
#include <signal.h>
 
char *dev="/dev/ttyUSB0";
int dev_fd=-1;
int addr;

static struct option opts[]=
{
	{ "device", required_argument,	0, 	'd' },	
	{ "address", required_argument,	0, 	'a' },	
	{ "level", required_argument,	0, 	'l' },	
	{ "set", required_argument,	0, 	's' },	
	{ "increment", required_argument,	0, 	'i' },	
	{ "track", required_argument,	0, 	't' },	
	{ "program", required_argument,	0, 	'p' },	
	{ "reset", no_argument,	0, 		'r' },	
	{ "stop", no_argument,	0, 		'x' },	
	{ "start", no_argument,	0, 		'X' },	
	{ "lamon", no_argument,	0, 		'L' },	
	{ "version", no_argument,	0, 	'V' },
	{0}	
};

#define PON(i,t) printf("-%c --%s\t" t "\n",opts[i].val,opts[i].name)

static void print_usage()
{
	PON( 0, "device to use [/dev/ttyUSB0]" );
	PON( 1, "device address [broadcast]" );
	PON( 2, "set level (r,g,b)" );
	PON( 3, "set target (r,g,b)" );
	PON( 4, "set increment (r,g,b)" );
	PON( 5, "set track" );
	PON( 6, "start program" );
	PON( 7, "reset" );
	PON( 8, "stop program" );
	PON( 9, "start program" );
	PON(10, "run load average monitor" );
	printf("-h --help\n-V --version\n");
}

static void setupDev()
{
	struct termios tio;
	
	if(dev_fd>=0)
	{
		tcflush(dev_fd,TCIFLUSH);
		return;
	}
	
	dev_fd=open(dev,O_RDWR|O_NOCTTY);
	if(dev_fd<0)
	{
		perror(dev);
		exit(-1);
	}
	tcgetattr(dev_fd,&tio);
	cfmakeraw(&tio);
	tio.c_cflag=B9600|CS8|CLOCAL;
	cfsetispeed(&tio,B9600);
	cfsetospeed(&tio,B9600);
	tio.c_oflag=0;
	tio.c_cc[VMIN]=8;
	tio.c_cc[VTIME]=10;
	if(tcsetattr(dev_fd,TCSANOW,&tio))
		perror("serial port mode");
}

static unsigned short usp_crc;
void process_crc(unsigned char ucData)
{
	int i;
	usp_crc^=ucData;
	for(i=0;i<8;i++)
	{ // Process each Bit
		if(usp_crc&1)
		{ 
			usp_crc >>=1; 
			usp_crc^=0xA001;
		}
		else
		{
			usp_crc >>=1;
		}
	}
}

void serial_send_raw(unsigned char sb)
{
	write(dev_fd,&sb,1);	
}

void serial_send_cooked(unsigned char sb)
{
	process_crc(sb); // track CRC
	if(sb==0xCA)
	{
		serial_send_raw(0xCB); // Physically out
		serial_send_raw(0);
	}
	else if(sb==0xCB)
	{
		serial_send_raw(0xCB);
		serial_send_raw(1);
	} 
	else
	{
		serial_send_raw(sb);
	}
}

static void sendcmd(unsigned char cmd,unsigned char *data,int datalen)
{
	setupDev();
	serial_send_raw(0xca);
	usp_crc=0x173f;
	serial_send_cooked((addr>>16)&0xff);
	serial_send_cooked((addr>>8)&0xff);
	serial_send_cooked((addr>>0)&0xff);
	serial_send_cooked((datalen>>8)&0xff);
	serial_send_cooked((datalen>>0)&0xff);
	serial_send_cooked(cmd);
	while(datalen-->0)
		serial_send_cooked(*data++);
	int crc=usp_crc;
	serial_send_cooked((crc>>8)&0xff);
	serial_send_cooked((crc>>0)&0xff);
}

static void sendreset()
{
	sendcmd(0xfe,NULL,0);
}

static void sendreg(int offset,char *spec,int maxargs)
{
	// Specify a number of registers
	unsigned char data[5];
	int len=0;
	char *p;
	
	spec=strdup(spec);
	
	for(p=strtok(spec,",");p && maxargs--;p=strtok(NULL,","))
	{
		if(!*p)
			offset++;
		else
			data[++len]=strtol(p,NULL,0);	
	}
	if(len)
	{
		data[0]=offset;
		sendcmd(0x7e,data,len+1);	
	}
	free(spec);
}

static void sendprog(int pn)
{
	char buffer[32];
	
	pn=200+pn*3;
	
	// We need to write hi/lo, otherwise the ChromoFlex
	// will ignore the command
	snprintf(buffer,sizeof(buffer),"0,%u",pn);
	sendreg(21,buffer,2); 
}

static void initrunla()
{
	// Take control
	sendreg(18,"1",1);
	// Increment
	sendreg(8,"1,1,1",3);
	// Track 1/100s
	sendreg(17,"1",1);
}

static void runla()
{
	struct sysinfo si;
	int r=0,g=0xff,b=0;
	int or=0,og=0,ob=0;
	
	initrunla();
	signal(SIGHUP,initrunla);
	while(1)
	{
		if(or!=r || og!=g || ob!=b)
		{
			char buffer[64];
			sprintf(buffer,"%u,%u,%u",r,g,b);
			sendreg(4,buffer,3);
			or=r;
			og=g;
			ob=b;
		}
		sleep(1);
		sysinfo(&si);
		double v=si.loads[0]/65536.0;
		
		if(v<1.0)
		{
			// Green
			r=0;
			g=v*255;
		}
		else if(v<2.0)
		{
			// Fade from green to yellow
			r=(v-1.0)*255;
			g=0xff;
		}
		else if(v<3.0)
		{
			r=0xff;
			g=1.0-(v-2)*255;
		}
		else
		{
			r=0xff;
			g=0;
		}
	}
}

static void doParams(int argc,char **argv)
{
	while(1)
	{
		int ch=getopt_long_only(argc,argv,"d:a:l:i:t:p:s:xXrL?hV",opts,0);
		if(ch<0)
			break;

		switch(ch)
		{
			case 'V':
				printf("usp3 " VERSION " compiled " __DATE__ " " __TIME__ "\n");
				exit(0);
				
			case 'd':
				dev=optarg;
				break;
				
			case 'a':
				addr=strtol(optarg,NULL,0);
				break;
				
			case 'l':
				sendreg(0,optarg,3);
				break;
				
			case 's':
				sendreg(4,optarg,3);
				break;
				
			case 'i':
				sendreg(8,optarg,3);
				break;
				
			case 't':
				sendreg(17,optarg,1);
				break;
				
			case 'x':
				sendreg(18,"1",1);
				break;
				
			case 'X':
				sendreg(18,"0",1);
				break;
				
			case 'p':
				sendprog(strtol(optarg,NULL,0));
				break;

			case 'r':
				sendreset();
				break;
				
			case 'L':
				runla();
				break;
				
			default:
			case '?':
			case 'h':
				print_usage();
				exit(0);
		}
	}
	if(optind<argc)
	{
		fprintf(stderr,"Ignoring excess command line arguments\n");
	}
}

int main(int argc,char **argv)
{
	doParams(argc,argv);
	return 0;
}
