//2.35a
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sched.h>
#include <getopt.h>
#include <termios.h>

uint8_t debug=0;

uint8_t uart_path[255] = "/dev/ttyUSB0";
uint32_t tty_speed=57600;
uint8_t stop = 0;
uint8_t mode = 0;
int uart_fd=0;

#define BUFFER_LEN 512
uint8_t buffer[BUFFER_LEN];

#define PREAMBLE "\x2f\x76\x2A\x24"
#define PREAMBLE_LEN (4)
#define HEADER_LEN (PREAMBLE_LEN+1)
#define ID_OFFSET (4)
#define CRC_LEN (4)

struct packet {
	uint8_t size;
	uint8_t data[512];
};

struct s_header {
	uint8_t preamble[4];
	uint8_t id;
};


#define STATUS_ID (0x53)
#define STATUS_REQ_REMINDER  "\x00\x00\xcc\x6d\xfd\x2b"
#define STATUS_REQ_REMINDER_LEN 6

#define STATUS_LEN (HEADER_LEN+CRC_LEN+156)
#define SPECTRUM_LEN (HEADER_LEN+257) //no crc
#define PARAM_LEN (HEADER_LEN+CRC_LEN+43)
#define PARAM_RESPONSE_LEN (HEADER_LEN+1)
#define PARAM_ID (0x58)

#define SPECTRUM_ID (0x52)
#define SPECTRUM_REQ_REMINDER "\x00\x1e\x09\x12\x73"
#define SPECTRUM_REQ_REMINDER_LEN 5

struct s_spectrum {
	uint8_t v[0xFF];
};

struct s_status {
	uint8_t ch_num; //number of channels (0x00-0x05)
	uint8_t ch[5]; //actual channel frequency (0x00-0xFF); 0x00=432.000MHz; 0xFF=470.250MHz; Spacing=0.150MHz
	float rssi_min[5];
	float rssi[5];
	uint16_t servo[16];
	float temperature;
	int8_t afc;
	uint8_t param_ver_major;
	uint8_t param_ver_minor;
	uint8_t rssi_avg;
	uint8_t mode; //TX=0x54; RX=0x52
	uint16_t bind_code; //bind code (big-int)
	uint8_t failsafe;
	uint8_t servo_map[16]; //target servo for each of 0...15 servos
	uint8_t ppm_pin; //0x00=Sponge board/ULRS Mini; 0x01=Normal pinout; 0x02=LRSMAX
	uint8_t rssi_ch; //0xff=as PWM, 0x0f as PPM (channel 15) 
	uint8_t buzzer; //0x00=mutted, 0x01=on
	uint8_t ppm_mode; //0x00=normal, 0x01=inverted
};

#define MODE_TX (0x54)
#define MODE_RX (0x52)
#define PPM_SPONGE (0x00)
#define PPM_NORMAL (0x01)
#define PPM_LRSMAX (0x02)
#define BUZZER_OFF (0x00)
#define BUZZER_ON (0x01)
#define PPM_MODE_INVERTED (0x01)
#define PPM_MODE_NORMAL (0x00)
#define RSSI_AS_PWM (0xff)
#define FAILSAFE_ON (0x01)
#define FAILSAFE_OFF (0x00)

void print_hex(const uint8_t *arr,int len);

void spectrum_parse(struct s_spectrum *s ,uint8_t *buf) {
	uint8_t i;

	memcpy(s->v,buf+6,0xFF);
	
}

void spectrum_print(struct s_spectrum *s) {
	uint8_t i;

	printf("Spectrum analyzer:\n");
	for (i=0;i<0xFF;i++) {
		printf("\t%.3f MHz, dBm: %i\n",432.f+i*0.15f,-128+s->v[i]/2);
	}

}

void status_parse(struct s_status *s ,uint8_t *buf) {
	uint8_t i;

	memcpy(&s->ch_num,buf+6,1);
	
	memcpy(s->ch,buf+7,5);
	
	for (i=0;i<5;i++)
		memcpy(s->rssi_min+i,buf+12+i*4,4);

	for (i=0;i<5;i++)
		memcpy(s->rssi+i,buf+32+i*4,4);
	
	for (i=0;i<16;i++) {
		memcpy(s->servo+i,buf+52+i*2,2);
		s->servo[i] = ntohs(s->servo[i]);
		s->servo[i] /= 2;
	}

	memcpy(&s->temperature,buf+104,4);

	memcpy(&s->afc,buf+108,1);

	memcpy(&s->param_ver_major,buf+111,1);
	memcpy(&s->param_ver_minor,buf+112,1);

	memcpy(&s->rssi_avg,buf+113,1);

	memcpy(&s->mode,buf+114,1);

	memcpy(&s->bind_code,buf+116,2);
	s->bind_code = ntohs(s->bind_code);

	memcpy(&s->failsafe,buf+126,1);
	
	for (i=0;i<16;i++) 
		memcpy(s->servo_map+i,buf+128+i,1);

	memcpy(&s->ppm_pin,buf+144,1);

	memcpy(&s->rssi_ch,buf+145,1);

	memcpy(&s->buzzer,buf+147,1);

	memcpy(&s->ppm_mode,buf+148,1);
	
}


void status_print(struct s_status *s) {
	uint8_t i;
	printf("\tNumber of channels: %u\n",s->ch_num);

	for (i=0;i<s->ch_num;i++) {
		printf("\tChannel %u: %u (%.3f MHz), RSSI: %.0f (%.0f)\n",i,s->ch[i],432.f+s->ch[i]*0.15f,s->rssi[i],s->rssi_min[i]);
	}
	printf("\tRSSI average: %i\n",(int16_t)(-130+s->rssi_avg*0.526f));

	printf("\tServo values:\n");	
	for (i=0;i<16;i++) {
		printf("\t\t[%u]: [%u]\n",i+1,s->servo[i]);
	}

	printf("\tServo mapping:\n");	
	for (i=0;i<16;i++) {
		printf("\t\t[%u] => [%u]\n",i+1,s->servo_map[i]+1);
	}

	printf("\tTemperature: %.1f\n",s->temperature);

	printf("\tAFC: %i\n",(int32_t)(s->afc*156.25f));

	printf("\tParam version: %u.%u\n",s->param_ver_major,s->param_ver_minor);

	if (s->mode==MODE_TX) printf("\tMode: TX\n");
	else if (s->mode==MODE_RX) printf("\tMode: RX\n");
	else printf("\tMode: Unknown\n");

	if (s->ppm_pin==PPM_SPONGE) printf("\tPPM Pin: Sponge board/ULRS Mini\n");
	else if (s->ppm_pin==PPM_LRSMAX) printf("\tPPM Pin: LRSMAX\n");
	else if (s->ppm_pin==PPM_NORMAL) printf("\tPPM Pin: Normal pinout\n");
	else printf("\tPPM Pin: Unknown\n");

	printf("\tBind code: %u\n",s->bind_code);

	if (s->failsafe==FAILSAFE_ON) printf("\tFailsafe: on\n");
	else if (s->failsafe==FAILSAFE_OFF) printf("\tFailsafe: off\n");
	else printf("\tFailsafe: Unknown\n");

	printf("\tRSSI channel: ");
	if (s->rssi_ch==RSSI_AS_PWM) printf("dedicated PWM\n");
	else printf("%i\n",s->rssi_ch);

	printf("\tBuzzer: ");
	if (s->buzzer==BUZZER_ON) printf("On\n");
	else if (s->buzzer==BUZZER_OFF) printf("Off\n");
	else printf("Unknown!\n");

	printf("\tPPM Mode (normal/inverted): ");
	if (s->ppm_mode==PPM_MODE_NORMAL) printf("Normal\n");
	else if (s->ppm_mode==PPM_MODE_INVERTED) printf("Inverted\n");
	else printf("Unknown!\n");


}

struct s_param {
	uint8_t ch_num; //number of channels (0x00-0x05)
	//padding 1 byte
	uint8_t ch[5]; //actual channel frequency (0x00-0xFF); 0x00=432.000MHz; 0xFF=470.250MHz; Spacing=0.150MHz
	uint8_t mode; //TX=0x54; RX=0x52
	uint8_t servo_map[16]; //target servo for each of 0...15 servos
	uint8_t ppm_pin; //0x00=Sponge board/ULRS Mini; 0x01=Normal pinout; 0x02=LRSMAX
	//padding 16 bytes
	uint16_t bind_code; //bind code (big-int)
	uint8_t rssi_ch;
	uint8_t buzzer;
	uint8_t ppm_mode;
};

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

unsigned char reverse(unsigned char b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

uint32_t reverse32(uint32_t x)
{
	x = ((x >> 1) & 0x55555555u) | ((x & 0x55555555u) << 1);
	x = ((x >> 2) & 0x33333333u) | ((x & 0x33333333u) << 2);
	x = ((x >> 4) & 0x0f0f0f0fu) | ((x & 0x0f0f0f0fu) << 4);
	x = ((x >> 8) & 0x00ff00ffu) | ((x & 0x00ff00ffu) << 8);
	x = ((x >> 16) & 0xffffu) | ((x & 0xffffu) << 16);
	return x;
}

uint32_t crc32(unsigned char const *addr, size_t num)
{
	uint32_t i,crc=0;
	uint8_t b;

	crc = 0x314d9b87;
	const uint32_t polynomial = 0x04C11DB7; /* divisor is 32bit */

	while (num--) {
		b = (*addr);
		addr++;
		b = reverse(b);

		crc ^= (uint32_t)(b << 24); /* move byte into MSB of 32bit CRC */

		for (i = 0; i < 8; i++) {
			if ((crc & 0x80000000) != 0) { /* test for MSB = bit 31 */
				crc = (uint32_t)((crc << 1) ^ polynomial);
			} else {
				crc <<= 1;
			}
		}
	}

	crc = reverse32(crc);
	return crc^0x768af41a;

}

int8_t uart_send(uint8_t *buf, uint8_t len) {
	int ret;
	if (debug) print_hex(buf,len);
	ret=write(uart_fd,buf,len);
	if (ret==-1 && errno==EAGAIN) return 0;
	if (ret!=len) {
		if (debug) printf("Error writing to serial!\n");
		return -1;
	}
	if (debug) printf("OK\n");
	return 0;
}

speed_t get_tty_speed(uint32_t v) {
	switch(v) {
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		default: return 0;
	}
}

int uart_open(const char *path, int flags) {
	printf("Openning UART port %s (%u)\n",path,tty_speed);
	int ret = open(path, flags);

	if (ret<0) {
		printf("open failed on %s [%i] [%s]\n",path,errno,strerror(errno));
		return ret;
	}

	if (get_tty_speed(tty_speed)==0) {
		printf("Incorrect serial speed: %u\n",tty_speed);
		return -1;
	}

	struct termios options;
	fcntl(ret, F_SETFL, FNDELAY);                    // Open the device in nonblocking mode

	// Set parameters
	tcgetattr(ret, &options);                        // Get the current options of the port
	bzero(&options, sizeof(options));               // Clear all the options

	if(cfsetispeed(&options, get_tty_speed(tty_speed)) < 0 || cfsetospeed(&options,
				get_tty_speed(tty_speed)) < 0) {
		return -1;
	}

options.c_cflag = ( CLOCAL | CREAD |  CS8);    // Configure the device : 8 bits, no parity, no control
//options.c_iflag |= ( IGNPAR | IGNBRK );
//options.c_cc[VTIME]=0;                          // Timer unused
//options.c_cc[VMIN]=0;                           // At least on character before satisfy reading


//	fcntl(ret, F_SETFL, FNDELAY); 

//	options.c_cflag &= ~(CSIZE | PARENB);
//	options.c_cflag |= CS8;


//	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
//			INLCR | PARMRK | INPCK | ISTRIP | IXON);

//	options.c_oflag = 0;


//	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	cfmakeraw(&options);

	tcflush(ret, TCIFLUSH);
	if (tcsetattr(ret, TCSANOW, &options) !=0) {
		printf("Error configuring serial port!\n");
		return -1;
	}
	if (debug) printf("OK\n");
	return ret; 
}

void uart_close() {
	if (debug) printf("Closing serial\n");
	close(uart_fd);
}

void param_set_defaults(struct s_param *p) {
	//set any params in here
	p->bind_code = 2188;
	p->ch_num=5;
	p->ch[0]=0xb2;
	p->ch[1]=0xb3;
	p->ch[2]=0xb4;
	p->ch[3]=0xb5;
	p->ch[4]=0xb6;
	p->ppm_pin=PPM_SPONGE;
	p->mode=MODE_TX;
	p->buzzer=BUZZER_ON;
	p->rssi_ch=RSSI_AS_PWM;
	p->ppm_mode=PPM_MODE_NORMAL;

}

uint8_t param_from_config(struct s_param *p) {
	FILE *f;
	unsigned int val[27];
    int i=0;
    int rv;
    int num_values;

    f=fopen("params.txt","r");
    if (f==NULL){
        printf("file params.txt doesn't exist?!\n");
        return 0;
    }

    while (i < 27) {
        rv = fscanf(f, "%x%*[^\n]",&val[i]);

        if (rv != 1) {
        	printf("Error reading params file!");
            return 0;
        }

        printf("%u: %u\n",i,val[i]);

        i++;
    }
    fclose(f);

    p->ch_num=val[0];
    p->ch[0]=val[1];
	p->ch[1]=val[2];
	p->ch[2]=val[3];
	p->ch[3]=val[4];
	p->ch[4]=val[5];
	p->mode=val[6];
	for (i=0;i<16;i++) {
		p->servo_map[i]=val[i+7];
	}
	p->ppm_pin=val[23];
   	p->bind_code = val[24];
   	p->buzzer = val[25];
   	p->ppm_mode = val[26];

    return 1;

}

void param_init(struct s_param *param) {
	uint8_t i;
	memset(param,0,sizeof(struct s_param));
	param->mode = MODE_TX; //default to TX
	for (i=0;i<16;i++) param->servo_map[i]=i; //default servo mapping
}

void param_print(struct s_param *p) {
	uint8_t i;
	printf("\tNumber of channels: %u\n",p->ch_num);

	for (i=0;i<p->ch_num;i++) {
		printf("\tChannel %u: %u (%.3f MHz)\n",i,p->ch[i],432.f+p->ch[i]*0.15f);
	}

	if (p->mode==MODE_TX) printf("\tMode: TX\n");
	else if (p->mode==MODE_RX) printf("\tMode: RX\n");
	else printf("\tMode: Unknown\n");

	printf("\tServo mapping:\n");	
	for (i=0;i<16;i++) {
		printf("\t\t[%u] => [%u]\n",i+1,p->servo_map[i]+1);
	}

	if (p->ppm_pin==PPM_SPONGE) printf("\tPPM Pin: Sponge board/ULRS Mini\n");
	else if (p->ppm_pin==PPM_NORMAL) printf("\tPPM Pin: Normal pinout\n");
	else printf("\tPPM Pin: Unknown\n");

	printf("\tBind code: %u\n",p->bind_code);

	printf("\tRSSI channel: ");
	if (p->rssi_ch==RSSI_AS_PWM) printf("dedicated PWM\n");
	else printf("%i\n",p->rssi_ch);

	printf("\tBuzzer: ");
	if (p->buzzer==BUZZER_ON) printf("On\n");
	else if (p->buzzer==BUZZER_OFF) printf("Off\n");
	else printf("Unknown!\n");

	printf("\tPPM Mode (normal/inverted): ");
	if (p->ppm_mode==PPM_MODE_NORMAL) printf("Normal\n");
	else if (p->ppm_mode==PPM_MODE_INVERTED) printf("Inverted\n");
	else printf("Unknown!\n");
}

uint8_t param_serialize(struct s_param *param, uint8_t *target, uint8_t *len) {
	uint32_t crc;
	uint8_t _len = 0;
	memcpy(target,PREAMBLE,PREAMBLE_LEN);
	_len+=PREAMBLE_LEN;
	target[ID_OFFSET] = PARAM_ID;
	_len+=1;

	memcpy(target+5,&param->ch_num,1);
	_len+=1;
	memset(target+6,0x29,1); //0x29
	_len+=1;
	memcpy(target+7,param->ch,5);
	_len+=5;
	memcpy(target+12,&param->mode,1);
	_len+=1;
	memcpy(target+13,param->servo_map,16);
	_len+=16;
	memcpy(target+29,&param->ppm_pin,1);
	_len+=1;
	memset(target+30,0,16); //padding
	_len+=16;
	uint16_t v = htons(param->bind_code);
	memcpy(target+46,&v,2);
	_len+=2;

	crc = crc32(target,_len);
	memcpy(target+48,&crc,4);
	_len+=4;

	(*len) = _len;
	return _len;
}

uint8_t status_req_serialize(uint8_t *target, uint8_t *len) {
	uint8_t _len = 0;
	memcpy(target,PREAMBLE,PREAMBLE_LEN);
	_len+=PREAMBLE_LEN;

	target[_len] = STATUS_ID;
	_len+=1;

	memcpy(target+_len,STATUS_REQ_REMINDER,STATUS_REQ_REMINDER_LEN);
	_len+=STATUS_REQ_REMINDER_LEN;

	(*len) = _len;
	return _len;
}

uint8_t spectrum_req_serialize(uint8_t *target, uint8_t *len) {
	uint8_t _len = 0;
	memcpy(target,PREAMBLE,PREAMBLE_LEN);
	_len+=PREAMBLE_LEN;

	target[_len] = SPECTRUM_ID;
	_len+=1;

	memcpy(target+_len,SPECTRUM_REQ_REMINDER,SPECTRUM_REQ_REMINDER_LEN);
	_len+=SPECTRUM_REQ_REMINDER_LEN;

	(*len) = _len;
	return _len;
}

void print_hex(const uint8_t *arr,int len) {
	int i;
	for (i = 0; i < len; i ++) {
		printf("%02x ", arr[i] & 0xff);
	}
}


void print_usage() {
	printf("Usage: -p [uart_port] -s [speed] -m [mode]\n");
	printf("\tmode=0 (default) read status\n");
	printf("\tmode=1 read spectrum\n");
	printf("\tmode=2 set params from params.txt\n");
}

int set_defaults(int c, char **a) {
	int option;
	while ((option = getopt(c, a,"p:s:m:")) != -1) {
		switch (option)  {
			case 'p': strcpy((char*)uart_path,optarg); break;
			case 's': tty_speed = atoi(optarg); break;
			case 'm': mode = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
				  break;
		}
	}
	return 0;
} 

void request_status() {
	uint8_t ret;
	uint8_t buffer[16];
	status_req_serialize(buffer,&ret);
	printf("Requesting status...\n");
	if (uart_send(buffer,ret)) {
		printf("Error getting status\n");
		return;
	}
}



void request_spectrum() {
	uint8_t ret;
	uint8_t buffer[16];
	spectrum_req_serialize(buffer,&ret);
	printf("Requesting spectrum...\n");
	if (uart_send(buffer,ret)) {
		printf("Error getting spectrum\n");
		return;
	}
}

int get_packet(uint8_t *buffer, void (*ping)()) {
	uint8_t c = 0;
	uint8_t state = 0;
	uint16_t len = 0;
	uint16_t _len = 0; 
	uint32_t crc;
	int ret;

	uint8_t xx=0;

	while (!stop) {
		c++;
		ret = read(uart_fd,buffer+len,BUFFER_LEN-len);
		if (ret>0) {
			len+=ret;
			if (debug) printf("Got: %i\n",ret);
		} else if (ret<0 && errno == EAGAIN) {
		} else {
			perror("Error reading\n");
			return -1;
		}

		if ((state==0) && (len>=HEADER_LEN)) { //awaiting header
			if 	(strncmp((const char*)buffer,PREAMBLE,PREAMBLE_LEN)==0) {
				state++;
			} else {
				printf("Packet mismatch\n");
				len=0;
			}
		}

		if (state==1) { //wait for full package
			switch (buffer[ID_OFFSET]) {
				case STATUS_ID:
					if (len>=STATUS_LEN) {
						_len = STATUS_LEN;
						state++;
					}
					break;
				case SPECTRUM_ID:
					if (len>=SPECTRUM_LEN) {
						_len = SPECTRUM_LEN;
						state+=2; //SPECTRUM does not have crc
					}
					break;
				case PARAM_ID:
					if (len>=PARAM_RESPONSE_LEN) {
						_len = PARAM_RESPONSE_LEN;
						state+=2; //no crc
					}
					break;

				default:
					printf("Unknown packet: %u\n",buffer[ID_OFFSET]);
					print_hex(buffer,len);
					printf("\n\n");
					len=0;
					state=0;
			}
		}

		if (state==2) { //verify crc
			memcpy(&crc,buffer+_len-CRC_LEN,CRC_LEN);
			if (crc==crc32(buffer,_len-CRC_LEN)) state++;
			else {
				return 0;
			}
		}

		if (state==3) {
			return buffer[ID_OFFSET];
		}

		usleep(100000);
		if (c==10) { //send status request every 0.5s
			c=0;
			ping();
		}	
	}

	return 0;
}



int main(int argc, char *argv[])
{
	int ret;
	uint8_t len;
	struct s_param p;
	struct s_status s;
	struct s_spectrum a;

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	if (set_defaults(argc,argv)) return -1;

	//uart_fd = uart_open((const char*)uart_path, O_RDWR | O_NOCTTY | O_SYNC | O_NDELAY | O_NONBLOCK);
	uart_fd = uart_open((const char*)uart_path, O_RDWR | O_NOCTTY | O_NDELAY );
	if (uart_fd<0) {
		return -1;
	}

	printf("Testing connection... \n");
	ret = get_packet(buffer, request_status);
	if (stop) {
		uart_close();
		return -1;
	}
	if (debug) printf("OK (%i)\n\n",ret);

	if (mode==2) {
		param_init(&p);
		param_from_config(&p);
		param_serialize(&p,buffer,&len);

		printf("Sending params:\n");
		param_print(&p);

		if (uart_send(buffer,len)) {
			printf("Error writing to UART\n");
			uart_close();
			return -1;
		}
		printf("Sent. Waiting for response...\n");
	}

	if (mode==0 || mode==2) {
		do {
			ret = get_packet(buffer,request_status);
			if (ret==PARAM_ID) {
				printf("Param save ok. (%i)\n",ret);
				printf("REMEMBER to change serial speed if you changed the mode (TX<->RX). For RX use 19200, for TX 57600\n");
			}
			else if (ret==STATUS_ID) {
				printf("Got status (%i)\n",ret);
				status_parse(&s,buffer);
				status_print(&s);
				break;
			} else printf("Got: %i\n",ret);
		} while (!stop && ret!=-1);
	}

	if (mode==1) {
		do {
			ret = get_packet(buffer,request_spectrum);
			if (ret==SPECTRUM_ID) {
				printf("Got spectrum (%i)\n",ret);
				spectrum_parse(&a,buffer);
				spectrum_print(&a);
				break;
			} else printf("Got: %i\n",ret);
		} while (!stop && ret!=-1);
	}

	uart_close();

	printf("Bye.\n\n");
	return 0; 
}


