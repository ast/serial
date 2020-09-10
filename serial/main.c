//
//  main.c
//  serial
//
//  Created by Albin Stigö on 2019-11-10.
//  Copyright © 2019 Albin Stigo. All rights reserved.
//

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;
    
    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }
    
    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);
    
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */
    
    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    
    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;
    
    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }
    
    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */
    
    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


int main()
{
    char *portname = "/dev/cu.usbserial-A50285BI";
    int fd;
    
    // O_NOCTTY = don't make controlling terminal
    // O_SYNC = data is transfered to hardware before return
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */
    
    /* simple output
     wlen = write(fd, "Hello!\n", 7);
     if (wlen != 7) {
     printf("Error from write: %d, %d\n", wlen, errno);
     }
     tcdrain(fd);    /* delay for output */
    
    /* simple noncanonical input */
    while(true) {
        uint8_t buf[200];
        ssize_t rdlen;
        
        rdlen = read(fd, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
            
            //buf[rdlen] = 0;
            //printf("Read %ld: \"%s\"\n", rdlen, buf);
            
            printf("Read %ld:", rdlen);
            for (int i = 0; i < rdlen; i++) {
                printf(" 0x%x", buf[i]);
            }
            printf("\n");
            
            uint8_t chksum = 0;
            for (int i = 0; i < rdlen-1; i++) {
                chksum += buf[i];
            }
            printf("chksum: 0x%x\n", chksum);
            
        } else if (rdlen < 0) {
            printf("Error from read: %ld: %s\n", rdlen, strerror(errno));
        } else {  /* rdlen == 0 */
            printf("Timeout from read\n");
        }
        /* repeat read to get full message */
    }
}
