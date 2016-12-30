/*
 * Copyright (c) 2016, CESAR.
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license. See the LICENSE file for details.
 *
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* FIXME: Remove this header */
#include "include/nrf24.h"

#include "include/comm.h"

struct serial_opts {
	char	tty[24];
	int	vmin;
	int	virtualfd;
	int	realfd;
	int	knotd;
};

static struct serial_opts serial_opts;

int hal_comm_init(const char *pathname, const struct nrf24_mac *mac)
{
	struct stat st;
	/* Setting default value */
	if (!pathname)
		return -EINVAL;

	strncpy(serial_opts.tty, pathname, sizeof(serial_opts.tty));
	serial_opts.vmin = 8;

	if (stat(serial_opts.tty, &st) < 0)
		return -errno;
	return serial_opts.tty[strlen(serial_opts.tty)-1] - '0';
}
int hal_comm_deinit(void)
{
	return -ENOSYS;
}

int hal_comm_socket(int domain, int protocol)
{
	return -ENOSYS;
}

int hal_comm_close(int sockfd)
{
	return close(sockfd);
}
/* Non-blocking read operation. Returns -EGAIN if there isn't data available */
ssize_t hal_comm_read(int sockfd, void *buffer, size_t count)
{
	return read(sockfd, buffer, count);
}

/* Blocking write operation. Returns -EBADF if not connected */
ssize_t hal_comm_write(int sockfd, const void *buffer, size_t count)
{
	return write(sockfd, buffer, count);
}

int hal_comm_listen(int sockfd)
{
	struct termios term;
	int ttyfd;

	memset(&term, 0, sizeof(term));
	/*
	 * 8-bit characters, no parity bit,no Bit mask for data bits
	 * only need 1 stop bit
	 */
	term.c_cflag &= ~PARENB;
	term.c_cflag &= ~CSTOPB;
	term.c_cflag &= ~CSIZE;
	term.c_cflag |= CS8;
	/* No flow control*/
	term.c_cflag &= ~CRTSCTS;
	/* Read block until 2 bytes arrives */
	term.c_cc[VMIN] = 2;
	/* Or 0.075 seconds read timeout */
	term.c_cc[VTIME] = 0.75;
	/* Turn on READ & ignore ctrl lines */
	term.c_cflag |= CREAD | CLOCAL;

	cfsetospeed(&term, B9600);
	cfsetispeed(&term, B9600);

	ttyfd = open(serial_opts.tty, O_RDWR | O_NOCTTY);
	if (ttyfd < 0)
		return -errno;
	tcflush(ttyfd, TCIFLUSH);
	tcsetattr(ttyfd, TCSANOW, &term);

	return ttyfd;
}

/* Non-blocking operation. Returns -EGAIN if there isn't a new client */
int hal_comm_accept(int sockfd, uint64_t *addr)
{
	return -ENOSYS;
}

/* Blocking operation. Returns -ETIMEOUT */
int hal_comm_connect(int sockfd, uint64_t *addr)
{
	return -ENOSYS;
}
