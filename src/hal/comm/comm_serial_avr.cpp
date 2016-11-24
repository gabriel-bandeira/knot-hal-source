/*
 * Copyright (c) 2016, CESAR.
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license. See the LICENSE file for details.
 *
 */
#include <Arduino.h>

#include <avr_unistd.h>
#include <avr_errno.h>

/* FIXME: Remove this header */
#include "include/nrf24.h"

#include "comm.h"

#define SERIAL_DATA_RATE 9600

int hal_comm_init(const char *pathname, const struct nrf24_mac *mac)
{
	if (strcmp(pathname, "serial0") == 0)
		Serial.begin(SERIAL_DATA_RATE);
	else
		return -ENOSYS;
	return 0;
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
	Serial.end();
	return 0;
}

/* Non-blocking read operation. Returns -EGAIN if there isn't data available */
ssize_t hal_comm_read(int sockfd, void *buffer, size_t count)
{
	int available = Serial.available();
	if (!available)
		return -EAGAIN;
	ssize_t index = 0;
	char* tempBuffer = (char*)buffer;
	for (index = 0; index < available && index < count; index++)
		tempBuffer[index] = Serial.read();
	return index;
}

/* Blocking write operation. Returns -EBADFD if not connected */
ssize_t hal_comm_write(int sockfd, const void *buffer, size_t count)
{
	if (!Serial)
		return -EBADFD;
	return Serial.write((char*)buffer, count);
}

int hal_comm_listen(int sockfd)
{
	return 0;
}

/* Non-blocking operation. Returns -EGAIN if there isn't a new client */
int hal_comm_accept(int sockfd, uint64_t *addr)
{
	return 0;
}

/* Blocking operation. Returns -ETIMEOUT */
int hal_comm_connect(int sockfd, uint64_t *addr)
{
	return -ENOSYS;
}
