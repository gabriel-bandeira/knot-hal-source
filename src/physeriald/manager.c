/*
 * Copyright (c) 2016, CESAR.
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license. See the LICENSE file for details.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <glib.h>

#include "include/nrf24.h"
#include "include/comm.h"
#include "manager.h"

/* Application packet size maximum, same as knotd */
#define PACKET_SIZE_MAX			512
#define KNOTD_UNIX_ADDRESS		"knot"
#define THING_TO_PHYEMUD_UNIX_SOCKET	":thing:phyemud"

#define KNOT_SUCCESS				0
#define KNOT_PROTOCOL_TOKEN_LEN			40
#define KNOT_PROTOCOL_UUID_LEN			36
#define KNOT_MSG_REGISTER_REQ			0x10

static int commfd;

struct session {
	unsigned int thing_id;	/* Thing event source */
	unsigned int knotd_id;	/* KNoT event source */
	GIOChannel *knotd_io;	/* Knotd GIOChannel reference */
	GIOChannel *thing_io;	/* Knotd GIOChannel reference */
};

typedef struct __attribute__ ((packed)) {
	uint8_t			type;
	uint8_t			payload_len;
} knot_msg_header;

typedef struct __attribute__ ((packed)) {
	knot_msg_header		hdr;
	int8_t			result;
	char			uuid[KNOT_PROTOCOL_UUID_LEN];
	char			token[KNOT_PROTOCOL_TOKEN_LEN];
} knot_msg_credential; // hdr + 40 + 36 bytes

static int connect_unix(void)
{
	struct sockaddr_un addr;
	int sock;

	sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	if (sock < 0)
		return -errno;

	/* Represents unix socket from seriald to knotd */
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path + 1, KNOTD_UNIX_ADDRESS,
					strlen(KNOTD_UNIX_ADDRESS));

	if (connect(sock, (struct sockaddr *) &addr, sizeof(addr)) == -1)
		return -errno;

	return sock;
}

static void knotd_io_destroy(gpointer user_data)
{
	struct session *session = user_data;

	printf("knotd_io_destroy\n\r");
	session->knotd_id = 0;
	session->knotd_io = NULL;
}

static gboolean knotd_io_watch(GIOChannel *io, GIOCondition cond,
							gpointer user_data)
{
	struct session *session = user_data;
	char buffer[PACKET_SIZE_MAX];
	int thing_sock, knotd_sock;
	ssize_t readbytes_knotd;

	if (cond & (G_IO_ERR | G_IO_HUP | G_IO_NVAL)) {
		printf("Wrong cond\n\r");
		return FALSE;
	}

	printf("Setting sockets\n\r");
	knotd_sock = g_io_channel_unix_get_fd(io);
	thing_sock = g_io_channel_unix_get_fd(session->thing_io);

	printf("Incoming data from knotd (%d)\n\r", knotd_sock);

	readbytes_knotd = read(knotd_sock, buffer, sizeof(buffer));
	if (readbytes_knotd < 0) {
		printf("read_knotd() error\n\r");
		return FALSE;
	}
	printf("RX_KNOTD: '%ld'\n\r", readbytes_knotd);

	if (hal_comm_write(thing_sock, buffer, readbytes_knotd) < 0) {
		printf("send_thing() error\n\r");
		return FALSE;
	}

	return TRUE;
}
/* If thing initiated disconnection decrement ref count */
static void generic_io_destroy(gpointer user_data)
{
	struct session *session = user_data;
	printf("generic_io_destroy\n\r");

	if (session->thing_id > 0) {
		g_io_channel_shutdown(session->thing_io, FALSE, NULL);
		g_io_channel_unref(session->thing_io);
	}
}

/*static void sendAuth(GIOChannel *io)
{
	knot_msg_credential msg_c;
	knot_msg_header msg_h;
	int sock;
//typedef struct __attribute__ ((packed)) {
//  uint8_t     type = KNOT_MSG_REGISTER_REQ;
//  uint8_t     payload_len = size_of(knot_msg_credential);
//} knot_msg_header;
	msg_h.type = KNOT_MSG_REGISTER_REQ;
	msg_h.payload_len = sizeof(knot_msg_credential) - sizeof(knot_msg_header);

//typedef struct __attribute__ ((packed)) {
//  knot_msg_header   hdr;
//  int8_t      result = KNOT_SUCCESS;
//  char      uuid[KNOT_PROTOCOL_UUID_LEN] = "4aa3f51b-185e-46c0-ba60-be274cd90000";
//  char      token[KNOT_PROTOCOL_TOKEN_LEN] = "4d482d9a1c338c385c674ff08a6867d171d5d878";
//} knot_msg_credential; // hdr + 40 + 36 bytes
	msg_c.hdr = msg_h;
	msg_c.result = KNOT_SUCCESS;
	msg_c.uuid[0] = '4';
	msg_c.uuid[1] = 'a';
	msg_c.uuid[2] = 'a';
	msg_c.uuid[3] = '3';
	msg_c.uuid[4] = 'f';
	msg_c.uuid[5] = '5';
	msg_c.uuid[6] = '1';
	msg_c.uuid[7] = 'b';
	msg_c.uuid[8] = '-';
	msg_c.uuid[9] = '1';
	msg_c.uuid[10] = '8';
	msg_c.uuid[11] = '5';
	msg_c.uuid[12] = 'e';
	msg_c.uuid[13] = '-';
	msg_c.uuid[14] = '4';
	msg_c.uuid[15] = '6';
	msg_c.uuid[16] = 'c';
	msg_c.uuid[17] = '0';
	msg_c.uuid[18] = '-';
	msg_c.uuid[19] = 'b';
	msg_c.uuid[20] = 'a';
	msg_c.uuid[21] = '6';
	msg_c.uuid[22] = '0';
	msg_c.uuid[23] = '-';
	msg_c.uuid[24] = 'b';
	msg_c.uuid[25] = 'e';
	msg_c.uuid[26] = '2';
	msg_c.uuid[27] = '7';
	msg_c.uuid[28] = '4';
	msg_c.uuid[29] = 'c';
	msg_c.uuid[30] = 'd';
	msg_c.uuid[31] = '9';
	msg_c.uuid[32] = '0';
	msg_c.uuid[33] = '0';
	msg_c.uuid[34] = '0';
	msg_c.uuid[35] = '0';
	msg_c.token[0] = '4';
	msg_c.token[1] = 'd';
	msg_c.token[2] = '4';
	msg_c.token[3] = '8';
	msg_c.token[4] = '2';
	msg_c.token[5] = 'd';
	msg_c.token[6] = '9';
	msg_c.token[7] = 'a';
	msg_c.token[8] = '1';
	msg_c.token[9] = 'c';
	msg_c.token[10] = '3';
	msg_c.token[11] = '3';
	msg_c.token[12] = '8';
	msg_c.token[13] = 'c';
	msg_c.token[14] = '3';
	msg_c.token[15] = '8';
	msg_c.token[16] = '5';
	msg_c.token[17] = 'c';
	msg_c.token[18] = '6';
	msg_c.token[19] = '7';
	msg_c.token[20] = '4';
	msg_c.token[21] = 'f';
	msg_c.token[22] = 'f';
	msg_c.token[23] = '0';
	msg_c.token[24] = '8';
	msg_c.token[25] = 'a';
	msg_c.token[26] = '6';
	msg_c.token[27] = '8';
	msg_c.token[28] = '6';
	msg_c.token[29] = '7';
	msg_c.token[30] = 'd';
	msg_c.token[31] = '1';
	msg_c.token[32] = '7';
	msg_c.token[33] = '1';
	msg_c.token[34] = 'd';
	msg_c.token[35] = '5';
	msg_c.token[36] = 'd';
	msg_c.token[37] = '8';
	msg_c.token[38] = '7';
	msg_c.token[39] = '8';

	sock = g_io_channel_unix_get_fd(io);
	if (hal_comm_write(sock, (void*)&msg_c, sizeof(msg_c)) < 0) {
		printf("write_knotd() error\n\r");
	}
}*/

static gboolean generic_io_watch(GIOChannel *io, GIOCondition cond,
							gpointer user_data)
{
	struct session *session = user_data;
	char buffer[PACKET_SIZE_MAX];
	ssize_t nbytes;
	int sock, knotdfd, offset, msg_size, err, remaining;
	ssize_t i;

	//printf("watch 0\n\r");
	if (cond & (G_IO_ERR | G_IO_HUP | G_IO_NVAL)) {
		session->thing_id = 0;
		return FALSE;
	}

	//printf("watch 1\n\r");
	sock = g_io_channel_unix_get_fd(io);

	printf("Generic IO Watch, reading from (%d)\n\r", sock);

	memset(&buffer, 0, sizeof(buffer));
	nbytes = hal_comm_read(sock, (void*)buffer, sizeof(buffer));
	if (nbytes < 0) {
		printf("read() error\n");
		return FALSE;
	}
	printf("Read (%zu) bytes from thing\n\r", nbytes);

	printf("Opt type = (0x%02X), Payload length = (%d)\n", buffer[0],
								buffer[1]);

	//printf("Read %s from thing.\n\r", buffer+2);
	printf("Read \"");
	for(i = 0; i < nbytes; i++)
		printf("%c ", buffer[(int)i]);
	printf("\" from thing\n\r");
	/*
	 * At the moment there isn't a header describing the size of
	 * the datagram. The field 'payload_len' (see buffer[1]) defined
	 * at knot_protocol.h is being used to determine the expected
	 * datagram length.
	 */
	msg_size = buffer[1] + 2;

	offset = (int)nbytes;
	/* If payload + header (2 Bytes) < nbytes, keep reading */
	while (offset < msg_size) {
		remaining = sizeof(buffer) - offset;
		if (remaining > 0)
			nbytes = hal_comm_read(sock, buffer + offset,
								remaining);
		else
			goto done;
		err = errno;
		/* Only consider nbytes < 0 when errno is not EAGAIN*/
		if (nbytes < 0 && err != EAGAIN) {
			/* Malformed datagram: ignore received data */
			goto done;
		} else if (nbytes > 0)
			offset += nbytes;
	}

	printf("Total bytes read = %d\n", offset);
	//printf("Read %s from thing.\n\r", buffer);
	printf("Read \"");
	for(i = 0; i < (ssize_t)offset; i++)
		printf("%c ", buffer[(int)i]);
	printf("\" from thing\n\r");

	knotdfd = g_io_channel_unix_get_fd(session->knotd_io);

	//sendAuth(io);
	if (write(knotdfd, buffer, msg_size) < 0) {
		printf("write_knotd() error\n\r");
		return FALSE;
	}

done:

	return TRUE;
}

static int serial_start(const char *pathname)
{
	GIOCondition cond = G_IO_IN | G_IO_ERR | G_IO_HUP | G_IO_NVAL;
	struct session *session;
	GIOChannel *io;
	int knotdfd;
	//knot_msg_header msg_h;
	//knot_msg_credential msg_c;

	commfd = hal_comm_init(pathname, NULL);
	printf("commfd = (%d)\n\r", commfd);
	if (commfd < 0) {
		hal_comm_close(commfd);
		return -errno;
	}

	knotdfd = connect_unix();
	printf("knotdfd = (%d)\n\r", knotdfd);
	if (knotdfd < 0) {
		return FALSE;
	}

	printf("Serial server started\n\r");

	/* Tracking thing connection & data */
	io = g_io_channel_unix_new(commfd);
	g_io_channel_set_flags(io, G_IO_FLAG_NONBLOCK, NULL);
	g_io_channel_set_close_on_unref(io, TRUE);

	printf("serial_start 0\n\r");

	session = g_new0(struct session, 1);

	printf("serial_start 1\n\r");
	/* Watch knotd socket */
	session->knotd_io = g_io_channel_unix_new(knotdfd);
	g_io_channel_set_flags(session->knotd_io, G_IO_FLAG_NONBLOCK, NULL);
	g_io_channel_set_close_on_unref(session->knotd_io, TRUE);

	printf("serial_start 2\n\r");
	session->knotd_id = g_io_add_watch_full(session->knotd_io,
							G_PRIORITY_DEFAULT,
							cond,
							knotd_io_watch, session,
							knotd_io_destroy);
	printf("serial_start 3\n\r");
	g_io_channel_unref(session->knotd_io);

	session->thing_io = io;

	session->thing_id = g_io_add_watch_full(io, G_PRIORITY_DEFAULT,
						cond, generic_io_watch, session,
							generic_io_destroy);
	g_io_channel_unref(io);

	//sendAuth(io);
	printf("Tudo certo\n\r");

	return 0;
}

static void serial_stop(void)
{
	printf("serial_stop\n\r");
	hal_comm_close(commfd);
}

int manager_start(const char *serial)
{
	if (serial)
		return serial_start(serial);

	return 0;
}

void manager_stop(void)
{
	serial_stop();

	printf("Manager stop\n");
}
