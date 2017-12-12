/* Copyright (C) 2005 6WIND <jean-mickael.guerin@6wind.com>
 * Copyright (C) 1999 Kunihiro Ishiguro
 *
 * This file was derived from bits of rtadv.c, part of of GNU Zebra.
 *
 * GNU Zebra is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * GNU Zebra is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Zebra; see the file COPYING.  If not, write to the Free
 * Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.  
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <netdb.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <linux/un.h>
#include <netinet/icmp6.h>
#include <netinet/in.h>
#include <netinet/ip6.h>

static void f_error(const char *x, ...)
{
	char msg[1024];
	va_list ap;

	va_start(ap, x);
	vsnprintf(msg, sizeof(msg), x, ap);
	va_end(ap);
	perror(msg);
	exit(EXIT_FAILURE);
}

void send_solicitation(int if_index)
{
    struct sockaddr_in6 addr;
    struct icmp6_hdr    message = { .icmp6_type = ND_ROUTER_SOLICIT };
    struct iovec iov;
    struct msghdr msg;
    struct cmsghdr *cmsgptr;
    struct in6_pktinfo *pkt;
    int    zero = 0;
    int    one = 1;
    int    two55 = 255;
    u_char all_nodes_addr[] = { 0xff,0x02,0,0,0,0,0,0,0,0,0,0,0,0,0,2 };    
    char   adata[CMSG_SPACE(sizeof(struct in6_pktinfo))];
    int fd;

    memset(&addr, 0, sizeof(addr));
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(IPPROTO_ICMPV6);
    memcpy(&addr.sin6_addr, all_nodes_addr, sizeof(addr.sin6_addr));

    fd = socket(AF_INET6, SOCK_RAW, IPPROTO_ICMPV6);
    if (fd < 0)
	f_error("socket");

    if (setsockopt(fd, IPPROTO_IPV6, IPV6_MULTICAST_IF, &if_index,
		   sizeof(if_index)) < 0)
    if (setsockopt(fd, IPPROTO_IPV6, IPV6_PKTINFO, &one,
		   sizeof(one)) < 0)
    if (setsockopt(fd, IPPROTO_IPV6, IPV6_MULTICAST_LOOP, &zero,
		   sizeof(zero)) < 0)
	f_error("setsockopt IPV6_MULTICAST_LOOP");
    if (setsockopt(fd, IPPROTO_IPV6, IPV6_UNICAST_HOPS, &two55, sizeof(two55)) < 0)
	f_error("setsockopt IPV6_UNICAST_HOPS");
    if (setsockopt(fd, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &two55, sizeof(two55)) < 0)
	f_error("setsockopt IPV6_MULTICAST_HOPS");

    memset(adata, 0, sizeof(adata));

    memset(&iov, 0, sizeof(iov));
    iov.iov_base = &message;
    iov.iov_len = sizeof(message);

    memset(&msg, 0, sizeof(msg));
    msg.msg_name = (void *) &addr;
    msg.msg_namelen = sizeof(addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = (void *) adata;
    msg.msg_controllen = sizeof(adata);
    msg.msg_flags = 0;

    cmsgptr = CMSG_FIRSTHDR(&msg);
    cmsgptr->cmsg_len = CMSG_LEN(sizeof(struct in6_pktinfo));
    cmsgptr->cmsg_level = IPPROTO_IPV6;
    cmsgptr->cmsg_type = IPV6_PKTINFO;

    pkt = (struct in6_pktinfo *) CMSG_DATA(cmsgptr);
    memset (&pkt->ipi6_addr, 0, sizeof(struct in6_addr));
    pkt->ipi6_ifindex = if_index;

    if (sendmsg(fd, &msg, 0) < 0) {
	perror("sendmsg");
	exit(1);
    }
    close(fd);

}

struct option opts[] = {
	{ "help",  0, NULL, 'h' },
	{ NULL,    0, NULL, 0 }
};

void usage(const char *mesg)
{
    if (mesg)
	fputs(mesg, stderr);
    fprintf(stderr, "solicitation [options] ifname\n"
	    "  options:\n"
	    "    -h|--help             Show this usage message\n");
}

int main(int argc, char *argv[])
{
    int c;

    while ((c = getopt_long(argc, argv, "h", opts, NULL)) != -1) {
	switch (c) {
	case 'h':
	    usage(NULL);
	    exit(0);
	default:
	    usage(NULL);
	    exit(1);
	}
    }

    if (optind >= argc) {
	usage("Missing interface name");
	exit(1);
    }
    send_solicitation(if_nametoindex(argv[optind]));
    return 0;
}
