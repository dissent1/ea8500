/*
 * Copyright (c) 2011, Cisco Systems, Inc.
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#define __USE_GNU
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/shm.h>
#include <errno.h>
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
#include <time.h>
#endif
#include <mqueue.h>
#include <libnetfilter_queue/libnetfilter_queue.h>
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
#include <libnetfilter_queue/linux_nfnetlink_queue.h>
#endif
#include <arpa/inet.h>
#include <linux/netfilter.h>            /* for NF_ACCEPT */
#include <linux/ip.h> /* for iphdr */
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
#include <linux/igmp.h> /* for igmphdr */
#endif

#define MAX_QUEUE_HANDLER			10
#define MAX_IP_SIZE					65535
#define TRIGGER_MASK 				0x0000007f		
#define PAGE_SIZE					getpagesize()
#define ROUNDUP(_size, _align)		((((_size) + (_align))/(_align))*(_align))

// Kernel sets the attributes for the packet. The maximum attributes size can be calculated from the linux kernel source code.
// For 2.6.31.8, it is 64 bytes.
#if 0	// linux/net/netfilter/nfnetfilter_queue.c
	...
	static struct sk_buff *nfqnl_build_packet_message(struct nfqnl_instance *queue, struct nf_queue_entry *entry)
	{
		...
		size =	  NLMSG_SPACE(sizeof(struct nfgenmsg))				// 4 bytes
			+ nla_total_size(sizeof(struct nfqnl_msg_packet_hdr))		// 8 bytes
			+ nla_total_size(sizeof(u_int32_t)) /* ifindex */			// 4 bytes
			+ nla_total_size(sizeof(u_int32_t)) /* ifindex */			// 4 bytes
#ifdef CONFIG_BRIDGE_NETFILTER
			+ nla_total_size(sizeof(u_int32_t)) /* ifindex */			// 4 bytes
			+ nla_total_size(sizeof(u_int32_t)) /* ifindex */			// 4 bytes
#endif
			+ nla_total_size(sizeof(u_int32_t)) /* mark */			// 4 bytes
			+ nla_total_size(sizeof(struct nfqnl_msg_packet_hw))			// 12 bytes
			+ nla_total_size(sizeof(struct nfqnl_msg_packet_timestamp));	// 16 bytes

		...
		nlh = NLMSG_PUT(skb, 0, 0,
				NFNL_SUBSYS_QUEUE << 8 | NFQNL_MSG_PACKET,
				sizeof(struct nfgenmsg));							// 4 bytes
		...
	}
	...
#endif
#define ATTR_SIZE					100

#define PC_MASK						    0x1f000000
#define SET_PC_MARK(_mark, _value)		_mark = (_mark & ~PC_MASK) | (_value & PC_MASK)

// TODO: Read queue numbers and mark values from command line arguments. For now, just use the default values.
#define DEFAULT_MARK_FORWARD_DNS			(0x11 << 24)
#define DEFAULT_MARK_PASS					(0x12 << 24)
#define DEFAULT_MARK_BLOCK					(0x13 << 24)
#define DEFAULT_MARK_CHECK					(0x14 << 24)
#define DEFAULT_MARK_DONE					(0x10 << 24)

#define DEFAULT_QUEUE_NUM_HTTP_REQUEST		10
#define DEFAULT_QUEUE_NUM_DNS_QUERY			11
#define DEFAULT_QUEUE_NUM_DNS_RESPONSE		12
#define DEFAULT_QUEUE_NUM_TCP_SYN_WWW		13
#ifdef CONFIG_BUILD_AUTO_CONFIG
#define DEFAULT_QUEUE_NUM_SETUP_HTTP		14
#define DEFAULT_QUEUE_NUM_SETUP_NON_HTTP	15
#endif
#define DEFAULT_QUEUE_NUM_TRIGGER			22
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
#define DEFAULT_QUEUE_NUM_IGMP			    30
#endif

static unsigned int mark_pass = DEFAULT_MARK_PASS;
static unsigned int mark_block = DEFAULT_MARK_BLOCK;
static unsigned int mark_check = DEFAULT_MARK_CHECK;
static unsigned int mark_forward = DEFAULT_MARK_FORWARD_DNS;

static int queue_num_http_request = DEFAULT_QUEUE_NUM_HTTP_REQUEST;
static int queue_num_dns_query = DEFAULT_QUEUE_NUM_DNS_QUERY;
static int queue_num_dns_response = DEFAULT_QUEUE_NUM_DNS_RESPONSE;
static int queue_num_tcp_syn_www = DEFAULT_QUEUE_NUM_TCP_SYN_WWW;
#ifdef CONFIG_BUILD_AUTO_CONFIG
static int queue_num_setup_http = DEFAULT_QUEUE_NUM_SETUP_HTTP;
static int queue_num_setup_non_http = DEFAULT_QUEUE_NUM_SETUP_NON_HTTP;
#endif
static int queue_num_trigger = DEFAULT_QUEUE_NUM_TRIGGER;
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
static int queue_num_igmp = DEFAULT_QUEUE_NUM_IGMP;
#endif
////

typedef enum {
	MODULE_UNKNOWN = 0,
	MODULE_PARENTAL_CONTROL,
	MODULE_TRIGGER,
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
    MODULE_IGMP,
#endif
} module_t;

//
// Interface to data processing server.
//

#define IPC_KEY				0x4321
#define IPC_MODE			0660

#define SHM_KEY				IPC_KEY
#define SHM_MODE			IPC_MODE
#define SHM_SZ				ROUNDUP(MAX_IP_SIZE + ATTR_SIZE, PAGE_SIZE)

#define PC_MSG_KEY			IPC_KEY
#define PC_MSG_MODE			IPC_MODE
#define PC_MSG_TYPE_REQUEST		1
#define PC_MSG_TYPE_RESPONSE	2

#define TR_MSG_NAME			"/trigger_mq"
#define TR_MSG_MODE			IPC_MODE
#define TR_MSG_TYPE			1
#define TR_MSG_MAXMSG			100

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
#define IGMP_MSG_MAXMSG 100
#define IGMP_MSG_NAME "/igmp_mq"
#define IGMP_MSG_MODE IPC_MODE
#endif

typedef enum {
	PC_REQUEST_UNKNOWN = 0,
	PC_REQUEST_DNS_QUERY,
	PC_REQUEST_DNS_RESPONSE,
	PC_REQUEST_HTTP_REQUEST,
	PC_REQUEST_TCP_SYN_WWW,
#ifdef CONFIG_BUILD_AUTO_CONFIG
	PC_REQUEST_SETUP_HTTP,
	PC_REQUEST_SETUP_NON_HTTP,
#endif
} pc_request_t;

typedef enum {
	PC_RESPONSE_UNKNOWN = 0,
	PC_RESPONSE_PASS,
	PC_RESPONSE_BLOCK,
	PC_RESPONSE_CHECK,
	PC_RESPONSE_DONE,
	PC_RESPONSE_FORWARD,
} pc_response_t;

typedef struct {
	int type;
	pc_request_t request;
	int data_offset;
	int data_len;
} msg_pc_request_t;

typedef struct {
	int type;
	pc_response_t response;
} msg_pc_response_t;

typedef struct {
	int id;
	struct in_addr ip_addr;
} msg_trigger_t;

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
typedef struct {
	unsigned char type;
	unsigned int group_address;
	unsigned char smac[6];
} msg_igmp_t;
#endif

////

typedef struct {
	module_t module;
	pc_request_t pc_request;
} callback_args_t;

typedef struct {
	struct nfq_q_handle *qh;
	int num;
	callback_args_t args;
	char *comment;
} queue_handler_t;

static int pc_queueId;
static mqd_t tr_queueId;

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
static mqd_t igmp_queueId;
#endif

static int shmId;
static char *shmBase;
static int option_verbose = 0;			// TODO: Turn it on by command line option(-v).

#define VERBOSE(_fmt, _args...)			fprintf(stderr, _fmt, ## _args);
#define VERBOSE_IF(_fmt, _args...)		if (option_verbose) { VERBOSE(_fmt, ## _args); }

#if defined(DEBUG)
#define INFO(_fmt, _args ...)			VERBOSE_IF(_fmt, ## _args)
#define ERROR(_fmt, _args ...)			VERBOSE(_fmt, ## _args)
#define DEBUG0(_fmt, _args...)			VERBOSE_IF(_fmt, ## _args)

#else
#define INFO(_fmt, _args ...)			VERBOSE_IF(_fmt, ## _args)
#define ERROR(_fmt, _args ...)			VERBOSE(_fmt, ## _args)
#define DEBUG0(_fmt, _args...)
#endif

/*
 * Procedure	: callback
 * Purpose	:
 * Parameters	: 
 * Return		:
 * Notes		:
 */
static int callback(struct nfq_q_handle *qh, struct nfgenmsg *nfmsg, struct nfq_data *nfa, void *data);

/*
 * Procedure	: pc_process
 * Purpose	:
 * Parameters	: 
 * Return		:
 * Notes		:
 */
static pc_response_t pc_process(pc_request_t request, char *data, int data_len);

/*
 * Procedure	: tr_process
 * Purpose	:
 * Parameters	: 
 * Return		:
 * Notes		:
 */
static int tr_process(int id, struct in_addr ip_addr);

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
static int igmp_process(unsigned char *smac, unsigned int group_address, unsigned char type);
#endif

/*
 * Procedure	: sig_handler
 * Purpose	: Handle a signal.
 * Parameters	: 
 *	signo	[IN]	signal number.
 * Return		: <none>
 * Notes		: <none>
 */
static void sig_handler(int signo);

/*
 * Procedure	: printmsg
 * Purpose	: Print out message.
 * Parameters	: 
 *	msg		[IN] message
 * Return		:
 * Notes		: signal safe function.
 */
static void printmsg(char *msg);

int main(int argc, char **argv)
{
	struct nfq_handle *h = NULL;
	queue_handler_t qh_tbl[MAX_QUEUE_HANDLER];
	int n_handler = 0;
	int i;
	int rv;
	void *qbuf = NULL;
	int qbuf_sz = 0;
	int fd;
	struct sigaction action;
	msg_pc_response_t rcvmsg;
	struct mq_attr tr_attr; 
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
	struct mq_attr igmp_attr;
#endif

	(void)argc;
	
	INFO("starting %s built at %s %s\n", argv[0], __DATE__, __TIME__);

	//
	// Set up signals.
	//
	action.sa_handler = sig_handler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = 0;
	
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGSEGV, &action, NULL);

	action.sa_handler = sig_handler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = SA_RESTART;
	
	sigaction(SIGALRM, &action, NULL);

	// Set up message queue for parental control.
	if ((pc_queueId = msgget(PC_MSG_KEY, IPC_CREAT | PC_MSG_MODE)) == -1) {
		ERROR("msgget() failed: %s\n", strerror(errno));
		goto __exit;
	}
	while (msgrcv(pc_queueId, &rcvmsg, sizeof(rcvmsg)-sizeof(long), PC_MSG_TYPE_RESPONSE, IPC_NOWAIT) > 0);	// Clear pending response messages.

	// Set up POSIX message queue for trigger.
	tr_attr.mq_maxmsg = TR_MSG_MAXMSG; 
	tr_attr.mq_msgsize = sizeof(msg_trigger_t);
	if ((tr_queueId = mq_open(TR_MSG_NAME, O_WRONLY | O_CREAT, TR_MSG_MODE, &tr_attr)) == -1) {
		ERROR("mq_open() failed: %s\n", strerror(errno));
		goto __exit;
	}

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
    igmp_attr.mq_maxmsg = IGMP_MSG_MAXMSG; 
	igmp_attr.mq_msgsize = sizeof(msg_igmp_t);
	if ((igmp_queueId = mq_open(IGMP_MSG_NAME, O_WRONLY | O_CREAT, IGMP_MSG_MODE, &igmp_attr)) == -1) {
		ERROR("mq_open() failed: %s\n", strerror(errno));
		goto __exit;
	}
#endif

	// Set up shared memory.
	if ((shmId = shmget(SHM_KEY, SHM_SZ, IPC_CREAT | SHM_MODE)) == -1) {
		ERROR("Failed to shmget() - %s\n", strerror(errno));
		goto __exit;
	}
	else if ((shmBase = (char *)shmat(shmId, 0, 0)) == (char *)-1 ) {
		ERROR("Failed to shmat() - %s\n", strerror(errno));
		goto __exit;
	}

	// Set up netfilter queue.
	n_handler = 0;
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_http_request;
		qh_tbl[n_handler].args.module = MODULE_PARENTAL_CONTROL;
		qh_tbl[n_handler].args.pc_request = PC_REQUEST_HTTP_REQUEST;
		qh_tbl[n_handler].comment = "HTTP REQUEST";
		n_handler++;
	}
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_dns_query;
		qh_tbl[n_handler].args.module = MODULE_PARENTAL_CONTROL;
		qh_tbl[n_handler].args.pc_request = PC_REQUEST_DNS_QUERY;
		qh_tbl[n_handler].comment = "DNS QUERY";
		n_handler++;
	}
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_dns_response;
		qh_tbl[n_handler].args.module = MODULE_PARENTAL_CONTROL;
		qh_tbl[n_handler].args.pc_request = PC_REQUEST_DNS_RESPONSE;
		qh_tbl[n_handler].comment = "DNS RESPONSE";
		n_handler++;
	}
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_tcp_syn_www;
		qh_tbl[n_handler].args.module = MODULE_PARENTAL_CONTROL;
		qh_tbl[n_handler].args.pc_request = PC_REQUEST_TCP_SYN_WWW;
		qh_tbl[n_handler].comment = "TCP SYN WWW";
		n_handler++;
	}
#ifdef CONFIG_BUILD_AUTO_CONFIG
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_setup_http;
		qh_tbl[n_handler].args.module = MODULE_PARENTAL_CONTROL;
		qh_tbl[n_handler].args.pc_request = PC_REQUEST_SETUP_HTTP;
		qh_tbl[n_handler].comment = "SETUP HTTP";
		n_handler++;
	}
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_setup_non_http;
		qh_tbl[n_handler].args.module = MODULE_PARENTAL_CONTROL;
		qh_tbl[n_handler].args.pc_request = PC_REQUEST_SETUP_NON_HTTP;
		qh_tbl[n_handler].comment = "SETUP NON HTTP";
		n_handler++;
	}
#endif
	if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_trigger;
		qh_tbl[n_handler].args.module = MODULE_TRIGGER;
		qh_tbl[n_handler].comment = "TRIGGER";
		n_handler++;
	}
#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
    if (n_handler < MAX_QUEUE_HANDLER) {
		qh_tbl[n_handler].qh = NULL;
		qh_tbl[n_handler].num = queue_num_igmp;
		qh_tbl[n_handler].args.module = MODULE_IGMP;
		qh_tbl[n_handler].comment = "IGMP";
		n_handler++;
	}
#endif
	DEBUG0("opening library handle\n");
	h = nfq_open();
	if (!h) {
		ERROR("error during nfq_open()\n");
		goto __exit;
	}

	DEBUG0("unbinding existing nf_queue handler for AF_INET (if any)\n");
	if (nfq_unbind_pf(h, AF_INET) < 0) {
		ERROR("error during nfq_unbind_pf()\n");
		goto __exit;
	}

	DEBUG0("binding nfnetlink_queue as nf_queue handler for AF_INET\n");
	if (nfq_bind_pf(h, AF_INET) < 0) {
		ERROR("error during nfq_bind_pf()\n");
		goto __exit;
	}

	DEBUG0("unbinding existing nf_queue handler for AF_INET6\n");
	if( nfq_unbind_pf(h, AF_INET6) < 0 )
	{
		ERROR("error during nfq_unbind_pf(AF_INET6)\n");
		goto __exit;

	}

	DEBUG0("binding nfnetlink_queue as nf_queue handler for AF_INET6\n");
	if( nfq_bind_pf(h, AF_INET6) < 0 )
	{
		ERROR("error during nfq_bind_pf(AF_INET6)\n");
		goto __exit;
	}

	for (i = 0; i < n_handler; i++) {
		DEBUG0("binding this socket to queue '%u' for %s\n", qh_tbl[i].num, qh_tbl[i].comment);
		qh_tbl[i].qh = nfq_create_queue(h, qh_tbl[i].num, &callback, (void *)&qh_tbl[i].args);
		if (!qh_tbl[i].qh) {
			ERROR("nfq_create_queue() failed\n");
			goto __exit;
		}
	}

	for (i = 0; i < n_handler; i++) {
		if (nfq_set_mode(qh_tbl[i].qh, NFQNL_COPY_PACKET, MAX_IP_SIZE) < 0) {
			ERROR("nfq_set_mode(%d) failed\n", i);
			goto __exit;
		}
	}

	// Use shared memory as receiving buffer.
	// The buffer should be aligned with 4 bytes. Share memory is supposed to be aligned with page size.
	qbuf = shmBase;
	qbuf_sz = SHM_SZ;

	INFO("%s started\n", argv[0]);

	//
	// Fetch packets from netfilter queue.
	//
	fd = nfq_fd(h);
	while ((rv = recv(fd, qbuf, qbuf_sz, 0)) && rv >= 0) {
		if (nfq_handle_packet(h, qbuf, rv) != 0) {
			// Unexpected!!! It will cause memory leaks.
			// TODO: Dequeue it from netfilter queue by force. Is it possible???
			ERROR("nfq_handle_packet() failed...rv(%d)\n", rv);
		}
	}

__exit:

	for (i = 0; i < n_handler; i++) {
		DEBUG0("unbinding from queue %u\n", qh_tbl[i].num);
		if (qh_tbl[i].qh) {
			nfq_destroy_queue(qh_tbl[i].qh);
		}
	}

#ifdef INSANE
        /* normally, applications SHOULD NOT issue this command, since
         * it detaches other programs/sockets from AF_INET, too ! */
        DEBUG0("unbinding from AF_INET\n");
        nfq_unbind_pf(h, AF_INET);

        DEBUG0("unbinding from AF_INET6\n");
        nfq_unbind_pf(h, AF_INET6);
#endif

	DEBUG0("closing library handle\n");
	if (h) {
		nfq_close(h);
	}

	INFO("%s stopped\n", argv[0]);

	exit(0);
}

int callback(struct nfq_q_handle *qh, struct nfgenmsg *nfmsg, struct nfq_data *nfa, void *data)
{
	int id = 0;
	struct nfqnl_msg_packet_hdr *ph = nfq_get_msg_packet_hdr(nfa);
	u_int32_t verdict = NF_ACCEPT;
	u_int32_t mark = nfq_get_nfmark(nfa); 
	unsigned char *payload;
	int data_len;
	callback_args_t *args = (callback_args_t *)data;
	module_t module = (args != NULL) ? args->module : MODULE_UNKNOWN;
	
	// Unused parameters.
	(void)nfmsg;

	if (ph){
		id = ntohl(ph->packet_id);
	}
    else {
        INFO("nfq: no packet_header\n");
        return -1;
    }

	switch (module) 
	{
	case MODULE_PARENTAL_CONTROL:
		{
			verdict = NF_REPEAT;		// Go back to iptables/netfilter.
			SET_PC_MARK(mark, DEFAULT_MARK_DONE); // Set default mark as mask value.	(mark & PC_MASK) should not be "0".

			data_len = nfq_get_payload(nfa, &payload);

            if (data_len < 0) {
                INFO("nfq: no payload\n");
                return -1;
            }

			if (data_len > 0) {
				pc_request_t request = (args != NULL) ? args->pc_request : PC_REQUEST_UNKNOWN;
				pc_response_t response;
		
				// Check this packet against data processing server.
				response = pc_process(request, (char *)payload, data_len);
		
				switch (response) {
				default:
				case PC_RESPONSE_DONE: 	// This site is allowed at this time.
					// Use default mark value above.
					break;
				case PC_RESPONSE_PASS: 	// The site is found in whitelist, so make it allowed all the time.
					SET_PC_MARK(mark, mark_pass);
					break;
				case PC_RESPONSE_BLOCK:	// The site is blocked, so client web browser is redirected to blocking page for authentication.
					SET_PC_MARK(mark, mark_block);
#ifdef CONFIG_BUILD_AUTO_CONFIG
					if (request == PC_REQUEST_SETUP_HTTP || request == PC_REQUEST_HTTP_REQUEST) {
#else
					if (request == PC_REQUEST_HTTP_REQUEST) {
#endif
						verdict = NF_DROP;	// Drop original packet.
					}
					break;
				case PC_RESPONSE_CHECK:	// Cannot make the decision at this time, so continue to check the traffic.
					SET_PC_MARK(mark, mark_check);
					break;
				case PC_RESPONSE_FORWARD:	// This packet should be forwareded.
					SET_PC_MARK(mark, mark_forward);
					break;
				}
			}
		}
		break;

	case MODULE_TRIGGER:
		{
			verdict = NF_ACCEPT;

			data_len = nfq_get_payload(nfa, &payload);
			if (data_len > 0) 
			{
                struct iphdr *iph = ((struct iphdr *)payload);

                if (iph->version == IPVERSION)
                {
                	struct in_addr saddr = {iph->saddr};
                	int trigger_id = (mark & TRIGGER_MASK);

                	// Check this packet against data processing server.
                	tr_process(trigger_id, saddr);
                }
                else
                {
                	//TODO: Young <-- You should implement the tr_process() function for IPv6
                }
            }
		}
		break;

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
	case MODULE_IGMP:
		{
			verdict = NF_ACCEPT;
			data_len = nfq_get_payload(nfa, &payload);
			struct iphdr *iph;
			if (data_len > 0) 
			{
				iph = ((struct iphdr *)payload);
				if (iph->protocol == 0x2) 
				{
					struct igmphdr *igmph;

					igmph = (struct igmphdr *)((char *)payload + iph->ihl * 4);
					if (igmph->type == 0x12 || igmph->type == 0x16 || igmph->type == 0x17) 
					{
						// IGMP Host Membership Report or Leave Group
						struct nfqnl_msg_packet_hw *smac = nfq_get_packet_hw(nfa);
						if (smac) 
						{
							// no need of ntohl
							igmp_process((unsigned char *)(smac->hw_addr), igmph->group, igmph->type);
						}
					}
				}
			}
		}
		break;
#endif
	default:
		break;
	}

	// nfq_set_verdict2() expects mark as host byte order unlike nfq_set_verdict_mark() in version 0.0.17.
	return nfq_set_verdict2(qh, id, verdict, mark, 0, NULL);
}

pc_response_t pc_process(pc_request_t request, char *data, int data_len)
{
	msg_pc_request_t sndmsg;
	msg_pc_response_t rcvmsg;

	sndmsg.type = PC_MSG_TYPE_REQUEST;
	sndmsg.request = request;
	sndmsg.data_offset = data - shmBase;	// Data is written in shared memory, so send the offset from shared memory base.
	sndmsg.data_len = data_len;

	if (msgsnd(pc_queueId, &sndmsg, sizeof(sndmsg)-sizeof(long), 0) < 0) {
		ERROR("msgsnd() failed: %s\n", strerror(errno));
	}
	// Wait for response from data processing server.
	else if (msgrcv(pc_queueId, &rcvmsg, sizeof(rcvmsg)-sizeof(long), PC_MSG_TYPE_RESPONSE, 0) < 0) {	// Blocked until getting the response.
		ERROR("msgrcv() failed: %s\n", strerror(errno));
	}

	return rcvmsg.response;
}

int tr_process(int id, struct in_addr saddr)
{
	msg_trigger_t sndmsg;
	struct timespec timeout = {0, 1000};	// {sec, usec} 1 ms

	sndmsg.id = id;
	sndmsg.ip_addr = saddr;

	if (mq_timedsend(tr_queueId, (char *)&sndmsg, sizeof(sndmsg), 0, &timeout) == -1) {
		ERROR("mq_send() failed: %s\n", strerror(errno));
	}

	return 0;
}

#ifdef CONFIG_BUILD_WL_IGMP_SNOOPING
int igmp_process(unsigned char *smac, unsigned int group_address, unsigned char type)
{
	msg_igmp_t sndmsg;
	struct timespec timeout = {0, 1000};	// {sec, usec} 1 ms
	
	sndmsg.type = type;
	sndmsg.group_address = group_address;
	memcpy(sndmsg.smac, smac, 6);
	//printf("igmp_process send mac %02x:%02x:%02x:%02x:%02x:%02x\n", *smac, *(smac+1), *(smac+2), *(smac+3), *(smac+4), *(smac+5));

	if (mq_timedsend(igmp_queueId, (char *)&sndmsg, sizeof(sndmsg), 0, &timeout) == -1) {
		ERROR("mq_send() failed: %s\n", strerror(errno));
	}

	return 0;
}
#endif

void sig_handler(int signo)
{
	if (signo == SIGSEGV) {
		printmsg("nfqrecv: segmentation fault\n");

		// TODO: Clean up iptables' rules to queue packets.

		_exit(-1);										// Terminate process to prevent from infinite signal callback.
	}
}

void printmsg(char *msg)
{
	int count = 0;

	while (msg[count] != 0 && count < 256) {	// Calculate string length. strlen() is not safe in signal handler.
		count++;
	}
	
	if (write(1, msg, count)) {
		/* should handle error case */
	}
}
