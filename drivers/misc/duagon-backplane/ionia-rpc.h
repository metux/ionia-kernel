#ifndef __DUAGON_RPC_H
#define __DUAGON_RPC_H

#include <linux/types.h>

#define IONIA_RPC_PROTO_PFS		0x01
#define IONIA_RPC_PROTO_SOCKET		0x02
#define IONIA_RPC_PROTO_IPTCOM_PD	0x03
#define IONIA_RPC_PROTO_IPTCOM_MD	0x04
#define IONIA_RPC_PROTO_IO		0x06
#define IONIA_RPC_PROTO_AI		0x07
#define IONIA_RPC_PROTO_AO		0x07
#define IONIA_RPC_PROTO_LOG		0x08
#define IONIA_RPC_PROTO_SER_COM		0x09
#define IONIA_RPC_PROTO_SVC		0x0F
#define IONIA_RPC_PROTO_MVB		0x10
#define IONIA_RPC_PROTO_WTB		0x11
#define IONIA_RPC_PROTO_PWR		0x12

struct uart_port;

int ionia_log_channel_enable(struct uart_port *port, int mode, int mask);
u32 ionia_rpc_read_u32(struct uart_port *port);

#endif /* __DUAGON_RPC_H */
