
#include <linux/types.h>
#include <linux/kernel.h>

#include "ionia-serial.h"
#include "ionia-rpc.h"
#include "ionia-rpc-log.h"

struct uart_port;

int ionia_rpc_send(struct uart_port* port, u8 proto, u8 cmd, char *outb, size_t outsz, char *inb, size_t insz)
{
	ionia_uart_putc(port, proto);
	ionia_uart_putc(port, cmd);
	return 0;
}

int ionia_rpc_send_hdr(struct uart_port* port, u8 proto, u8 cmd)
{
	ionia_uart_putc(port, proto);
	ionia_uart_putc(port, cmd);
	return 0;
}

int ionia_rpc_send_u32(struct uart_port* port, u32 data)
{
	char* c = (char*)&data;
	data = cpu_to_be32(data);

	ionia_uart_putc(port, c[0]);
	ionia_uart_putc(port, c[1]);
	ionia_uart_putc(port, c[2]);
	ionia_uart_putc(port, c[3]);
	return 0;
}

u32 ionia_rpc_read_u32(struct uart_port *port)
{
	u32 data;
	char *c = (char*)&data;

	c[0] = ionia_uart_getc(port);
	c[1] = ionia_uart_getc(port);
	c[2] = ionia_uart_getc(port);
	c[3] = ionia_uart_getc(port);
	return be32_to_cpu(data);
}

int ionia_log_channel_enable(struct uart_port *port, int mode, int mask)
{
	u32 package = mode + (mask << 16);
	ionia_rpc_send_hdr(port, IONIA_RPC_PROTO_LOG, IONIA_LOG_CMD_CHANNEL_ENABLE);
	ionia_rpc_send_u32(port, package);
	return 0;
}
