/*
 *  Duagon Ionia backplane core Linux support
 *
 *  Copyright (c) 2017 Enrico Weigelt, metux IT consult <enrico.weigelt@gr13.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#ifndef __DUAGON_IONIA_SERIAL_H
#define __DUAGON_IONIA_SERIAL_H

struct platform_device;
struct uart_port;

void ionia_uart_dump(struct uart_port *port);

int  ionia_serial_init(struct platform_device *pdev);
void ionia_serial_dumpall(struct platform_device *pdev);

int ionia_uart_getc(struct uart_port *port);
int ionia_uart_putc(struct uart_port *port, char c);

#endif /* __DUAGON_IONIA_SERIAL_H */
