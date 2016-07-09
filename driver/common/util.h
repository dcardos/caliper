/* ****************************************************************************
 * $Id: util.h 1340 2007-02-19 21:36:44Z grg $
 *
 * Module: nf2util.h
 * Project: NetFPGA 2 Linux Kernel Driver
 * Description: Header file for kernel driver
 *
 * Change history:
 *
 */

#ifndef _UTIL_H
#define _UTIL_H	1

#define PATHLEN		80

uint8_t *parseip(char *str);
uint8_t *parsemac(char *str);
uint16_t cksm(uint16_t length, uint32_t buf[]);

#endif
