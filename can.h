#ifndef _CAN_H
#define _CAN_H
void can_init();
int can_send(unsigned char *payload, unsigned char payload_len);
int can_read(unsigned char *buf, unsigned char *buf_len);
#endif
