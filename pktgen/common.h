#ifndef COMMON_H
#define COMMON_H


typedef unsigned int (*length_source)();
typedef unsigned long long (*delay_source)();


unsigned long long get_now_ns();
char* ip(unsigned int i);


#endif
