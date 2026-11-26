/* Host test build: the subset of Microchip's GenericTypeDefs.h the firmware uses. */
#ifndef HOST_GENERIC_TYPEDEFS_H
#define HOST_GENERIC_TYPEDEFS_H
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int DWORD;
typedef unsigned long long QWORD;
typedef signed short SHORT;
typedef unsigned char BOOL;
typedef union { BYTE Val; } BYTE_VAL;
typedef union { WORD Val; BYTE v[2]; } WORD_VAL;
typedef union { DWORD Val; BYTE v[4]; } DWORD_VAL;
#endif
