/* Host test build: the CBUS definitions the firmware uses (values as in MERG cbusdefs).
 * The real cbusdefs8j.h / cbusdefs8h.h are not in the CANEther repository. */
#ifndef HOST_CBUSDEFS_H
#define HOST_CBUSDEFS_H
#define MANU_MERG       165
#define MTYP_CANEther   34
#define MTYP_CANGC1e    11
#define CPUM_MICROCHIP  1
#define PB_CAN          1
#define P18F2585        13
#define OPC_RTOF        0x06
#define OPC_QNN         0x0D
#define OPC_RQNP        0x10
#define OPC_RQMN        0x11
#define OPC_SNN         0x42
#define OPC_RQNN        0x50
#define OPC_NNACK       0x52
#define OPC_WRACK       0x59
#define OPC_BOOT        0x5C
#define OPC_NNRST       0x5E
#define OPC_NVRD        0x71
#define OPC_RQNPN       0x73
#define OPC_ACON        0x90
#define OPC_NVSET       0x96
#define OPC_NVANS       0x97
#define OPC_PARAN       0x9B
#define OPC_NAME        0xE2
#define OPC_PARAMS      0xEF
#define OPC_ACDAT       0xF6
#endif
