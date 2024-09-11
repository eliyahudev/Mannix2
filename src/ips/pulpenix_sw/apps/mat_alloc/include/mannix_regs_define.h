// base address for registers
#define GPP_BASE_ADDR	0x1A400000  
// base address for memory
#define GPP_BASE_MEM	0x1A480000

// TH define here the name of the register
// you want to write and read. those registers
// would decide how the accelerator works

// define 
// #define CNN_ADDRX     GPP_BASE_ADDR + 0
// #define CNN_ADDRY     GPP_BASE_ADDR + 4
// #define CNN_ADDRZ     GPP_BASE_ADDR + 8
// #define CNN_ADDRB     GPP_BASE_ADDR + 12
// #define CNN_XM     GPP_BASE_ADDR + 16
// #define CNN_XN     GPP_BASE_ADDR + 20
// #define CNN_YM     GPP_BASE_ADDR + 24
// #define CNN_YN     GPP_BASE_ADDR + 28
// #define CNN_START     GPP_BASE_ADDR + 32
// #define CNN_DONE     GPP_BASE_ADDR + 36
// #define CNN_STRIDE     GPP_BASE_ADDR + 40
// #define CNN_DEPTH     GPP_BASE_ADDR + 44
// #define POOL_ADDRX     GPP_BASE_ADDR + 48
// #define POOL_ADDRZ     GPP_BASE_ADDR + 52
// #define POOL_XM     GPP_BASE_ADDR + 56
// #define POOL_XN     GPP_BASE_ADDR + 60
// #define POOL_PM     GPP_BASE_ADDR + 64
// #define POOL_PN     GPP_BASE_ADDR + 68
// #define POOL_START     GPP_BASE_ADDR + 72
// #define POOL_DONE     GPP_BASE_ADDR + 76
// #define POOL_STRIDE     GPP_BASE_ADDR + 80
// #define FC_ADDRX     GPP_BASE_ADDR + 84
// #define FC_ADDRY     GPP_BASE_ADDR + 88
// #define FC_ADDRB     GPP_BASE_ADDR + 92
// #define FC_ADDRZ     GPP_BASE_ADDR + 96
// #define FC_XM     GPP_BASE_ADDR + 100
// #define FC_XN     GPP_BASE_ADDR + 104
// #define FC_YM     GPP_BASE_ADDR + 108
// #define FC_YN     GPP_BASE_ADDR + 112
// #define FC_START     GPP_BASE_ADDR + 116
// #define FC_DONE     GPP_BASE_ADDR + 120

/*#define GPP_BASE_ADDR   0x1A400000
#define GPP_BASE_MEM    0x1A480000
#define CNN_ADDRX       GPP_BASE_ADDR +0X0004
#define CNN_ADDRY       GPP_BASE_ADDR +0X0008
#define CNN_ADDRZ       GPP_BASE_ADDR +0X000C
#define CNN_XM          GPP_BASE_ADDR +0X0010
#define CNN_XN          GPP_BASE_ADDR +0X0014
#define CNN_YM          GPP_BASE_ADDR +0X0018
#define CNN_YN          GPP_BASE_ADDR +0X001C
#define CNN_STRAT       GPP_BASE_ADDR +0X0020
#define CNN_DONE        GPP_BASE_ADDR +0X0024

#define POOL_ADDRX      ( GPP_BASE_ADDR + 10*4)//0x00000048 )
#define POOL_ADDRZ      ( GPP_BASE_ADDR + 11*4)//0x00000048 )
#define POOL_XM         ( GPP_BASE_ADDR + 12*4)//0x00000048 )
#define POOL_XN         ( GPP_BASE_ADDR + 13*4)//0x00000048 )  
#define POOL_PM         ( GPP_BASE_ADDR + 14*4)//0x00000048 )
#define POOL_PN         ( GPP_BASE_ADDR + 15*4)//0x00000048 )
#define POOL_STRIDE     ( GPP_BASE_ADDR + 16*4)//0x00000048 )
#define POOL_START      ( GPP_BASE_ADDR + 16*4)//0x00000048 )
#define POOL_DONE       ( GPP_BASE_ADDR + 17*4)//0x00000048 )

				        
#define FC_ADDRX       ( GPP_BASE_ADDR + 18*4)//0x00000048 )
#define FC_ADDRY       ( GPP_BASE_ADDR + 19*4)//0x0000004c )
#define FC_ADDRZ       ( GPP_BASE_ADDR + 21*4)//0x00000050 )
#define FC_ADDRB       ( GPP_BASE_ADDR + 20*4)//0x00000054 )
#define FC_XM          ( GPP_BASE_ADDR + 22*4)//0x00000058 )
#define FC_XN          ( GPP_BASE_ADDR + 23*4)//0x0000005c )
#define FC_YM          ( GPP_BASE_ADDR + 24*4)//0x00000060 )
#define FC_YN          ( GPP_BASE_ADDR + 25*4)//0x00000064 )
#define FC_START       ( GPP_BASE_ADDR + 26*4)//0x00000068 )
#define FC_DONE        ( GPP_BASE_ADDR + 27*4)//0x0000006c )*/
