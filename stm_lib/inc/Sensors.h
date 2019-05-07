
//______________APDS9960______________//

#define APDS9960_I2C_ADDR       0x39
#define APDS9960_ATIME          0x81
#define APDS9960_CONTROL        0x8F
#define APDS9960_ENABLE         0x80

#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B

#define APDS9960_PON            0x01
#define APDS9960_AEN            0x02
#define APDS9960_PEN            0x04
#define APDS9960_WEN            0x08
#define APSD9960_AIEN           0x10
#define APDS9960_PIEN           0x20
#define APDS9960_GEN            0x40
#define APDS9960_GVALID         0x01

#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_AGAIN           AGAIN_4X

//------------------------------------------------//
