/*
 * spi_mdm6600.h -- Serial peheripheral interface framing layer for MDM6600 modem.
 *
 */

//20110504 ws.yang@lge.com ..add to header of spi mdm6600 [S]
#define IFX_SPI_FRAME_SIZE  (2048*2)
#define IFX_SPI_HEADER_SIZE 8
#define IFX_SPI_MAX_BUF_SIZE (IFX_SPI_FRAME_SIZE - IFX_SPI_HEADER_SIZE)
#define IFX_SPI_DEFAULT_BUF_SIZE (IFX_SPI_FRAME_SIZE - IFX_SPI_HEADER_SIZE)
#define IFX_SPI_MAJOR			153	/* assigned */
#define IFX_N_SPI_MINORS		2	/* ... up to 256 */
//20110504 ws.yang@lge.com ..add to header of spi mdm6600 [E]

//20110614 ws.yang@lge.com ..add to measure of spi speed [S]
//#define SPI_SPEED_MEASUREMENT
#ifdef SPI_SPEED_MEASUREMENT
#define SPI_TX_RX_THROUGHTPUT
#endif
//20110614 ws.yang@lge.com ..add to measure of spi speed [E]

//#define SPI_BUFFER_DUMP_LOG
#ifdef SPI_BUFFER_DUMP_LOG
typedef struct {
	__u8 ea:1;
	__u8 cr:1;
	__u8 d:1;
	__u8 server_chn:5;
} __attribute__ ((packed)) address_field;

typedef struct {
	__u8 ea:1;
	__u8 len:7;
} __attribute__ ((packed)) short_length;

typedef struct {
	__u8 ea:1;
	__u8 l_len:7;
	__u8 h_len;
} __attribute__ ((packed)) long_length;

typedef struct {
	address_field addr;
	__u8 control;
	short_length length;
} __attribute__ ((packed)) short_frame_head;

typedef struct {
	short_frame_head h;
	__u8 data[0];
} __attribute__ ((packed)) short_frame;

typedef struct {
	address_field addr;
	__u8 control;
	long_length length;
	__u8 data[0];
} __attribute__ ((packed)) long_frame_head;

typedef struct {
	long_frame_head h;
	__u8 data[0];
} __attribute__ ((packed)) long_frame;

#define GET_LONG_LENGTH(a) ( ((a).h_len << 7) | ((a).l_len) )
#endif

//#define SPI_DEBUG_MODE
#ifdef SPI_DEBUG_MODE
#define SPI_DEBUG(format, args...) printk("[SPI] : %s %d: " format "\n", __FUNCTION__, __LINE__, ## args)
#else
#define SPI_DEBUG(format, args...)
#endif

#define SPI_PRINTK(format, args...) printk("[SPI] : %s %d: " format "\n", __FUNCTION__, __LINE__, ## args)

