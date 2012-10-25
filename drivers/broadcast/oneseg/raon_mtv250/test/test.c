#include "test.h"


//#define _TS_FILE_DUMP_ENABLE


static int fd_dmb_dev; /* MTV device file descriptor. */
static volatile int tv_mode; /* 0: 1seg, 1: T_DMB, 2: FM */

RTV_IOCTL_REGISTER_ACCESS ioctl_register_access;
RTV_IOCTL_TEST_GPIO_INFO gpio_info;

#define TDMB_ALL_CH

const uint32_t tdmb_korea_tbl[] =
{
#ifdef TDMB_ALL_CH
	175280, 177008, 178736, //1 // KDMB 7
	181280, 183008, 184736, //2 //KDMB 8
	187280, 189008, 190736, //3 //KDMB 9
	193280, 195008, 196736, //4 //KDMB 10
	199280, 201008, 202736, //5 KDMB 11
	205280, 207008, 208736, //6 KDMB 12
	211280, 213008, 214736  //7 KDMB 13
#else
	181280, 183008, 184736, //2 //KDMB 8
	205280, 207008, 208736, //6 KDMB 12
#endif
};


typedef struct
{
	unsigned int ch;
	unsigned int freq;
} TISDBTFREQ;

static const TISDBTFREQ gtISDBTFreqTable[] = 
{
#ifndef DxB_SBTVD_INCLUDE
	{13, 473143}, {14, 479143}, {15, 485143}, {16, 491143}, {17, 497143}, {18, 503143},
	{19, 509143}, {20, 515143}, {21, 521143}, {22, 527143}, {23, 533143}, {24, 539143},
	{25, 545143}, {26, 551143}, {27, 557143}, {28, 563143}, {29, 569143}, {30, 575143},
	{31, 581143}, {32, 587143}, {33, 593143}, {34, 599143}, {35, 605143}, {36, 611143},
	{37, 617143}, {38, 623143}, {39, 629143}, {40, 635143}, {41, 641143}, {42, 647143},
	{43, 653143}, {44, 659143}, {45, 665143}, {46, 671143}, {47, 677143}, {48, 683143},
	{49, 689143}, {50, 695143}, {51, 701143}, {52, 707143}, {53, 713143}, {54, 719143},
	{55, 725143}, {56, 731143}, {57, 737143}, {58, 743143}, {59, 749143}, {60, 755143},
	{61, 761143}, {62, 767143},	
#else
	{14, 473143}, {15, 479143}, {16, 485143}, {17, 491143}, {18, 497143}, {19, 503143},
	{20, 509143}, {21, 515143}, {22, 521143}, {23, 527143}, {24, 533143}, {25, 539143},
	{26, 545143}, {27, 551143}, {28, 557143}, {29, 563143}, {30, 569143}, {31, 575143},
	{32, 581143}, {33, 587143}, {34, 593143}, {35, 599143}, {36, 605143}, {37, 611143},
	{38, 617143}, {39, 623143}, {40, 629143}, {41, 635143}, {42, 641143}, {43, 647143},
	{44, 653143}, {45, 659143}, {46, 665143}, {47, 671143}, {48, 677143}, {49, 683143},
	{50, 689143}, {51, 695143}, {52, 701143}, {53, 707143}, {54, 713143}, {55, 719143},
	{56, 725143}, {57, 731143}, {58, 737143}, {59, 743143}, {60, 749143}, {61, 755143},
	{62, 761143}, {63, 767143}, {64, 773143}, {65, 779143}, {66, 785143}, {67, 791143},
	{68, 797143}, {69, 803143},
#endif
};

#define CLEAR_STDIN		while(getc(stdin) != '\n')


#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
static int fd_tsif_dev;
#endif

#ifdef _TS_FILE_DUMP_ENABLE
static FILE *fd_tdmb_ts;
static FILE *fd_isdbt_ts;
#endif

static int tsif_run_flag;
static volatile int dmb_thread_run;
static pthread_t tsp_read_thread_cb; 
static pthread_t tsp_consumer_thread_cb; 

static struct tsp_queue tsp_pool_cb;
static struct tsp_queue tsp_queue_cb;


#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)
	static MTV_TS_PKT_INFO tsp_pool[MAX_NUM_TS_BUF];
#else
	static TDMB_CIF_TS_INFO tsp_pool[MAX_NUM_TS_BUF];
#endif

static struct mrevent ts_read_event;


#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
static void tsif_run(int run)
{
	if(run == 1)
	{
		/* Start TSI */
		if(ioctl(fd_tsif_dev, 0xAABB) != 0)
		{
			printf("TSI start error");					
		}	
	}
	else
	{		
		/*	Stop TSI */
		if(ioctl(fd_tsif_dev, 0xAACC) != 0) 
		{
			printf("TSI stop error");
		}
	}

	tsif_run_flag = run;
}
#endif

static void queue_reset(struct tsp_queue *q_ptr)
{
	q_ptr->count = 0;
	q_ptr->front = 0;
	q_ptr->rear = 0;
}

static void queue_init(struct tsp_queue *q_ptr, char *name)
{
	strcpy(q_ptr->name, name);
	pthread_mutex_init(&q_ptr->mutex, 0);
	queue_reset(q_ptr);
}

static void queue_deinit(struct tsp_queue *q_ptr)
{
	pthread_mutex_destroy(&q_ptr->mutex);
}


static void queue_put(struct tsp_queue *q_ptr, unsigned int buf_addr)
{
	pthread_mutex_lock(&q_ptr->mutex);

	// fifo full?
	if(q_ptr->count != MAX_NUM_TS_BUF)
	{
		q_ptr->buf_addr[q_ptr->rear] = buf_addr;
		q_ptr->rear = (q_ptr->rear+1) % MAX_NUM_TS_BUF;
		q_ptr->count++;
	}
	else
		printf("[queue_put] %s queue Full!\n", q_ptr->name);

	pthread_mutex_unlock(&q_ptr->mutex);
}


static void * queue_get(struct tsp_queue *q_ptr)
{
	void *buf;

	pthread_mutex_lock(&q_ptr->mutex);
	
	// fifo empty?
	if(q_ptr->count != 0)
	{		
		buf = (void *)q_ptr->buf_addr[q_ptr->front];
		q_ptr->front = (q_ptr->front+1) % MAX_NUM_TS_BUF;
		q_ptr->count--;
	}
	else
	{
		buf = NULL;
		//printf("[queue_get] %s queue empty!\n", q_ptr->name);
	}

	pthread_mutex_unlock(&q_ptr->mutex);

	return buf;
}


static void tsp_buf_init(void)
{
	int i;
#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)
	MTV_TS_PKT_INFO *tsp;
#else
	TDMB_CIF_TS_INFO *tsp;
#endif
		
	queue_init(&tsp_pool_cb, "ts_free_queue");
	queue_init(&tsp_queue_cb, "ts_data_queue");

	for(i=0; i<MAX_NUM_TS_BUF; i++)
	{
		tsp = &tsp_pool[i];
		queue_put(&tsp_pool_cb, (unsigned int)tsp);
	}
}

static void ts_buf_deinit(void)
{
	queue_deinit(&tsp_pool_cb);
	queue_deinit(&tsp_queue_cb);
}

void *dmb_read_thread(void *arg)
{
#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)
	MTV_TS_PKT_INFO *tsp;
#else
	TDMB_CIF_TS_INFO *tsp;
	int len;
#endif

#if defined(RTV_IF_SPI) 	
	int dev = fd_dmb_dev;
#elif defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_MPEG2_PARALLEL_TSIF) || defined(RTV_IF_QUALCOMM_TSIF) 
	int dev = fd_tsif_dev; 
#endif	  

	printf("[dmb_read_thread] Entered\n");

	for(;;)
	{
		if(dmb_thread_run == 0)
		{
			break;
		}

#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)			
		tsp = (MTV_TS_PKT_INFO *)queue_get(&tsp_pool_cb);		
#else
		tsp = (TDMB_CIF_TS_INFO *)queue_get(&tsp_pool_cb);		
#endif
		if(tsp == NULL) 
		{
			printf("[dmb_read_thread] tsp_pool_cb full!!!\n");
			continue;
		}

		// Read a TSP into a free buffer.
#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)					
		tsp->len = read(dev, tsp->msc_buf, MAX_READ_TSP_SIZE);		
		if(tsp->len > 0)			
#else
		len = read(dev, tsp, sizeof(TDMB_CIF_TS_INFO));		
		if(len > 0)			
#endif
		{			
			/* Enqueue a TSP to data queue. */
			queue_put(&tsp_queue_cb, (unsigned int)tsp);

			/* Send the data-event to consumer. */
			mrevent_trigger(&ts_read_event);
		}		
		else
		{
			//printf("[dmb_read_thread] read() fail: %d, dmb_thread_run:%d\n", ret, dmb_thread_run);
			
			queue_put(&tsp_pool_cb, (unsigned int)tsp);
		}

		usleep(5 * 1000);
	}

	printf("[dmb_read_thread] Exit...\n");			

	pthread_exit((void *)NULL);
}


void check_video_pid(unsigned char *buf, int len)
{
#if 1
	unsigned int i;

	for (i = 0; i < len; i+= 188) 
	{
		printf("[0x%02X], [0x%02X], [0x%02X], [0x%02X] (len: %d)\n", buf[i], buf[i+1], buf[i+2], buf[i+3], len);			
	}

#else	
	int i;
	static unsigned int expect_cnt = 0xff;
	unsigned int cur_cnt = 0;
	unsigned int pid = 0;

	unsigned int check_pid;

	//check_pid = 0x113;
	check_pid = 0x281;

	//UARTprintf("\n");
	for (i = 0; i < len; i+= 188) 
	{
		if (buf[i] != 0x47) 
		{
			printf("wrong TS sync byte (%x)\n", buf[i]);
			break;
		}
		
		pid = ((buf[i + 1] & 0x1f) << 8) | buf[i + 2];
		if (pid == check_pid) 
		{
			cur_cnt = (buf[i + 3] & 0x0f);
			//printf("V(%d)", cur_cnt);
			if (expect_cnt == 0xff)
				expect_cnt = cur_cnt;
			if (expect_cnt != cur_cnt) 
			{
				printf("\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
				printf("TS seq problem. expected %d, got %d\n", expect_cnt, cur_cnt);
				expect_cnt = cur_cnt;
			}
			else
			{
				//printf("seq OK\n");
			}
			expect_cnt = (expect_cnt + 1) % 16;
		}
	}
#endif	
}


void *dmb_consumer_thread(void *arg)
{
#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)
	MTV_TS_PKT_INFO *tsp;
#else
	TDMB_CIF_TS_INFO *tsp;
	int i;
#endif
	
	printf("[dmb_consumer_thread] Entered\n");
	
	for(;;)
	{
		/* Wait for the data event. */
		mrevent_wait(&ts_read_event);

		/* Clear the data event. */
		mrevent_reset(&ts_read_event);

		if(dmb_thread_run == 0)
		{
			break;
		}

#if !defined(RTV_TDMB_MULTI_SUB_CHANNEL_ENABLE)
		while((tsp=(MTV_TS_PKT_INFO *)queue_get(&tsp_queue_cb)) != NULL)
		{			
	#ifdef _TS_FILE_DUMP_ENABLE
			switch( tv_mode ) /* 0: 1seg, 1: T_DMB, 2: FM */
			{
				case 0 :
					fwrite(tsp->msc_buf, sizeof(char), tsp->len, fd_isdbt_ts);
					break;

				case 1 :
					fwrite(tsp->msc_buf, sizeof(char), tsp->len, fd_tdmb_ts);
					break;

				case 2 :
					break;

				default: 
					break;
			}
	#endif
			check_video_pid(tsp->msc_buf, tsp->len);
			
			queue_put(&tsp_pool_cb, (unsigned int)tsp);
		}
#else
	while((tsp=(TDMB_CIF_TS_INFO *)queue_get(&tsp_queue_cb)) != NULL)
	{
		for(i=0; i<RTV_MAX_NUM_MULTI_SUB_CHANNEL; i++)
		{
			if(tsp->msc_size[i] != 0)
			{
				printf("\t (%d) Subch ID: %d: Subch Size: %d\n", i, tsp->msc_subch_id[i], tsp->msc_size[i]);
				printf("[0x%02X], [0x%02X], [0x%02X], [0x%02X]\n", tsp->msc_buf[i][0], tsp->msc_buf[i][1], tsp->msc_buf[i][2],tsp->msc_buf[i][3]);			
			}
		}
		
		queue_put(&tsp_pool_cb, (unsigned int)tsp);
	}
#endif
	}

	printf("[dmb_consumer_thread] Exit...\n");			

	pthread_exit((void *)NULL);
}


static int run_threads(void)
{
	int ret;
	
	if(dmb_thread_run == 1)
		return 0;

	dmb_thread_run = 1;
	
	tsp_buf_init();
	mrevent_init(&ts_read_event);
		
	ret = pthread_create(&tsp_read_thread_cb, NULL, dmb_read_thread, NULL);
	if(ret < 0)
	{
		printf("thread create error: %d\n", ret);
		return ret;
	}

	ret = pthread_create(&tsp_consumer_thread_cb, NULL, dmb_consumer_thread, NULL);
	if(ret < 0)
	{
		printf("thread create error: %d\n", ret);
		return ret;
	}	

	return ret;
}

static void exit_threads(void)
{
	if(dmb_thread_run == 0)
		return;
	
	dmb_thread_run = 0;

#if 0
	pthread_kill(tsp_read_thread_cb, SIGINT);
	pthread_kill(tsp_consumer_thread_cb, SIGINT);

//	pthread_cancel(tsp_read_thread_cb);
//	pthread_cancel(tsp_consumer_thread_cb);
#else	
	mrevent_trigger(&ts_read_event);
#endif
			
	pthread_join(tsp_read_thread_cb, NULL);
	pthread_join(tsp_consumer_thread_cb, NULL);
	ts_buf_deinit();
}


static void test_TDMB()
{
	int key, ret;
	int num_tbl;
	int i;
	unsigned int lock_mask, sub_ch_id, ch_freq_khz;
	E_RTV_COUNTRY_BAND_TYPE country_band_type; 
	IOCTL_TDMB_SUB_CH_INFO sub_ch_info;
	IOCTL_TDMB_SIGNAL_INFO dm_info;
//	BOOL power_on = FALSE;
	unsigned char fic_buf[384];
	char devname[32];	
	BOOL is_power_on = FALSE;
	
	num_tbl = sizeof(tdmb_korea_tbl) / sizeof(uint32_t);
	tv_mode = 1;

	sprintf(devname,"/dev/%s", RAONTV_DEV_NAME);
	fd_dmb_dev = open(devname, O_RDWR);
	if(fd_dmb_dev<0)
	{
		printf("Can't not open %s\n", devname);
		return;
	}

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
	sprintf(devname,"/dev/%s", "s3c-tsi");
	fd_tsif_dev = open(devname, O_RDWR);
	if(fd_tsif_dev<0)
	{
		printf("Can't not open %s\n",devname);
		return -1;
	}
#endif	

#ifdef  _TS_FILE_DUMP_ENABLE
	if((fd_tdmb_ts=fopen("./tdmb_ts.ts", "wb")) == NULL)
	{
		printf("tdmb_ts.ts file open error\n");
	}
#endif	

	tsif_run_flag = 0;

	if((ret=run_threads()) != 0)
	{
		printf("run_threads() failed: %d\n", ret);
	}

	while(1)
	{
		printf("===============================================\n");
		printf("\t0: TDMB Power ON\n");
		printf("\t1: TDMB Power OFF\n");
		printf("\t2: TDMB Scan freq\n");
		printf("\t3: TDMB Open Sub Channel\n");
		printf("\t4: TDMB Close Sub Channel\n");
		printf("\t5: TDMB Get Lockstatus\n");
		printf("\t6: TDMB Get Signal Info\n");
		printf("\t7: TDMB [TEST] Set Freq\n");
		printf("\t8: GPIO Write(Set) Test\n");
		printf("\t9: GPIO Read(Get) Test\n");
	
		printf("\tw: Write Reg\n");
		printf("\tr: Read Reg\n");
		printf("\ts: Stop TSIF\n");
		
		printf("\tq or Q: Quit\n");
		printf("===============================================\n");
   		
		//fflush(stdin);
		key = getc(stdin);				
		CLEAR_STDIN;
		
		switch( key )
		{
			case '0':
				printf("[T-DMB Power ON]\n");
			
				country_band_type = RTV_COUNTRY_BAND_KOREA;				
				if((ret=ioctl(fd_dmb_dev, IOCTL_TDMB_POWER_ON, &country_band_type)) < 0)
				{
					printf("IOCTL_TDMB_POWER_ON failed: %d\n", ret);
				}	
				is_power_on = TRUE;

#if 0
				////
				sub_ch_info.ch_freq_khz = 208736;
				sub_ch_info.sub_ch_id = 0;
				
				//sub_ch_info.ch_freq_khz = 184736;
				//sub_ch_info.sub_ch_id = 1;
				
				sub_ch_info.service_type = RTV_TDMB_SERVICE_VIDEO;
				if(ioctl(fd_dmb_dev, IOCTL_TDMB_OPEN_SUBCHANNEL, &sub_ch_info) < 0)
				{
					printf("IOCTL_TDMB_OPEN_SUBCHANNEL failed\n");
				}
				printf("\n");

//////////////////////////////////////test///////////////////////////////////////


#if 0
for(i=0;i < 400;i++)
{
/*
                            ioctl_register_access.Addr = 0x03;
			       ioctl_register_access.data[0] = 0x0f;
								
	 			if(ioctl(fd_dmb_dev, IOCTL_REGISTER_WRITE, &ioctl_register_access) < 0)
	 			{
	 				printf("IOCTL_REGISTER_WRITE failed\n");
	 			}			

*/								
				//ioctl_register_access.Addr = 0x00;
				
				if(ioctl(fd_dmb_dev, IOCTL_REGISTER_READ, &ioctl_register_access) < 0)
				{
					printf("IOCTL_REGISTER_READ failed\n");
				}

				printf("[%d] Address 0x%02x= 0x%02x\n", i, ioctl_register_access.Addr, ioctl_register_access.data[0]);				
				//printf("\n");	
	usleep(20000);
}
#endif
//////////////////////////////////////test///////////////////////////////////////			
#endif

				break;
				
			case '1':		
				printf("[T-DMB Power OFF]\n");
				if(ioctl(fd_dmb_dev, IOCTL_TDMB_POWER_OFF) < 0)
				{
					printf("IOCTL_TDMB_POWER_OFF failed\n");
				}	
				is_power_on = FALSE;
				break;

			case '2':
				printf("TDMB Scan freq\n");
				for(i=0; i<num_tbl; i++)
				{
					int k;
					
					printf("[TDMB Scan start] freq: %u\n", tdmb_korea_tbl[i]);

					if(ioctl(fd_dmb_dev, IOCTL_TDMB_SCAN_FREQ, &tdmb_korea_tbl[i]) < 0)
					{
						printf("scan failed on channel[%d]\n", tdmb_korea_tbl[i]);
					}	
					else
					{					
						// Add a channel to scaned list.
						printf("[TDMB Scan freq] CH DETECTED\n");	

						for(k=0; k<5; k++)
						{
							if((ret=ioctl(fd_dmb_dev, IOCTL_TDMB_READ_FIC, fic_buf)) < 0)
							{
								printf("read fic fail: %d\n", ret);
							}	
							else
							{
								printf("\tfic_buf[0]: 0x%02X, fic_buf[1]: 0x%02X, fic_buf[2]: 0x%02X\n", fic_buf[0], fic_buf[1], fic_buf[2]);
							}
						}		
					}

					printf("\n");
				}				
				break;

			case '3':
				printf("[T-DMB Open Sub Channel]\n");
				while(1)
				{
					printf("Input Channel freq(ex. 175280):");
					scanf("%u", &sub_ch_info.ch_freq_khz);   		
					CLEAR_STDIN;

					for(i=0; i<num_tbl; i++)
					{
						if(sub_ch_info.ch_freq_khz == tdmb_korea_tbl[i])
							goto call_tdmb_set_ch;
					}	

					if(i == num_tbl)
					{
						printf("Invalid T-DMB freq\n");
					}
				}
				
call_tdmb_set_ch	:
				printf("Input sub channel ID(ex. 0):");
				scanf("%u", &sub_ch_info.sub_ch_id);			
				CLEAR_STDIN;

				printf("Input service type(0: Video, 1:Audio, 2: Data):");
				scanf("%u", &sub_ch_info.service_type);			
				CLEAR_STDIN;

				if((ret=ioctl(fd_dmb_dev, IOCTL_TDMB_OPEN_SUBCHANNEL, &sub_ch_info)) < 0)
				{
					printf("IOCTL_TDMB_OPEN_SUBCHANNEL failed: %d\n", ret);
				}
				printf("\n");			
				break;

			case '4':
				printf("[TDMB Close Sub Channel]\n");
				
				printf("Input sub channel ID(ex. 0):");
				scanf("%u", &sub_ch_id);			
				CLEAR_STDIN;

				if(ioctl(fd_dmb_dev, IOCTL_TDMB_CLOSE_SUBCHANNEL, &sub_ch_id) < 0)
				{
					printf("IOCTL_TDMB_CLOSE_SUBCHANNEL failed\n");
				}
				printf("\n");			
				break;

			case '5':
				printf("[TDMB Get Lockstatus]\n");			
				if(ioctl(fd_dmb_dev, IOCTL_TDMB_GET_LOCK_STATUS, &lock_mask) < 0)
				{						
					printf("RTV_IOCTL_TDMB_GET_LOCK_STATUS failed\n");
				}

				printf("lock_mask = %d\n", lock_mask);			
				printf("\n");			
				break;

			case '6':
				printf("[TDMB Get Singal Info]\n");			
				if(ioctl(fd_dmb_dev, IOCTL_TDMB_GET_SIGNAL_INFO, &dm_info) < 0)
				{						
					printf("IOCTL_TDMB_GET_SIGNAL_INFO failed\n");
				}

				printf("ber = %f\n", (float)dm_info.ber/RTV_TDMB_BER_DIVIDER);
				printf("cer = %u\n", dm_info.cer);
				printf("cnr = %f\n", (float)dm_info.cnr/RTV_TDMB_CNR_DIVIDER);
				printf("rssi = %f\n", (float)dm_info.rssi/RTV_TDMB_RSSI_DIVIDER);
				printf("per = %u\n", dm_info.per);
				printf("\n");			
				break;

			case '7':
				printf("[T-DMB Set Freq]\n");
				while(1)
				{
					printf("Input Channel freq(ex. 175280):");
					scanf("%u", &ch_freq_khz);			
					CLEAR_STDIN;

					for(i=0; i<num_tbl; i++)
					{
						if(ch_freq_khz == tdmb_korea_tbl[i])
							goto call_tdmb_set_Freq;
					}	

					if(i == num_tbl)
					{
						printf("Invalid T-DMB freq\n");
					}
				}
				
call_tdmb_set_Freq	:
				if(ioctl(fd_dmb_dev, IOCTL_TEST_TDMB_SET_FREQ, &ch_freq_khz) < 0)
				{
					printf("IOCTL_TDMB_SET_FREQ failed\n");
				}
				printf("\n");			
				break;


			case '8':
				printf("[GPIO Write(Set) Test]\n");	

				printf("Select Pin Number:");
				scanf("%u" , &gpio_info.pin);	
				CLEAR_STDIN;

		retry_gpio_set:
				printf("Input Pin Level(0 or 1):");
				scanf("%u" , &gpio_info.value);	
				CLEAR_STDIN;
				if((gpio_info.value != 0) && (gpio_info.value != 1))
					goto retry_gpio_set;
				
				if(ioctl(fd_dmb_dev, IOCTL_TEST_GPIO_SET, &gpio_info) < 0)
				{						
					printf("IOCTL_TEST_GPIO_SET failed\n");
				}						
				break;

			case '9':
				printf("[GPIO Write(Set) Test]\n"); 
		
				printf("Select Pin Number:");
				scanf("%u" , &gpio_info.pin);	
				CLEAR_STDIN;

				if(ioctl(fd_dmb_dev, IOCTL_TEST_GPIO_GET, &gpio_info) < 0)
				{						
					printf("IOCTL_TEST_GPIO_GET failed\n");
				}		

				printf("Pin(%u): %u\n", gpio_info.pin, gpio_info.value);
				printf("\n");		
				break;
								
			case 'w':
				printf("[T-DMB] Register Write\n");
				printf("===============================================\n");
				printf("\t0: HOST_PAGE\n");
				printf("\t1: RF_PAGE\n");				
				printf("\t2: COMM_PAGE\n");
				printf("\t3: DD_PAGE\n");
				printf("\t4: MSC0_PAGE\n");
				printf("\t5: MSC1_PAGE\n");				
				printf("\t6: OFDM PAGE\n");
				printf("\t7: FEC_PAGE\n");
				printf("\t8: FIC_PAGE\n");
				printf("===============================================\n");
				
				printf("Select Page:");
				scanf("%x" , &ioctl_register_access.page);	
				CLEAR_STDIN;

				printf("Input Address(hex) :,  data(hex) : ");
				scanf("%x" , &ioctl_register_access.Addr);	
				CLEAR_STDIN;
				scanf("%x" , &ioctl_register_access.data[0]);
				CLEAR_STDIN;
				printf("Input Address: 0x%02x,  data: 0x%02x \n", ioctl_register_access.Addr, ioctl_register_access.data[0] );

				if(ioctl(fd_dmb_dev, IOCTL_REGISTER_WRITE, &ioctl_register_access) < 0)
				{
					printf("IOCTL_REGISTER_WRITE failed\n");
				}						
				break;
				
			case 'r':
				printf("[T-DMB] Register Read\n");
				printf("===============================================\n");
				printf("\t0: HOST_PAGE\n");
				printf("\t1: RF_PAGE\n");				
				printf("\t2: COMM_PAGE\n");
				printf("\t3: DD_PAGE\n");
				printf("\t4: MSC0_PAGE\n");
				printf("\t5: MSC1_PAGE\n");				
				printf("\t6: OFDM PAGE\n");
				printf("\t7: FEC_PAGE\n");
				printf("\t8: FIC_PAGE\n");
				printf("===============================================\n");
				
				printf("Select Page:");
				scanf("%x" , &ioctl_register_access.page);	
				CLEAR_STDIN;
				
				printf("Input Address(hex) : ");

				scanf("%x", &ioctl_register_access.Addr);
				CLEAR_STDIN;
				printf("Input Address: 0x%02x\n ", ioctl_register_access.Addr );

				if(ioctl(fd_dmb_dev, IOCTL_REGISTER_READ, &ioctl_register_access) < 0)
				{
					printf("IOCTL_REGISTER_READ failed\n");
				}

				printf("Address 0x%02x= 0x%02x\n",ioctl_register_access.Addr, ioctl_register_access.data[0]);				
				printf("\n");			
				break;

			case 's':
				break;

			case 'q':
			case 'Q':
				goto TDMB_EXIT;

			default:
				printf("[%c]\n", key);
		}
	} 

TDMB_EXIT:

#if 0
	
	exit_threads();

	//raise(SIGKILL);
//	raise(SIGUSR1);
#else
	if(is_power_on == TRUE)
	{
	//	printf("(TDMB_EXIT) IOCTL_TDMB_POWER_OFF\n");
		if(ioctl(fd_dmb_dev, IOCTL_TDMB_POWER_OFF) < 0)
		{
			printf("IOCTL_TDMB_POWER_OFF failed\n");
		}	
	}

	exit_threads();

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
	close(fd_tsif_dev);
#endif

//	printf("(TDMB_EXIT) close(fd_dmb_dev)\n");
	close(fd_dmb_dev);
//	printf("(TDMB_EXIT) close(fd_dmb_dev) END\n");

#ifdef _TS_FILE_DUMP_ENABLE
	fclose(fd_tdmb_ts);
#endif

#endif


	printf("TDMB_EXIT\n");
	
	return;
}


static void test_ISDBT(void)
{
	int key, ret;
	E_RTV_COUNTRY_BAND_TYPE country_band_type; 
	RTV_ISDBT_TMCC_INFO tmcc_info;
	IOCTL_ISDBT_SIGNAL_INFO dm_info;
	int num_tbl;
	int i;
	int ch;
	unsigned int lock_mask;
	char devname[32];

	tv_mode = 0;
	
	num_tbl = sizeof(gtISDBTFreqTable) / sizeof(TISDBTFREQ);

	sprintf(devname,"/dev/%s", RAONTV_DEV_NAME);
	fd_dmb_dev = open(devname, O_RDWR);
	if(fd_dmb_dev<0)
	{
		printf("Can't not open %s\n", devname);
		return;
	}

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
	sprintf(devname,"/dev/%s", "s3c-tsi");
	fd_tsif_dev = open(devname, O_RDWR);
	if(fd_tsif_dev<0)
	{
		printf("Can't not open %s\n",devname);
		return -1;
	}
#endif	

#ifdef  _TS_FILE_DUMP_ENABLE
	if((fd_isdbt_ts=fopen("./isdbt_ts.ts", "wb")) == NULL)
	{
		printf("isdbt_ts.ts file open error\n");
	}
#endif	

	tsif_run_flag = 0;

	
	while(1)
	{
		printf("===============================================\n");
		printf("\t0: 1seg Power ON\n");
		printf("\t1: 1seg Power OFF\n");
		printf("\t2: 1seg Scan freq\n");
		printf("\t3: 1seg Set Channel\n");
		printf("\t4: 1seg Get Lockstatus\n");
		printf("\t5: 1seg Get TMCC\n");
		printf("\t6: 1seg Get Signal Info\n");
		printf("\t7: 1seg Start TS\n");
		printf("\t8: 1seg Stop TS\n");
		
		printf("\tw: Write Reg\n");
		printf("\tr: Read Reg\n");
		
		printf("\tq or Q: Quit\n");
		printf("===============================================\n");
   		
		fflush(stdin);
		key = getc(stdin);				
		CLEAR_STDIN;
		
		switch( key )
		{
			case '0':
				printf("[ISDBT DTV Start]\n");

				if((ret=run_threads()) != 0)
				{
					printf("run_threads() failed: %d\n", ret);
				}
				
				country_band_type = RTV_COUNTRY_BAND_JAPAN;
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_POWER_ON, &country_band_type) != 0)
				{
					printf("IOCTL_ISDBT_POWER_ON error");
				}

	#if 0
				ch = 37;
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_SET_FREQ, &ch) < 0)
				{
					// scan fail
					printf("IOCTL_ISDBT_SET_FREQ failed\n");
				}	

				sleep(3);
				printf("After sleep()\n");

				ch = 37;
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_SET_FREQ, &ch) < 0)
				{
					// scan fail
					printf("IOCTL_ISDBT_SET_FREQ failed\n");
				}	
	#endif
				break;

			case '1':
				printf("[ISDBT DTV Stop]\n");
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_POWER_OFF) != 0)
				{
					printf("IOCTL_ISDBT_POWER_OFF error");
				}
				break;
		
			case '2':
				for(i=0; i<num_tbl; i++)
				{
					printf("[ISDB-T Scan start] (ch: %u), freq: %u\n", gtISDBTFreqTable[i].ch, gtISDBTFreqTable[i].freq);

					if(ioctl(fd_dmb_dev, IOCTL_ISDBT_SCAN_FREQ, &gtISDBTFreqTable[i].ch) < 0)
					{
						printf("scan failed on channel[%d]\n",gtISDBTFreqTable[i].ch);
					}		
					
					// Add a channel to scaned list.

					printf("\n");
				}				
				break;

			case '3':

				printf("Input Channel num(ex.13):");
				scanf("%d",&ch);
				CLEAR_STDIN;
				printf("[ISDBT Set Freq] (ch: %u)\n", ch);

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
		   		if(tsif_run_flag != 1)
					tsif_run(1); /*  Run TSI */
#endif

				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_SET_FREQ, &ch) < 0)
				{
					// scan fail
					printf("IOCTL_ISDBT_SET_FREQ failed\n");
				}								
				printf("\n");
				break;

			case '4':
				printf("[ISDB-T Get Lockstatus]\n");
				
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_GET_LOCK_STATUS, &lock_mask) < 0)
				{
					printf("IOCTL_ISDBT_GET_LOCK_STATUS failed\n");
				}

				printf("isdbt_lock_mask = %d\n", lock_mask);
				
				printf("\n");			
				break;

			case '5':
				printf("[ISDB-T Get Tmcc]\n");				
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_GET_TMCC, &tmcc_info) < 0)
				{
					printf("IOCTL_ISDBT_GET_TMCC failed\n");
				}

				printf("tmcc_info.eCodeRate = %d\n", tmcc_info.eCodeRate);
				printf("tmcc_info.eGuard = %d\n", tmcc_info.eGuard);
				printf("tmcc_info.eInterlv = %d\n", tmcc_info.eInterlv);
				printf("tmcc_info.eModulation = %d\n", tmcc_info.eModulation);
				printf("tmcc_info.eSeg = %d\n", tmcc_info.eSeg);
				printf("tmcc_info.eTvMode = %d\n", tmcc_info.eTvMode);
				printf("tmcc_info.fEWS = %d\n", tmcc_info.fEWS);
				
				printf("\n");			
				break;

			case '6':
				printf("[ISDB-T Get Singal Info]\n");			
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_GET_SIGNAL_INFO, &dm_info) < 0)
				{						
					printf("IOCTL_ISDBT_GET_SIGNAL_INFO failed\n");
				}

				printf("ber = %f\n", (float)dm_info.ber/(float)RTV_ISDBT_BER_DIVIDER);
				printf("cnr = %f\n", (float)dm_info.cnr/(float)RTV_ISDBT_CNR_DIVIDER);
				printf("rssi = %f\n", (float)dm_info.rssi/(float)RTV_ISDBT_RSSI_DIVIDER);
				printf("per = %u\n", dm_info.per);
				printf("\n");			
				break;

			case '7':
				printf("[ISDBT Start TS]\n");				
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_START_TS) != 0)
				{
					printf("IOCTL_ISDBT_START_TS error");
				}
				break;
				
			case '8':
				printf("[ISDBT Start TS]\n");				
				if(ioctl(fd_dmb_dev, IOCTL_ISDBT_STOP_TS) != 0)
				{
					printf("IOCTL_ISDBT_STOP_TS error");
				}
				break;

			case 'w':
				printf("[ISDB-T Register Write\n");
				printf("===============================================\n");
				printf("\t0: HOST_PAGE\n");
				printf("\t1: RF_PAGE\n");				
				printf("\t2: COMM_PAGE\n");
				printf("\t3: DD_PAGE\n");
				printf("\t4: MSC0_PAGE\n");
				printf("\t5: MSC1_PAGE\n");				
				printf("\t6: OFDM PAGE\n");
				printf("\t7: FEC_PAGE\n");
				printf("===============================================\n");
				
				printf("Select Page:");
				scanf("%x" , &ioctl_register_access.page);	
				CLEAR_STDIN;

				printf("Input Address(hex) :,  data(hex) : ");
				scanf("%x" , &ioctl_register_access.Addr);	
				CLEAR_STDIN;
				scanf("%x" , &ioctl_register_access.data[0]);
				CLEAR_STDIN;
				printf("Input Address: 0x%02x,  data: 0x%02x \n", ioctl_register_access.Addr, ioctl_register_access.data[0] );

				if(ioctl(fd_dmb_dev, IOCTL_REGISTER_WRITE, &ioctl_register_access) < 0)
				{
					printf("IOCTL_REGISTER_WRITE failed\n");
				}			
				
				break;
				
			case 'r':
				printf("[ISDB-T Register Read\n");
				printf("===============================================\n");
				printf("\t0: HOST_PAGE\n");
				printf("\t1: RF_PAGE\n");				
				printf("\t2: COMM_PAGE\n");
				printf("\t3: DD_PAGE\n");
				printf("\t4: MSC0_PAGE\n");
				printf("\t5: MSC1_PAGE\n");				
				printf("\t6: OFDM PAGE\n");
				printf("\t7: FEC_PAGE\n");
				printf("===============================================\n");
				
				printf("Select Page:");
				scanf("%x" , &ioctl_register_access.page);	
				CLEAR_STDIN;
				
				printf("Input Address(hex) : ");

				scanf("%x", &ioctl_register_access.Addr);
				CLEAR_STDIN;
				
				printf("Input Address: 0x%02x\n ", ioctl_register_access.Addr );
		
				if(ioctl(fd_dmb_dev, IOCTL_REGISTER_READ, &ioctl_register_access) < 0)
				{
					printf("IOCTL_REGISTER_READ failed\n");
				}

				printf("Address 0x%02x= 0x%02x\n",ioctl_register_access.Addr, ioctl_register_access.data[0]);
				
				printf("\n");			
				break;
				
			case 'q':
			case 'Q':
				goto ISDBT_EXIT;

			default:
				printf("[%c]\n", key);
		}
	} 

ISDBT_EXIT:

	if(ioctl(fd_dmb_dev, IOCTL_TDMB_POWER_OFF) < 0)
	{
		printf("IOCTL_TDMB_POWER_OFF failed\n");
	}	

#if defined(RTV_IF_MPEG2_SERIAL_TSIF) || defined(RTV_IF_SPI_SLAVE) || defined(RTV_IF_QUALCOMM_TSIF) || defined(RTV_IF_MPEG2_PARALLEL_TSIF)
	tsif_run(0); /*  Stop TSI */
	close(fd_tsif_dev);
#endif

	close(fd_dmb_dev);

	exit_threads();

#ifdef _TS_FILE_DUMP_ENABLE
	fclose(fd_isdbt_ts);
#endif


	return;	
}

int main(void)
{
	int key;

	dmb_thread_run = 0;
  
	while(1)
	{
		printf("===============================================\n");
//		printf("\t1: TDMB Test\n");
		printf("\t2: ISDBT Test\n");
	//	printf("\t3: FM Test\n");
		printf("\tq or Q: Quit\n");
		printf("===============================================\n");
   		
		fflush(stdin);
		key = getc(stdin);				
		CLEAR_STDIN;
		
		switch( key )
		{
			case '1':
				test_TDMB();				
				break;

			case '2':				
				test_ISDBT();
				break;

			case 'q':
			case 'Q':
				goto APP_MAIN_EXIT;

			default:
				printf("[%c]\n", key);
		}
	} 

APP_MAIN_EXIT:

	return 0;
}
