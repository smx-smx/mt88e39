#ifndef __MT88E39_H
#define __MT88E39_H

#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#endif

#ifndef EXIT_FAILURE
#define EXIT_FAILURE -1
#endif
typedef enum {
	STS_STBY = 0,	//stand by
	STS_MLEN,		//getting message size (in bytes)
	//STS_UWAIT,
	STS_DONE		//finished
} mt88e_state;

#define TYPE_MDMF 0x80
#define TYPE_MDMF_TEST 0x81
#define TYPE_MDMF_WAIT 0x82
#define TYPE_SDMF 0x04
#define TYPE_SDMF_WAIT 0x06
#define TYPE_RESERVER 0x0B

#define MAX_LENGTH 255
#endif
