#ifndef TBL_H
#define TBL_H

extern unsigned char TblData[3500];



typedef enum { /* Defines data type of entries in parameter */
	/* table. Corresponds to fields of type ValPT */
	IntPT, /* Signed 4 byte integer. */
	FltPT, /* 8 byte IEEE floating point (double) */
	StrPT, /* String, null terminated, 0-8 bytes. */
	UTCPT, /* UTC date and time, 1 s resolution. */
	PosPT, /* Geographical position. (char *) */
	AmxPT /* AMX time and date. */
} TypePT;
typedef union { /* Defines a value in the parameter table. */
	/* Corresponds to elements of type TypePT. */
	signed int I; /* Signed 4 byte integer. */
	double F; /* 8 byte IEEE floating point. */
	char S[8];/* Geographical position,5m resolution. */
	char P[13];/* Geographical position,5m resolution. */
	unsigned char utc[8];
} ValPT;

#pragma pack(1)
typedef struct {
	char Code[5]; /* Ascii code for the parameter. */
	/* Up to 4 characters, null terminated */
	/* if less than 4. Case ignored. */
	unsigned short Grp:16; /* Group of parameters of which this */
	/* parameter is a member. One semaphore */
	/* is shared by all parameters in a group. */
	unsigned int Smph:32; /* ID of the semaphore which protects */
	/* this parameter. 4 bytes*/
	unsigned char T; /* The data type of this parameter. */
	ValPT V; /* The value of this parameter. */
} TableEntryPT;

int WritePT(const char *code, void *p);
int ReadPT(const char *code, void *p);

void tblInit();
extern TableEntryPT DataTbl[140];
int FastLog2(int x);


#endif

