#ifndef TROUBLE_CODE_READER_H
#define TROUBLE_CODE_READER_H


typedef struct _TROUBLE_CODE
{
    char *code;
    char *description;
    char *pending;
    int foundCount;
} TROUBLE_CODE;

extern TROUBLE_CODE master_trouble_list[];

int display_trouble_codes(void);
int handle_read_codes(char *, int);
void ready_trouble_codes(void);
void printTroubleCodes(char *buf, unsigned long bufSize);
void initializeUnknownList();
void destroyUnknownList();

#endif
