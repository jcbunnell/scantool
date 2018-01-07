#ifndef TROUBLE_CODE_READER_H
#define TROUBLE_CODE_READER_H

int display_trouble_codes(void);
int import_trouble_codes(void);
int handle_read_codes(char *, int);
void ready_trouble_codes(void);
void printTroubleCodes(char *buf, size_t bufSize);
void populate_trouble_codes_list(void);

#endif
