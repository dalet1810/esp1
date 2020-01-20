// duart.h

void uart_task(void *v);
esp_err_t get_saved_blob(char *, int);
int getArgs(char *, char **, int);
esp_err_t save_nm_blob(char *, char *);
esp_err_t get_named_blob(char *, char *, int);
