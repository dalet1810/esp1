// duart.h

void uart_task(void *v);
esp_err_t get_saved_blob(char *, int);
int getArgs(char *, char **, int);
esp_err_t save_nm_blob(char *, char *);
esp_err_t get_named_blob(char *, char *, int);

esp_err_t save_nm_str(char *, char *);
esp_err_t get_named_str(char *, char *, int);

void nvs_starter();
void loadnmstr(int *out, char *name, int maxout);

#define SVLINEMAX 70
char xline[SVLINEMAX];
int xvec[30];
