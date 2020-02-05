// make ESPBAUD=921600 ESPPORT=/dev/ttyUx

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
//#include "cmd_decl.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <esp_log.h>

#include <esp_task_wdt.h>

//static const char* TAG = "defi";

#define CONFIG_ESP_CONSOLE_UART_NUM 0
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE 115200

static void
initialize_console(void)
{
   /* Drain stdout before reconfiguring it */
   fflush(stdout);
   fsync(fileno(stdout));

   /* Disable buffering on stdin */
   setvbuf(stdin, NULL, _IONBF, 0);

   /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
   esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
   /* Move the caret to the beginning of the next line on '\n' */
   esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

   /* Configure UART. Note that REF_TICK is used so that the baud rate remains
    * correct while APB frequency is changing in light sleep mode.
    */
   const uart_config_t uart_config = {
           .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
           .data_bits = UART_DATA_8_BITS,
           .parity = UART_PARITY_DISABLE,

           .stop_bits = UART_STOP_BITS_1,
#ifdef UART_SCLK_REF_TICK
           .source_clk = UART_SCLK_REF_TICK,
#endif
   };
   /* Install UART driver for interrupt-driven reads and writes */
   ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
           256, 0, 0, NULL, 0) );
   ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

   /* Tell VFS to use UART driver */
   esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

   /* Initialize the console */
   esp_console_config_t console_config = {
           .max_cmdline_args = 8,
           .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
           .hint_color = atoi(LOG_COLOR_CYAN)
#endif
   };
   ESP_ERROR_CHECK( esp_console_init(&console_config) );

   /* Configure linenoise line completion library */
   /* Enable multiline editing. If not set, long commands will scroll within single line.  */
   linenoiseSetMultiLine(1);

   /* Tell linenoise where to get command completions and hints */
   linenoiseSetCompletionCallback(&esp_console_get_completion);
   linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

   /* Set command history size */
   linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
   /* Load command history from filesystem */
   linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

int
getArgs(char *lp, char **Ar, int nargs)
{
    char **ar = Ar, *p, q;

    p = lp;
    while(*p) {
    if((ar - Ar) == nargs)
      break;
    while((*p == ' ') || (*p == '\t'))
      p++;
    if(*p == '"' || *p=='\'')
      q = *p++;
    else
      q = 0;
    *ar++ = p;
    while(*p) {
      if(q) {
     if(*p == q) {
       *p++ = 0;
       break;
     }
      } else
    if((*p == ' ') || (*p == '\t')) {
       *p++ = 0;
       break;
     }
      p++;
    }
    }
    *ar = 0;
    return ar - Ar;
}

#define STORAGE_NAMESPACE "storage"

/*---*/
esp_err_t save_nm_str(char *sv, char *nv_name)
{
    nvs_handle anvs_handle;
    esp_err_t err;

    err = nvs_flash_init();
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &anvs_handle);
ESP_ERROR_CHECK(err);
    if (err != ESP_OK) {
        printf("save_nm_str open err:%d\n", err);
	return err;
    }

    size_t required_size = 0;  // value will default to 0, if not set yet in NVS

    // Read previously saved str if available
    uint32_t* run_time = malloc(sizeof(uint32_t) + 70);

    // Write value including string size 32 bit
    if(required_size < 200) { //limit size!
        required_size = sizeof(uint32_t);
        required_size += strlen(sv) + 1;
    }
    //run_time[0] = required_size;
    strcpy((char *)(run_time), sv);

    printf("save_nm_str rqd size:%d\n", required_size);
    printf("save_nm_str nv_name:%s\n", nv_name);
    printf("save_nm_str string<%s>\n", (char *)(run_time));
    err = nvs_set_str(anvs_handle, nv_name, (char *)run_time);
    free(run_time);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(anvs_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(anvs_handle);
    return ESP_OK;
}

esp_err_t get_named_str(char *saved, char *svname, int savemax)
{
    nvs_handle my_handle;
    esp_err_t err;

    err = nvs_flash_init(); //nvs_starter?
    saved[0] = (char)0;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS

    err = nvs_get_str(my_handle, svname, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    if (required_size == 0) {
        printf("get_named_str (%s): nothing saved yet!\n", svname);
    } else {
        printf("get_named_str rqd size %d only\n", required_size);
        err = nvs_get_str(my_handle, svname, saved, &required_size);
        if (err != ESP_OK) {
            return err;
        }
	//printf("get_named_str rq size %d, saved[0] %04X\n", required_size, saved[0]);
	//printf("get_named_str (%s) bytes <%x,%x,%x,%x,%x>\n", svname, saved[0], saved[1], saved[2], saved[3], saved[4]);
	//printf("get_named_str (%s) string <%s>\n", svname, saved);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

/*---*/

esp_err_t print_what_saved(void)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read restart counter
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "restart_conter", &restart_counter);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    printf("Restart counter = %d\n", restart_counter);

    // Read run time blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, "run_time", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    printf("a counter:\n");
    if (required_size == 0) {
        printf("Nothing saved yet!\n");
    } else {
        uint32_t* run_time = malloc(required_size);
        err = nvs_get_blob(my_handle, "run_time", run_time, &required_size);
        if (err != ESP_OK) {
            free(run_time);
            return err;
        }
        //for (int i = 0; i < required_size / sizeof(uint32_t); i++) {
        //    printf("%d: %d\n", i + 1, run_time[i]);
        //}
	printf("read blob rq size %d, run_time[0] %04X\n", required_size, run_time[0]);
	printf("read blob string <%s>\n", (char *)(run_time+1));

        char **arsplit;
        arsplit = malloc(sizeof(char *) * 20);
        int arcnt = getArgs((char *)(run_time+1), arsplit, 20);
	printf("arcnt %d, 1st <%s>\n", arcnt, arsplit[0]);

        free(run_time);
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}
esp_err_t save_a_counter(int val)
{
    nvs_handle nvs1_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs1_handle);
    if (err != ESP_OK) return err;

    // Read
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(nvs1_handle, "restart_conter", &restart_counter);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

/*
    // Write ?
    restart_counter = val;
    err = nvs_set_i32(nvs1_handle, "restart_conter", restart_counter);
    if (err != ESP_OK) return err;
*/
 
    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(nvs1_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(nvs1_handle);
    return ESP_OK;
}

/* Save new blob value in NVS
 */
esp_err_t save_a_blob(char *sv)
{
    nvs_handle anvs_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &anvs_handle);
    if (err != ESP_OK) return err;

    // do not read the size of memory space required for blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    //err = nvs_get_blob(anvs_handle, "run_time", NULL, &required_size);
    //if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Read previously saved blob if available
    uint32_t* run_time = malloc(sizeof(uint32_t) + 70);
#if 0
    if (required_size > 0) {
        err = nvs_get_blob(anvs_handle, "run_time", run_time, &required_size);
        if (err != ESP_OK) {
            free(run_time);
            return err;
        }
    }
#endif

    // Write value including string size 32 bit
    if(required_size < 200) { //limit size!
        required_size = sizeof(uint32_t);
        required_size += strlen(sv) + 1;
    }
    //run_time[required_size / sizeof(uint32_t) - 1] = xTaskGetTickCount() * portTICK_PERIOD_MS;
    run_time[0] = required_size;
    strcpy((char *)(run_time+1), sv);

    printf("save rqd size:%d\n", required_size);
    printf("save string <%s>\n", (char *)(run_time+1));
    err = nvs_set_blob(anvs_handle, "run_time", run_time, required_size);
    free(run_time);

    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(anvs_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(anvs_handle);
    return ESP_OK;
}

void nvs_starter()
{
    esp_err_t err = nvs_flash_init();
//nvs_flash_erase();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    //err = print_what_saved();
    //if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));

    //err = save_a_counter(210);
    //if (err != ESP_OK) printf("Error (%s) saving restart counter to NVS!\n", esp_err_to_name(err));

    //err = save_a_blob("A0 123 456");
    //if (err != ESP_OK) printf("Error (%s) saving blob to NVS!\n", esp_err_to_name(err));
}

#define SVLINEMAX 70
void loadnmstr(int *out, char *name, int maxint)
{
    char sv2[SVLINEMAX] = "^";
    int nargs = 0;
    char **arline;
    get_named_str(sv2, name, SVLINEMAX);
    arline = malloc(sizeof(char *) * 10);
    nargs = getArgs(sv2, arline, 10);
    printf("loadnmstr nargs=%d {", nargs);
    for(int j=0; j<nargs; j++) { printf("[%d:%s (%d)] ", j, arline[j], atoi(arline[j])); }
    printf("}\n");
    for(int j=1; j<nargs; j++) { //skip 1st (name)
	if(arline[j][0] == 'E') {
	    out[j-1] = -1;
	    break;
	}
        out[j-1] = atoi(arline[j]);
    }
}

char xline[SVLINEMAX];

void
uart_task(void *v)
{
  int doneflag = 0;
  char svline[SVLINEMAX] = "@";
  char sv2[SVLINEMAX] =    "#";

 initialize_console();
 const char *prompt = LOG_COLOR_I "defi> " LOG_RESET_COLOR;

 int probe_status = linenoiseProbe();
 if (probe_status) { /* zero indicates success */
   printf("\n"
    "Your terminal application does not support escape sequences.\n"
    "Line editing and history features are disabled.\n"
    "On Windows, try using Putty instead.\n");
   linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
   /* Since the terminal doesn't support escape sequences,
    * don't use color codes in the prompt.
    */
   prompt = "defi> ";
#endif //CONFIG_LOG_COLORS
   }
  int nargs = 0;
  char **arline;
  arline = malloc(sizeof(char *) * 10);
  char *mem = malloc(sizeof(char) * 80);
 
   while(true) {
       /* Get a line using linenoise.  */
       char* line = linenoise(prompt);
       if (line == NULL) { /* Ignore empty lines */
           continue;
       }
       if(strncmp(line, "done", 4) == 0) {
         linenoiseFree(line);
	 doneflag = 1;
         break;
       }
       if(strncmp(line, "save", 4) == 0) {
         strncpy(svline, (char *)(line+5), SVLINEMAX); //"save "
         linenoiseFree(line);
	 doneflag = 2;
         break;
       }
       if(strncmp(line, "erase", 5) == 0) {
	 doneflag = 3;
         break;
       }
       if(strncmp(line, "nsave", 5) == 0) {
         strncpy(svline, (char *)(line+6), SVLINEMAX); //"nsave "
         linenoiseFree(line);
	 doneflag = 4;
         break;
       }
       if(strncmp(line, "reads", 5) == 0) {
	 doneflag = 5;
         nargs = getArgs(line, arline, 10);

         if(nargs>1) {
           printf("reads:%s\n", arline[1]);
           get_named_str(svline, arline[1], SVLINEMAX);
           printf("reads:[%s] <%s>\n", arline[1], svline);
           strncpy(xline, svline, SVLINEMAX); //last reads for export

           nargs = getArgs(svline, arline, 10);
           printf("reads nargs=%d\n<<", nargs);
           for(int j=0; j<nargs; j++) { printf("%d:%s (%d),", j, arline[j], atoi(arline[j])); }
           printf(">>\n");
         }
         continue;
       }
       printf("line:<%s>\n", line);

nargs = getArgs(line, arline, 10);
printf("nargs=%d\n", nargs);
printf("arline=%s\n", *arline);
for(int j=0; j<nargs; j++) { printf("%d:%s (%d)\n", j, arline[j], atoi(arline[j])); } 
       /* linenoise allocates line buffer on the heap, so need to free it */
       linenoiseFree(line);
   }
printf("done!\n");
if(doneflag == 2) {
  printf("save str <%s> at %s\n", svline, "str0");
  save_nm_str(svline, "str0");
  //save_a_blob(svline);
}
if(doneflag == 3) {
  printf("erase.\n");
  nvs_flash_erase();
}
if(doneflag == 4) {
  printf("nsave.<%s>\n", svline);
  strncpy(sv2, svline, SVLINEMAX);
  nargs = getArgs(svline, arline, 10);
  printf("nsave nargs %d [%s]:<%s>\n", nargs, arline[0], sv2);
  save_nm_str(sv2, arline[0]);
}
if(doneflag == 1) {
  printf("get str:\n");
  int err = get_named_str(mem, "pm0", 80);
  ESP_ERROR_CHECK( err );
  printf("get str %s:%d <%s>\n", "pm0", err, mem);

  //strncpy(xline, mem, SVLINEMAX); //last pm0 on done
  printf("xline:<%s>\n", xline);
}
if(v != NULL)
  vTaskDelete(NULL);
}

/*
void
app_main(void)
{
  nvs_starter();

    uart_task(NULL);

    int* pulp = (int *)malloc(sizeof(int *) * 20);
    loadnmstr(pulp, "pm0", 20);
    printf("\npulp 1:"); for(int j=0; j<20; j++) { printf("%d:(%d),", j, pulp[j]); if(pulp[j]==(-1))break; } 
    loadnmstr(pulp, "pm2", 10);
    printf("\npulp 2:"); for(int j=0; j<20; j++) { printf("%d:(%d),", j, pulp[j]); if(pulp[j]==(-1))break; } 

    printf("realy done!\n");
}
*/
