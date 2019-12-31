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
   /* Enable multiline editing. If not set, long commands will scroll within
    * single line.
    */
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

/*
static void
initialize_nvs(void)
{
   esp_err_t err = nvs_flash_init();
   if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
       ESP_ERROR_CHECK( nvs_flash_erase() );
       err = nvs_flash_init();
   }
   ESP_ERROR_CHECK(err);
}
*/

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

static void
uart_task(void *v)
{
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
  //char **AR;
  //AR = malloc(sizeof(char *) * 10);
 
   while(true) {
       /* Get a line using linenoise.
        * The line is returned when ENTER is pressed.
        */
       char* line = linenoise(prompt);
       if (line == NULL) { /* Ignore empty lines */
           continue;
       }
       if(strncmp(line, "done", 4) == 0) {
         linenoiseFree(line);
         break;
       }
       printf("line:<%s>\n", line);

char *lp = strdup(line);
int nargs = getArgs(lp, &line, 10);
printf("nargs=%d\n", nargs);
for(int j=0; j<nargs; j++) { printf("%d:%d\n", j, lp[j]); } 
       /* linenoise allocates line buffer on the heap, so need to free it */
       linenoiseFree(line);
   }
printf("done!\n");
if(v != NULL)
  vTaskDelete(NULL);
}

void
app_main(void)
{
 //initialize_nvs();
#if 0
 char *pv = "";

BaseType_t ret;
ret = xTaskCreate(uart_task, "uart console",
      4*1024, // usStackDepth */
      (void *)pv,
      tskIDLE_PRIORITY, // priority
      NULL //&pv->xHandle
  );
if(ret != pdPASS) {
  printf("failed to start uart_task!\n");
}
#else
    uart_task(NULL);
    printf("realy done!\n");
#endif

}
