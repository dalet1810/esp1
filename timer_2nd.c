/* Timer group-hardware timer example
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#include "stdio.h"

#define BLINK_GPIO 18
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define TIMER_INTERVAL0_SEC   (0.01) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1.0)   // sample test interval for the second timer

//#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
//#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

static int flgpr=0;
int flop=0;
int tctr=0;

void gpio_task_input();

xQueueHandle timer_queue;

static xQueueHandle gpio_evt_queue = NULL;


/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    if(flgpr > 2)
        return;
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;

        gpio_set_level(BLINK_GPIO, flop);
        flop ^= 1;
        tctr += 1;

    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    //if( evt.type != TEST_WITHOUT_RELOAD)
    //    xQueueSendFromISR(timer_queue, &evt, NULL);

    if( tctr > 18 )
        xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    //timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //printf("starting timer %d with interval %f sec\n", timer_idx, timer_interval_sec);

    timer_start(TIMER_GROUP_0, timer_idx);
}

void tg0_timer_interrupt(int timer_idx, int strt)
{
    if(strt==1) {
        timer_enable_intr(TIMER_GROUP_0, timer_idx);
    } else {
        timer_disable_intr(TIMER_GROUP_0, timer_idx);
    }
}


/*
 * The main task of this example program
 */
static void timer_example_evt_task(void *arg)
{
    //extern int flgpr = 0;
    int flip=0;
    //while (1) {
    for (int i=0; i<20; i++) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        if(flgpr<2){flgpr++;}else{flgpr=10;}

        /* Print information that the timer reported an event */
        if (evt.type == TEST_WITHOUT_RELOAD) {
            if(flgpr<6)printf("\n    Example timer without reload\n");
            break;
        } else if (evt.type == TEST_WITH_RELOAD) {
            if(flgpr<6)printf("\n    Example timer with auto reload\n");
        } else {
            if(flgpr<6)printf("\n    UNKNOWN EVENT TYPE\n");
        }
        if(flgpr<6)printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);

        /* Print the timer values passed by event */
        if(flgpr<6)printf("------- EVENT TIME -------- flip:%d\n", flip);
        if(flgpr<6)print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        if(flgpr<6)printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value(evt.timer_group, evt.timer_idx, &task_counter_value);
        if(flgpr<6)print_timer_counter(task_counter_value);
    }
    printf("timer task done (%d)...\n", tctr);
    tg1_timer_interrupt(TEST_WITHOUT_RELOADP, 0);

    if(tctr>20) {
        tctr = 0;
    }
    //timer_disable_intr(0, 0);
    //vQueueDelete(timer_queue);

/*
    int v=0;
    for(int p=0; p<=20; p++ ) {
        gpio_set_level(BLINK_GPIO, v);
        v ^= 1;
        vTaskDelay(TIMER_INTERVAL0_SEC / portTICK_PERIOD_MS);
    }
*/
}

/*
 * In this example, we will test hardware timer0 and timer1 of timer group0.
 */
void app_main()
{

        vTaskDelay(1 / portTICK_PERIOD_MS);

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    //input handling
    void inpin_conf();
    inpin_conf();

    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC); //no interrupt
    if(0)
        example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    printf("interval0:(%f)\n", TIMER_INTERVAL0_SEC);
    printf("NOT USED: interval1:(%f)\n", TIMER_INTERVAL1_SEC);


    gpio_pad_select_gpio(17);
    gpio_set_direction(17, GPIO_MODE_OUTPUT);
    gpio_set_level(17, 0);

/*
setbuf(stdout, NULL);
    printf("when ready hit c...");
    while(getchar() != 'c') {
        vTaskDelay(0.1 / portTICK_PERIOD_MS);
    }
    printf("ready...\n");

*/
    gpio_set_level(17, 1);
    while(1) {
        gpio_task_input();
        vTaskDelay(5.0 / portTICK_PERIOD_MS);
    }

}

void start_timed_pulse()
{
    //example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    tg0_timer_interrupt(TEST_WITHOUT_RELOADP, 1);
}

//input pin with interrupt
//TaskHandle_t inp_Handle;

void gpio_task_input()
{
    uint32_t io_num;
    for(int i=0; i<10; i++) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            start_timed_pulse(i);
            break;
        }
    }
    //vTaskDelete(inp_handle);
}
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
void inpin_conf()
{
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    //xTaskCreate(gpio_task_input, "gpio_task_input", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
}
