/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "osapi.h"
#include "at_custom.h"
#include "user_interface.h"
#include "mem.h"
#include "espconn.h"
#include "secret.h"

//Range: 10000 ~ 268435455(0xfffffff) us
#define FPM_SLEEP_MAX_TIME 0xFFFFFFF

static os_timer_t blink_timer;
static const int pin = 1;
static volatile int count = 0;

//192.168.0.27 =  0xc0a8001b
static const char remote_ip[4] = {192, 168, 0, 27};


static void ICACHE_FLASH_ATTR
at_exeCmdTest(uint8_t id)
{
    count = 0;
    at_response_ok();
}

static void ICACHE_FLASH_ATTR wifi_init(struct espconn *esp_conn)
{

    // UDP specific protocol structure.
    static esp_udp udp_proto;

    struct ip_info ipinfo;
    // Set station mode - we will talk to a WiFi router.
    wifi_set_opmode_current(STATION_MODE);

    // Set up the network name and password.
    struct station_config sc;
    strncpy(sc.ssid, SSID, 32);
    strncpy(sc.password, PASSWD, 64);
    wifi_station_set_config(&sc);
    wifi_station_dhcpc_start();

    wifi_get_ip_info(STATION_IF, &ipinfo);


    udp_proto.remote_port = 3456;
    udp_proto.local_port = 1234;

    memcpy(udp_proto.remote_ip, remote_ip, 4);
    
    memcpy(udp_proto.local_ip, &ipinfo.ip.addr, 4);


    esp_conn->type = ESPCONN_UDP;
    esp_conn->state = ESPCONN_NONE;
    esp_conn->proto.udp = &udp_proto;

    espconn_create(esp_conn);
//    espconn_regist_recvcb(&udp_conn, recv_cb);
}

/*
 * Call-back for when the blink timer expires. This toggles the GPIO 4 state, and send a UDP packet about the new state
 */
static void ICACHE_FLASH_ATTR blink_cb(void *arg)
{
    struct espconn *esp_conn = arg;

  //Do blinky stuff
  if (GPIO_REG_READ(GPIO_IN_ADDRESS) & (1<<pin))
  {
    // set gpio low
   // gpio_output_set(0, (1<<pin), 0, 0);
   count++;

    espconn_sendto(esp_conn,"smellypanda%d\n", 13);
  }
  else
  {
    // set gpio high
    //gpio_output_set((1<<pin), 0, 0, 0);
    espconn_sendto(esp_conn,"pandasmelly\n", 13);
  }


    // Send a UDP packet 
    //char message[16];
    //int len = os_sprintf(message, "LED state - %d.\n", led_state);
}

/*
 * Sets the interval of the timer controlling the blinking of the LED.
 */
static void ICACHE_FLASH_ATTR set_blink_timer(uint16_t interval, struct espconn *esp_conn)
{
    // configure UART TXD to be GPIO1, set as input
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1);
    gpio_output_set(0, 0, 0, (1<<pin));

    os_timer_disarm(&blink_timer);
    os_timer_setfn(&blink_timer, (os_timer_func_t *)blink_cb, esp_conn);
    os_timer_arm(&blink_timer, interval, 1);
}

at_funcationType at_custom_cmd[] = {
    {"+TEST", 5, NULL, NULL, NULL, at_exeCmdTest},
};

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/

uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR user_rf_pre_init(void)
{
    system_phy_freq_trace_enable(at_get_rf_auto_trace_from_flash());
}

// interrupt handler this function will be executed on any edge of GPIO0
static void  gpio_intr_handler(void * args)
{
    struct espconn *esp_conn;
    esp_conn = args;

    uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);

    // if the interrupt was by GPIO2
    if (gpio_status & BIT(2))
    {
        //clear interrupt status for GPIO2
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(2));

        // Send a UDP packet
        char message[16];
        int len = os_sprintf(message, "smellypanda - %d.\n", count);
        espconn_sendto(esp_conn,message, len);
    }
}

void ICACHE_FLASH_ATTR configure_gpio(struct espconn *esp_conn)
{
    uint32_t gpio_status;

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO2_U);
    //arg1=high arg2=low, arg3=enable(mask), arg4=0(output),input
    gpio_output_set(0, 0, 0, GPIO_ID_PIN(2)); // set gpio 2 as input

    // Disable interrupts by GPIO
    ETS_GPIO_INTR_DISABLE();

    // Attach interrupt handle to gpio interrupts.
    // You can set a void pointer that will be passed to interrupt handler each interrupt

    ETS_GPIO_INTR_ATTACH(gpio_intr_handler, esp_conn);

    //clear interrupt status
    gpio_status	= GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

    // enable interrupt for GPIO
     gpio_pin_intr_state_set(GPIO_ID_PIN(2), GPIO_PIN_INTR_NEGEDGE);

     ETS_GPIO_INTR_ENABLE();
}


void fpm_wakup_cb_func(void)
{        
    count++;    
        
//  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
    
    wifi_fpm_close();
    wifi_set_opmode(STATION_MODE);
    wifi_station_connect();
}


void ICACHE_FLASH_ATTR prepare_sleep()
{ 
    os_printf("prepare_sleep\n");
    wifi_station_disconnect();
    
    // set WiFi mode to null mode.
    wifi_set_opmode(NULL_MODE);

    // light sleep
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

    // enable force sleep
    wifi_fpm_open();
    
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
    PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO0_U);
    //arg1=high arg2=low, arg3=enable(mask), arg4=0(output),input
    gpio_output_set(0, 0, 0, GPIO_ID_PIN(0)); // set gpio 0 as input

    wifi_enable_gpio_wakeup(0, GPIO_PIN_INTR_LOLEVEL);
    
    // Set wakeup callback
    wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func);
    wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
    
}



void ICACHE_FLASH_ATTR user_init(void)
{
    static struct espconn esp_conn;

    at_init();

    at_cmd_array_regist(&at_custom_cmd[0], sizeof(at_custom_cmd)/sizeof(at_custom_cmd[0]));

    gpio_init();

    wifi_init(&esp_conn);

//    configure_gpio(&esp_conn);
    
    prepare_sleep();

    // Start the LED timer at 5s per change.
    //set_blink_timer(5000, &esp_conn);
}
