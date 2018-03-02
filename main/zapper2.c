/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_err.h"
#include "driver/timer.h"
#include "driver/spi_slave.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include <byteswap.h>

//#define DEBUG_REMOTE
#define DEBUG_ACCEL


#define ACCEL_X_DOWN 12000
#define ACCEL_Y_DOWN -2000
#define ACCEL_Z_DOWN 13500
#define STILLNESS 3800
#define BUTTON_LOCKDOWN 0xef8afc
#define BUTTON_LOCKDOWN_SET 0xef8af0 /*both buttons*/
#define BUTTON_RELEASE 0xef8af3


#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (3)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)


#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1  
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0 
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0 
#define I2C_EXAMPLE_MASTER_SCL_IO          26
#define I2C_EXAMPLE_MASTER_SDA_IO          25
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000


#define GPIO_ZAP 15
#define GPIO_VIBE 2

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
#define EXAMPLE_WIFI_SSID "PublicWork" //"interstacks1" //MakerswarmDemo
#define EXAMPLE_WIFI_PASS "publicnet63!" //"stax15222"    //lotsofbots

short sample[8];
short lock[3];
char lockdown = 0;
void lock_position() {
  lock[0] = sample[0];
  lock[1] = sample[1];
  lock[2] = sample[2];
}

static const char *TAG = "example";


    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER,
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .timer_sel  = LEDC_LS_TIMER
        },
    };


void zap(int i) {
  if(i) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_ZAP));
  } else {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_ZAP));
  }
}

void vibe(int i) {
  if(i) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_VIBE));
  } else {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_VIBE));
  }
}

static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}



//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
  //printf("SETUP\n");
  //WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<GPIO_HANDSHAKE));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
  //printf("TRANS\n");  
  //WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<GPIO_HANDSHAKE));
}

static uint32_t isr_pulse_len = 0;
static uint32_t rf_inter_len = 0;
static uint32_t rf_bits = 0;
static uint32_t rf_last = 0;
static uint32_t rf_lastcnt = 0;
static uint32_t rf_cnt = 0;
static char rf_state = 0;
static char rf_sent = 0;
static xQueueHandle gpio_evt_queue = NULL;
static intr_handle_t s_timer_handle;

static void timer_isr(void* arg)
{
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
    if(isr_pulse_len < 0x70000000) 
      isr_pulse_len += 1;
    if(rf_inter_len < 0x70000000) {
      rf_inter_len += 1;
      if(rf_sent && rf_inter_len > 8000) {
        int arg = 0;
        rf_sent = 0;
        xQueueSendFromISR(gpio_evt_queue, &arg, NULL);
      }
    }
    
}
#define ESP_INTR_FLAG_DEFAULT 0
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t lvl = gpio_get_level(gpio_num);
    if(rf_state == 0) {
      if(isr_pulse_len > 300) { //start bit
#ifdef DEBUG_REMOTE
        gpio_num = isr_pulse_len;
        //xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);        
#endif
        rf_cnt = 0;
        rf_bits = 0;
        rf_state = 1;
      }
    } else {
      int pt = (isr_pulse_len > 8 && isr_pulse_len < 19) ? 1 :
        (isr_pulse_len > 32 && isr_pulse_len < 40) ? -1 : 0;
#ifdef DEBUG_REMOTE
      if(rf_cnt == 0) {
        gpio_num = isr_pulse_len;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
      }
#endif
      if(!pt) {
        //gpio_num = isr_pulse_len;
        //xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);        
        rf_state = 0;
      } else if(pt > 0) { //short
        if(rf_state == 1) {
          rf_state = 2;
        } else if(rf_state == 3) {
          rf_bits <<= 1; //0 bit
          rf_cnt += 1;
          rf_state = 1;          
        } else {
          rf_state = 0;
        }
      } else { //long
        if(rf_state == 1) {
          rf_state = 3;
        } else if(rf_state == 2) {
          rf_bits = 1 | (rf_bits<<1); //1-bit
          rf_cnt += 1;          
          rf_state = 1;
        } else {
          rf_state = 0;
        }
      }
    }
    if(rf_cnt == 24) {
      rf_state = 0;
      rf_cnt = 0;
      if(rf_last == rf_bits && rf_inter_len < 3500) {
        rf_lastcnt += 1;
        if(rf_lastcnt == 2) {
#ifndef DEBUG_REMOTE          
          xQueueSendFromISR(gpio_evt_queue, &rf_bits, NULL);
#endif
          rf_sent = 1;
        }
        rf_inter_len = 0;
      } else {
        rf_lastcnt = 1;
        rf_last = rf_bits;
        rf_inter_len = 0;        
      }
    }
    isr_pulse_len = 0;
}
void init_timer(int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,
            .counter_en = false,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = true,
            .divider = 119   /* 1 us per tick */
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, NULL, 0, &s_timer_handle);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

ledc_timer_config_t ledc_timer = {
  .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
  .freq_hz = 1000,                      // frequency of PWM signal
  .speed_mode = LEDC_HS_MODE,           // timer mode
  .timer_num = LEDC_HS_TIMER            // timer index
};

//1Khz - 10Khz seems to work
void beep_on(int freq) {
  ledc_timer.freq_hz = freq;
  ledc_timer_config(&ledc_timer);
  ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 4000);
  ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
  zap(1);
  vibe(1);
}
void beep_off() {
  ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel,0);
  ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);  
  //ledc_stop(ledc_channel[0].speed_mode, ledc_channel[0].channel,0);
    zap(0);
  vibe(0);    
}


static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
#ifdef DEBUG_REMOTE
          printf("DBG: %d\n", io_num);
#else      
          if(io_num) beep_on(2000);
          else beep_off();
          printf("BUTTON: %x\n", io_num);
#endif
          if(io_num == BUTTON_LOCKDOWN_SET) {
            lock_position();
            lockdown = 1;            
          }
          if(io_num == BUTTON_LOCKDOWN) {
            lock_position();
            lockdown = 1;
          }
          if(io_num == BUTTON_RELEASE) {
            lockdown = 0;
          }
          
        }
    }
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
      printf("connecting %d\n",event->event_id);
      ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
      printf("got ip %d\n",event->event_id);            
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      printf("discon %d\n",event->event_id);      
      ESP_ERROR_CHECK(esp_wifi_connect());
      xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}


void init_wifi() {
  
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  //  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  
  wifi_config_t wifi_config = {
    .sta = {
      .ssid = EXAMPLE_WIFI_SSID,
      .password = EXAMPLE_WIFI_PASS,
      //      .scan_method = DEFAULT_SCAN_METHOD,
      //      .sort_method = DEFAULT_SORT_METHOD,
      //      .threshold.rssi = DEFAULT_RSSI,
      //      .threshold.authmode = DEFAULT_AUTHMODE,
    },
  };
  //strcpy((char*) &wifi_config.sta.password[0],"stax15222");
  
  ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA)); //STA) );
  
  ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  
#if 0
  ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_ca_cert(ca_pem_start, ca_pem_bytes) );
  ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_cert_key(client_crt_start, client_crt_bytes, \
                                                      client_key_start, client_key_bytes, NULL, 0) );
  ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EXAMPLE_EAP_ID, strlen(EXAMPLE_EAP_ID)) );
  if (EXAMPLE_EAP_METHOD == EAP_PEAP || EXAMPLE_EAP_METHOD == EAP_TTLS) {
    ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EXAMPLE_EAP_USERNAME, strlen(EXAMPLE_EAP_USERNAME)) );
    ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EXAMPLE_EAP_PASSWORD, strlen(EXAMPLE_EAP_PASSWORD)) );
  }  
  ESP_ERROR_CHECK( esp_wifi_sta_wpa2_ent_enable(&config) );
  //  ESP_ERROR_CHECK( esp_wifi_start() );
#endif
  ESP_ERROR_CHECK( esp_wifi_start() );
}


#define SENSOR_ADDR (0x68<<1)
#define WRITE_BIT                          I2C_MASTER_WRITE
#define ACK_CHECK_EN                       0x1
#define CMD_START 0x23
static void i2c_test_task(void* arg) {
  i2c_port_t i2c_num = I2C_EXAMPLE_MASTER_NUM;
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, SENSOR_ADDR | WRITE_BIT, 1);
  //write 0x6b=0 ; power on
  i2c_master_write_byte(cmd, 0x6B , 1);
  i2c_master_write_byte(cmd, 0, 1);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  while(1) {
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_ADDR | WRITE_BIT, 1);
    i2c_master_write_byte(cmd, 0x3b, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SENSOR_ADDR | READ_BIT, 1);
    i2c_master_read(cmd, &sample[0], 13, ACK_VAL);
    i2c_master_read_byte(cmd, &sample[13], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //read 14 (accel) Xh, Xl, Yh, Yl, Zh, Zl, TempH, TempL, (gyro) Xh,X, y,y, z,z
    int i;
    for(i=0; i<8; i++) {
      sample[i] = __bswap_16(sample[i]);
    }
    
    //could be 333ms
    vTaskDelay(333 / portTICK_RATE_MS);
    if(lockdown) {
      int diffdown =
        abs(sample[0] - lock[0]) +
        abs(sample[1] - lock[1]) +
        abs(sample[2] - lock[2]);
      printf("D=%d\n",diffdown);
      if(diffdown > STILLNESS) beep_on(2000);
      else beep_off();
    }
#ifdef DEBUG_ACCELx
    printf("X=%hd %hd %hd + %hd + %hd %hd %hd\n",
           sample[0],sample[1],sample[2],sample[3],sample[4],sample[5],sample[6],sample[7]);
#endif
  }

}

void app_main()
{
    int ch;
    int n=0;
    esp_err_t ret;

    gpio_config_t io_conf;

    zap(0);
    vibe(0);
    lockdown = 0;
    
    ESP_ERROR_CHECK( nvs_flash_init() ); //need this for WIFI to work.
    
#if 0 //GPIO2 output  -- later
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_HANDSHAKE) | 0; //pin 
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
#endif

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<< GPIO_ZAP) | (1ULL<< GPIO_VIBE) ; //pin 25
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    init_timer(10); //10uSec
    
    init_wifi();
    
    io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE; //POSEDGE; //interrupt of rising edge
    io_conf.pin_bit_mask = (1ULL<<4) | 0; //pin 4
    io_conf.mode = GPIO_MODE_INPUT;     //set as input mode    
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);     //install gpio isr service
    gpio_isr_handler_add(4, gpio_isr_handler, (void*) 4); //gpio 4
    //gpio_isr_handler_remove(GPIO_INPUT_IO_0);

    
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
#if 0 //don't need these w/ hard connection - they tend to interfere
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);
#endif
    
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */

#if 0
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);
#endif
    
    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
    i2c_example_master_init();
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);    


    while(1) {
        
#if 0
        printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
      
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("2. LEDC fade down to duty = 0\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("4. LEDC set duty = 0 without fade\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
#endif
        
        vTaskDelay(10000 / portTICK_PERIOD_MS);

        {
          tcpip_adapter_ip_info_t ip;
#if 0
          if (tcpip_adapter_get_ip_info(ESP_IF_WIFI_STA, &ip) == 0) {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "IP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "MASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "GW:"IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
          }
#endif
        }
    }
}

