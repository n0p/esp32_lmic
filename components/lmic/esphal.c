#include "lmic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"


static const char* TAG = "LMIC_HAL";

extern const lmic_pinmap_t lmic_pins;

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

// -----------------------------------------------------------------------------
// ISR handler
static void IRAM_ATTR gpio_isr_handler(void * arg)
{
    uint32_t dionum = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &dionum, NULL);
}


// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
  ESP_LOGI(TAG, "Starting IO initialization");

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL<<lmic_pins.rst);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  // Setup all DIO pins to use interrupts
  // DIOs are activated on the rising edge
  io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
  io_conf.pin_bit_mask = ((1ULL<<lmic_pins.dio[0]) | (1ULL<<lmic_pins.dio[1]) | (1ULL<<lmic_pins.dio[2]));
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  // Create a queue to handle the event from ISR
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  // hook isr handler to the specific GPIO pins
  gpio_isr_handler_add(lmic_pins.dio[0], gpio_isr_handler, (void *) 0);
  gpio_isr_handler_add(lmic_pins.dio[1], gpio_isr_handler, (void *) 1);
  gpio_isr_handler_add(lmic_pins.dio[2], gpio_isr_handler, (void *) 2);

  ESP_LOGI(TAG, "Finished IO initialization");
}

void hal_pin_rxtx (u1_t val) {
  // unused
}


// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.pin_bit_mask = (1<<lmic_pins.rst);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;

  if(val == 0 || val == 1) { // drive pin
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    gpio_set_level(lmic_pins.rst, val);
  } else { // keep pin floating
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
  }
}

// Called by the os_runloop routing to check
// if there are any pending interrupts that need
// to be services
void hal_io_check()
{
    uint32_t io_num;
    // Block 10 ticks if message is not immediately
    // available
    if (xQueueReceive(gpio_evt_queue, &io_num, (TickType_t) 10))
    {
        ESP_LOGI(TAG, "Received interrupt from DIO[%d]!", io_num);
        radio_irq_handler(io_num);
    }
}

// -----------------------------------------------------------------------------
// SPI

static spi_device_handle_t spi_handle;

static void hal_spi_init (uint8_t spihost) 
{
  ESP_LOGI(TAG, "Starting SPI initialization");
  esp_err_t ret;

  // We assume SPI bus master is already
  // initialized before a call to spi_bus_add_device()
  // below.

  // initialize the spi device
  spi_device_interface_config_t devcfg={
    .clock_speed_hz = 10000000,
    .mode = 1,
    .spics_io_num = lmic_pins.nss,
    .queue_size = 7,
    .address_bits = 8,
  };

  // Attach this device to the SPI Bus
  ret = spi_bus_add_device((spi_host_device_t) spihost, &devcfg, &spi_handle);
  assert(ret==ESP_OK);

  ESP_LOGI(TAG, "Finished SPI initialization");
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t data) {
  uint8_t rxData = 0;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8;
  t.rxlength = 8;
  t.tx_buffer = &data;
  t.rx_buffer = &rxData;
  esp_err_t ret = spi_device_transmit(spi_handle, &t);
  assert(ret == ESP_OK);

  return (u1_t) rxData;
}

int hal_spi_transfer(uint8_t addr, uint8_t * txdata, uint8_t * rxdata, size_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = size * 8; // size in bits
    t.rx_buffer = rxdata;
    t.tx_buffer = txdata;
    t.addr = addr;

    if (spi_device_transmit(spi_handle, &t) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error on SPI Transfer!");
        return 0;
    }

    return size;
}

// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () 
{
  ESP_LOGI(TAG, "Starting initialisation of timer");
  int timer_group = TIMER_GROUP_0;
  int timer_idx = TIMER_1;
  timer_config_t config;
  config.alarm_en = 0;
  config.auto_reload = 0;
  config.counter_dir = TIMER_COUNT_UP;
  config.divider = TIMER_DIVIDER;
  config.intr_type = 0;
  config.counter_en = TIMER_PAUSE;
  /*Configure timer*/
  timer_init(timer_group, timer_idx, &config);
  /*Stop timer counter*/
  timer_pause(timer_group, timer_idx);
  /*Load counter value */
  timer_set_counter_value(timer_group, timer_idx, 0x0);
  /*Start timer counter*/
  timer_start(timer_group, timer_idx);

  ESP_LOGI(TAG, "Finished initalisation of timer");
}

ll_u8_t hal_ticks () {
  uint64_t val;
  timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &val);
  return (ll_u8_t) val;
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static ll_s8_t delta_time(ll_u8_t time) {
    return (ll_s8_t)(time - hal_ticks());
}


void hal_waitUntil (ll_u8_t time) {

    ESP_LOGI(TAG, "Wait until (%lld)", time);
    ll_s8_t delta = delta_time(time);
    ESP_LOGI(TAG, "Wait for %lld (%d ms)", delta, osticks2ms(delta));
    while(hal_ticks() < time)
    {
        // yield to higher priority task
        taskYIELD();
    }

    ESP_LOGI(TAG, "Done waiting until");
}

// check and rewind for target time
u1_t hal_checkTimer (ll_u8_t time) {
  return 1;
}

int x_irq_level = 0;

// -----------------------------------------------------------------------------
// IRQ
// hal_disableIRQs()/hal_enableIRQs() in LMIC act more like mutex
// to prevent execution when one task is using a certain resource (such as the radio).
// So we use mutex here in place of truly disabling interrupts.
void hal_disableIRQs () {
  if(x_irq_level < 1){
     //taskDISABLE_INTERRUPTS();
  }
  x_irq_level++;
}

void hal_enableIRQs () {
  if(--x_irq_level == 0){
    //taskENABLE_INTERRUPTS();
  }
}

void hal_sleep () {
  // unused
}

// -----------------------------------------------------------------------------

void hal_init (uint8_t spi) 
{
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init(spi);
    // configure timer and interrupt handler
    hal_time_init();
}

void hal_failed () {
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}
