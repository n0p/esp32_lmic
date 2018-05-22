#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

#include "lmic.h"
#include "I2CMaster.h"
#include "SPIMaster.h"
#include "SSD1306.h"
#include "HTU21D.h"
#include "CayenneLPP.h"

u1_t NWKSKEY[16] = { 0xD6, 0x6D, 0xC7, 0x16, 0xE7, 0x90, 0xB8, 0x39, 0x8A, 0x6A, 0xE9, 0xA4, 0x15, 0x48, 0x21, 0x69 };
u1_t APPSKEY[16] = { 0x29, 0x0E, 0x5C, 0xF7, 0xA1, 0x41, 0x3C, 0x5A, 0x8B, 0xF9, 0x12, 0x96, 0xE3, 0xBA, 0x54, 0x40 };
u4_t DEVADDR = 0x26041938 ;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static const char *TAG = "MAIN";

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;
static const int deep_sleep_sec = 10; // 60 * 5; // Sleep every five minutes

// We initialize the SPI master here
SPIMaster spi(HSPI_HOST, GPIO_NUM_19, GPIO_NUM_27, GPIO_NUM_5); 
// Using the TTGO ESP32 Lora or Heltec ESP32 Lora board
// https://www.thethingsnetwork.org/forum/t/big-esp32-sx127x-topic-part-2/11973
extern "C" const lmic_pinmap_t lmic_pins = {
    .nss = 18,
    .rst = 14,
    .dio = {26, 33, 32},
    // MISO, MOSI, SCK
    //.spi = {19, 27, 5},
};


// I2CMaster (I2C Num, SDA, SCL)
I2CMaster i2c(I2C_NUM_1, GPIO_NUM_4, GPIO_NUM_15);
SSD1306   ssd(i2c, GPIO_NUM_16);
HTU21D    htu(i2c);
CayenneLPP lpp;


// Time to linger before going to deep sleep
const unsigned LINGER_TIME = 5000;

extern "C" void do_deepsleep(osjob_t * arg)
{
    // Turn off oled display
    ssd.PowerOff();
    // Enter deep sleep
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);    
}

extern "C" void do_send(osjob_t * arg)
{
    char tmpbuff[50];
    lpp.reset();

    ESP_LOGD(TAG, "do_send() called! ... Sending data!");
    if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGI(TAG, "OP_TXRXPEND, not sending!");
    } else {
        float temperature;
        float humidity;
        ssd.Fill(SSD1306::Black);
        // Prepare upstream data transmission at the next possible time.
        if (htu.readTemperature(&temperature))
        {
            sprintf(tmpbuff, "Temperature : %.2f C", temperature);
            ssd.GotoXY(0, 15);
            ssd.Puts(&tmpbuff[0], &Font_7x10, SSD1306::White);
            lpp.addTemperature(1, temperature);
            ESP_LOGI(TAG, "HTU21D Temperature : %.2f C", temperature);
        }
        if (htu.readHumidity(&humidity))
        {
            sprintf(tmpbuff, "Humidity   : %.2f%%", humidity);
            ssd.GotoXY(0,27);
            ssd.Puts(&tmpbuff[0], &Font_7x10, SSD1306::White);
            lpp.addRelativeHumidity(2, humidity);
            ESP_LOGI(TAG, "HTU21D Humidity : %.2f %%", humidity);
        }
        ssd.UpdateScreen();

        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0); 
        ESP_LOGD(TAG, "Packet queued");
    }
}

void do_receive()
{
    if (LMIC.dataLen > 0)
    {
        // TODO: Copy and process the data from LMIC.dataBeg to a buffer
        ESP_LOGD(TAG, "Received %d of data\n", LMIC.dataLen);
    }
}

// Callbacks from lmic, needs C linkage
extern "C" void onEvent (ev_t ev) {
    ESP_LOGI(TAG, "Event Time: %lld, %d", os_getTime(), ev);
    switch(ev) {
        case EV_TXCOMPLETE:
            ESP_LOGI(TAG, "EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
              ESP_LOGI(TAG, "Received ack");
            if (LMIC.dataLen) {
              ESP_LOGI(TAG, "Received %d bytes of payload\n", LMIC.dataLen);
            }
            // Schedule the send job at some dela
            os_setTimedCallback(&LMIC.osjob, os_getTime()+ms2osticks(LINGER_TIME), FUNC_ADDR(do_deepsleep));
            break;
         case EV_RXCOMPLETE:
            // data received in ping slot
            ESP_LOGI(TAG, "EV_RXCOMPLETE");
            do_receive();
            break;
          default:
            ESP_LOGI(TAG, "Unknown event: %d", ev);
            break;
    }
}

void os_runloop(void * arg) 
{
  // Update the send counter based on boot_count
  LMIC.seqnoUp = boot_count;
  // Send an Update to TTN
  ESP_LOGI(TAG, "Sending TTN Update!");
  do_send(NULL);
  // Run the lmic state machine
  // wait until we receive a TX_COMPLETE (transmission complete) event
  // and go in to deep sleep...
  while(true) {
    os_run();
  }
}

extern "C" void app_main(void)
{
  ++boot_count;
  ESP_LOGI(TAG, "Wake(%d) initializing ....", boot_count);
  // Devices attached to the I2C Bus
  i2c.init();
  ssd.init();
  htu.init();

  spi.init();
  // LMIC os_init has C linkage, make sure
  // we pass in the same HSPI_HOST to add
  // the SPI device
  os_init(HSPI_HOST);
  ESP_LOGI(TAG, "Initialize peripherals, doing LMIC reset ...");

  LMIC_reset();
  ESP_LOGI(TAG, "LMIC RESET");

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band
#elif defined(CFG_us915)
  LMIC_selectSubBand(0); 
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7,14);
  
  // Disable channel 1 to 8
  for(int i = 1; i <= 8; i++) LMIC_disableChannel(i);

  xTaskCreate(os_runloop, "os_runloop", 1024 * 2, (void* )0, 10, NULL);
}
