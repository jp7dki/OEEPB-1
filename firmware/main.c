#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "bsp/board.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "BME280.h"
#include "hardware/i2c.h"

#include "tusb.h"

#include "ff.h"
#include "diskio.h"
#include "msc_disks.h"

#include "EPD_GDEY029T94.h"

// pin define
#define DEBUG_TX_PIN 0
#define DEBUG_RX_PIN 1
#define PWR_ON_PIN 17
#define SWA_PIN 18
#define SWB_PIN 19
#define SWC_PIN 20
#define LED1_PIN 24
#define LED2_PIN 25
#define BAT_DET_PIN 26
#define BAT_DETON_PIN 27

// macro define
#define UART_DEBUG uart0
#define UART_DEBUG_BAUDRATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define EPD_WIDTH 296
#define EPD_HEIGHT 128

const char disp_string[] = "This is E-Paper Display Test! ";

enum{
	BLINK_NOT_MOUNTED = 250,
	BLINK_MOUNTED = 1000,
	BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void cdc_task(void);
void core1_entry(void);

int main(void)
{
    UINT i;
    UINT br,bw;


    sleep_ms(500);

    // hardware init
    gpio_init(DEBUG_TX_PIN);
    gpio_init(DEBUG_RX_PIN);
    gpio_set_dir(DEBUG_TX_PIN, GPIO_OUT);
    gpio_set_dir(DEBUG_RX_PIN, GPIO_IN);
    gpio_put(DEBUG_TX_PIN, 0);
    gpio_set_function(DEBUG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEBUG_RX_PIN, GPIO_FUNC_UART);

    gpio_init(SWA_PIN);
    gpio_init(SWB_PIN);
    gpio_init(SWC_PIN);
    gpio_set_dir(SWA_PIN, GPIO_IN);
    gpio_set_dir(SWB_PIN, GPIO_IN);
    gpio_set_dir(SWC_PIN, GPIO_IN);
    gpio_pull_up(SWA_PIN);
    gpio_pull_up(SWB_PIN);
    gpio_pull_up(SWC_PIN);

    gpio_init(LED1_PIN);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_put(LED1_PIN, 0);
    gpio_put(LED2_PIN, 0);

    gpio_init(DC_Pin);
    gpio_init(CS_Pin);
    gpio_init(EPD_SCK);
    gpio_init(EPD_MOSI);
    gpio_init(RES_Pin);
    gpio_init(BUSY_Pin);
    gpio_set_dir(DC_Pin, GPIO_OUT);
    gpio_set_dir(CS_Pin, GPIO_OUT);
    gpio_set_dir(EPD_SCK, GPIO_OUT);
    gpio_set_dir(EPD_MOSI, GPIO_IN);
    gpio_set_dir(RES_Pin, GPIO_OUT);
    gpio_set_dir(BUSY_Pin, GPIO_IN);
    gpio_put(DC_Pin, 0);
    gpio_put(CS_Pin, 0);
    gpio_put(BUSY_Pin, 0);
    gpio_put(RES_Pin, 0);

    spi_init(EPD_SPI, 100 * 1000);
    gpio_set_function(EPD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(EPD_MOSI, GPIO_FUNC_SPI);

    gpio_init(PWR_ON_PIN);
    gpio_init(BAT_DET_PIN);
    gpio_init(BAT_DETON_PIN);
    gpio_set_dir(PWR_ON_PIN, GPIO_OUT);
    gpio_set_dir(BAT_DET_PIN, GPIO_IN);
    gpio_set_dir(BAT_DETON_PIN, GPIO_OUT);
    gpio_put(PWR_ON_PIN, 1);
    gpio_put(BAT_DETON_PIN, 0);

    //---- UART for debug -----------------------
    uart_init(UART_DEBUG, 2400);
    int __unused actual = uart_set_baudrate(UART_DEBUG, UART_DEBUG_BAUDRATE);
    uart_set_hw_flow(UART_DEBUG, false, false);
    uart_set_format(UART_DEBUG, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_DEBUG, false);
    uart_set_irq_enables(UART_DEBUG, false, false);
    
    //---- BME280 initialize --------------------
    bme280_init();
    read_compensation_parameters();

    sleep_ms(500);

    //---- TUSB initialize ----------------------
    board_init();
    msc_init();

    tud_init(BOARD_TUD_RHPORT);

    // Core1 Task 
    multicore_launch_core1(core1_entry);

    // main roop
    while(1){
        tud_task();
        cdc_task();
    }

}

//------------------------------------------
// core1 entry
//------------------------------------------
// core1は電子ペーパの表示処理のみ行う
void core1_entry(void){
    FRESULT res;
    FATFS fs;
    FIL fsr;
    UINT res_num;
    char buf[100];
    int i,j,k;
    uint16_t count=1;
    char file_name[10];
    BYTE buffer[EPD_WIDTH*EPD_HEIGHT/8];
    BYTE *ImageData;

    //---- EPD init ----------------------
    EPD_HW_Init(); //EPD init Fast
    ImageData = (BYTE *)malloc(EPD_WIDTH*EPD_HEIGHT/8);

    gpio_put(LED1_PIN, 1);
    gpio_put(LED2_PIN, 1);

    f_mount(&fs, "", 0);
    sprintf(file_name, "0:%03d.BIN", count);

    res = f_open(&fsr, file_name, FA_READ);
    if(res==FR_OK){
        f_read(&fsr, buffer, EPD_WIDTH*EPD_HEIGHT/8, &res_num);

        memcpy(ImageData, buffer, EPD_WIDTH*EPD_HEIGHT/8);

        EPD_WhiteScreen_ALL(ImageData);
        EPD_DeepSleep();

    }

    while(1){
        if(gpio_get(SWA_PIN)==1){
            sleep_ms(100);
            count=0;
            while(gpio_get(SWA_PIN)==1){
                if(count==20){
                    gpio_put(PWR_ON_PIN, 0);        // power off
                }else{
                    count++;
                }
            }
        }

        if(gpio_get(SWB_PIN)==1){
            sleep_ms(100);
            count=0;
            while(gpio_get(SWA_PIN)==1){
                if(count==20){
                    // long push
                    
                }else{
                    count++;
                }
            }            
        }

        if(gpio_get(SWC_PIN)==1){

        }

        sleep_ms(2000);
        
        uint8_t buf[8];
        uint8_t b;

        char cbuf[40];
        sprintf(cbuf, "%d\r\n", b);
//            sprintf(cbuf, "%X, %X, %X, %X, %X, %X, %X\n\r", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
        uart_puts(UART_DEBUG, cbuf);
    }
}

//------------------------------------------
// Device callbacks
//------------------------------------------

// Invoked when device is mounted
void tud_mount_cd(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cd(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5mA from bus
void tud_suspended_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

//-------------------------------------------
// USB CDC
//-------------------------------------------
void cdc_task(void)
{
	// connected() check for DTR bit
	// Most but not all terminal client set this when makeing connection
	// if (tud_cdc_connected())
	{
		// connected and there are data available
		if(tud_cdc_available())
		{
			// read datas
			char buf[64];
			uint32_t count = tud_cdc_read(buf, sizeof(buf));
			(void) count;
			
			// Echo back
			// Note: Skip echo by commenting out write() and write_flush()
			// for throughtput test e.g
			//  $ dd if=/dev/zero of=/dev/ttyACM0 count 10000
			tud_cdc_write(buf, count);
			tud_cdc_write_flush();
		}
	}
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cd(uint8_t itf, bool dtr, bool rts)
{
	(void) itf;
	(void) rts;
	
	// TODO set some indicator
	if(dtr)
	{
		// Terminal connected
	}else{
		// Terminal disconnected
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cd(uint8_t itf)
{
	(void) itf;
}

//-------------------------------------------
// BLINKING TASK
//-------------------------------------------
void led_blinking_task(void)
{
	static uint32_t start_ms = 0;
	static bool led_state = false;
	
	// Blink every interval ms
	if(board_millis() - start_ms < blink_interval_ms) return;	// not enough time
	start_ms += blink_interval_ms;
	
	board_led_write(led_state);
	led_state = 1 - led_state;		// toggle
} 

/*----------------------------------*/
DWORD get_fattime (void)
{
	return 1;
}