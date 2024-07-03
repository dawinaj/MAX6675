#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include "MAX6675.h"

static const char *TAG = "Test";

void init_spi()
{
	spi_bus_config_t bus_cfg = {
		.mosi_io_num = GPIO_NUM_NC, // GPIO_NUM_23,
		.miso_io_num = GPIO_NUM_19,
		.sclk_io_num = GPIO_NUM_18,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
		.data4_io_num = GPIO_NUM_NC,
		.data5_io_num = GPIO_NUM_NC,
		.data6_io_num = GPIO_NUM_NC,
		.data7_io_num = GPIO_NUM_NC,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER,
	};

	spi_bus_initialize(SPI3_HOST, &bus_cfg, 0);
}

extern "C" void app_main(void)
{
	init_spi();

	MAX6675 tc(SPI3_HOST, GPIO_NUM_5);
	tc.init();

	while (true)
	{
		float tempc = tc.get_float_celsius();
		ESP_LOGI(TAG, "Temp: %f *C", tempc);
		vTaskDelay(pdMS_TO_TICKS(500));
	}

	tc.deinit();
}
