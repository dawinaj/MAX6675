#ifndef MAX6675_H
#define MAX6675_H

#include <esp_log.h>
#include <esp_check.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

class MAX6675
{
	static constexpr const char *const TAG = "MAX6675";

public:
	using out_t = uint16_t;

private:
	spi_host_device_t spi_host;
	gpio_num_t cs_gpio;
	int clk_hz;
	spi_device_handle_t spi_hdl;
	mutable spi_transaction_t trx;

public:
	MAX6675(spi_host_device_t sh, gpio_num_t csg, int chz = 4'000'000) : spi_host(sh), cs_gpio(csg), clk_hz(chz), spi_hdl(nullptr), trx({})
	{
		ESP_LOGI(TAG, "Constructed with host: %d, pin: %d", spi_host, cs_gpio);
	}
	~MAX6675() = default;

	esp_err_t init()
	{
		assert(!spi_hdl);

		const spi_device_interface_config_t dev_cfg = {
			.command_bits = 0,
			.address_bits = 0,
			.dummy_bits = 0,
			.mode = 0,
			.duty_cycle_pos = 0,
			.cs_ena_pretrans = 0,
			.cs_ena_posttrans = 0,
			.clock_speed_hz = clk_hz,
			.input_delay_ns = 0,
			.spics_io_num = cs_gpio,
			.flags = SPI_DEVICE_HALFDUPLEX, // SPI_DEVICE_NO_DUMMY
			.queue_size = 1,
			.pre_cb = NULL,
			.post_cb = NULL,
		};

		ESP_RETURN_ON_ERROR(
			spi_bus_add_device(spi_host, &dev_cfg, &spi_hdl),
			TAG, "Error in spi_bus_add_device!");

		trx.flags = SPI_TRANS_USE_RXDATA;
		trx.tx_buffer = nullptr;
		trx.length = 0;
		trx.rxlength = 16;

		return ESP_OK;
	}

	esp_err_t deinit()
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_bus_remove_device(spi_hdl),
			TAG, "Error in spi_bus_remove_device!");
		spi_hdl = nullptr;

		return ESP_OK;
	}

	//

	esp_err_t acquire_spi(TickType_t timeout = portMAX_DELAY) const
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_device_acquire_bus(spi_hdl, timeout),
			TAG, "Error in spi_device_acquire_bus!");

		return ESP_OK;
	}

	esp_err_t release_spi() const
	{
		assert(spi_hdl);

		spi_device_release_bus(spi_hdl); // return void
		return ESP_OK;
	}

	//

	inline esp_err_t send_trx() const
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_device_polling_start(spi_hdl, &trx, portMAX_DELAY),
			TAG, "Error in spi_device_polling_start!");

		return ESP_OK;
	}

	inline esp_err_t recv_trx(TickType_t timeout = portMAX_DELAY) const
	{
		assert(spi_hdl);

		esp_err_t ret = spi_device_polling_end(spi_hdl, timeout);

		if (ret == ESP_OK || ret == ESP_ERR_TIMEOUT) [[likely]]
			return ret;

		ESP_LOGE(TAG, "Error in spi_device_polling_end!");
		return ret;
	}

	inline out_t parse_trx() const
	{
		out_t raw = SPI_SWAP_DATA_RX(*reinterpret_cast<const uint32_t *>(trx.rx_data), 16);
		if (raw & 0b100) [[unlikely]]
		{
			ESP_LOGE(TAG, "Thermocouple is not connected!");
			return -1;
		}
		return raw >> 3;
	}

	inline float get_float_celsius(TickType_t timeout = portMAX_DELAY) const
	{
		__attribute__((__unused__)) esp_err_t ret;
		out_t raw;

		ESP_GOTO_ON_ERROR(
			send_trx(),
			label_err, TAG, "Error in send_trx!");

		ESP_GOTO_ON_ERROR(
			recv_trx(),
			label_err, TAG, "Error in recv_trx!");

		raw = parse_trx();
		ESP_GOTO_ON_FALSE(
			raw != out_t(-1),
			ESP_FAIL, label_err, TAG, "Error in parse_trx!");

		return raw * 0.25f;

	label_err:
		return -1;
	}
};

#endif
