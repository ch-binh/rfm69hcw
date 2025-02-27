#include "../inc/rfm69_hw.h"

static rfm69_hw_ops_t rfm_hw_ops;

void rfm69_set_hw_spec(rfm69_hw_ops_t *ops)
{
    rfm_hw_ops = *ops; // Copy the function pointers
}

int rfm69_spi_read_reg(uint8_t reg, uint8_t *data, uint8_t size)
{
    if (rfm_hw_ops.spi_read)
    {
        return rfm_hw_ops.spi_read(reg, data, size);
    }
    return -1; // Error: function not set
}

int rfm69_spi_write_reg(uint8_t reg, uint8_t data)
{
    if (rfm_hw_ops.spi_write)
    {
        rfm_hw_ops.spi_write(reg, data);
        return 0;
    }
    return -1; // Error: function not set
}

int rfm69_spi_write_frame(uint8_t *data, uint8_t len)
{
    if (rfm_hw_ops.spi_write_frame)
    {
        rfm_hw_ops.spi_write_frame(data, len);
        return 0;
    }
    return -1; // Error: function not set
}

int rfm69_gpio_dio2_write(uint8_t val)
{
    if (rfm_hw_ops.gpio_dio2_write)
    {
        rfm_hw_ops.gpio_dio2_write(val);
        return 0;
    }
    return -1; // Error: function not set
}

int rfm69_gpio_dio0_write(uint8_t val)
{
    if (rfm_hw_ops.gpio_dio0_write)
    {
        rfm_hw_ops.gpio_dio0_write(val);
        return 0;
    }
    return -1; // Error: function not set
}

int rfm69_gpio_dio0_read(void)
{
    if (rfm_hw_ops.gpio_dio0_write)
    {
        return rfm_hw_ops.gpio_dio0_read();
    }
    return -1; // Error: function not set
}

void rfm69_delay_us(uint32_t us)
{
    if (rfm_hw_ops.delay_us)
    {
        rfm_hw_ops.delay_us(us);
    }
}