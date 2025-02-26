#include "../inc/rfm69_hw.h"


static rfm69_hw_ops_t spi_ops;

void rfm69_set_hw_spec(rfm69_hw_ops_t *ops)
{
    spi_ops = *ops; // Copy the function pointers
}

int rfm69_spi_read_reg(uint8_t reg, uint8_t *data, uint8_t size)
{
    if (spi_ops.spi_read)
    {
        return spi_ops.spi_read(reg, data, size);
    }
    return -1; // Error: function not set
}

int rfm69_spi_write_reg(uint8_t reg, uint8_t data)
{
    if (spi_ops.spi_write)
    {
        spi_ops.spi_write(reg, data);
        return 0;
    }
    return -1; // Error: function not set
}