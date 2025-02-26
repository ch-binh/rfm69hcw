#ifndef RFM69_HW_H
#define RFM69_HW_H


#include <stdint.h>

/*
void rfm69_com_init(void)
{
    rfm69_hw_ops_t ops = {
        .spi_write = hal_spi_write_reg,
        .spi_read = hal_spi_read_reg};
    rfm69_set_hw_spec(&ops);
}
*/

typedef struct
{
    int (*spi_write)(uint8_t reg, uint8_t data);
    int (*spi_read)(uint8_t reg, uint8_t *data, uint8_t size);
    int (*gpio_dio_write)(uint8_t val);
    int (*reset)(void);
} rfm69_hw_ops_t;

void rfm69_set_hw_spec(rfm69_hw_ops_t *ops);

int rfm69_spi_write_reg(uint8_t reg, uint8_t data);
int rfm69_spi_read_reg(uint8_t reg, uint8_t *data, uint8_t size);

int rfm69_gpio_dio_write(uint8_t val);

#endif // RFM69_SPI_H