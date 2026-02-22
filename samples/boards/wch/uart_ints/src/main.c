#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <stdio.h>

static const char msg[] = "Hello from UART ISR!\r\n";
static size_t tx_pos = 0;

static void uart_isr(const struct device *dev, void *user_data)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_tx_ready(dev)) {
			int sent = uart_fifo_fill(dev, (const uint8_t *)&msg[tx_pos], strlen(msg) - tx_pos);
			tx_pos += sent;
			if (tx_pos >= strlen(msg)) {
				tx_pos = 0;
			}
		}
	}
}

int main(void)
{
	const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(uart_dev)) {
		printf("UART device is not ready\n");
		return -1;
	}

	uart_irq_callback_set(uart_dev, uart_isr);

	k_msleep(1000);

	uart_irq_tx_enable(uart_dev);

	while (1) {
		k_msleep(1);
	}

	return 0;
}
