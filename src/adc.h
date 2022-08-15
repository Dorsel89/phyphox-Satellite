#ifndef _ADC_H   /* Include guard */
#define _ADC_H

#define SCAP_NODE DT_NODELABEL(button9)
#define SCAP_GPIO DT_GPIO_LABEL(SCAP_NODE, gpios)
#define SCAP_PIN DT_GPIO_PIN(SCAP_NODE, gpios)
#define SCAP_FLAGS DT_GPIO_FLAGS(SCAP_NODE, gpios)

static struct k_timer timer_bas;
static struct k_work work_bas;

void init_BAS();
#endif