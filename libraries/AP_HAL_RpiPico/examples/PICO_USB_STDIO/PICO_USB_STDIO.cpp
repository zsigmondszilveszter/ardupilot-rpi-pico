#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    printf("This is the first message on USB channel.\n");
    while (true) {
        printf("Hello Ardupilot, from Raspberry Pi Pico!\n");
        sleep_ms(1000);
    }
    return 0;
}
