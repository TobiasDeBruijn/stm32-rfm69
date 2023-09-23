# RFM69 library adapted for STM32
The Arduino RFM69 library adapted for STM32

You'll likely have to make some changes to `include/rfm69.h` and `src/rfm69.c`, as the imports are
unfortunately STM board specific. I use the STM32L1xx, if you don't, you need to change them.

Make sure to add a call to `interrupt_handler` in your RFM69 DIO0 interrupt handler, if you want to receive any data.

Note that this project will **NOT** compile on its own, it is dependant on STM32 HAL type definitions,
which you must supply.