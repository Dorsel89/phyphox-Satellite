#include <kernel.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>

#include "ds18b20.h"

#define DQ 20

const struct device * gpio_dev;


//////////////////////////////////////////////////////////////////////////////
// OW_RESET - performs a reset on the one-wire bus and
// returns the presence detect. Reset is 480us. Presence checked
// another 70us later.
//
unsigned char ow_reset(void)
{
    unsigned char presence;

    gpio_dev = device_get_binding("GPIO_0");
    if(gpio_dev==NULL){
        printk("GPIO_0 not found\n");
        
    }else{
        printk("all good\n");
    }

    gpio_pin_configure(gpio_dev, DQ, GPIO_OUTPUT_INACTIVE); //pull DQ line low

    k_sleep(K_USEC(480)); // leave it low for 480us

    gpio_pin_set(gpio_dev, DQ, 1);  // allow line to return high

    k_sleep(K_USEC(70)); // wait for presence

    gpio_pin_configure(gpio_dev, DQ, GPIO_INPUT);
    if(gpio_pin_get(gpio_dev, DQ))  // get presence signal
            presence = 1;
    else  
            presence = 0;

    k_sleep(K_USEC(410)); // wait for end of timeslot


    return(presence); // presence signal returned

} // 0=presence, 1 = no part


//////////////////////////////////////////////////////////////////////////////
// READ_BIT - reads a bit from the one-wire bus. The delay
// required for a read is 15us, so the DELAY routine won't work.
// We put our own delay function in this routine in the form of a
// for() loop.
//
unsigned char read_bit(void)
{
    unsigned char presence;

    gpio_dev = device_get_binding("GPIO_0");
    gpio_pin_configure(gpio_dev, DQ, GPIO_OUTPUT_INACTIVE); // pull DQ low to start timeslot

    k_sleep(K_USEC(2));

     gpio_pin_configure(gpio_dev, DQ, GPIO_INPUT);  // then return high

    k_sleep(K_USEC(15));    // delay 15us from start of timeslot
   
    if(gpio_pin_get(gpio_dev, DQ))  // get presence signal
            presence = 1;
    else  
            presence = 0;

    return(presence); // return value of DQ line
}


//////////////////////////////////////////////////////////////////////////////
// WRITE_BIT - writes a bit to the one-wire bus, passed in bitval.
//
void write_bit(char bitval)
{
    gpio_dev = device_get_binding("GPIO_0");
    gpio_pin_configure(gpio_dev, DQ, GPIO_OUTPUT_INACTIVE); // pull DQ low to start timeslot

    k_sleep(K_USEC(5));

    if(bitval==1) gpio_pin_set(gpio_dev, DQ, 1); // return DQ high if write 1

    k_sleep(K_USEC(115)); // hold value for remainder of timeslot


    gpio_pin_set(gpio_dev, DQ, 1);
}


//////////////////////////////////////////////////////////////////////////////
// READ_BYTE - reads a byte from the one-wire bus.
//
unsigned char read_byte(void)
{
    unsigned char i;
    unsigned char value = 0;

    for (i=0;i<8;i++)
    {
        if(read_bit()) value|=0x01<<i;  // reads byte in, one byte at a time and then shifts it left
        
        k_sleep(K_USEC(103));   // wait for rest of timeslot
    }

    return(value);
}



//////////////////////////////////////////////////////////////////////////////
// WRITE_BYTE - writes a byte to the one-wire bus.
//
void write_byte(char val)
{
    unsigned char i;
    unsigned char temp;

    for (i=0; i<8; i++) // writes byte, one bit at a time
    {
        temp = val>>i; // shifts val right 'i' spaces
        temp &= 0x01; // copy that bit to temp
        write_bit(temp); // write bit in temp into
    }

    k_sleep(K_USEC(104));
}


int read_temperature(void)
{
    char get[10];
    char temp_lsb,temp_msb;
    int k;
    int temp_c;

    ow_reset();
    write_byte(0xCC); //Skip ROM
    write_byte(0x44); // Start Conversion

    k_sleep(K_USEC(600));

    ow_reset();
    write_byte(0xCC); // Skip ROM
    write_byte(0xBE); // Read Scratch Pad

    for (k=0;k<9;k++){get[k]=read_byte();}
    // printk("\n ScratchPAD DATA = %X%X%X%X%X\n",get[8],get[7],get[6],get[5],get[4],get[3],get[2],get[1],get[0]);
    
    temp_msb = get[1]; // Sign byte + lsbit
    temp_lsb = get[0]; // Temp data plus lsb
    if (temp_msb <= 0x80){temp_lsb = (temp_lsb/2);} // shift to get whole degree
    temp_msb = temp_msb & 0x80; // mask all but the sign bit
    if (temp_msb >= 0x80) {temp_lsb = (~temp_lsb)+1;} // twos complement
    if (temp_msb >= 0x80) {temp_lsb = (temp_lsb/2);}// shift to get whole degree
    if (temp_msb >= 0x80) {temp_lsb = ((-1)*temp_lsb);} // add sign bit

    temp_c = (int)temp_lsb;

    printk( "TempC= %d degrees C\n", temp_c ); // print temp. C

    return temp_c;
}