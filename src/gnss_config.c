/*
    a program for sending NMEA PUBX and UBX protocol messages
    to a ublox gnss module to change things like baud rate,
    message rate, and desired NMEA sentences, etc.

    /* ublox m8 datasheet:
    https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

    I'm using a M8030 chip on a quadcopter GPS module:
        - Team Blacksheep M8.2
        - BOM: ~$11

    NOTE: It's critical the chip is genuine ublox and not a clone. To detect this, connect to the module via
    UART and have a stream (eg. using minicom) to read the output in the terminal. Then disconnect power from
    only the module and reconnect it. A stream of `GPTXT` data will be emitted. You can look up what it should and
    shouldn't look like for a genuine chip. This program likely won't work on a clone chip.
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID uart1   // change as needed
#define BAUD_RATE 115200  // default BAUD rate for the module for initial connection. can be changed later.
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 4   // change as needed
#define UART_RX_PIN 5   // change as needed

void on_uart_rx(void);
int get_checksum(char *string);
void uart_tx_setup(void);
void uart_rx_setup(void);
void compile_message(char *nmea_msg, char *raw_msg, char *checksum,
                     char *terminator);
int extract_baud_rate(char *string);
void send_nmea(int testrun);
void send_ubx(int testrun);
void fire_nmea_msg(char *msg, int testrun);
void fire_ubx_msg(char *msg, size_t len, int testrun);


void on_uart_rx() {
    // for reading the raw output to a buffer and printing it to the console
    size_t len = 255;  // size of the buffer in bytes
    char buffer[len];  // make a buffer of size `len` for the raw message
    uart_read_blocking(UART_ID, buffer, len);
    printf("\n%s\n-------------\n", buffer);
}


int get_checksum(char *string) {
    // adapted from: https://github.com/craigpeacock/NMEA-GPS/blob/master/gps.c
    char *checksum_str;
	// int checksum;
	int calculated_checksum = 0;
    // printf("calculating checksum\n");
    char duplicate[strlen(string)];
    strcpy(duplicate, string); // preserve the original string 

	// Checksum is postcede by *
	checksum_str = strchr(duplicate, '*');
	if (checksum_str != NULL){
		// Remove checksum from duplicate
		*checksum_str = '\0';
		// Calculate checksum, starting after $ (i = 1)
		for (int i = 1; i < strlen(duplicate); i++) {
			calculated_checksum = calculated_checksum ^ duplicate[i];  // exclusive OR
		}
        printf("Calculated checksum (int): %u\n", calculated_checksum);
        return calculated_checksum;
	} else {
		// printf("Error: Checksum missing or NULL NMEA message\r\n");
		return 0;
	}
	return 0;
}


void compile_message(char *nmea_msg, char *raw_msg, char *checksum,
                     char *terminator) {
    // add each component to the `nmea_msg` array
    strcat(nmea_msg, raw_msg);     // add the base message
    strcat(nmea_msg, checksum);    // add the checksum
    strcat(nmea_msg, terminator);  // finally, add the termination sequence
    // printf("\ncatted: %s\n", nmea_msg);
}


void uart_tx_setup(void) {
    // initialize UART on the pico but only what's needed for transmission
    // so that the writes aren't interrupted by interrupts when the module
    // starts spitting out GNSS data.
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true);
}


void uart_rx_setup(void) {
    // finish initializing the RX UART...
    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
    // char msg[] = "0xB5,0x62,0x0A,0x04,0x00,0x00,0x0E,0x34,\0";
    // uint8_t msg[] = { 0xB5,0x62,0x0A,0x04,0x00,0x00,0x0E,0x34,00 };
}


int extract_baud_rate(char *string) {
    // extract the new baud from the message
    printf("extracting baud rate....\n");
    char *token;
    token = strtok(string, ",");
    for (int i=0; i<5; i++) {
        // skip to the 5th field of the message
        token = strtok(NULL, ",");
    }
    return atoi(token);
}

void fire_ubx_msg(char *msg, size_t len, int testrun) {
    printf("firing off UBX message...\n");
    if (testrun == 0) {
        // send out the message multiple times. BAUD_RATE in particular needs this treatment.
        for (int i=0; i<5; i++) {
            uart_write_blocking(UART_ID, msg, len);
        }
    }
}

void fire_nmea_msg(char *msg, int testrun) {
    printf("firing off NMEA message...\n");
    for (int k = 0; k < 5; k++) {
        // send out the message multiple times. BAUD_RATE in particular needs this treatment.
        if (testrun == 0) {
            for (int i=0; i<strlen(msg); i++) {
                // write the message char by char.
                uart_putc_raw(UART_ID, msg[i]);
            }
        } else {
            for (int i=0; i<strlen(msg); i++) {
                // print the chars for debugging
                printf("%c|", msg[i]);
            }
        }
        printf("\n");
    }
}

void send_nmea(int testrun) {
    // below are some NMEA PUBX messages to be modified as needed.
    // checksum values (immediately following `*`) are generated automatically
    char update_baud_rate[] = "$PUBX,41,1,3,3,115200,0*";  // update baud rate
    char enable_zda[] = "$PUBX,40,ZDA,1,1,1,0*";           // enable ZDA
    char disable_gsv[] = "$PUBX,40,GSV,0,0,0,0*";          // disable GSV
    char disable_vtg[] = "$PUBX,40,VTG,0,0,0,0*";          // disable VTG
    char disable_rmc[] = "$PUBX,40,RMC,0,0,0,0*";          // disable RMC
    char disable_gsa[] = "$PUBX,40,GSA,0,0,0,0*";          // disable GSA
    char disable_gll[] = "$PUBX,40,GLL,0,0,0,0*";          // disable GLL messages

    // --------------- modify below variables to control execution---------------

    char raw_msg[] = "$PUBX,40,GLL,0,0,0,0*";  // pick the desired message to write and just hardcode it here

    // ------------------------ end modifyable variables ------------------------

    int decimal_checksum;  // placeholder for the integer value checksum checksum
    decimal_checksum = get_checksum(raw_msg);  // calc the hex checksum and write it to the `checksum` array
    char checksum[2];  // placeholder for hexadecimal checksum
    strcpy(checksum, "");  // initialize to empty string to avoid junk values
    sprintf(checksum, "%x", decimal_checksum);  // convert the decimal checksum to hexadecimal
    // itoa(cs, checksum, 16);  // alternative to sprintf()
    // printf("%s", checksum);  // for debugging
    char msg_terminator[] = "\r\n";  // NMEA sentence terminator <cr><lr> == "\r\n"
    char nmea_msg[strlen(raw_msg) + strlen(msg_terminator) + strlen(checksum)];  // placeholder for final message
    strcpy(nmea_msg, "");  // initialize to empty string to avoid junk values
    compile_message(nmea_msg, raw_msg, checksum, msg_terminator);  // assemble the components into the final msg

    fire_nmea_msg(nmea_msg, testrun);

    if (strncmp(nmea_msg, update_baud_rate, 23) == 0 && (testrun == 0)) {
        int new_baud;
        new_baud = extract_baud_rate(update_baud_rate);
        // update the pico's UART baud rate to the newly set value.
        printf("updating baud rate to %d", new_baud);
        int __unused actual = uart_set_baudrate(UART_ID, new_baud);
    }

}

void send_ubx(int testrun) {
    // cfg_cfg_save_all will cause the changes to be permanent and persist through power cycles.
    uint8_t cfg_cfg_save_all[] = {
        0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,
        0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xAB
    };
    // below changes baud rate to `115200`
    uint8_t change_baud_rate[] = {
        0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,
        0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,
        0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E
    };
    // pick the desired message and send it.
    fire_ubx_msg(cfg_cfg_save_all, sizeof(change_baud_rate), testrun);
}

int main(void) {
    stdio_init_all();  // important so that printf() works
    uart_init(UART_ID, BAUD_RATE);
    uart_tx_setup();  // initialize UART Tx on the pico

    int testrun = 0;  // 1 to print the simulated transmission only, 0 to transmit it.
    printf("REMINDER: ENSURE `BAUD_RATE` IS CORRECT FOR INITIAL CONNECTION!\n\n");

    // TODO: be able to power on/off the module

    // send_nmea(testrun);  // comment out to not send anything
    send_ubx(testrun);   // comment out to not send anything

    uart_rx_setup();  // initialize UART Rx on the pico
    while (1)
        tight_loop_contents();
}
