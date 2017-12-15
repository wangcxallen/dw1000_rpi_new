/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple RX with diagnostics example code
 *
 *           This application waits for reception of a frame. After each frame received with a good CRC it reads some data provided by DW1000:
 *               - Diagnostics data (e.g. first path index, first path amplitude, channel impulse response, etc.). See dwt_rxdiag_t structure for more
 *                 details on the data read.
 *               - Accumulator values around the first path.
 *           It also reads event counters (e.g. CRC good, CRC error, PHY header error, etc.) after any event, be it a good frame or an RX error. See
 *           dwt_deviceentcnts_t structure for more details on the counters read.
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

 /*
 * Modified by:
 * Anh Luong <luong@eng.utah.edu>
 * Adwait Dongare
 */

#include <stdio.h>
#include <stdlib.h> // malloc, free
#include <unistd.h>
#include <string.h> // memset

#include <stdint.h> // for various uintXX_t, intXX_t types

#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME argv[0]

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define RADAR_FRAME_SN_IDX 1
#define RADAR_FRAME_DATA_IDX 6

// 992 samples for 16MHz PRF - 3968 bytes
// 1016 samples for 64MHz PRF - 4064 bytes
#define CIR_SAMPLES 1016 //1016

/* size of decawave rx buffer*/
#define RX_BUF_LEN 24

typedef unsigned long long uint64;
typedef signed long long int64;

struct cir_struct {
    int16 real;
    int16 img;
};

#define ACC_CHUNK 64 // bytes read at the same time

static void copyCIRToBuffer(uint8 *buffer, uint16 len);
static void saveCIRToFile(char *filename, int32 msgNo, uint64 rxTimestamp, struct cir_struct *cir);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static uint64 get_system_timestamp_u64(void);

/**
 * Application entry point.
 */
int main(int argc, char** argv) {

    /* Default communication configuration. We use here EVK1000's default mode (mode 3). */
    dwt_config_t config = {
        5,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
        DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_110K,     /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    // variables for DW1000 related data
    uint64 rx_timestamp;
    uint8 rx_buffer[RX_BUF_LEN];
    dwt_rxdiag_t diagnostics;
    uint8 *cir_buffer;
    struct cir_struct *cir;

    // experiment related variables
    int32 frame_count = 0;
    int32 current_frame_count = 0; 
    int32 max_frame_count;
    char filename[64];
    
    // status variables
    uint32 status_reg = 0;
    int32 frame_len = 0;

    if(argc > 1) {

        if(argc > 2 && sscanf(argv[2], "%ld", &max_frame_count) > 0) {

            printf("Recording %ld measurements for test %s\n", max_frame_count, argv[1]);

        } else {
            max_frame_count = -1;
            printf("Recording infinite measurements for test %s\n", argv[1]);
        }

        cir_buffer = (uint8 *) malloc(4*CIR_SAMPLES);
        cir = (struct cir_struct *) cir_buffer;

        if(cir_buffer == NULL) {
            printf("Could not allocate memory\r\n");
            return(0);
        }

        /* Start with board specific hardware init. */
        hardware_init();

        /* Reset and initialise DW1000. See NOTE 2 below.
         * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
         * performance. */
        reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
        spi_set_rate_low();
        if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
            printf("Unable to initialize UCODE\r\n");
            return(0);
        }
        spi_set_rate_high();

        /* Configure DW1000. */
        dwt_configure(&config);

        printf("running %s\n", APP_NAME);


    } else {
        printf("usage: %s <experiment name> (<number of measurements>)\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    

    
    /* Loop receiving frames. */
    while (1) {

        if(max_frame_count > 0 && frame_count >= max_frame_count) {
            printf("test complete\r\n");
            break;
        }

        printf("waiting for capture\r\n");

        /* Clear all data buffers */

        // clear RX data buffer        
        memset((void*) rx_buffer, 0, RX_BUF_LEN);

        // clear cir_buffer before next sampling
        memset((void*) cir_buffer, 0, 4*CIR_SAMPLES);

        // // clear diagnostic readings
        // memset((void*) &diagnostics, 0, sizeof(dwt_rxdiag_t));

        /* Activate reception immediately. See NOTE 3 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
            // spin wait until message is received
        };

        if (status_reg & SYS_STATUS_RXFCG) {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                memcpy((void *) &current_frame_count, (void *) &rx_buffer[RADAR_FRAME_DATA_IDX], sizeof(int32));
   
                // gather RX timestamp
                rx_timestamp = get_rx_timestamp_u64();

                copyCIRToBuffer((uint8 *) cir_buffer, 4*CIR_SAMPLES);

                dwt_readdiagnostics(&diagnostics);
                // printf("FP: %d, STD_NOISE: %d, MAX_NOISE: %d \r\n", diagnostics.firstPath, diagnostics.stdNoise, diagnostics.maxNoise);

                printf("MSG %lu received at %llu\r\n", current_frame_count, rx_timestamp);

                frame_count++;

                // generate new filename
                snprintf(filename, 64, "%s_%ld_cir.bin", argv[1], current_frame_count);

                printf("writing observations to %s\r\n", filename);

                // save data to file
                saveCIRToFile(filename, current_frame_count, rx_timestamp, cir);

            }




            // uint16 fp_int = diagnostics.firstPath / 64;
            //dwt_readaccdata((uint8 *)&cir, CIR_SAMPLES, (fp_int - 2) * 4);
            // dwt_readaccdata((uint8 *) cir_buffer, 4*CIR_SAMPLES + 1, 0);


            // Print CIR

            // printf("CIR Bytes:");
            // for(i = 0; i < 4*CIR_SAMPLES + 1; i++)
            // {
            //     printf("%01X", cir_buffer[i]);
            // }

            // printf("CIR Real: ");
            // for (i = 0; i < CIR_SAMPLES; i++) {
            //     printf("%04X ", cir[i].real);
            // }
            // printf("\n");

            // printf("CIR Imaginary: ");
            // for (i = 0; i < CIR_SAMPLES; i++) {
            //      printf("%04X ", cir[i].img);
            // }

            // printf("\n");
        } else {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }
    }

    cir = NULL;
    free(cir_buffer);
}

/*
 * TODO: Add function documentation
 */
static void copyCIRToBuffer(uint8 *buffer, uint16 len) {
    int loc = 0;
    uint16 toRead = 0;
    int lastRead = 0;

    // we need to create a buffer to discard the dummy byte
    uint8 buf[ACC_CHUNK+1];
    while(1) {
        memset((void *) buf, 0, ACC_CHUNK + 1);
        if(len > ACC_CHUNK) {
            // need to loop again
            // only read the max allowable number of characters
            toRead = ACC_CHUNK;
        } else {
            // we will cover the entire length in this iteration of the loop
            // only read the remaining length
            toRead = len;
            lastRead = 1;
        }
        // read from device
        dwt_readaccdata(buf, toRead, loc);
        // copy to the buffer
        memcpy((void *) &buffer[loc], (void *) &buf[1], toRead);
        if(lastRead) {
            break;
        }
        // decrease remaining length
        len = len - toRead;
        // increase pointer in buffer
        loc = loc + toRead;
    }
}

static void saveCIRToFile(char *filename, int32 msgNo, uint64 rxTimestamp, struct cir_struct *cir) {
    FILE *output_file;
    // double tof;
    // int i;
    // int j;

    output_file = fopen(filename, "wb");
    if (output_file == NULL){
        printf("unable to write\n");
    } else {

        // write message number
        fwrite((void*) &msgNo, sizeof(int32), 1, output_file);

        // write RX timestamp
        fwrite((void*) &rxTimestamp, sizeof(uint64), 1, output_file);

        // write CIR values
        fwrite((void*) cir, sizeof(struct cir_struct), CIR_SAMPLES, output_file);

        // close the open file        
        fclose(output_file);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_system_timestamp_u64()
 *
 * @brief Get the system in a 64-bit variable.
 *
 * @param  none
 *
 * @return  64-bit value of the system time.
 */
static uint64 get_system_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Accumulator values are complex numbers: one 16-bit integer for real part and one 16-bit value for imaginary part, for each sample. In this
 *    example, we chose to read 3 values below the first path index and 3 values above. It must be noted that the first byte read when accessing the
 *    accumulator memory is always garbage and must be discarded, that is why the data length to read is increased by one byte here.
 * 3. In this example, LDE microcode is loaded even if timestamps are not used because diagnostics values are computed during LDE execution. If LDE is
 *    not loaded and running, dwt_readdiagnostics will return all 0 values.
 * 4. Manual reception activation is performed here but DW1000 offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 5. We use polled mode of operation here to keep the example as simple as possible but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW1000 User Manual for more details on "interrupts".
 * 6. Here we chose to read only a few values around the first path index but it is possible and can be useful to get all accumulator values, using
 *    the relevant offset and length parameters. Reading the whole accumulator will require 4064 bytes of memory. First path value gotten from
 *    dwt_readdiagnostics is a 10.6 bits fixed point value calculated by the DW1000. By dividing this value by 64, we end up with the integer part of
 *    it. This value can be used to access the accumulator samples around the calculated first path index as it is done here.
 * 7. Event counters are never reset in this example but this can be done by re-enabling them (i.e. calling again dwt_configeventcounters with
 *    "enable" parameter set).
 * 8. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
