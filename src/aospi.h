// aospi.h - 2wire-SPI towards and from OSP nodes
/*****************************************************************************
 * Copyright 2024 by ams OSRAM AG                                            *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************/
#ifndef _AOSPI_H_
#define _AOSPI_H_


#include <stdint.h>   // uint8_t etc
#include <aoresult.h> // global error list


// Identifies lib version
#define AOSPI_VERSION "0.4.2"


// OSP uses telegrams of max 12 bytes
#define AOSPI_TELE_MAXSIZE 12


// Initializes the SPI OUT and IN controllers and their support pins.
void aospi_init();                             


// Sends the `txsize` bytes in buffer `tx` to the first OSP node.
aoresult_t aospi_tx(const uint8_t * tx, int txsize); 
// Sends the `txsize` bytes in buffer `tx` to the first OSP node. Waits for a response telegram and stores those bytes in buffer `rx` with size `rxsize`.
aoresult_t aospi_txrx(const uint8_t * tx, int txsize, uint8_t * rx, int rxsize, int *actsize=0); 


// Sets the direction mux so that the last OSP node is connected to the SPI slave (for an OSP chain using Loop).
void aospi_dirmux_set_loop();
// Sets the direction mux so that the first OSP node is connected to the SPI slave (for an OSP chain using BiDir).
void aospi_dirmux_set_bidir();
// Inspects the direction mux, returning iff the last node (Loop) is connected to the SPI slave block.
int aospi_dirmux_is_loop();
// Inspects the direction mux, returning iff the first node (BiDir) is connected to the SPI slave block.
int aospi_dirmux_is_bidir();


// The library tracks the number of transmitted telegrams via calls to aospi_tx() and aospi_txrx(). This function resets that counter to 0.
void aospi_txcount_reset();
// The library tracks the number of received telegrams via calls aospi_txrx(). This function resets that counter to 0.
void aospi_rxcount_reset();
// The library tracks the number of transmitted telegrams via calls to aospi_tx() and aospi_txrx(). This function reports that count.
int  aospi_txcount_get();
// The library tracks the number of received telegrams via calls to aospi_txrx(). This function reports that count.
int  aospi_rxcount_get();


// For testing! Sets the output-enable of the outgoing level shifter to `val`.
void aospi_outoena_set( int val );
// For testing! Returns the state of the output-enable of the outgoing level shifter.
int aospi_outoena_get();
// For testing! Sets the output-enable of the incoming level shifter to `val`.
void aospi_inoena_set( int val );
// For testing! Returns the state of the output-enable of the incoming level shifter.
int aospi_inoena_get( );


#endif




