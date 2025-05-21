// aospi.cpp - 2-wire SPI (and 1-wire Manchester) towards and from OSP nodes
/*****************************************************************************
 * Copyright 2024,2025 by ams OSRAM AG                                       *
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
#include <Arduino.h>         // Serial, uint8_t
#include "soc/gpio_struct.h" // GPIO.out_w1tc
#include <aoresult.h>        // aoresult_t
#include <aospi.h>           // own


// Phy selected in init()
static aospi_phy_t aospi_phy= aospi_phy_undef;


/*!
    @brief  Converts an aospi_phy_t physical layer to a string.
    @param  phy
            A physical layer (tag).
    @return A constant string.
    @note   If the the tag is not part of aospi_phy_t, the function
            returns "<unknown>".
*/
const char * aospi_phy_str( aospi_phy_t phy ) {
  switch( phy ) {
    case aospi_phy_undef : return "<undef>";
    case aospi_phy_mcua  : return "MCU-A";
    case aospi_phy_mcub  : return "MCU-B";
  //case aospi_phy_use   : return "USE";
  //case aospi_phy_ext   : return "ext";
  }
  // By not having a default in the switch, a good compiler gives a warning when a case/tag is forgotten.
  return "<unknown>" ;
}


// === counters =============================================================


static int aospi_txcount;
static int aospi_rxcount;


/*!
    @brief  The library tracks the number of transmitted telegrams 
            via calls to aospi_tx() and aospi_txrx().
            This function resets that counter to 0.
*/
void aospi_txcount_reset() {
  aospi_txcount= 0;
}


/*!
    @brief  The library tracks the number of received telegrams 
            via calls aospi_txrx().
            This function resets that counter to 0.
*/
void aospi_rxcount_reset() {
  aospi_rxcount= 0;
}


/*!
    @brief  The library tracks the number of transmitted telegrams 
            via calls to aospi_tx() and aospi_txrx().
            This function reports that count.
    @return The number of transmitted telegrams, that is the number
            of  calls to aospi_tx() and aospi_txrx(), since 
            aospi_txcount_reset().
*/
int  aospi_txcount_get() {
  return aospi_txcount;
}


/*!
    @brief  The library tracks the number of received telegrams 
            via calls to aospi_txrx().
            This function reports that count.
    @return The number of received telegrams, that is the number of 
            calls to aospi_txrx(), since aospi_rxcount_reset().
*/
int  aospi_rxcount_get() {
  return aospi_rxcount;
}


// === SPI OUT ==============================================================
// The SPI OUT send command telegrams to the first OSP node (SPI master)


// Use the standard Arduino API for SPIOUT
#include <SPI.h>


// The pins of the ESP32 are fixed in this library.
// OSP nodes receive using only SCLK and MOSI
#define AOSPI_OUT_SCLK  4
#define AOSPI_OUT_MOSI  5
#define AOSPI_OUT_SSEL -1
#define AOSPI_OUT_MISO -1
// There is one extra pin in use, to en/disable the output of the level
// shifter that is placed between SPIOUT and the first SAID.
// Note: a uni-directional level shifter is used.
// Note: The B-side (5V, SAID) is the one that is en/disabled
#define AOSPI_OUT_OENA  6


// digitalWrite(AOSPI_OUT_OENA,...) is too slow so we resort to register writes
#define AOSPI_OUT_OENA_CLR()  do GPIO.out_w1tc = 1UL << AOSPI_OUT_OENA; while(0)
#define AOSPI_OUT_OENA_SET()  do GPIO.out_w1ts = 1UL << AOSPI_OUT_OENA; while(0)


// Select which of the two SPI controllers of the ESP to use for OUT
#define AOSPI_OUT_BUS   HSPI


// For OSP nodes, the SPI frequency has to be 2.4MHz +/- 50%
// otherwise a timeout elapses inside the node, resulting in a com error.
#define AOSPI_OUT_TYPEA_FREQ  (2 * 2400 * 1000) // MCU mode type A: 1-wire Manchester
#define AOSPI_OUT_TYPEB_FREQ  (2400 * 1000)     // MCU mode type B: 2-wire SPI


// The SPI OUT master object
static SPIClass aospi_out(AOSPI_OUT_BUS);


/*!
    @brief  Initializes the SPI OUT controller
    @note   Assigns and sets up MOSI and SCLK pins.
            Also assigns and sets up the OENA pin.
            Drives all lines to default level.
*/
static void aospi_out_init() {
  // Couple the pins
  aospi_out.begin(AOSPI_OUT_SCLK, AOSPI_OUT_MISO, AOSPI_OUT_MOSI, AOSPI_OUT_SSEL);
  // Configure the extra pin (level-shifter output-enable)
  digitalWrite(AOSPI_OUT_OENA, LOW); // disable level shifter output
  pinMode(AOSPI_OUT_OENA, OUTPUT);
}


/*!
    @brief  Sends the `txsize` bytes in buffer `tx` to the first OSP node.
    @param  freq
            The frequency for the underlying SPI bus.
    @param  tx
            A pointer to a buffer of bytes to be sent.
    @param  txsize
            The number of bytes (of buffer tx) to be sent.
    @return aoresult_ok (no error checking on send possible)
    @note   While sending the OENA line is held high to enable the output
            of the level shifter. This is especially important after a
            RESET telegram because the first OSP node inspects the line
            status to redetermine its comms mode (MCU, LVDS, CAN, EOL).
*/
static aoresult_t aospi_tx_internal(int freq, const uint8_t * tx, int txsize) {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  // Send (see https://docs.arduino.cc/learn/communication/spi)
  aospi_out.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE0)); // TX uses SPI MODE 0
  AOSPI_OUT_OENA_SET(); // digitalWrite(AOSPI_OUT_OENA, HIGH); // enable level shifter output
    aospi_out.transferBytes( tx, NULL, txsize ); // No MISO
    // For the RESET telegram it is important to clear OENA immediately
  AOSPI_OUT_OENA_CLR(); // digitalWrite(AOSPI_OUT_OENA, LOW); // disable level shifter output
  aospi_out.endTransaction();
  // Update counters
  aospi_txcount++;
  // No checks possible on success
  return aoresult_ok;
}


// === SPIIN ================================================================
// The SPI IN receives response telegram from the OSP chain (SPI slave).
// The response may come from the first node of the chain (BiDir) or the last (Loop).


// Use a high level wrapper around ESP code
#include "slave/ESP32SPISlave.h"


// The pins of the ESP32 are fixed in this library.
// OSP nodes send using only SCLK and MOSI
// However the slave block requires an SSEL "in" (so we connect that to MSEL "out")
#define AOSPI_IN_SCLK       13
#define AOSPI_IN_MOSI       12
#define AOSPI_IN_SSEL       47
#define AOSPI_IN_MISO       -1
// Output pin externally wired to AOSPI_IN_SSEL to control slave select
#define AOSPI_IN_MSEL       21
// We need to release MSEL, for this we tap the clock line; SCLK is externally wired to CINT
#define AOSPI_IN_CINT        2 
// Finally, there are two extra pins in use to control the level shifters
// Note: two uni-directional level shifters are used: one for first node (BIDIR), one for last (LOOP).
// Note: The B-sides of level shifter (3V3, MCU side) are the ones that are en/disabled
// Only one is enabled with AOSPI_IN_OENA, AOSPI_IN_DIRL selects which.
#define AOSPI_IN_OENA        1 // Level-shifter output-enable
#define AOSPI_IN_DIRL       14 // To select direction Loop (or BiDir)


// digitalWrite(AOSPI_IN_MSEL_CLR,...) is too slow so we resort to register writes
// Use out1_w1tc.val (instead of out_w1tc) for pin numbers of 32 and higher
#define AOSPI_IN_OENA_CLR()  do GPIO.out_w1tc = 1UL << AOSPI_IN_OENA; while(0)
#define AOSPI_IN_OENA_SET()  do GPIO.out_w1ts = 1UL << AOSPI_IN_OENA; while(0)
#define AOSPI_IN_MSEL_CLR()  do GPIO.out_w1tc = 1UL << AOSPI_IN_MSEL; while(0)
#define AOSPI_IN_MSEL_SET()  do GPIO.out_w1ts = 1UL << AOSPI_IN_MSEL; while(0)
// digitalRead(AOSPI_IN_CINT) is too slow so we resort to register reads (GPIO.in.val for some ESP derivatives)
#define AOSPI_IN_CINT_GET() (GPIO.in & (1<<AOSPI_IN_CINT))


// Select which of the two SPI controllers of the ESP to use for IN
#define AOSPI_IN_BUS  FSPI


// A node responds worst-case in 1400 us. Forwarding a telegram takes 8us.
// A chain with n nodes, with a request/response to/from node k thus needs
//   1400+8×(n-1) us   for an answer (in Loop) and
//   1400+2×8×(k-1) us for an answer (in BiDir).
// The maximum number is BiDir with a longest chain of 1002 nodes.
#define AOSPI_IN_TIMEOUT_US      (1400+2*8UL*(1002-1))


static uint32_t aospi_txrx_us_;
/*!
    @brief  Returns the (round) trip time for the last call to `aospi_txrx()`.
    @return Time in us (micro seconds).
    @note   This the time recorded in the last call to `aospi_txrx()` so 
            that must precede the call to this function.
    @note   Since `aoosp_tx()` does not send a response, the MCU can not
            measure the trip time of those send-only telegrams.
    @note   Trip time is available for BiDir and for Loop.
    @note   The trip time is recorded starting with the first bit tx'ed by 
            the MCU and ending with the last bit rx'ed by the MCU.
    @note   The trip time (t_trip) includes sending the telegram sized txsize 
            (t_cmd) and receiving the response sized rxsize (t_resp), but 
            also includes the execution time of the command (t_exec) and the 
            forwarding time (t_fwd) of all intermediate nodes (k). 
            In some cases the executing node introduces a delay (t_delay), 
            typically 5us for SAID in BiDir, 0 otherwise.
    @note   For a BiDir trip the trip time is as follows:
            t_trip = k×t_fwd + t_cmd + t_exec + t_delay + t_resp + k×t_fwd
    @note   For a Loop trip the trip time on a chain of n nodes is as follows:
            t_trip = (n-1)×t_fwd + t_cmd + t_exec + t_delay + t_resp
    @note   The call is non destructive; it keeps its value until it is
            overwritten by a next call to `aospi_txrx()`.  
*/
uint32_t aospi_txrx_us() {
  return aospi_txrx_us_;
}


static int aospi_txrx_size_;
/*!
    @brief  Returns an estimate of the number of hops a command telegram and
            its response telegram need in a bidirectional round trip.
    @param  t_extra
            The sum of t_exec, the execution time of a command telegram, 
            typically 0; and t_delay, the artificial delay introduced by the 
            node, typically 5 for SAID in BiDir, 0 otherwise. 
            Default value is 5. Unit is micro seconds.
    @return Number of hops.
    @note   This returns the number of hops of the last call to `aospi_txrx()` 
            so that must precede the call to this function.
    @note   The number of hops is the number of OSP nodes the message needs
            to travel. In BiDir, the number of hops to travel to the n-th 
            node and back is 2(n-1). In other words hops/2+1 estimates the 
            address of the node.
    @note   The returned estimation of the number of hops is based on the 
            round trip time `aospi_txrx_us()` and the size of the command and 
            response telegrams of the last `aospi_txrx()`.
            The caller must pass t_extra which is defined as t_exec+t_delay.
            Since t_exec is usually 0 and t_delay is 5 for SAID in BiDir,
            the default value for t_extra is 5.
    @note   Every hop introduces a propagation delay of t_forward=7.5us.
            This is the basis for estimating the number of hops.
    @note   The call is non destructive; it keeps it value until it is
            overwritten by a next call to `aospi_txrx()`.  
    @note   Due to the asynchronous behavior in the daisy chain communication 
            the returned number of hops has a variance which increases with 
            the distance to the MCU.
*/
uint32_t aospi_txrx_hops(int t_extra) {
  // *8 for bits-to-bytes, *10 to get rid of dec point, +12 for rounding, bit rate 2.4Mhz;
  // t_proc = t_cmd+(t_exec+t_delay)+t_resp = t_cmd+t_extra+t_resp
  uint32_t t_proc = (aospi_txrx_size_*8*10+12)/24 + t_extra; 
  if( t_proc>aospi_txrx_us_ ) return 0;
  // Time for all hops together
  uint32_t t_hop = aospi_txrx_us_ - t_proc;
  // t_fwd=7.5us; dividing t_hop by t_fwd gives number of hops (add 7.5/2 for rounding)
  // (t_hop+7.5/2)/7.5 = (t_hop*100+3.75*100) / 7.5*100 = (t_hop*100+375) / 750
  return (t_hop*100+375)/750;
}



// The SPI IN slave object
static ESP32SPISlave aospi_in;


/*!
    @brief  Initializes the SPI IN controller
    @note   Assigns and sets up MOSI, SCLK and SSEL pins.
            Also assigns and sets up the extra pins (MSEL, CINT, OENA, DIRL).
            Drives all lines to default level (DIRL low, so mux in BiDir).
*/
static void aospi_in_init() {
  // Slave deselected (active low)
  pinMode     (AOSPI_IN_MSEL, OUTPUT);
  digitalWrite(AOSPI_IN_MSEL, HIGH  ); // do not swap with previous line (e.g. aoosp_topo.ino breaks on resetinit)
  // Clock monitoring (to determine when to release MSEL, we tap the clock line)
  // Clock tap was previously handled by interrupt (hence the name CINT, now polling)
  pinMode     (AOSPI_IN_CINT, INPUT);
  // Disable level shifter output
  digitalWrite(AOSPI_IN_OENA, LOW   );
  pinMode     (AOSPI_IN_OENA, OUTPUT);
  // Loop deselect
  digitalWrite(AOSPI_IN_DIRL, LOW   );
  pinMode     (AOSPI_IN_DIRL, OUTPUT);
  // SPI slave controller
  aospi_in.setDataMode(SPI_MODE0); // See note on SPI MODE below
  aospi_in.begin(AOSPI_IN_BUS, AOSPI_IN_SCLK, AOSPI_IN_MISO, AOSPI_IN_MOSI, AOSPI_IN_SSEL);
}


// Note on SPI MODE.
// Command telegrams are mastered by the MCU, but response telegrams are
// mastered by an OSP node. Responses come either from the first node
// (when using BiDir direction), or from the last node (Loop direction).
// Recall that the first node has its SIO configured in comms mode MCU, with
// P pulled up, and N pulled down. The last node has its SIO configured as
// EOL, with P pulled down, and N pulled up.
// This is relevant, because when an OSP node masters, its default clock
// line (N pad) follows this convention. In other words, when an OSP node
// masters in comms mode MCU (BiDir), it uses SPI MODE 0: clock (N pad)
// default low. When an OSP node masters in comms mode EOL (Loop), it
// uses SPI MODE 3: clock (N pad) default high. In both cases data is
// captured by the SPI slave in the MCU on the _rising_ clock. The MOSI
// line (P pad) does not have defaults in the SPI protocol.
// In OSP context, the MCU SPI slave is only receiving data (MOSI) it is
// not sending data (MISO) back to the master (OSP node). Sending would
// happen on the opposite edge (falling clock).
// Since the slave only needs to receive, it is only triggered by the
// rising edges of the clock. As a result, the ESP SPI slave driver can
// receive both BiDir and Loop telegrams using both MODE0 or MODE3.
// This is expected to hold for other MCUs as well but has not been checked.


// SPI slave buffers need to be 4 byte aligned.
static __attribute__((aligned(4))) uint8_t aospi_in_buf[AOSPI_TELE_MAXSIZE];


/*!
    @brief  Sends the `txsize` bytes in buffer `tx` to the first OSP node.
            Waits for a response telegram and stores those bytes in
            buffer `rx` with size `rxsize`.
    @param  freq
            The frequency for the underlying SPI bus.
    @param  tx
            A pointer to a buffer of bytes to be sent.
    @param  txsize
            The number of bytes (of buffer `tx`) to be sent.
    @param  rx
            A pointer to a caller allocated buffer of bytes to be received.
    @param  rxsize
            The size of the `rx` buffer.
    @param  actsize
            Output parameter set to the number of bytes actually received.
            Might be NULL.
    @return aoresult_spi_buf     if tx or rx is NULL
            aoresult_spi_buf     if txsize or rxsize is out of bounds (0..12)
            aoresult_assert      if the underlying driver behaves unexpectedly
            aoresult_spi_noclock if no response (clock) is received
            aoresult_spi_length  if the response had wrong number of bytes
            aoresult_ok          otherwise
    @note   Before sending, configure whether reception is from first (BiDir)
            or last (Loop) node in the OSP chain using aospi_dirmux_set_xxx()
    @note   While sending, this function enables the OUT level shifter.
            After sending, switches to reception by enabling the IN level 
            shifter (the one selected with aospi_dirmux_set_xxx), and then 
            activates SSEL.
            Reception terminates when a response is received (measured with
            CINT pin), which is fast (one telegram length or ~40us). If there
            is no response, reception terminates on timeout which is orders
            slower (see ~14000us AOSPI_IN_TIMEOUT_US).
    @note   Output parameter `actsize` might be set to NULL. In this case the 
            function will not perform a size test, will thus not return
            aoresult_spi_length. Instead the caller can inspect `*actsize`.
    @note   If caller knows how many bytes will be received, set `rxsize` to
            that amount and set `actsize` pointer to NULL. 
    @note   If caller does not knows how many bytes will be received, set 
            `rxsize` to largest possible telegram (ie AOSPI_TELE_MAXSIZE) 
            and pass an `actsize`. 
*/
static aoresult_t aospi_txrx_internal(int freq, const uint8_t * tx, int txsize, uint8_t * rx, int rxsize, int *actsize) {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  // Parameter check (skipping tx, done by caller because of possible Manchester encoding
  if( rxsize<0 || rxsize>AOSPI_TELE_MAXSIZE ) return aoresult_spi_buf;
  if( rx==0 )  return aoresult_spi_buf;

  // Put a buffer in the (background) reception queue
  if( aospi_in.numTransactionsInFlight()!=0 && aospi_in.numTransactionsCompleted()!=0 ) return aoresult_assert; // should not happen (buffers pending in flight)
  aospi_in.queue(NULL, aospi_in_buf, AOSPI_TELE_MAXSIZE ); // aospi_in doesn't send data to master (NULL), it only receives (aospi_in_buf)
  aospi_in.trigger();

  // record start time
  aospi_txrx_us_= micros(); 
  
  // aospi_tx() inlined here for speed
  aospi_out.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE0)); // TX uses mode 0
  AOSPI_OUT_OENA_SET(); // enable level shifter output - fast implementation of digitalWrite(AOSPI_OUT_OENA, HIGH)
  aospi_out.transferBytes( tx, NULL, txsize );
  AOSPI_OUT_OENA_CLR(); // disable level shifter output - fast implementation of digitalWrite(AOSPI_OUT_OENA, LOW)
  // aospi_out.endTransaction(); // Postpone this 6 us call, so that we can quickly enable reception - OSP gives 5us

  // Enable reception
  AOSPI_IN_OENA_SET();  // enable level shifter output - fast implementation of digitalWrite(AOSPI_IN_OENA, HIGH);
  delayMicroseconds(0); // Wait a tiny bit for the connect kicks in (and SCLK is right) before we "chip select" the SPIIN (1us is too long)
  AOSPI_IN_MSEL_CLR();  // enable "chip select" the SPIIN - fast implementation of digitalWrite(AOSPI_IN_MSEL, LOW );

    // WARNING: can not 'return' until MSEL is de-asserted
    
    // Now the background reception is running; have time to end the inlined spi_tx()
    aospi_out.endTransaction(); // takes ~6 us

    // Wait till data starts arriving (ie wait till first clock line flip)
    uint32_t start = micros(); // Capture start time for timeout
    uint32_t cint = AOSPI_IN_CINT_GET(); // Capture current state of clock line (via CINT tap)
    int      clkflip = 0; // Clock flip detected
    while( micros()-start<AOSPI_IN_TIMEOUT_US ) { // wait for first clock flip
      if( cint!=AOSPI_IN_CINT_GET() ) { clkflip=1; break; }
    };
    // Wait till whole telegram should have been received (all bits/clocks)
    uint32_t tele_us = (rxsize*800)/230; // 8 clocks per byte at 2.4MHz (both *100) - we assumed a slightly slower clock to be save.
    delayMicroseconds(tele_us);

    // record end time; sw overhead is 7+4
    aospi_txrx_us_= micros() - aospi_txrx_us_ - 11; 

  // Disable reception
  AOSPI_IN_MSEL_SET(); // stop chip select - fast implementation of digitalWrite(AOSPI_IN_MSEL, HIGH)
  AOSPI_IN_OENA_CLR(); // disable level shifter - fast implementation of digitalWrite(AOSPI_IN_OENA, LOW)

  // Get the received data
  if( aospi_in.numTransactionsInFlight()!=0 && aospi_in.numTransactionsCompleted()!=1 ) return aoresult_assert; // should not happen (buffers pending in flight)
  int rxact= aospi_in.numBytesReceived();
  memcpy(rx,aospi_in_buf,min(rxact,rxsize));

  // Update stats counters
  aospi_txcount++;
  aospi_rxcount++;
  aospi_txrx_size_ = txsize + rxact;

  // Return result
  if( clkflip==0 ) return aoresult_spi_noclock; // no clocks seen (the telegram might still have been received, but this still took too much time, so we flag that as an error)
  if( actsize==0 && rxact!=rxsize ) return aoresult_spi_length;  // wrong number of bytes received
  if( actsize!=0 ) *actsize=rxact;
  return aoresult_ok; // rx successful
}
// todo: Multiple aospi_tx[rx]s in a row have a 10ms wait now and then. Is this caused by 'slave' lib using freeRTOS?


// === Direction MUX ========================================================

/*!
    @brief  Sets the direction mux so that the last OSP node is connected 
            to the SPI slave (for an OSP chain using Loop).
    @note   The mux is not relevant for sending (spi_tx) only for
            reception (spi_txrx).
    @note   The OSP32 board has signaling LEDs connected to the direction 
            mux identifying its state.
    @note   Default state of the mux, after on aospi_init(), is direction BiDir.
*/
void aospi_dirmux_set_loop() {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  digitalWrite(AOSPI_IN_DIRL, 1 );
}


/*!
    @brief  Sets the direction mux so that the first OSP node is connected 
            to the SPI slave (for an OSP chain using BiDir).
    @note   The mux is not relevant for sending (spi_tx) only for
            reception (spi_txrx).
    @note   The OSP32 board has signaling LEDs connected to the direction 
            mux identifying its state.
    @note   Default state of the mux, after on aospi_init(), is direction BiDir.
*/
void aospi_dirmux_set_bidir() {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  digitalWrite(AOSPI_IN_DIRL, 0 );
}


/*!
    @brief  Inspects the direction mux, returning iff the last node (Loop) 
            is connected to the SPI slave block.
    @return If 0, the mux is connected to the first node (BiDir).
            Otherwise, the mux is connected to the last node (Loop).
*/
int  aospi_dirmux_is_loop() {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  return digitalRead(AOSPI_IN_DIRL);
}


/*!
    @brief  Inspects the direction mux, returning iff the first node (BiDir) 
            is connected to the SPI slave block.
    @return If 0, the mux is connected to the last node (Loop).
            Otherwise, the mux is connected to the first node (Bidir).
*/
int  aospi_dirmux_is_bidir() {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  return ! digitalRead(AOSPI_IN_DIRL);
}


// === Testing ==============================================================


/*!
    @brief  Sets the output-enable of the outgoing level shifter to `val`.
    @param  val
            0 to disable output (LED off), 1 to enable (LED on).
    @note   This function should not be called during normal operation.
            It is intended fo test purposes (test the PCB).
    @note   The OSP32 board has a signaling LED ("OUT") connected to 
            this line, identifying its state.
    @note   Do not use while telegrams are sent, since aospi_tx()/aospi_txrx()
            controls output-enable.
*/
void aospi_outoena_set( int val ) {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  digitalWrite(AOSPI_OUT_OENA, val);
}


/*!
    @brief  Returns the state of the output-enable of the outgoing level shifter.
    @return 0 when output disabled (LED off), 1 when enabled (LED on).
    @note   For testing, see aospi_outoena_set().
*/
int aospi_outoena_get( ) {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  return digitalRead(AOSPI_OUT_OENA);
}


/*!
    @brief  Sets the output-enable of the incoming level shifter to `val`.
    @param  val
            0 to disable output (LED off), 1 to enable (LED on).
    @note   This function should not be called during normal operation.
            It is intended fo test purposes (test the PCB).
    @note   The OSP32 board has a signaling LED ("IN") connected to 
            this line, identifying its state.
    @note   Do not use while telegrams are received, since aospi_txrx()
            controls output-enable.
*/
void aospi_inoena_set( int val ) {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  digitalWrite(AOSPI_IN_OENA, val);
}


/*!
    @brief  Returns the state of the output-enable of the incoming level shifter.
    @return 0 when output disabled (LED off), 1 when enabled (LED on).
    @note   For testing, see aospi_outoena_set().
*/
int aospi_inoena_get( ) {
  AORESULT_ASSERT( aospi_phy!=aospi_phy_undef );
  return digitalRead(AOSPI_IN_OENA);
}


// === Manchester ===========================================================


// Python program to generate the lookup table aospi_manchester[]
//
// # OSP uses Manchester encoding IEEE 802.4: a 1 bit is represented 
// # by a rising edge (0b01) and 0 bit by a falling edge (0b10).
// print("static const uint16_t aospi_manchester[] = {" );
// for i8 in range(256):
//   if i8%8 == 0 : print( f"  /*{i8:02X}*/", end="" )
//   i16=0
//   for b1 in range(7,-1,-1) :
//     b2 = 0b01 if i8 & (1<<b1) else 0b10 # IEEE 802.4
//     i16= i16*4+b2
//   print( f" 0x{i16:04X},", end="" )
//   if i8%8 == 7 : print()
// print("};");


// The lookup table, mapping 8 payload bits to 16 Manchester'ed bits
static const uint16_t aospi_manchester[] = {
  /*00*/ 0xAAAA, 0xAAA9, 0xAAA6, 0xAAA5, 0xAA9A, 0xAA99, 0xAA96, 0xAA95,
  /*08*/ 0xAA6A, 0xAA69, 0xAA66, 0xAA65, 0xAA5A, 0xAA59, 0xAA56, 0xAA55,
  /*10*/ 0xA9AA, 0xA9A9, 0xA9A6, 0xA9A5, 0xA99A, 0xA999, 0xA996, 0xA995,
  /*18*/ 0xA96A, 0xA969, 0xA966, 0xA965, 0xA95A, 0xA959, 0xA956, 0xA955,
  /*20*/ 0xA6AA, 0xA6A9, 0xA6A6, 0xA6A5, 0xA69A, 0xA699, 0xA696, 0xA695,
  /*28*/ 0xA66A, 0xA669, 0xA666, 0xA665, 0xA65A, 0xA659, 0xA656, 0xA655,
  /*30*/ 0xA5AA, 0xA5A9, 0xA5A6, 0xA5A5, 0xA59A, 0xA599, 0xA596, 0xA595,
  /*38*/ 0xA56A, 0xA569, 0xA566, 0xA565, 0xA55A, 0xA559, 0xA556, 0xA555,
  /*40*/ 0x9AAA, 0x9AA9, 0x9AA6, 0x9AA5, 0x9A9A, 0x9A99, 0x9A96, 0x9A95,
  /*48*/ 0x9A6A, 0x9A69, 0x9A66, 0x9A65, 0x9A5A, 0x9A59, 0x9A56, 0x9A55,
  /*50*/ 0x99AA, 0x99A9, 0x99A6, 0x99A5, 0x999A, 0x9999, 0x9996, 0x9995,
  /*58*/ 0x996A, 0x9969, 0x9966, 0x9965, 0x995A, 0x9959, 0x9956, 0x9955,
  /*60*/ 0x96AA, 0x96A9, 0x96A6, 0x96A5, 0x969A, 0x9699, 0x9696, 0x9695,
  /*68*/ 0x966A, 0x9669, 0x9666, 0x9665, 0x965A, 0x9659, 0x9656, 0x9655,
  /*70*/ 0x95AA, 0x95A9, 0x95A6, 0x95A5, 0x959A, 0x9599, 0x9596, 0x9595,
  /*78*/ 0x956A, 0x9569, 0x9566, 0x9565, 0x955A, 0x9559, 0x9556, 0x9555,
  /*80*/ 0x6AAA, 0x6AA9, 0x6AA6, 0x6AA5, 0x6A9A, 0x6A99, 0x6A96, 0x6A95,
  /*88*/ 0x6A6A, 0x6A69, 0x6A66, 0x6A65, 0x6A5A, 0x6A59, 0x6A56, 0x6A55,
  /*90*/ 0x69AA, 0x69A9, 0x69A6, 0x69A5, 0x699A, 0x6999, 0x6996, 0x6995,
  /*98*/ 0x696A, 0x6969, 0x6966, 0x6965, 0x695A, 0x6959, 0x6956, 0x6955,
  /*A0*/ 0x66AA, 0x66A9, 0x66A6, 0x66A5, 0x669A, 0x6699, 0x6696, 0x6695,
  /*A8*/ 0x666A, 0x6669, 0x6666, 0x6665, 0x665A, 0x6659, 0x6656, 0x6655,
  /*B0*/ 0x65AA, 0x65A9, 0x65A6, 0x65A5, 0x659A, 0x6599, 0x6596, 0x6595,
  /*B8*/ 0x656A, 0x6569, 0x6566, 0x6565, 0x655A, 0x6559, 0x6556, 0x6555,
  /*C0*/ 0x5AAA, 0x5AA9, 0x5AA6, 0x5AA5, 0x5A9A, 0x5A99, 0x5A96, 0x5A95,
  /*C8*/ 0x5A6A, 0x5A69, 0x5A66, 0x5A65, 0x5A5A, 0x5A59, 0x5A56, 0x5A55,
  /*D0*/ 0x59AA, 0x59A9, 0x59A6, 0x59A5, 0x599A, 0x5999, 0x5996, 0x5995,
  /*D8*/ 0x596A, 0x5969, 0x5966, 0x5965, 0x595A, 0x5959, 0x5956, 0x5955,
  /*E0*/ 0x56AA, 0x56A9, 0x56A6, 0x56A5, 0x569A, 0x5699, 0x5696, 0x5695,
  /*E8*/ 0x566A, 0x5669, 0x5666, 0x5665, 0x565A, 0x5659, 0x5656, 0x5655,
  /*F0*/ 0x55AA, 0x55A9, 0x55A6, 0x55A5, 0x559A, 0x5599, 0x5596, 0x5595,
  /*F8*/ 0x556A, 0x5569, 0x5566, 0x5565, 0x555A, 0x5559, 0x5556, 0x5555,
};


// Encodes the count bytes in incoming buffer bufi using the IEEE 802.4 
// Manchester standard, and writes the encoded stream to bufo.
// Note bufo needs to be twice as big as bufi (not checked).
static void aospi_manchester_encode(const uint8_t *bufi, int count, uint8_t *bufo ) {
  for( uint8_t i=0; i<count; i++ ) {
    uint16_t w= aospi_manchester[ bufi[i] ]; 
    *bufo++ = w >> 8; // transmission is big endian
    *bufo++ = w & 0xFF;
  }
}


// === MAIN =================================================================


/*!
    @brief  Sends the `txsize` bytes in buffer `tx` to the first OSP node,
	        using the selected physical layer.
    @param  tx
            A pointer to a buffer of bytes to be sent.
    @param  txsize
            The number of bytes (of buffer tx) to be sent.
    @return aoresult_spi_buf if tx is NULL
            aoresult_spi_buf if txsize is out of bounds (0..12)
	          aoresult_ok (no error checking on send possible)
    @note   With `aospi_init()` the physical layer is selected.
	          This function just dispatches to the actual implementation.
*/
aoresult_t aospi_tx(const uint8_t * tx, int txsize) { 
  // Parameter checks
  if( txsize<0 || txsize>AOSPI_TELE_MAXSIZE ) return aoresult_spi_buf;
  if( tx==0 )  return aoresult_spi_buf;
  // Dispatch
  switch( aospi_phy ) {
    case aospi_phy_mcua  : {
      uint8_t tx_man[AOSPI_TELE_MAXSIZE*2];
      aospi_manchester_encode(tx,txsize,tx_man);
      return aospi_tx_internal(AOSPI_OUT_TYPEA_FREQ,tx_man,txsize*2);
      break;
    }
    case aospi_phy_mcub  : {
      return aospi_tx_internal(AOSPI_OUT_TYPEB_FREQ,tx,txsize);
      break;
    }
    default : {
      AORESULT_ASSERT( false ); // others not yet supported
      return aoresult_assert; 
      break;
    }
  }
}


/*!
    @brief  Sends the `txsize` bytes in buffer `tx` to the first OSP node,
	          using the selected physical layer. Waits for a response telegram 
            and stores those bytes in buffer `rx` with size `rxsize`.
    @param  tx
            A pointer to a buffer of bytes to be sent.
    @param  txsize
            The number of bytes (of buffer `tx`) to be sent.
    @param  rx
            A pointer to a caller allocated buffer of bytes to be received.
    @param  rxsize
            The size of the `rx` buffer.
    @param  actsize
            Output parameter set to the number of bytes actually received.
            Might be NULL.
    @return aoresult_spi_buf     if tx or rx is NULL
            aoresult_spi_buf     if txsize or rxsize is out of bounds (0..12)
            aoresult_assert      if the underlying driver behaves unexpectedly
            aoresult_spi_noclock if no response (clock) is received
            aoresult_spi_length  if the response had wrong number of bytes
            aoresult_ok          otherwise
    @note   Before sending, configure whether reception is from first (BiDir)
            or last (Loop) node in the OSP chain using aospi_dirmux_set_xxx()
    @note   Output parameter `actsize` might be set to NULL. In this case the 
            function will not perform a size test, will thus not return
            aoresult_spi_length. Instead the caller can inspect `*actsize`.
    @note   If caller knows how many bytes will be received, set `rxsize` to
            that amount and set `actsize` pointer to NULL. 
    @note   If caller does not knows how many bytes will be received, set 
            `rxsize` to largest possible telegram (ie AOSPI_TELE_MAXSIZE) 
            and pass an `actsize`. 
    @note   With `aospi_init()` the physical layer is selected.
	          This function just dispatches to the actual implementation.
            Recall that the physical layer is only different fro the transmit
            part, reception is always 2-wire SPI.
*/
aoresult_t aospi_txrx(const uint8_t * tx, int txsize, uint8_t * rx, int rxsize, int *actsize) {
  // Parameter checks
  if( txsize<0 || txsize>AOSPI_TELE_MAXSIZE ) return aoresult_spi_buf;
  if( tx==0 )  return aoresult_spi_buf;
  // Dispatch
  switch( aospi_phy ) {
    case aospi_phy_mcua  : {
      uint8_t tx_man[AOSPI_TELE_MAXSIZE*2];
      aospi_manchester_encode(tx,txsize,tx_man);
      return aospi_txrx_internal(AOSPI_OUT_TYPEA_FREQ, tx_man, txsize*2, rx, rxsize, actsize);
      break;
    }
    case aospi_phy_mcub  : {
      return aospi_txrx_internal(AOSPI_OUT_TYPEB_FREQ, tx, txsize, rx, rxsize, actsize);
      break;
    }
    default : {
      AORESULT_ASSERT( false ); // others not yet supported
      return aoresult_assert; 
      break;
    }
  }
}


/*!
    @brief  Initializes the SPI OUT and IN controllers and their support pins.
    @param  phy
            A physical layer (tag), indicating which physical layer to use 
            for subsequent calls to aospi_tx() and aospi_txrx.
              aospi_phy_mcua: MCU uses type A (1-wire Manchester)
              aospi_phy_mcub: MCU uses type B (2-wire SPI)
    @note   The OSP32 board's default is type B, so that is also the default
            value when calling this function. For type A physical layer some
            jumpers have to be set, see e.g. aospi/examples/aospi_mcua
    @note   Drives all (SPI and support) lines to their default level.
            The default direction is BiDir.
            Prints completion to Serial.
*/
void aospi_init( aospi_phy_t phy ) {
  AORESULT_ASSERT( aospi_phy==aospi_phy_undef ); // double init?
  AORESULT_ASSERT( phy==aospi_phy_mcua || phy==aospi_phy_mcub ); 
  aospi_out_init();
  aospi_in_init();
  Serial.printf("spi: init(%s)\n", aospi_phy_str(phy) );
  aospi_phy = phy;
}

