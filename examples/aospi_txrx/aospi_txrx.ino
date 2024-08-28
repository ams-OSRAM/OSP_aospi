// aospi_txrx.ino - demo that switches on some LEDs but also queries nodes for some info
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
#include <aospi.h>


/*
DESCRIPTION
In this demo, a small set of telegrams has been hand-constructed
(including for example CRC), and responses are hand destructed.
Normally one would use the aoosp lib for that. The telegrams are
passed directly to/from the SPI layer (aospi lib).

The amount of telegrams being send is minimal, just enough to
switch on the LEDs in channel 0 of the first SAID.

There is not much error handling, other then printing result codes.

HARDWARE
The demo runs on the OSP32 board. It only uses the first SAID.
But it receives responses so a 'closed' OSP chain is needed. 
There are two options for that (1) "loop": connect OUT to IN with a
wire (or connect OUT to IN of a demo board and connect the OUT of 
that demo board to the IN of OSP32). (2) "bidir": plug in terminator
in OUT (or connect OUT to IN of a demo board and plug a terminator 
in OUT of that demo board).

Set the #if in setup() to match the loop/bidir choice made for the hardware.
In Arduino select board "ESP32S3 Dev Module".

OUTPUT
Welcome to aospi_txrx.ino
version: result 0.1.10 spi 0.2.7
spi: init

reset(0x000) ok
initbidir(0x001) ok -> 2 nodes
clrerror(0x000) ok
goactive(0x000) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
telegram count: tx 5 rx 1

mode bidir
identify(0x001) ok -> id 40
identify(0x001) ok -> 00 00 00 40
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
mode bidir
identify(0x001) ok -> id 40
identify(0x001) ok -> 00 00 00 40
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
mode bidir
identify(0x001) ok -> id 40
identify(0x001) ok -> 00 00 00 40
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
...
*/


#define BITS_MASK(n)                  ((1<<(n))-1)                           // series of n bits: BITS_MASK(3)=0b111 (max n=31)
#define BITS_SLICE(v,lo,hi)           ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) ) // takes bits [lo..hi) from v: BITS_SLICE(0b11101011,2,6)=0b1010


void tele_reset() {
  const uint8_t reset[] = {0xA0, 0x00, 0x00, 0x22};
  aoresult_t result = aospi_tx( reset, sizeof reset );
  Serial.printf("reset(0x000) %s\n", aoresult_to_str(result) );
}


void tele_initbidir() {
  const uint8_t initbidir[] = {0xA0, 0x04, 0x02, 0xA9};
  uint8_t resp[4+2];
  aoresult_t result = aospi_txrx( initbidir, sizeof initbidir, resp, sizeof resp);
  int last = BITS_SLICE(resp[0],0,4)<<6 | BITS_SLICE(resp[1],2,8);
  Serial.printf("initbidir(0x001) %s -> %d nodes\n", aoresult_to_str(result), last );
}


void tele_initloop() {
  const uint8_t initloop[] = {0xA0, 0x04, 0x03, 0x86};
  uint8_t resp[4+2];
  aoresult_t result = aospi_txrx( initloop, sizeof initloop, resp, sizeof resp);
  int last = BITS_SLICE(resp[0],0,4)<<6 | BITS_SLICE(resp[1],2,8);
  Serial.printf("initloop(0x001) %s -> %d nodes\n", aoresult_to_str(result), last );
}


void tele_clrerror() {
  const uint8_t clrerror[] = {0xA0, 0x00, 0x01, 0x0D};
  aoresult_t result = aospi_tx( clrerror, sizeof clrerror);
  Serial.printf("clrerror(0x000) %s\n", aoresult_to_str(result) );
}


void tele_goactive() {
  const uint8_t goactive[] = {0xA0, 0x00, 0x05, 0xB1};
  aoresult_t result = aospi_tx( goactive, sizeof goactive);
  Serial.printf("goactive(0x000) %s\n", aoresult_to_str(result) );
}


void tele_setpwmchn_hi() {
  const uint8_t setpwmchn[] = {0xA0, 0x07, 0xCF, 0x00, 0xFF, 0x08, 0x88, 0x00, 0x11, 0x08, 0x88, 0x94};
  aoresult_t result = aospi_tx( setpwmchn, sizeof setpwmchn);
  Serial.printf("setpwmchn(0x001,0,0x0888,0x0011,0x0888) %s\n", aoresult_to_str(result) );
}


void tele_setpwmchn_lo() {
  const uint8_t setpwmchn[] = {0xA0, 0x07, 0xCF, 0x00, 0xFF, 0x00, 0x11, 0x08, 0x88, 0x00, 0x11, 0x17};
  aoresult_t result = aospi_tx( setpwmchn, sizeof setpwmchn);
  Serial.printf("setpwmchn(0x001,0,0x0011,0x0888,0x0011) %s\n", aoresult_to_str(result) );
}


void tele_identify() {
  const uint8_t identify[] = {0xA0, 0x04, 0x07, 0x3A};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(0x001) %s -> id %lX\n", aoresult_to_str(result), id );
}


// Since the response size is know tele_identify() is the way to go.
// If the response size would not be not known, the below tele_identify_alt() is the way to go.
void tele_identify_alt() {
  const uint8_t identify[] = {0xA0, 0x04, 0x07, 0x3A};
  uint8_t resp[AOSPI_TELE_MAXSIZE];
  int actsize;
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp, &actsize);
  Serial.printf("identify(0x001) %s ->", aoresult_to_str(result) );
  for( int i=3; i<actsize-1; i++) Serial.printf(" %02X", resp[i] );
  Serial.printf("\n");
}


void setup() {
  Serial.begin(115200);
  Serial.printf("\n\nWelcome to aospi_txrx.ino\n");
  Serial.printf("version: result %s spi %s\n", AORESULT_VERSION, AOSPI_VERSION );

  aospi_init();
  Serial.printf("\n");

  // Minimal sequence to illuminate a SAID
  tele_reset(); delayMicroseconds(150); // RESET time one node
  #if 1
    aospi_dirmux_set_bidir(); // On OSP32 board dir led is now green
    tele_initbidir();
  #else
    aospi_dirmux_set_loop(1); // On OSP32 board dir led is now yellow
    tele_initloop();
  #endif
  tele_clrerror();
  tele_goactive();
  tele_setpwmchn_hi();

  Serial.printf("telegram count: tx %d rx %d\n", aospi_txcount_get(), aospi_rxcount_get() );
}


void loop() {
  Serial.printf( "\nmode %s\n", aospi_dirmux_is_loop() ? "loop" : "bidir" );
  tele_identify();
  tele_identify_alt();
  delay(1000);
  tele_setpwmchn_lo();
  delay(1000);
  tele_setpwmchn_hi();
}

