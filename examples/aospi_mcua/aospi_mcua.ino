// aospi_mcua.ino - demo that controls LEDs and gets info using MCU mode type A
/*****************************************************************************
 * Copyright 2025 by ams OSRAM AG                                            *
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
This is an advanced demo: we reconfigure the OSP32 board (V11 or higher is 
needed) to bypass the SAID OUT which uses MCU mode type B. Instead we attach 
the SAIDbasic board directly to the level shifter. This allows us to test the 
MCU mode type A. MCU mode type A is used in RGBI and in SAID with a default 
OTP image.
This demo is a near copy of aospi_txrx.ino, the only difference is the 
physical layer between the MCU and the first node: type A instead of type B. 
Recall that physical layer type A (1-wire Manchester encoded) only differs 
from type B (2-wire SPI with clock and data) in the transmit, in reception 
both use 2-wire SPI. 
Software wise, the main worry is selecting the correct physical layer, that 
happens in the call aospi_init(aospi_phy_mcua).

HARDWARE
The demo runs on the OSP32 board with a reconfiguration: a new OUT port is 
created bypassing SAID OUT; see the readme.md for how to do that. The new
OUT port is typically connected to the SAIDBbasic board, which is looped back.
In Arduino select board "ESP32S3 Dev Module".

WARNING
The RGBIs and the SAID v1.0 generate one extra clock transition before 
sending a response telegram in BiDir mode. The aospi driver does not (yet)
support that. These are the supported setups.
               Loop BiDir
  RGBI          yes  no
  SAID v1.0     yes  no
  SAID v1.1     yes  yes
Older EVKs have still SAID v1.0. See 
https://github.com/ams-OSRAM/OSP_aotop/tree/main/extras/manuals/saidversions
for instructions how to find out the SAIDbasic has v1.0 or v1.1 SAID in head
position.

BEHAVIOR
The first RGB triplet in the chain blinks magenta and green.
The Serial output shows status and twice the identify of that node.

OUTPUT
Welcome to aospi_mcua.ino
version: result 0.4.6 spi 0.5.9
spi: init(MCU-A)

reset(0x000)    ok
initloop(0x001) ok -> 8 nodes
clrerror(0x000) ok
goactive(0x000) ok
readstat(0x001) ok -> stat 0x80
identify(0x001) ok -> id 40
identify(0x001) ok -> 00 00 00 40

setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
setpwm(0x001,0x0888,0x0011,0x0888) ok
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwm(0x001,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
setpwm(0x001,0x0888,0x0011,0x0888) ok
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwm(0x001,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
setpwm(0x001,0x0888,0x0011,0x0888) ok
*/


#define BITS_MASK(n)                  ((1<<(n))-1)                           // series of n bits: BITS_MASK(3)=0b111 (max n=31)
#define BITS_SLICE(v,lo,hi)           ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) ) // takes bits [lo..hi) from v: BITS_SLICE(0b11101011,2,6)=0b1010


void tele_reset() {
  const uint8_t reset[] = {0xA0, 0x00, 0x00, 0x22};
  aoresult_t result = aospi_tx( reset, sizeof reset );
  Serial.printf("reset(0x000)    %s\n", aoresult_to_str(result) );
}


void tele_initbidir() {
  const uint8_t initbidir[] = {0xA0, 0x04, 0x02, 0xA9};
  uint8_t resp[4+2];
  aoresult_t result = aospi_txrx( initbidir, sizeof initbidir, resp, sizeof resp);
  int last = BITS_SLICE(resp[0],0,4)<<6 | BITS_SLICE(resp[1],2,8);
  Serial.printf("initbidir(0x001) %s -> %d nodes\n", aoresult_to_str(result), last );
  // Serial.print("  resp="); for(int i=0; i<sizeof resp; i++) Serial.printf(" %02X",resp[i]); Serial.print(".\n");
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


void tele_setpwmchn_hi() { // magenta (for SAID)
  const uint8_t setpwmchn[] = {0xA0, 0x07, 0xCF, 0x00, 0xFF, 0x08, 0x88, 0x00, 0x11, 0x08, 0x88, 0x94};
  aoresult_t result = aospi_tx( setpwmchn, sizeof setpwmchn);
  Serial.printf("setpwmchn(0x001,0,0x0888,0x0011,0x0888) %s\n", aoresult_to_str(result) );
}


void tele_setpwmchn_lo() {  // green (for SAID)
  const uint8_t setpwmchn[] = {0xA0, 0x07, 0xCF, 0x00, 0xFF, 0x00, 0x11, 0x08, 0x88, 0x00, 0x11, 0x17};
  aoresult_t result = aospi_tx( setpwmchn, sizeof setpwmchn);
  Serial.printf("setpwmchn(0x001,0,0x0011,0x0888,0x0011) %s\n", aoresult_to_str(result) );
}


void tele_setpwm_hi() { // magenta (for RGBI)
  const uint8_t setpwm[] = {0xA0, 0x07, 0x4F, 0x08, 0x88, 0x00, 0x11, 0x08, 0x88, 0x24 };
  aoresult_t result = aospi_tx( setpwm, sizeof setpwm);
  Serial.printf("setpwm(0x001,0x0888,0x0011,0x0888) %s\n", aoresult_to_str(result) );
}


void tele_setpwm_lo() {  // green (for RGBI)
  const uint8_t setpwm[] = {0xA0, 0x07, 0x4F, 0x00, 0x11, 0x08, 0x88, 0x00, 0x11, 0xA7}; 
  aoresult_t result = aospi_tx( setpwm, sizeof setpwm);
  Serial.printf("setpwm(0x001,0x0011,0x0888,0x0011) %s\n", aoresult_to_str(result) );
}


void tele_readstat() {
  const uint8_t identify[] = {0xA0, 0x04, 0x40, 0x11};
  uint8_t resp[4+1];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint8_t stat = resp[3];
  Serial.printf("readstat(0x001) %s -> stat 0x%02X\n", aoresult_to_str(result), stat );
  // Serial.print("  resp="); for(int i=0; i<sizeof resp; i++) Serial.printf(" %02X",resp[i]); Serial.print(".\n");
}


void tele_identify() {
  const uint8_t identify[] = {0xA0, 0x04, 0x07, 0x3A};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(0x001) %s -> id %lX\n", aoresult_to_str(result), id );
  // Serial.print("  resp="); for(int i=0; i<sizeof resp; i++) Serial.printf(" %02X",resp[i]); Serial.print(".\n");
}


// Since the response size is known tele_identify() above is the way to go.
// If the response size would not be not known, tele_identify_alt() below is the way to go.
void tele_identify_alt() {
  const uint8_t identify[] = {0xA0, 0x04, 0x07, 0x3A};
  uint8_t resp[AOSPI_TELE_MAXSIZE];
  int actsize;
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp, &actsize);
  Serial.printf("identify(0x001) %s ->", aoresult_to_str(result) );
  for( int i=3; i<actsize-1; i++) Serial.printf(" %02X", resp[i] );
  Serial.printf("\n");
  // Serial.print("  resp="); for(int i=0; i<sizeof resp; i++) Serial.printf(" %02X",resp[i]); Serial.print(".\n");
}


void setup() {
  Serial.begin(115200);
  Serial.printf("\n\nWelcome to aospi_mcua.ino\n");
  Serial.printf("version: result %s spi %s\n", AORESULT_VERSION, AOSPI_VERSION );

  // The key difference with aospi_txrx.ino is the next line: initialization of 
  // the aospi library selecting a different physical layer (MCU mode Type A).
  aospi_init(aospi_phy_mcua);
  Serial.printf("\n");

  // Minimal sequence to illuminate a SAID
  tele_reset(); delayMicroseconds(150); // RESET time one node
  // Init with matching dirmux
  #if 0
    aospi_dirmux_set_bidir(); // On OSP32 board dirmux LED is now green
    tele_initbidir();
  #else
    aospi_dirmux_set_loop();  // On OSP32 board dirmux LED is now yellow
    tele_initloop();
  #endif
  // Switch to active (need to clear error flags first)
  tele_clrerror();
  tele_goactive();
  // Some reads to check that works too
  tele_readstat();
  tele_identify();
  tele_identify_alt();

  // Now start the blinking
  Serial.printf("\n");
  tele_setpwmchn_hi(); tele_setpwm_hi(); // SAID or RGBI; either will be happy
  delay(5000);
}


void loop() {
  tele_setpwmchn_lo(); tele_setpwm_lo(); // SAID or RGBI; either will be happy
  delay(2000);
  tele_setpwmchn_hi(); tele_setpwm_hi(); // SAID or RGBI; either will be happy
  delay(2000);
}

