// aospi_tx.ino - transmit only demo, switching on some LEDs
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
#include <aospi.h>

/*
DESCRIPTION
In this demo, a small set of telegrams has been hand-constructed 
(including for example CRC), Normally one would use the aoosp lib for 
that. The telegrams are passed directly to the SPI layer (aospi lib).

The amount of telegrams being send is minimal, just enough to
switch on the LEDs in channel 0 of the first SAID.

There is not much error handling, other then printing result codes.

HARDWARE
The demo runs on the OSP32 board. It only uses the first SAID, and
does not receive responses, so no demo board needs to be attached.
In Arduino select board "ESP32S3 Dev Module".

BEHAVIOR
The first RGB (L1.0 aka OUT0) of SAID OUT blinks magenta and green.

OUTPUT
Welcome to aospi_tx.ino
version: result 0.4.1 spi 0.5.1
spi: init

reset(0x000) ok
initbidir(0x001) ok
clrerror(0x000) ok
goactive(0x000) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok

setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
setpwmchn(0x001,0,0x0888,0x0011,0x0888) ok
setpwmchn(0x001,0,0x0011,0x0888,0x0011) ok
...
*/


void tele_reset() {
  const uint8_t reset[] = {0xA0, 0x00, 0x00, 0x22};
  aoresult_t result = aospi_tx( reset, sizeof reset );
  Serial.printf("reset(0x000) %s\n", aoresult_to_str(result) );
}


void tele_initbidir() {
  const uint8_t initbidir[] = {0xA0, 0x04, 0x02, 0xA9};
  aoresult_t result = aospi_tx( initbidir, sizeof initbidir);
  Serial.printf("initbidir(0x001) %s\n", aoresult_to_str(result) );
}


void tele_initloop() {
  const uint8_t initloop[] = {0xA0, 0x04, 0x03, 0x86};
  aoresult_t result = aospi_tx( initloop, sizeof initloop);
  Serial.printf("initloop(0x001) %s\n", aoresult_to_str(result) );
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


void setup() {
  Serial.begin(115200);
  Serial.printf("\n\nWelcome to aospi_tx.ino\n");
  Serial.printf("version: result %s spi %s\n", AORESULT_VERSION, AOSPI_VERSION );

  aospi_init();
  Serial.printf("\n");

  // Minimal sequence to illuminate a SAID
  tele_reset(); delayMicroseconds(150); // RESET time one node
  aospi_dirmux_set_bidir();
  tele_initbidir(); delay(15); // Worst case response time (1000 nodes Bidir)
  tele_clrerror();
  tele_goactive();
  tele_setpwmchn_hi();
  Serial.printf("\n");
}


void loop() {
  delay(1000);
  tele_setpwmchn_lo();
  delay(1000);
  tele_setpwmchn_hi();
}

