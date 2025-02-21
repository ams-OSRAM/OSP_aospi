// aospi_bringup.ino - test if communication between MCU and OSP node is operational
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
This sketch just sends a RESET telegram to a node. We check whether that 
node forwards the telegram. The checking is done using a logic analyzer. 
A next step is sending INITBIDIR.

HARDWARE
The demo runs on the OSP32 board. Add pull-ups on both SION and SIOP 
of OUT, and connect a logic analyzer to them.
In Arduino select board "ESP32S3 Dev Module".

BEHAVIOR
See readme.md for how to capture and analyze the telegrams.

OUTPUT
Nothing printed to Serial; check the logic analyzer.
*/


const uint8_t reset[]     = {0xA0, 0x00, 0x00, 0x22}; // RESET(000)
const uint8_t initbidir[] = {0xA0, 0x04, 0x02, 0xA9}; // INITBIDIR(001)


void setup() {
  aospi_init();
}


void loop() {
  aospi_tx( reset, sizeof reset );
  delayMicroseconds(150); 
  aospi_tx( initbidir, sizeof initbidir );
  delay(1000);
}
