// aospi_time.ino - measuring round trip time of command-responses
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


typedef void (*func_t)(void);


/*
DESCRIPTION
This demo measures, in a BiDirectional chain, the round trip time for two 
telegrams:  READSTAT (5 byte responses) and IDENTIFY (8 byte responses).
Both telegrams are sent to each node in chain of length 5 (and then 10 
times to allow some averaging).
The measured times are compared with each other, to check two aspects.
The first is the impact of addressing nodes further in the chain 
(adding the fast forwarding time for each intermediate node).
The second aspect is the impact of longer telegrams.

HARDWARE
The demo runs on the OSP32 board. It expects the SAIDlooker
board to be connected to the OUT and IN connector of OSP32 (loop), 
so that a chain of 5 SAIDs is established.
In Arduino select board "ESP32S3 Dev Module".

BEHAVIOR
Nothing to be seen, firmware only retrieves status and identify.

OUTPUT
Welcome to aospi_time.ino
version: result 0.4.1 spi 0.5.1
spi: init

reset(000) ok
initbidir(001) ok -> 5 nodes [132 us, hops 13]
clrerror(000) ok
goactive(000) ok

readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [35 us, hops 0]
readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [35 us, hops 0]
readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [34 us, hops 0]
readstat(001) ok -> stat 80 [35 us, hops 0]

readstat(002) ok -> stat 80 [50 us, hops 2]
readstat(002) ok -> stat 80 [49 us, hops 2]
readstat(002) ok -> stat 80 [48 us, hops 2]
readstat(002) ok -> stat 80 [50 us, hops 2]
readstat(002) ok -> stat 80 [50 us, hops 2]
readstat(002) ok -> stat 80 [48 us, hops 2]
readstat(002) ok -> stat 80 [49 us, hops 2]
readstat(002) ok -> stat 80 [49 us, hops 2]
readstat(002) ok -> stat 80 [50 us, hops 2]
readstat(002) ok -> stat 80 [48 us, hops 2]

readstat(003) ok -> stat 80 [65 us, hops 4]
readstat(003) ok -> stat 80 [64 us, hops 4]
readstat(003) ok -> stat 80 [65 us, hops 4]
readstat(003) ok -> stat 80 [62 us, hops 4]
readstat(003) ok -> stat 80 [63 us, hops 4]
readstat(003) ok -> stat 80 [64 us, hops 4]
readstat(003) ok -> stat 80 [64 us, hops 4]
readstat(003) ok -> stat 80 [63 us, hops 4]
readstat(003) ok -> stat 80 [63 us, hops 4]
readstat(003) ok -> stat 80 [64 us, hops 4]

readstat(004) ok -> stat 80 [79 us, hops 6]
readstat(004) ok -> stat 80 [80 us, hops 6]
readstat(004) ok -> stat 80 [77 us, hops 6]
readstat(004) ok -> stat 80 [79 us, hops 6]
readstat(004) ok -> stat 80 [79 us, hops 6]
readstat(004) ok -> stat 80 [78 us, hops 6]
readstat(004) ok -> stat 80 [77 us, hops 6]
readstat(004) ok -> stat 80 [79 us, hops 6]
readstat(004) ok -> stat 80 [79 us, hops 6]
readstat(004) ok -> stat 80 [78 us, hops 6]

readstat(005) ok -> stat 80 [95 us, hops 8]
readstat(005) ok -> stat 80 [95 us, hops 8]
readstat(005) ok -> stat 80 [95 us, hops 8]
readstat(005) ok -> stat 80 [93 us, hops 8]
readstat(005) ok -> stat 80 [93 us, hops 8]
readstat(005) ok -> stat 80 [94 us, hops 8]
readstat(005) ok -> stat 80 [95 us, hops 8]
readstat(005) ok -> stat 80 [93 us, hops 8]
readstat(005) ok -> stat 80 [94 us, hops 8]
readstat(005) ok -> stat 80 [95 us, hops 8]

identify(001) ok -> id 40 [45 us, hops 0]
identify(001) ok -> id 40 [46 us, hops 0]
identify(001) ok -> id 40 [45 us, hops 0]
identify(001) ok -> id 40 [45 us, hops 0]
identify(001) ok -> id 40 [45 us, hops 0]
identify(001) ok -> id 40 [44 us, hops 0]
identify(001) ok -> id 40 [45 us, hops 0]
identify(001) ok -> id 40 [45 us, hops 0]
identify(001) ok -> id 40 [44 us, hops 0]
identify(001) ok -> id 40 [45 us, hops 0]

identify(002) ok -> id 40 [58 us, hops 2]
identify(002) ok -> id 40 [59 us, hops 2]
identify(002) ok -> id 40 [58 us, hops 2]
identify(002) ok -> id 40 [58 us, hops 2]
identify(002) ok -> id 40 [58 us, hops 2]
identify(002) ok -> id 40 [59 us, hops 2]
identify(002) ok -> id 40 [58 us, hops 2]
identify(002) ok -> id 40 [59 us, hops 2]
identify(002) ok -> id 40 [58 us, hops 2]
identify(002) ok -> id 40 [59 us, hops 2]

identify(003) ok -> id 40 [75 us, hops 4]
identify(003) ok -> id 40 [73 us, hops 4]
identify(003) ok -> id 40 [72 us, hops 4]
identify(003) ok -> id 40 [75 us, hops 4]
identify(003) ok -> id 40 [73 us, hops 4]
identify(003) ok -> id 40 [72 us, hops 4]
identify(003) ok -> id 40 [75 us, hops 4]
identify(003) ok -> id 40 [74 us, hops 4]
identify(003) ok -> id 40 [73 us, hops 4]
identify(003) ok -> id 40 [73 us, hops 4]

identify(004) ok -> id 40 [90 us, hops 6]
identify(004) ok -> id 40 [89 us, hops 6]
identify(004) ok -> id 40 [90 us, hops 6]
identify(004) ok -> id 40 [89 us, hops 6]
identify(004) ok -> id 40 [88 us, hops 6]
identify(004) ok -> id 40 [89 us, hops 6]
identify(004) ok -> id 40 [89 us, hops 6]
identify(004) ok -> id 40 [90 us, hops 6]
identify(004) ok -> id 40 [90 us, hops 6]
identify(004) ok -> id 40 [90 us, hops 6]

identify(005) ok -> id 40 [105 us, hops 8]
identify(005) ok -> id 40 [104 us, hops 8]
identify(005) ok -> id 40 [105 us, hops 8]
identify(005) ok -> id 40 [105 us, hops 8]
identify(005) ok -> id 40 [105 us, hops 8]
identify(005) ok -> id 40 [104 us, hops 8]
identify(005) ok -> id 40 [104 us, hops 8]
identify(005) ok -> id 40 [105 us, hops 8]
identify(005) ok -> id 40 [105 us, hops 8]
identify(005) ok -> id 40 [104 us, hops 8]

The number of hops should be twice the node's address

Tforward is ~7.5us; in bidir this should add 15us per node
  readstat(001) average 34 us
  readstat(002) average 49 us; delta 15 us
  readstat(003) average 64 us; delta 15 us
  readstat(004) average 79 us; delta 15 us
  readstat(005) average 94 us; delta 15 us
  identify(001) average 45 us
  identify(002) average 58 us; delta 13 us
  identify(003) average 74 us; delta 16 us
  identify(004) average 89 us; delta 15 us
  identify(005) average 105 us; delta 16 us
Payload increase from 1 to 4 bytes, is 24 bits a 2.4MHz, should be 10 us
  identify(001) readstat(001) delta 11 us
  identify(002) readstat(002) delta 9 us
  identify(003) readstat(003) delta 10 us
  identify(004) readstat(004) delta 10 us
  identify(005) readstat(005) delta 11 us
*/


#define BITS_MASK(n)                  ((1<<(n))-1)                           // series of n bits: BITS_MASK(3)=0b111 (max n=31)
#define BITS_SLICE(v,lo,hi)           ( ((v)>>(lo)) & BITS_MASK((hi)-(lo)) ) // takes bits [lo..hi) from v: BITS_SLICE(0b11101011,2,6)=0b1010


void tele_reset000() {
  const uint8_t reset[] = {0xA0, 0x00, 0x00, 0x22};
  aoresult_t result = aospi_tx( reset, sizeof reset );
  Serial.printf("reset(000) %s\n", aoresult_to_str(result) );
}
void tele_initbidir001() {
  const uint8_t initbidir[] = {0xA0, 0x04, 0x02, 0xA9};
  uint8_t resp[4+2];
  aoresult_t result = aospi_txrx( initbidir, sizeof initbidir, resp, sizeof resp);
  int last = BITS_SLICE(resp[0],0,4)<<6 | BITS_SLICE(resp[1],2,8);
  Serial.printf("initbidir(001) %s -> %d nodes [%ld us, hops %ld]\n", aoresult_to_str(result), last,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_initloop001() {
  const uint8_t initloop[] = {0xA0, 0x04, 0x03, 0x86};
  uint8_t resp[4+2];
  aoresult_t result = aospi_txrx( initloop, sizeof initloop, resp, sizeof resp);
  int last = BITS_SLICE(resp[0],0,4)<<6 | BITS_SLICE(resp[1],2,8);
  Serial.printf("initloop(001) %s -> %d nodes [%ld us]\n", aoresult_to_str(result), last,aospi_txrx_us() );
}
void tele_clrerror000() {
  const uint8_t clrerror[] = {0xA0, 0x00, 0x01, 0x0D};
  aoresult_t result = aospi_tx( clrerror, sizeof clrerror);
  Serial.printf("clrerror(000) %s\n", aoresult_to_str(result) );
}
void tele_goactive000() {
  const uint8_t goactive[] = {0xA0, 0x00, 0x05, 0xB1};
  aoresult_t result = aospi_tx( goactive, sizeof goactive);
  Serial.printf("goactive(000) %s\n", aoresult_to_str(result) );
}


void tele_readstat001() {
  const uint8_t identify[] = {0xA0, 0x04, 0x40, 0x11};
  uint8_t resp[4+1];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint8_t stat = resp[3];
  Serial.printf("readstat(001) %s -> stat %X [%ld us, hops %ld]\n", aoresult_to_str(result), stat,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_readstat002() {
  const uint8_t identify[] = {0xA0, 0x08, 0x40, 0x41};
  uint8_t resp[4+1];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint8_t stat = resp[3];
  Serial.printf("readstat(002) %s -> stat %X [%ld us, hops %ld]\n", aoresult_to_str(result), stat,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_readstat003() {
  const uint8_t identify[] = {0xA0, 0x0C, 0x40, 0x94};
  uint8_t resp[4+1];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint8_t stat = resp[3];
  Serial.printf("readstat(003) %s -> stat %X [%ld us, hops %ld]\n", aoresult_to_str(result), stat,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_readstat004() {
  const uint8_t identify[] = {0xA0, 0x10, 0x40, 0xE1};
  uint8_t resp[4+1];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint8_t stat = resp[3];
  Serial.printf("readstat(004) %s -> stat %X [%ld us, hops %ld]\n", aoresult_to_str(result), stat,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_readstat005() {
  const uint8_t identify[] = {0xA0, 0x14, 0x40, 0x34};
  uint8_t resp[4+1];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint8_t stat = resp[3];
  Serial.printf("readstat(005) %s -> stat %X [%ld us, hops %ld]\n", aoresult_to_str(result), stat,aospi_txrx_us(), aospi_txrx_hops() );
}


void tele_identify001() {
  const uint8_t identify[] = {0xA0, 0x04, 0x07, 0x3A};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(001) %s -> id %lX [%ld us, hops %ld]\n", aoresult_to_str(result), id,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_identify002() {
  const uint8_t identify[] = {0xA0, 0x08, 0x07, 0x6A};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(002) %s -> id %lX [%ld us, hops %ld]\n", aoresult_to_str(result), id,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_identify003() {
  const uint8_t identify[] = {0xA0, 0x0C, 0x07, 0xBF};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(003) %s -> id %lX [%ld us, hops %ld]\n", aoresult_to_str(result), id,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_identify004() {
  const uint8_t identify[] = {0xA0, 0x10, 0x07, 0xCA};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(004) %s -> id %lX [%ld us, hops %ld]\n", aoresult_to_str(result), id,aospi_txrx_us(), aospi_txrx_hops() );
}
void tele_identify005() {
  const uint8_t identify[] = {0xA0, 0x14, 0x07, 0x1F};
  uint8_t resp[4+4];
  aoresult_t result= aospi_txrx( identify, sizeof identify, resp, sizeof resp);
  uint32_t id = (uint32_t)(resp[3])<<24 | (uint32_t)(resp[4])<<16 | (uint32_t)(resp[5])<<8 | (uint32_t)(resp[6]);
  Serial.printf("identify(005) %s -> id %lX [%ld us, hops %ld]\n", aoresult_to_str(result), id,aospi_txrx_us(), aospi_txrx_hops() );
}


uint32_t repeat( int n, func_t func) {
  uint32_t sum=0;
  for(int i=0; i<n; i++) {
    func();
    sum+=aospi_txrx_us();
    delay(1);
  }
  Serial.printf("\n");
  return (sum+n/2)/n;
}

void setup() {
  Serial.begin(115200);
  Serial.printf("\n\nWelcome to aospi_time.ino\n");
  Serial.printf("version: result %s spi %s\n", AORESULT_VERSION, AOSPI_VERSION );

  aospi_init();
  Serial.printf("\n");

  tele_reset000(); delayMicroseconds(150); 
  aospi_dirmux_set_bidir(); 
  tele_initbidir001();
  tele_clrerror000();
  tele_goactive000();
  Serial.printf("\n");

  uint32_t readstat_us[5];
  readstat_us[0]= repeat(10,tele_readstat001);
  readstat_us[1]= repeat(10,tele_readstat002);
  readstat_us[2]= repeat(10,tele_readstat003);
  readstat_us[3]= repeat(10,tele_readstat004);
  readstat_us[4]= repeat(10,tele_readstat005);

  uint32_t identify_us[5];
  identify_us[0]= repeat(10,tele_identify001);
  identify_us[1]= repeat(10,tele_identify002);
  identify_us[2]= repeat(10,tele_identify003);
  identify_us[3]= repeat(10,tele_identify004);
  identify_us[4]= repeat(10,tele_identify005);

  Serial.printf("The number of hops should be twice the node's address\n\n");

  Serial.printf("Tforward is ~7.5us; in bidir this should add 15us per node\n");
  for( int i=0; i<5; i++ ) {
    Serial.printf("  readstat(%03X) average %ld us",i+1,readstat_us[i]);
    if( i>0 ) Serial.printf("; delta %ld us",readstat_us[i]-readstat_us[i-1]);
    Serial.printf("\n");
  }
  for( int i=0; i<5; i++ ) {
    Serial.printf("  identify(%03X) average %ld us",i+1,identify_us[i]);
    if( i>0 ) Serial.printf("; delta %ld us",identify_us[i]-identify_us[i-1]);
    Serial.printf("\n");
  }

  Serial.printf("Payload increase from 1 to 4 bytes, is 24 bits a 2.4MHz, should be 10 us\n");
  for( int i=0; i<5; i++ ) {
    Serial.printf("  identify(%03X) readstat(%03X) delta %ld us\n",i+1,i+1,identify_us[i]-readstat_us[i]);
  }

}


void loop() {
}

