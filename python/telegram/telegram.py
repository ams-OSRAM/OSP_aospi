import sys
import crcmod # only needed for CRC checking


# Create 
ospcrcfunc = crcmod.mkCrcFun(0b100101111, initCrc=0, rev=False, xorOut=0)


# Lookup table for PSI value to actual meaning
psilookup=['0','1','2','3','4','-','6','8']


# Lookup table for TID value to telegram name
commandlookup=[
  "reset"         , # 0x00
  "clrerror"      , # 0x01
  "initbidir"     , # 0x02
  "initloop"      , # 0x03
  "gosleep"       , # 0x04
  "goactive"      , # 0x05
  "godeepsleep"   , # 0x06
  "identify"      , # 0x07
  "p4errbidir"    , # 0x08
  "p4errloop"     , # 0x09
  "asktinfo"      , # 0x0A
  "askvinfo"      , # 0x0B
  "readmult"      , # 0x0C
  "setmult"       , # 0x0D
  "<unknown>"     , # 0x0E
  "sync"          , # 0x0F
  "<unknown>"     , # 0x10
  "idle"          , # 0x11
  "foundry"       , # 0x12
  "cust"          , # 0x13
  "burn"          , # 0x14
  "aread"         , # 0x15
  "load"          , # 0x16
  "gload"         , # 0x17
  "i2cread"       , # 0x18
  "i2cwrite"      , # 0x19
  "<unknown>"     , # 0x1A
  "<unknown>"     , # 0x1B
  "<unknown>"     , # 0x1C
  "<unknown>"     , # 0x1D
  "readlast"      , # 0x1E
  "<unknown>"     , # 0x1F
  "<unknown>"     , # 0x20
  "clrerror_sr"   , # 0x21
  "<unknown>"     , # 0x22
  "<unknown>"     , # 0x23
  "gosleep_sr"    , # 0x24
  "goactive_sr"   , # 0x25
  "godeepsleep_sr", # 0x26
  "<unknown>"     , # 0x27
  "<unknown>"     , # 0x28
  "<unknown>"     , # 0x29
  "<unknown>"     , # 0x2A
  "<unknown>"     , # 0x2B
  "<unknown>"     , # 0x2C
  "setmult_sr"    , # 0x2D
  "<unknown>"     , # 0x2E
  "<unknown>"     , # 0x2F
  "<unknown>"     , # 0x30
  "idle_sr"       , # 0x31
  "foundry_sr"    , # 0x32
  "cust_sr"       , # 0x33
  "burn_sr"       , # 0x34
  "aread_sr"      , # 0x35
  "load_sr"       , # 0x36
  "gload_sr"      , # 0x37
  "i2cread_sr"    , # 0x38
  "i2cwrite_sr"   , # 0x39
  "<unknown>"     , # 0x3A
  "<unknown>"     , # 0x3B
  "<unknown>"     , # 0x3C
  "<unknown>"     , # 0x3D
  "<unknown>"     , # 0x3E
  "<unknown>"     , # 0x3F
  "readstat"      , # 0x40
  "<unknown>"     , # 0x41
  "readtempstat"  , # 0x42
  "<unknown>"     , # 0x43
  "readcomst"     , # 0x44
  "<unknown>"     , # 0x45
  "readledst"     , # 0x46
  "<unknown>"     , # 0x47
  "readtemp"      , # 0x48
  "<unknown>"     , # 0x49
  "readotth"      , # 0x4A
  "setotth"       , # 0x4B
  "readsetup"     , # 0x4C
  "setsetup"      , # 0x4D
  "readpwm[chn]"  , # 0x4E
  "setpwm[chn]"   , # 0x4F
  "readcurchn"    , # 0x50
  "setcurchn"     , # 0x51
  "readtcoeff"    , # 0x52
  "settcoeff"     , # 0x53
  "readadc"       , # 0x54
  "setadc"        , # 0x55
  "readi2ccfg"    , # 0x56
  "seti2ccfg"     , # 0x57
  "readotp"       , # 0x58
  "setotp"        , # 0x59
  "readtestdata"  , # 0x5A
  "settestdata"   , # 0x5B
  "readadcdat"    , # 0x5C
  "testscan"      , # 0x5D
  "<unknown>"     , # 0x5E
  "settestpw"     , # 0x5F
  "<unknown>"     , # 0x60
  "<unknown>"     , # 0x61
  "<unknown>"     , # 0x62
  "<unknown>"     , # 0x63
  "<unknown>"     , # 0x64
  "<unknown>"     , # 0x65
  "<unknown>"     , # 0x66
  "<unknown>"     , # 0x67
  "<unknown>"     , # 0x68
  "<unknown>"     , # 0x69
  "<unknown>"     , # 0x6A
  "setotth_sr"    , # 0x6B
  "<unknown>"     , # 0x6C
  "setsetup_sr"   , # 0x6D
  "<unknown>"     , # 0x6E
  "setpwm_sr"     , # 0x6F
  "setpwmchn_sr"  , # 0x6F
  "<unknown>"     , # 0x70
  "setcurchn_sr"  , # 0x71
  "<unknown>"     , # 0x72
  "settcoeff_sr"  , # 0x73
  "<unknown>"     , # 0x74
  "setadc_sr"     , # 0x75
  "<unknown>"     , # 0x76
  "seti2ccfg_sr"  , # 0x77
  "<unknown>"     , # 0x78
  "setotp_sr"     , # 0x79
  "<unknown>"     , # 0x7A
  "<unknown>"     , # 0x7B
  "<unknown>"     , # 0x7C
  "<unknown>"     , # 0x7D
  "<unknown>"     , # 0x7E
  "settestpw_sr"  , # 0x7F
]


# Convert sys.argv (which should be like 'run  A0 15 02 6F 50 30')
# to a list of 8-bit integers. This list has size 4..12.
# Returns empty list if sys.argv is not a list of hex numbers.
def getbytes() :
  if len(sys.argv)-1 < 4 :
    print(f"ERROR: telegrams must be at least 4 telebytes long - not {len(sys.argv)-1}")
    return []
  if len(sys.argv)-1 > 12 :
    print(f"ERROR: telegrams must be at most 12 telebytes long - not {len(sys.argv)-1}")
    return []
  telebytes=[]
  for arg in sys.argv[1:] :
    try:
      telebyte = int(arg,16)
      if telebyte<0 or telebyte>255 :
        print(f"ERROR: telegrams must be at most 12 telebytes long - not {len(sys.argv)-1}")
        return []
      telebytes.append(telebyte)
    except ValueError :
      print(f"ERROR: telegrams telebytes must be hex - not {arg}")
      return []
  return telebytes


# Converts a list of 8-bit integers to a string of bits.
# [0xA0,0x15,0x02,0x6F,0x50,0x30] -> "101000000001010100100110011011110101000000110000"
def bytes2bits(telebytes) :
  bits = ""
  for telebyte in telebytes :
    bits += ("0000000"+bin(telebyte)[2:])[-8:]
  return bits


# Converts a list of 8-bit integers to a pretty printed telegram dissection.
def printdissection(telebytes) :
  bits=bytes2bits(telebytes)

  #           +---------------+---------------+---------------+---------------+---------------+---------------+
  print("          ",end="")
  for index,telebyte in enumerate(telebytes) :
    print( "+"+"-"*(8*2-1), end="" )
  print("+")

  # byteval   |      A0       |      15       |      02       |      6F       |      50       |      30       |
  print("byteval   ",end="")
  for index,telebyte in enumerate(telebytes) :
    print( f"|{f'{telebyte:02X}': ^{8*2-1}}", end="" )
  print("|")

  # byteix    |0 0 0 0 0 0 0 0|1 1 1 1 1 1 1 1|2 2 2 2 2 2 2 2|3 3 3 3 3 3 3 3|4 4 4 4 4 4 4 4|5 5 5 5 5 5 5 5|
  print("byteix    ",end="")
  for index,telebyte in enumerate(telebytes) :
    print( "|"+(f"{index: 2X}"*8)[1:], end="" )
  print("|")

  # bitix     |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
  print("bitix     ",end="")
  for index,telebyte in enumerate(telebytes) :
    print( f"|7 6 5 4 3 2 1 0", end="" )
  print("|")

  # bitval    |1 0 1 0 0 0 0 0|0 0 0 1 0 1 0 1|0 0 0 0 0 0 1 0|0 1 1 0 1 1 1 1|0 1 0 1 0 0 0 0|0 0 1 1 0 0 0 0|
  print("bitval    ",end="")
  for index,bit in enumerate(bits) :
    if index%8==0 : print( f"|{bit}", end="" )
    else : print( f"{bit:>2}", end="" )
  print("|")

  #           +-------+-------+-----------+---+-+-------------+-------------------------------+---------------+
  psa = len(bits)-32
  print("          ",end="")
  print( "+"+"-"*(4*2-1), end="" )
  print( "+"+"-"*(4*2-1), end="" )
  print( "+"+"-"*(6*2-1), end="" )
  print( "+"+"-"*(2*2-1), end="" )
  print( "+"+"-"*(1*2-1), end="" )
  print( "+"+"-"*(7*2-1), end="" )
  if psa>0 : print( "+"+"-"*(psa*2-1), end="" )
  print( "+"+"-"*(8*2-1), end="" )
  print("+")

  # field     |preambl|      address      | psi |   command   |            payload            |      crc      |

  print("field     ",end="")
  print( f"|{'preambl': ^{4*2-1}}", end="" )
  print( f"|{'address': ^{10*2-1}}", end="" )
  print( f"|{'psi': ^{3*2-1}}", end="" )
  print( f"|{'command': ^{7*2-1}}", end="" )
  if psa>0 : print( f"|{'payload': ^{psa*2-1}}", end="" )
  print( f"|{'crc': ^{8*2-1}}", end="" )
  print("|")

  # bin       | 1010  |    0000000101     | 010 |   0000010   |   01101111    :   01010000    |   00110000    |
  preamble4 = bits[0:0+4]
  address10 = bits[4:4+10]
  psi3 = bits[14:14+3]
  command7 = bits[17:17+7]
  payload = bits[24:24+psa]
  crc8 = bits[24+psa:24+psa+8]

  print("bin       ",end="")
  print( f"|{preamble4: ^{4*2-1}}", end="" )
  print( f"|{address10: ^{10*2-1}}", end="" )
  print( f"|{psi3: ^{3*2-1}}", end="" )
  print( f"|{command7: ^{7*2-1}}", end="" )
  if psa>0 : 
    for i in range(psa//8) :
      sep = "|" if i==0 else ":"
      print( f"{sep}{payload[8*i:8*i+8]: ^{8*2-1}}", end="" )
  print( f"|{crc8: ^{8*2-1}}", end="" )
  print("|")

  # hex       |  0xA  |       0x005       | 0x2 |    0x02     |     0x6F      :     0x50      |   0x30 (ok)   |
  preamble4v = int(preamble4,2)
  address10v = int(address10,2)
  psi3v = int(psi3,2)
  command7v = int(command7,2)
  payloadv = int('0'+payload,2)
  crc8v = int(crc8,2)

  crc_expect = ospcrcfunc( bytes(telebytes[:-1]) )

  print("hex       ",end="")
  s= f"0x{preamble4v:X}"
  print( f"|{s:^{4*2-1}}", end="" )
  s= f"0x{address10v:03X}"
  print( f"|{s: ^{10*2-1}}", end="" )
  s= f"0x{psi3v:X}"
  print( f"|{s: ^{3*2-1}}", end="" )
  s= f"0x{command7v:02X}"
  print( f"|{s: ^{7*2-1}}", end="" )
  if psa>0 : 
    for i in range(psa//8) :
      payload8v = int(payload[8*i:8*i+8],2)
      s= f"0x{payload8v:02X}"
      sep = "|" if i==0 else ":"
      print( f"{sep}{s: ^{8*2-1}}", end="" )
  if crc8v==crc_expect : s=f"0x{crc8v:02X} (ok)"
  else : s= f"0x{crc8v:02X} (ERR) 0x{crc_expect:02X}"
  print( f"|{s: ^{8*2-1}}", end="" )
  print("|")

  # meaning   |   -   |         5         |  2  |  initbidir  |      111      :      80       |    48 (ok)    |
  print("meaning   ",end="")
  s= f"-"
  print( f"|{s:^{4*2-1}}", end="" )
  s= f"{address10v}"
  print( f"|{s: ^{10*2-1}}", end="" )
  s= f"{psilookup[psi3v]}"
  print( f"|{s: ^{3*2-1}}", end="" )
  s= f"{commandlookup[command7v]}"
  print( f"|{s: ^{7*2-1}}", end="" )
  if psa>0 : 
    for i in range(psa//8) :
      payload8v = int(payload[8*i:8*i+8],2)
      s= f"{payload8v}"
      sep = "|" if i==0 else ":"
      print( f"{sep}{s: ^{8*2-1}}", end="" )
  if crc8v==crc_expect : s=f"{crc8v} (ok)"
  else : s= f"{crc8v} (ERR) {crc_expect}"
  print( f"|{s: ^{8*2-1}}", end="" )
  print("|")

  #           +-------+-------------------+-----+-------------+-------------------------------+---------------+
  print("          ",end="")
  print( "+"+"-"*(4*2-1), end="" )
  print( "+"+"-"*(10*2-1), end="" )
  print( "+"+"-"*(3*2-1), end="" )
  print( "+"+"-"*(7*2-1), end="" )
  if psa>0 : print( "+"+"-"*(psa*2-1), end="" )
  print( "+"+"-"*(8*2-1), end="" )
  print("+")


# Gets telegram telebytes from command line and pretty prints dissected telegram
def main():
  telebytes=getbytes()
  if telebytes==[] : 
    print("SYNTAX run <hexbyte>...")
  else :
    printdissection(telebytes)


if __name__ == "__main__":
  main()

