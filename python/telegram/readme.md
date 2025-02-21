# Telegram

A python script to dissect and pretty print telegrams, with CRC check.

## Introduction

This is a Python application that, when fed with the bytes that
make up a telegram, pretty prints a dissection of those bytes 
into telegram fields. 

As a bonus the application computes and checks the OSP CRC 
of the telegram bytes.


## File architecture

This application uses the following support files

- `setup.bat` first file to run, sets up a virtual Python environment.
  You might need to tweak the line that sets the `LOCATION` of the python executable.
- `requirements.txt` is used by `setup.bat` to install packages.
  In this case, the file contains a reference to a CRC library.
- `run.bat` actually runs `telegram.py`.
  It checks if `setup` has been run.
- `telegram.py` the script started by `run.bat`.
- `clean.bat` deletes the virtual environment.
- `readme.md` this file.


## Run

Once the project is setup (`setup.bat`), run the python app via `run.bat`. 
We must pass it the bytes that make up the telegram.
The bytes must be passed in hex, space separated.

This example shows the result of pretty printing the response 
for an initbidir telegram of a chain of length 5.

```
(env) OSP_aospi\python\telegram>run A0 15 02 6F 50 30
          +---------------+---------------+---------------+---------------+---------------+---------------+
byteval   |      A0       |      15       |      02       |      6F       |      50       |      30       |
byteix    |0 0 0 0 0 0 0 0|1 1 1 1 1 1 1 1|2 2 2 2 2 2 2 2|3 3 3 3 3 3 3 3|4 4 4 4 4 4 4 4|5 5 5 5 5 5 5 5|
bitix     |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
bitval    |1 0 1 0 0 0 0 0|0 0 0 1 0 1 0 1|0 0 0 0 0 0 1 0|0 1 1 0 1 1 1 1|0 1 0 1 0 0 0 0|0 0 1 1 0 0 0 0|
          +-------+-------+-----------+---+-+-------------+-------------------------------+---------------+
field     |preambl|      address      | psi |   command   |            payload            |      crc      |
bin       | 1010  |    0000000101     | 010 |   0000010   |   01101111    :   01010000    |   00110000    |
hex       |  0xA  |       0x005       | 0x2 |    0x02     |     0x6F      :     0x50      |   0x30 (ok)   |
meaning   |   -   |         5         |  2  |  initbidir  |      111      :      80       |    48 (ok)    |
          +-------+-------------------+-----+-------------+-------------------------------+---------------+
```

If we pass the wrong CRC as last byte, we get

```
(env) OSP_aospi\python\telegram>run A0 15 02 6F 50 31
          +---------------+---------------+---------------+---------------+---------------+---------------+
byteval   |      A0       |      15       |      02       |      6F       |      50       |      31       |
byteix    |0 0 0 0 0 0 0 0|1 1 1 1 1 1 1 1|2 2 2 2 2 2 2 2|3 3 3 3 3 3 3 3|4 4 4 4 4 4 4 4|5 5 5 5 5 5 5 5|
bitix     |7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|
bitval    |1 0 1 0 0 0 0 0|0 0 0 1 0 1 0 1|0 0 0 0 0 0 1 0|0 1 1 0 1 1 1 1|0 1 0 1 0 0 0 0|0 0 1 1 0 0 0 1|
          +-------+-------+-----------+---+-+-------------+-------------------------------+---------------+
field     |preambl|      address      | psi |   command   |            payload            |      crc      |
bin       | 1010  |    0000000101     | 010 |   0000010   |   01101111    :   01010000    |   00110001    |
hex       |  0xA  |       0x005       | 0x2 |    0x02     |     0x6F      :     0x50      |0x31 (ERR) 0x30|
meaning   |   -   |         5         |  2  |  initbidir  |      111      :      80       |  49 (ERR) 48  |
          +-------+-------------------+-----+-------------+-------------------------------+---------------+
```

Run `OSP_aoapi\python\cleanall.bat` to remove the virtual environment.
It can always be regenerated with `setup.bat`.

(end)

