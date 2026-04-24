# HOW TO RUN THE DANG THING

quartus:
programming -> blast top and bottom

terminal one:
locate to vdatp files
scp -r cli levitator@levitator.local:/home/levitator/pi_hamish

terminal two:
ssh levitator@levitator.local
ls to see files, locate cli folder
gcc cli.c -o cli -lm
./cli

terminal three:
locate to client.c (pc_cli) files.
gcc client.c -o client.exe -lws2_32
client.exe
(client.exe just runs the exe file)


Current setup:
Laptop -> TCP → Raspberry Pi -> SPI -> FPGA -> PWM?? idrk -> Transducers -> Acoustic field WOOHOO