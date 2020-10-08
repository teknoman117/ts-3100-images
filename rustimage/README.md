A bare metal rust image for i386EX
----------------------------------

Why? Who knows. I was bored.

Build Instructions
------------------
~~~~
git clone https://github.com/teknoman117/ts3100-images
cd ts3100-images/rustimage
as --32 -o startup.o src/startup.s
cargo build --release
objcopy -O binary target/i386-baremetal/release/rustimage.elf rom.bin
minipro -p CAT28C64B -w rom.bin
~~~~
