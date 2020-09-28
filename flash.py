#!/usr/bin/python3.7

import serial
import binascii

with open('flash.bin', 'rb') as f:
    image = f.read(1024*512)

interface = serial.Serial('/dev/ttyUSB0', 115200, timeout=60)

sync_packet = bytes([0xa5] * 16)
interface.write(sync_packet)

# synchronize with board
response = interface.read(1)[0]
if response != 0x5a:
    raise ValueError("did not receive sync byte in response to sync packet (got: {})".format(response))

# erase cmos
print("sending clear cmos")
clear_cmos_packet = bytes([0x60])
interface.write(clear_cmos_packet)
while True:
    response = interface.read(1)[0]
    if response == 0xff:
        break
    elif response != 0x5a:
        raise ValueError("did not receive command ack in response to write command (got: {})".format(response))

response = interface.read(1)[0]
if response != 0xbf:
    raise ValueError("did not receive cmos clear complete (got: {})".format(response))

for sector in range(0, 1024*512, 65536):
    # start write
    write_packet = bytes([0x50])
    interface.write(write_packet)
    response = interface.read(1)[0]
    if response != 0xff:
        raise ValueError("did not receive command ack in response to write command (got: {})".format(response))

    # send packet
    print("sending sector {} image (64 KiB)".format(int(sector / 65536)))
    for idx in range(sector, sector + 65536, 1024):
        while True:
            chunk = image[idx:idx+1024]
            crc = binascii.crc32(chunk)

            interface.write(chunk)
            interface.write(crc.to_bytes(4, byteorder='little', signed=False))

            # disard 8 bytes
            response = interface.read(1)[0]
            if response == 0xEF:
                break
            else:
                print("retrying chunk")

    response = interface.read(1)[0]
    if response != 0xec:
        raise ValueError("did not receive write complete (got: {})".format(response))

    # flash write packet
    flash_packet = bytes([0x10])
    interface.write(flash_packet)
    response = interface.read(1)[0]
    if response != 0xff:
        raise ValueError("did not receive command ack in response to write command (got: {})".format(response))

    address_packet = bytes([int(sector / 65536)])
    interface.write(address_packet)
    response = interface.read(1)[0]
    if response != 0xdd:
        raise ValueError("did not receive start ack in response to sa (got: {})".format(response))

    print("sent flash write")

    response = interface.read(1)[0]
    if response != 0xdc:
        raise ValueError("did not receive erase complete (got: {})".format(response))

    print("> sector erase complete")

    acks = 0
    while True:
        response = interface.read(1)[0]
        if response == 0xda:
            print("acks - ", acks)
            acks = acks + 1
        elif response == 0xdb:
            break
        else:
            raise ValueError("did not receive program complete (got: {})".format(response))

    print("> sector program complete")
