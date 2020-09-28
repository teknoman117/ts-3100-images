# Copyright 2020 Nathaniel R. Lewis
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

.section .text
.global main
.code32

# Stack: @ 4 KiB, grows down
# Sector writing tool

# Return codes
# 0x5A: Sync ACK
# 0xFE: Command NACK
# 0xFF: Command ACK
# 0xEF: Chunk ACK
# 0xEE: RX NACK
# 0xED: Chunck Checksum NACK
# 0xEC: Write Complete
# 0xDF: Flash Write Complete
# 0xDE: Flash Write Sector Select Error
# 0xDD: Flash Write Sector ACK
# 0xDC: Sector erase complete
# 0xDB: Sector program complete
# 0xCF: Flash Read Complete
# 0xCE: Flash Read Sector Select Error
# 0xCD: Flash Read Sector ACK
# 0xBF: CMOS Clear Complete


# Commands
# 0xA5: Sync byte
# 0x10 -> Begin (64 KiB) page write
# 0x11 -> Begin (64 KiB) page read
# 0x50 -> Write Buffer
# 0x51 -> Read Buffer
# 0x60 -> Erase CMOS

main:
    # Build a CRC32 LUT at %ebx
    mov $0x00001000, %ebx
    call build_crc32

    # synchronize with transmitter
    mov $0x2f8, %edx
    call synchronize

.main.command:
    call read_char
    test $0xff, %ah
    jz .main.command.received
    mov $0xFE, %al
    call write_char
    jmp .main.command

.main.command.received:
    # check command
    cmp $0x10, %al
    je .main.writeflash
    cmp $0x11, %al
    je .main.readflash
    cmp $0x50, %al
    je .main.writebuffer
    cmp $0x60, %al
    je .main.clearcmos
    cmp $0xA5, %al
    je .main.syncbyte
    mov $0xFE, %al
    call write_char
    jmp .main.command

.main.syncbyte:
    mov $0x5A, %al
    call write_char
    jmp .main.command

    # fill buffer with 64 KiB from host
.main.writebuffer:
    mov $0xff, %al
    call write_char
    movl $0x00010000, %esi

    # receive in 1 KiB chunks
.main.writebuffer.chunk:
    mov %esi, %edi
    mov $0x404, %ecx # 1 KiB chunk + 32 bit CRC
.main.writebuffer.chunk.next_byte:
    call read_char
    test $0xff, %ah
    jnz .main.writebuffer.chunk.error
    stosb
    loop .main.writebuffer.chunk.next_byte

    # Check CRC32
    mov $0x00001000, %ebx
    mov $0x400, %ecx
    call compute_crc32
    sub $4, %edi
    movl (%edi), %ecx
    cmpl %ecx, %eax
    jne .main.writebuffer.chunk.checksum_error
    mov $0xEF, %al
    call write_char
    movl %edi, %esi
    test $0x0001FFFF, %esi
    jnz .main.writebuffer.chunk
    mov $0xEC, %al
    call write_char
    jmp .main.command

.main.writebuffer.chunk.error:
    mov $0xEE, %al
    call write_char
    call synchronize
    jmp .main.writebuffer.chunk

.main.writebuffer.chunk.checksum_error:
    mov $0xED, %al
    call write_char
    jmp .main.writebuffer.chunk

.main.writeflash:
    mov $0xff, %al
    call write_char
    movl $0x00010000, %esi

    # get sector id
    call read_char
    test $0xff, %ah
    jnz .main.writeflash.select.error
    and $0x00000007, %eax
    movl %eax, %edi
    shll $16, %edi

    mov $0xdd, %al
    call write_char

    # trigger sector erase
    movl $0x00100000, %ebx
    movb $0xAA, 0x555(%ebx)
    movb $0x55, 0x2AA(%ebx)
    movb $0x80, 0x555(%ebx)
    movb $0xAA, 0x555(%ebx)
    movb $0x55, 0x2AA(%ebx)
    movb $0x30, (%ebx, %edi)

    # wait until D7 is set
.main.writeflash.erase_incomplete:
    movb (%ebx, %edi), %al
    test $0x80, %al
    jz .main.writeflash.erase_incomplete
    mov $0xdc, %al
    call write_char

    # write data to flash
    mov $0x10000, %ecx
.main.writeflash.next_byte:
    push %edx
    lodsb
    movb $0xAA, 0x555(%ebx)
    movb $0x55, 0x2AA(%ebx)
    movb $0xA0, 0x555(%ebx)
    movb %al, (%ebx, %edi)
.main.writeflash.program_incomplete:
    movb (%ebx, %edi), %dl
    cmp %dl, %al
    jne .main.writeflash.program_incomplete
    pop %edx
    inc %edi
    loop .main.writeflash.next_byte
    mov $0xdb, %al
    call write_char
    jmp .main.command

.main.writeflash.select.error:
    mov $0xDE, %al
    call write_char
    call synchronize
    jmp .main.command

.main.readflash:
    mov $0xff, %al
    call write_char
    
    # get sector id
    call read_char
    test $0xff, %ah
    jnz .main.readflash.select.error
    and $0x00000007, %eax
    movl %eax, %esi
    mov $0xCD, %al
    call write_char

    # perform copy
    shll $16, %esi
    addl $0x00100000, %esi
    movl $0x00010000, %edi
    mov $0x4000, %ecx
    rep movsd

    mov $0xCF, %al
    call write_char
    jmp .main.command

.main.readflash.select.error:
    mov $0xCE, %al
    call write_char
    call synchronize
    jmp .main.command

.main.clearcmos:
    mov $0xff, %al
    call write_char
    mov $0xE, %esi
    mov $114, %ecx
.main.clearcmos.next_byte:
    mov %esi, %eax
    out %al, $0x70
    xor %eax, %eax
    out %al, $0x71
    inc %esi
    loop .main.clearcmos.next_byte
    mov $0xBF, %al
    call write_char
    jmp .main.command

# Wait until 16 consecutive sync bytes are received
# %dx -> serial port base
synchronize:
    push %esi
    xor %esi, %esi
.synchronize.read_char:
    call read_char
    test $0xff, %ah
    jnz synchronize
    cmp $0xA5, %al
    jne synchronize
    inc %esi
    test $0xf, %esi
    jnz .synchronize.read_char
    mov $0x5A, %al
    call write_char
    pop %esi
    ret

# Reads byte from serial port
# %edx -> serial port base
# %al -> return: contains char
# %ah -> return: error bits
read_char:
    add $5, %edx
.read_char.not_ready:
    in %dx, %al
    and $0x8B, %al
    jz .read_char.not_ready
    and $0x8A, %al
    xchg %ah, %al
    sub $5, %edx
    in %dx, %al
    ret

# Writes a byte to serial port
# %edx -> serial port base
# %al -> char to write
write_char:
    out %al, %dx
    add $5, %edx
.write_char.not_ready:
    in %dx, %al
    and $0x20, %al
    jz .write_char.not_ready
    sub $5, %edx
    ret

# Build the crc32 lookup table
# %ebx: pointer to beginning of table (1 KiB in size)
build_crc32:
    push %esi
    xor %esi, %esi
.build_crc32.fill:
    movl $0x8, %ecx
    movl %esi, %eax
.build_crc32.fill.compute:
    shrl $1, %eax
    jnc .build_crc32.fill.skip
    xorl $0xEDB88320, %eax
.build_crc32.fill.skip:
    loop .build_crc32.fill.compute
    movl %eax, (%ebx, %esi, 4)
    inc %esi
    test $0xff, %esi
    jnz .build_crc32.fill
    pop %esi
    ret

# Compute CRC32 for some data
# %ebx: pointer to the table
# %esi: pointer to data
# %ecx: number of bytes
# %eax: on return, crc32
compute_crc32:
    push %esi
    push %edx
    xor %eax, %eax
    xor %edx, %edx
    not %eax
.compute_crc32.next_byte:
    mov %al, %dl
    xor (%esi), %dl
    inc %esi
    shrl $8, %eax
    xor (%ebx, %edx, 4), %eax
    loop .compute_crc32.next_byte
    not %eax
    pop %edx
    pop %esi
    ret

.section .data
