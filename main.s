.section .text
.global main
.code32

main:
    # dx holds serial port
    movw $0x2f8, %dx

    # erase RTC
    mov $0xE, %esi
.rtc.erase:
    mov %esi, %eax
    out %al, $0x70
    xor %eax, %eax
    out %al, $0x71
    inc %esi
    test $0x7F, %esi
    jnz .rtc.erase

    # rtc message
    movl $(message3 + 0x03400000), %esi
    movw $message_len3, %cx
    call print_string

    # dump RTC contents
    xor %esi, %esi
.rtc.display.rows:
    push $2
    push %esi
    call print_hex
    movb $':', %al
    call putchar
    movb $' ', %al
    call putchar

.rtc.display.columns:
    mov %esi, %eax
    out %al, $0x70
    in $0x71, %al
    push $2
    push %eax
    call print_hex
    movb $' ', %al
    call putchar
    inc %esi
    test $0xf, %esi
    jnz .rtc.display.columns

    movb $'\r', %al
    call putchar
    movb $'\n', %al
    call putchar

    test $0x7f, %esi
    jnz .rtc.display.rows

    # enter autoselect mode
    movb $0xAA, (0x00100555)
    movb $0x55, (0x001002AA)
    movb $0x90, (0x00100555)

    # display manufacturer ID
    movl $(message1 + 0x03400000), %esi
    movw $message_len1, %cx
    call print_string
    movl $0x00100000, %ebx
    movzxb (%ebx), %eax
    push $2
    push %eax
    call print_hex
    movb $'\r', %al
    call putchar
    movb $'\n', %al
    call putchar

    # display product ID
    movl $(message2 + 0x03400000), %esi
    movw $message_len2, %cx
    call print_string
    movl $0x00100001, %ebx
    movzxb (%ebx), %eax
    push $2
    push %eax
    call print_hex
    movb $'\r', %al
    call putchar
    movb $'\n', %al
    call putchar

    # sector protect verify
    movl $(message4 + 0x03400000), %esi
    movw $message_len4, %cx
    call print_string
    mov $0x00100002, %ebx
    xor %edi, %edi
.sector.print:
    mov %edi, %esi
    shl $16, %esi
    lea (%ebx, %esi), %eax
    push $6
    push %eax
    call print_hex
    movb $':', %al
    call putchar
    movb $' ', %al
    call putchar
    movzxb (%ebx, %esi), %eax
    test $0xff, %al
    jnz .sector.protected
    movl $(message5 + 0x03400000), %esi
    movw $message_len5, %cx
    jmp .sector.display
.sector.protected:
    movl $(message6 + 0x03400000), %esi
    movw $message_len6, %cx
.sector.display:
    call print_string
    inc %edi
    test $0x7, %edi
    jnz .sector.print

    # exit autoselect mode
    movb $0xF0, (%ebx)

.forever:
    jmp .forever


# Put a character
putchar:
    out %al, %dx
    add $5, %dx
.putchar.not_ready:
    in   %dx, %al
    and  $0x40, %eax
    jz .putchar.not_ready
    sub $5, %edx
    ret

# Print string to serial port
# DS - segment containing string
# ESI - pointer to string
# ECX - length of string
# clobbers esi, ecx
print_string:
    outsb
    add  $5, %edx
.print_string.not_ready:
    in   %dx, %al
    and  $0x40, %eax
    jz .print_string.not_ready
    sub $5, %edx
    loop print_string
    ret

# Print hex digit to serial port
print_hex:
    # create stack frame, backup registers
    enter $12, $0
    mov %esi, %ss:-4(%ebp)
    mov %edi, %ss:-8(%ebp)
    mov %ebx, %ss:-12(%ebp)

    # copy arguments
    mov %ss:8(%ebp), %esi # value to print
    mov %ss:12(%ebp), %ecx # nibbles to produce
    mov $(print_hex.lut + 0x03400000), %ebx

    # produce a hex string
    mov %esp, %edi
    sub %ecx, %esp
    dec %edi
    std
.print_hex.convert:
    mov %esi, %eax
    shr $4, %esi
    and $0xF, %eax
    xlatb
    stosb
    loop .print_hex.convert
    cld

.print_hex.display:
    # print string
    mov %edi, %esi
    inc %esi
    mov %ss:12(%ebp), %ecx # nibbles to produce
    call print_string

    mov %ss:-4(%ebp), %esi
    mov %ss:-8(%ebp), %edi
    mov  %ss:-12(%ebp), %ebx
    leave
    ret $8

.section .data

print_hex.lut: .ascii "0123456789ABCDEF"

message1: .ascii "Manufacturer ID: "
.equ message_len1, .-message1

message2: .ascii "Device ID: "
.equ message_len2, .-message2

message3: .ascii "RTC Data\r\n"
.equ message_len3, .-message3

message4: .ascii "Flash Sector Protect Verify\r\n"
.equ message_len4, .-message4

message5: .ascii "Unprotected\r\n"
.equ message_len5, .-message5

message6: .ascii "Protected\r\n"
.equ message_len6, .-message6
