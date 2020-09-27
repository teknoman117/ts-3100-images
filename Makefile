PROJECT = rom
BIN = $(PROJECT).elf
PROGFILE = $(PROJECT).ihx
AS = as
LD = ld
SRCS = start.s main.s
OBJ = $(SRCS:.s=.o)
ASFLAGS =
LDFLAGS =

all: $(PROGFILE)

install: $(PROGFILE)
	minipro -p CAT28C64B -f ihex -w $(PROGFILE)

$(PROGFILE): $(BIN)
	objcopy --output-target=ihex $(BIN) $(PROGFILE)

$(BIN): $(OBJ)
	$(LD) -T $(PROJECT).ld -o $(BIN) $(LDFLAGS) $(OBJ)

%.o: %.s
	$(AS) -o $@ $< $(CFLAGS)

clean:
	rm -f $(BIN) $(PROGFILE) $(OBJ)
