# See LICENSE.SiFive for license details

debug_rom = debug_rom.sv debug_rom_one_scratch.sv

PREFIX ?= ${RISCV_GCC_BIN}/${RISCV_XX}
GCC = 		${PREFIX}-gcc
OBJCOPY =	${PREFIX}-objcopy
OBJDUMP =	${PREFIX}-objdump
PYTHON?=python

all: $(debug_rom) debug_rom.dump

%.sv: %.img
	$(PYTHON) gen_rom.py $<

%.img: %.bin
	dd if=$< of=$@ bs=256 count=1

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.elf: $(findstring debug_rom, $(debug_rom)).S link.ld
	$(GCC) $(if  $(findstring one_scratch,$@),,-DSND_SCRATCH=1) -I$(RISCV)/include -Tlink.ld $< -nostdlib -fPIC -static -Wl,--no-gc-sections -o $@

%.dump: %.elf
	$(OBJDUMP) -d $< --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data  > $@


clean:
	rm -f *.img *.dump *.bin *.sv
