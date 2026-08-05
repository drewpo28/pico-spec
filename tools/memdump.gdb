# memdump.gdb — dump Z80 address space + all 8 physical RAM pages (128K)
# Usage (in Debug Console): source tools/memdump.gdb
# Then run task "Pico: Memory Dump" (Ctrl+Alt+D)

# Logical Z80 address space via current mapping. Handles BOTH layouts —
# pico-spec (4x16K slots) and pico-speccy (8x8K slots since the Next MMU
# refactor) — in pure GDB command language (the pico toolchain gdb has no
# Python). 8-slot builds write picospec_s{0-7}.bin (8K each) + the Next
# MMU/nextreg state to /tmp/picospec_next.txt; memdump.py picks whichever
# set is present.
shell rm -f /tmp/picospec_s0.bin /tmp/picospec_s1.bin /tmp/picospec_s2.bin /tmp/picospec_s3.bin /tmp/picospec_s4.bin /tmp/picospec_s5.bin /tmp/picospec_s6.bin /tmp/picospec_s7.bin /tmp/picospec_mem0.bin /tmp/picospec_mem1.bin /tmp/picospec_mem2.bin /tmp/picospec_mem3.bin /tmp/picospec_next.txt
set $nslots = sizeof(MemESP::ramCurrent)/sizeof(MemESP::ramCurrent[0])
if $nslots == 8
  if MemESP::ramCurrent[0] != 0
    dump binary memory /tmp/picospec_s0.bin MemESP::ramCurrent[0] (MemESP::ramCurrent[0] + 8192)
  end
  if MemESP::ramCurrent[1] != 0
    dump binary memory /tmp/picospec_s1.bin MemESP::ramCurrent[1] (MemESP::ramCurrent[1] + 8192)
  end
  if MemESP::ramCurrent[2] != 0
    dump binary memory /tmp/picospec_s2.bin MemESP::ramCurrent[2] (MemESP::ramCurrent[2] + 8192)
  end
  if MemESP::ramCurrent[3] != 0
    dump binary memory /tmp/picospec_s3.bin MemESP::ramCurrent[3] (MemESP::ramCurrent[3] + 8192)
  end
  if MemESP::ramCurrent[4] != 0
    dump binary memory /tmp/picospec_s4.bin MemESP::ramCurrent[4] (MemESP::ramCurrent[4] + 8192)
  end
  if MemESP::ramCurrent[5] != 0
    dump binary memory /tmp/picospec_s5.bin MemESP::ramCurrent[5] (MemESP::ramCurrent[5] + 8192)
  end
  if MemESP::ramCurrent[6] != 0
    dump binary memory /tmp/picospec_s6.bin MemESP::ramCurrent[6] (MemESP::ramCurrent[6] + 8192)
  end
  if MemESP::ramCurrent[7] != 0
    dump binary memory /tmp/picospec_s7.bin MemESP::ramCurrent[7] (MemESP::ramCurrent[7] + 8192)
  end
  set logging file /tmp/picospec_next.txt
  set logging overwrite on
  set logging redirect on
  set logging enabled on
  printf "mmu: %02X %02X %02X %02X %02X %02X %02X %02X\n", MemESP::mmu[0], MemESP::mmu[1], MemESP::mmu[2], MemESP::mmu[3], MemESP::mmu[4], MemESP::mmu[5], MemESP::mmu[6], MemESP::mmu[7]
  printf "next_rom_mask=%02X nextRamReady=%d divmmc_mapped=%d romInUse=%d\n", MemESP::next_rom_mask, MemESP::nextRamReady, MemESP::divmmc_mapped, MemESP::romInUse
  printf "NextReg: sel=%02X 7FFD=%02X DFFD=%02X 1FFD=%02X 123B=%02X\n", NextReg::selected, NextReg::port7FFD, NextReg::portDFFD, NextReg::port1FFD, NextReg::port123B
  printf "reg05=%02X reg07=%02X reg08=%02X reg22=%02X reg69=%02X\n", NextReg::reg[0x05], NextReg::reg[0x07], NextReg::reg[0x08], NextReg::reg[0x22], NextReg::reg[0x69]
  printf "reg50..57: %02X %02X %02X %02X %02X %02X %02X %02X\n", NextReg::reg[0x50], NextReg::reg[0x51], NextReg::reg[0x52], NextReg::reg[0x53], NextReg::reg[0x54], NextReg::reg[0x55], NextReg::reg[0x56], NextReg::reg[0x57]
  printf "reg12=%02X reg13=%02X reg14=%02X reg15=%02X reg6B=%02X\n", NextReg::reg[0x12], NextReg::reg[0x13], NextReg::reg[0x14], NextReg::reg[0x15], NextReg::reg[0x6B]
  printf "divmmc: automap=%d conmem=%d mapram=%d bank=%d mapped=%d p0lo=%08X p0hi=%08X\n", DivMMC::automap, DivMMC::conmem, DivMMC::mapram, DivMMC::bank, MemESP::divmmc_mapped, (unsigned)MemESP::page0_lo, (unsigned)MemESP::page0_hi
  dump binary memory /tmp/picospec_nxtmmc.bin MemESP::nextDivRomBase (MemESP::nextDivRomBase + 8192)
  dump binary memory /tmp/picospec_nextrom.bin MemESP::nextRomBase (MemESP::nextRomBase + 65536)
  if DivMMC::bank_ptr[0] != 0
    dump binary memory /tmp/picospec_divbank0.bin DivMMC::bank_ptr[0] (DivMMC::bank_ptr[0] + 8192)
  end
  if DivMMC::bank_ptr[1] != 0
    dump binary memory /tmp/picospec_divbank1.bin DivMMC::bank_ptr[1] (DivMMC::bank_ptr[1] + 8192)
  end
  if DivMMC::bank_ptr[2] != 0
    dump binary memory /tmp/picospec_divbank2.bin DivMMC::bank_ptr[2] (DivMMC::bank_ptr[2] + 8192)
  end
  if DivMMC::bank_ptr[3] != 0
    dump binary memory /tmp/picospec_divbank3.bin DivMMC::bank_ptr[3] (DivMMC::bank_ptr[3] + 8192)
  end
  if MemESP::altRomBase != 0
    dump binary memory /tmp/picospec_altrom.bin MemESP::altRomBase (MemESP::altRomBase + 32768)
  end
  printf "reg8C=%02X altrom_rd=%d altrom_wr=%d altrom_sel=%d\n", NextReg::reg[0x8C], MemESP::altrom_rd, MemESP::altrom_wr, MemESP::altrom_sel
  dump binary memory /tmp/picospec_page0B.bin MemESP::nextRamPtr[0x0B] (MemESP::nextRamPtr[0x0B] + 8192)
  dump binary memory /tmp/picospec_page10.bin MemESP::nextRamPtr[0x10] (MemESP::nextRamPtr[0x10] + 8192)
  dump binary memory /tmp/picospec_page11.bin MemESP::nextRamPtr[0x11] (MemESP::nextRamPtr[0x11] + 8192)
  printf "slots: %08X %08X %08X %08X %08X %08X %08X %08X\n", (unsigned)MemESP::ramCurrent[0], (unsigned)MemESP::ramCurrent[1], (unsigned)MemESP::ramCurrent[2], (unsigned)MemESP::ramCurrent[3], (unsigned)MemESP::ramCurrent[4], (unsigned)MemESP::ramCurrent[5], (unsigned)MemESP::ramCurrent[6], (unsigned)MemESP::ramCurrent[7]
  printf "nextRomBase=%08X nextDivRomBase=%08X\n", (unsigned)MemESP::nextRomBase, (unsigned)MemESP::nextDivRomBase
  printf "palCtrl=%02X palIdx=%02X border=%d\n", NEXTVID::palCtrl, NEXTVID::palIdx, VIDEO::borderColor
  printf "rawPal0 ink0-7: %03X %03X %03X %03X %03X %03X %03X %03X\n", NEXTVID::rawPal[0][0], NEXTVID::rawPal[0][1], NEXTVID::rawPal[0][2], NEXTVID::rawPal[0][3], NEXTVID::rawPal[0][4], NEXTVID::rawPal[0][5], NEXTVID::rawPal[0][6], NEXTVID::rawPal[0][7]
  printf "rawPal0 pap16-23: %03X %03X %03X %03X %03X %03X %03X %03X\n", NEXTVID::rawPal[0][16], NEXTVID::rawPal[0][17], NEXTVID::rawPal[0][18], NEXTVID::rawPal[0][19], NEXTVID::rawPal[0][20], NEXTVID::rawPal[0][21], NEXTVID::rawPal[0][22], NEXTVID::rawPal[0][23]
  printf "lut0 ink0-7: %02X %02X %02X %02X %02X %02X %02X %02X\n", NEXTVID::lut[0][0], NEXTVID::lut[0][1], NEXTVID::lut[0][2], NEXTVID::lut[0][3], NEXTVID::lut[0][4], NEXTVID::lut[0][5], NEXTVID::lut[0][6], NEXTVID::lut[0][7]
  printf "lut0 pap16-23: %02X %02X %02X %02X %02X %02X %02X %02X\n", NEXTVID::lut[0][16], NEXTVID::lut[0][17], NEXTVID::lut[0][18], NEXTVID::lut[0][19], NEXTVID::lut[0][20], NEXTVID::lut[0][21], NEXTVID::lut[0][22], NEXTVID::lut[0][23]
  printf "lut4 pap16-23: %02X %02X %02X %02X %02X %02X %02X %02X\n", NEXTVID::lut[4][16], NEXTVID::lut[4][17], NEXTVID::lut[4][18], NEXTVID::lut[4][19], NEXTVID::lut[4][20], NEXTVID::lut[4][21], NEXTVID::lut[4][22], NEXTVID::lut[4][23]
  set logging enabled off
else
  dump binary memory /tmp/picospec_mem0.bin MemESP::ramCurrent[0] (MemESP::ramCurrent[0] + 16384)
  dump binary memory /tmp/picospec_mem1.bin MemESP::ramCurrent[1] (MemESP::ramCurrent[1] + 16384)
  dump binary memory /tmp/picospec_mem2.bin MemESP::ramCurrent[2] (MemESP::ramCurrent[2] + 16384)
  dump binary memory /tmp/picospec_mem3.bin MemESP::ramCurrent[3] (MemESP::ramCurrent[3] + 16384)
end

# All 8 physical RAM pages (128K) via MemESP::ram[N]._int->p
# Note: pages backed by PSRAM_SPI or SWAP read whatever was last synced into the
# cache buffer at _int->p; non-cached PSRAM/SWAP regions cannot be dumped here.
dump binary memory /tmp/picospec_ram0.bin MemESP::ram[0]._int->p (MemESP::ram[0]._int->p + 16384)
dump binary memory /tmp/picospec_ram1.bin MemESP::ram[1]._int->p (MemESP::ram[1]._int->p + 16384)
dump binary memory /tmp/picospec_ram2.bin MemESP::ram[2]._int->p (MemESP::ram[2]._int->p + 16384)
dump binary memory /tmp/picospec_ram3.bin MemESP::ram[3]._int->p (MemESP::ram[3]._int->p + 16384)
dump binary memory /tmp/picospec_ram4.bin MemESP::ram[4]._int->p (MemESP::ram[4]._int->p + 16384)
dump binary memory /tmp/picospec_ram5.bin MemESP::ram[5]._int->p (MemESP::ram[5]._int->p + 16384)
dump binary memory /tmp/picospec_ram6.bin MemESP::ram[6]._int->p (MemESP::ram[6]._int->p + 16384)
dump binary memory /tmp/picospec_ram7.bin MemESP::ram[7]._int->p (MemESP::ram[7]._int->p + 16384)

set logging file /tmp/picospec_regs.txt
set logging overwrite on
set logging redirect on
set logging enabled on

printf "AF=%04X\n",  (unsigned short)((Z80::regA << 8) | (Z80::carryFlag ? Z80::sz5h3pnFlags | 1 : Z80::sz5h3pnFlags))
printf "BC=%04X\n",  (unsigned short)Z80::regBC.word
printf "DE=%04X\n",  (unsigned short)Z80::regDE.word
printf "HL=%04X\n",  (unsigned short)Z80::regHL.word
printf "AFx=%04X\n", (unsigned short)Z80::regAFx.word
printf "BCx=%04X\n", (unsigned short)Z80::regBCx.word
printf "DEx=%04X\n", (unsigned short)Z80::regDEx.word
printf "HLx=%04X\n", (unsigned short)Z80::regHLx.word
printf "IX=%04X\n",  (unsigned short)Z80::regIX.word
printf "IY=%04X\n",  (unsigned short)Z80::regIY.word
printf "SP=%04X\n",  (unsigned short)Z80::regSP.word
printf "PC=%04X\n",  (unsigned short)Z80::regPC.word
printf "I=%02X\n",   (unsigned char)Z80::regI
printf "R=%02X\n",   (unsigned char)Z80::regR
printf "IM=%d\n",    (int)Z80::modeINT
printf "IFF1=%d\n",  (int)Z80::ffIFF1
printf "IFF2=%d\n",  (int)Z80::ffIFF2
printf "halted=%d\n",(int)Z80::halted
printf "bankLatch=%d\n",  (int)MemESP::bankLatch
printf "romLatch=%d\n",   (int)MemESP::romLatch
printf "videoLatch=%d\n", (int)MemESP::videoLatch
printf "romInUse=%d\n",   (int)MemESP::romInUse
printf "pagingLock=%d\n", (int)MemESP::pagingLock
printf "page0ram=%d\n",   (int)MemESP::page0ram
printf "tstates=%u\n",    (unsigned int)CPU::tstates
printf "ram0_type=%d\n",  (int)MemESP::ram[0]._int->mem_type
printf "ram1_type=%d\n",  (int)MemESP::ram[1]._int->mem_type
printf "ram2_type=%d\n",  (int)MemESP::ram[2]._int->mem_type
printf "ram3_type=%d\n",  (int)MemESP::ram[3]._int->mem_type
printf "ram4_type=%d\n",  (int)MemESP::ram[4]._int->mem_type
printf "ram5_type=%d\n",  (int)MemESP::ram[5]._int->mem_type
printf "ram6_type=%d\n",  (int)MemESP::ram[6]._int->mem_type
printf "ram7_type=%d\n",  (int)MemESP::ram[7]._int->mem_type

set logging enabled off
set logging redirect off

echo \nMemory dump files written to /tmp/picospec_mem{0-3}.bin, /tmp/picospec_ram{0-7}.bin and /tmp/picospec_regs.txt\n
