/*

WD1793 Floppy Disk controller emulation

Copyright ©2017 Juan Carlos González Amestoy

(Adaptation to ESPectrum / Betadisk (C) 2025 Víctor Iborra [Eremus])

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <stdlib.h>
#include <stdio.h>
#include "wd1793.h"
#include "Debug.h"
#include "Config.h"
#include "CPU.h"
#include "Ports.h"
#include "MemESP.h"
#include "OSDMain.h"
#include "messages.h"
#include "Z80_JLS/z80.h"
#if !PICO_RP2040
#include "td0.h"
#include "psram_spi.h"
#include "trdos_boot.h"
#include <string.h>
#endif

static bool sclConvertToTRD(rvmWD1793 *wd);
#if !PICO_RP2040
static void mbdFlushTrack(rvmWD1793 *wd); // defined below; also re-declared near its callers
#endif

// Shared 8 KB track scratch for raw-format loads: FDI whole-track bulk read and
// TD0 streaming decode fetch a track in ONE SD multi-block read instead of one
// SPI transaction per sector (~1.4 ms each on plain SPI cards). RP2350-only —
// the FDI/TD0 raw paths are all under #if !PICO_RP2040.
#if !PICO_RP2040
static uint8_t g_rawTrkDataBuf[8192];
#endif

// SCL-translated track-0 cache (was per-fdd Track0[2304], 2 copies). Only one fdd
// owns it at a time; a different fdd clears the previous owner's sclConverted flag
// so the buffer is regenerated. On RP2350 it ALIASES the first 2304 B of
// g_rawTrkDataBuf to save SRAM: SCL never uses g_rawTrkDataBuf, and FDI/TD0 never
// use this cache, for the SAME disk — they can only collide ACROSS drives (SCL in
// one + FDI/TD0 in another). invalidateSclCacheForScratch() handles that: it drops
// the SCL cache at every g_rawTrkDataBuf (re)fill, so track-0 is reconverted on the
// next SCL read (a few ms, only in that rare mix). RP2040 has no FDI/TD0 raw path,
// so it keeps a dedicated buffer.
static rvmWD1793 *s_scl_track0_owner = nullptr;
#if !PICO_RP2040
static unsigned char* const s_scl_track0 = (unsigned char*)g_rawTrkDataBuf;
static inline void invalidateSclCacheForScratch() {
    if (s_scl_track0_owner) {
        s_scl_track0_owner->sclConverted = false;
        s_scl_track0_owner = nullptr;
    }
}
#else
static unsigned char s_scl_track0[2304];
static inline void invalidateSclCacheForScratch() {}
#endif

static unsigned char* claim_scl_track0(rvmWD1793 *wd) {
    if (s_scl_track0_owner && s_scl_track0_owner != wd) {
        s_scl_track0_owner->sclConverted = false;
    }
    s_scl_track0_owner = wd;
    return s_scl_track0;
}

// #pragma GCC optimize("O3")

//Step rates
static const uint32_t srate[8][4]={
  {1500,3000,5000,7500}, //1mhz mfm test
  {750,1500,2500,3750}, //2mhz mfm test
  {1500,3000,5000,7500}, //1mhz fm test
  {750,1500,2500,3750}, //2mhz fm test
  {92,95,99,104}, //1mhz mfm !test
  {46,47,49,52}, //2mhz mfm !test
  {92,95,99,104}, //1mhz fm !test
  {46,47,49,52}, //2mhz fm !test
};

// Sectdatapos = (cursect * 392) + 146 + 16;
static const uint16_t sectdatapos[16]= { 162,554,946,1338,1730,2122,2514,2906,3298,3690,4082,4474,4866,5258,5650,6042 };

#define mark 0xa1a1a1
#define indexMark 0xC2
#define sectorMark 0xA1

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#if !PICO_RP2040
static uint16_t vgCrc(uint16_t crc, uint8_t byte);
void td0LoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side);
#endif

#ifndef ESP_PLATFORM
#define heap_caps_calloc(n, size, caps) calloc(n, size)
#define MALLOC_CAP_8BIT 0
#endif

IRAM_ATTR static void _end(rvmWD1793 *wd) {
  wd->status &= ~kRVMWD177XStatusBusy;
  wd->state = kRVMWD177XNone;
  wd->stepState = kRVMWD177XStepIdle;
  wd->control &= ~(kRVMWD177XWriting|kRVMWD177XDRQ);
  wd->retry = 15;
  wd->control |= kRVMWD177XINTRQ; //TODO: ADD A INTERRUPT HANDLER
}

IRAM_ATTR void _do(rvmWD1793 *wd) {

  switch(wd->state) {

    case kRVMWD177XSettingHeader: {

      if(wd->control & kRVMWD177XHLD) { // Head load
        wd->state=wd->next;
        _do(wd);
        return;
      }

      // RVM plays motor audio sample here

      wd->control|=kRVMWD177XHLD;
      // wd->c=kRVMWD177XSettingHeaderTime * ((wd->control&kRVMWD177XTest)?1:0);
      wd->c=1;
      wd->state=kRVMWD177XSettingEnd;
      wd->stepState=kRVMWD177XStepWaiting;
      return;
    }

    case kRVMWD177XSettingEnd: {
      wd->control|=kRVMWD177XHLT;
      wd->state=wd->next;
      _do(wd);
      return;
    }

    case kRVMWD177XTypeI0: {
      if(wd->command & kRVMWD177XHeadBit) {
        wd->next=kRVMWD177XTypeI1;
        wd->state=kRVMWD177XSettingHeader;
      } else {
        wd->control&=~(kRVMWD177XHLD|kRVMWD177XHLT);
        wd->state=kRVMWD177XTypeI1;
      }
      _do(wd);
      return;
    }

    case kRVMWD177XTypeI1: {
      if(wd->command & kRVMWD177XStepInOut) {
         //StepIn or stepOut
         if(wd->command & 0x20) {
           //Step Out
           //printf("Step Out\n");
           wd->control&=~kRVMWD177XDire;
         } else {
           //Step In
           //printf("Step In\n");
           wd->control|=kRVMWD177XDire;

         }
         if(wd->command & kRVMWD177XUpdateBit) {
           wd->state=kRVMWD177XTypeIUpdate;
         } else {
           wd->state=kRVMWD177XTypeISeek;
         }
      } else {
        if(wd->command & 0x20) {//Step
          if(wd->command & kRVMWD177XUpdateBit) {
            wd->state=kRVMWD177XTypeIUpdate;
          } else {
            wd->state=kRVMWD177XTypeISeek;
          }
        } else {
          if(wd->command & 0x10) { // Seek

          } else { // Restore
            wd->track=0xff;
            wd->data=0;
          }

          // printf("Seeking disk %d to track %d (disk in track: %d)\n",wd->diskS,wd->data,wd->disk[wd->diskS]->t);
          wd->dsr=wd->data;
          wd->state=kRVMWD177XTypeICheck;
        }
      }

      _do(wd);
      return;
    }

    case kRVMWD177XTypeICheck: {
      if(wd->track==wd->dsr) {
        wd->state=kRVMWD177XTypeIEnd;
      } else {
        if(wd->dsr>wd->track) {
          wd->control|=kRVMWD177XDire;
        } else {
          wd->control&=~kRVMWD177XDire;
        }
        wd->state=kRVMWD177XTypeIUpdate;
      }
      _do(wd);
      return;
    }

    case kRVMWD177XTypeIUpdate: {

      if(wd->control & kRVMWD177XDire) wd->track++;
      else wd->track--;

      //printf("UPDATE TRACK %d\n",wd->track);
      wd->state=kRVMWD177XTypeISeek;
      _do(wd);
      return;
    }

    case kRVMWD177XTypeISeek: {

      if(!(wd->control & kRVMWD177XDire) && (wd->disk[wd->diskS]->s & kRVMwdDiskOutTrack0)) {

        // printf("No seek track 0 end\n");
        wd->track=0;
        wd->state=kRVMWD177XTypeIEnd;
        _do(wd);
        return;

      } else {

        // printf("Seek track: %d side: %d\n", wd->track, wd->side);

        rvmwdDiskStep(wd, wd->control & kRVMWD177XDire ? 0x100 : 0x300);

        wd->fdd_clicks++;

        wd->c=(srate[(wd->control & kRVMWD177XRateSelect) ^ 0x4][wd->command & 0x3]) >> 3; // Value for 1 bit per diskstep / 8

        // printf("wd->c: %d, RateSelect: %d\n",wd->c,(wd->control & kRVMWD177XRateSelect) ^ 0x4);
        //printf("RATE: %llu\n",wd->c);
        //printf("STEP\n");

        wd->stepState=kRVMWD177XStepWaiting;
        if(!(wd->command & 0xe0)) //Seek or restore
          wd->state=kRVMWD177XTypeICheck;
        else
          wd->state=kRVMWD177XTypeIEnd;

        return;

      }

    }

    case kRVMWD177XTypeIEnd: {
      if(wd->command & kRVMWD177XVerifyBit) {
        //printf("Verify\n");
        if(wd->control & kRVMWD177XHLD) {
          wd->next=kRVMWD177XTypeIHeadSet;
          wd->state=kRVMWD177XSettingHeader;
        } else {
          wd->retry=5; //5 retrys
          wd->state=kRVMWD177XReadHeader;
          wd->next=kRVMWD177XTypeIHeaderReaded;
        }

        _do(wd);
        return;
      } else {
        wd->control|=kRVMWD177XINTRQ;
        wd->status|=kRVMWD177XStatusSetHead;
        wd->status&=~kRVMWD177XStatusBusy;
        wd->stepState=kRVMWD177XStepIdle;
      }
      return;
    }

    case kRVMWD177XTypeIHeadSet: {
      wd->retry=5; //5 retrys
      wd->state=kRVMWD177XReadHeader;
      wd->next=kRVMWD177XTypeIHeaderReaded;
      _do(wd);
      return;
    }

    case kRVMWD177XReadHeader: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      // _waitMark(wd);
      // printf("Waitmark ReadHeader\n");
      wd->stepState=kRVMWD177XStepWaitingMark;
      wd->marka = mark;

      wd->headerI=0xff;
      wd->state=kRVMWD177XReadHeaderBytes;
      return;

    }

    case kRVMWD177XReadAddressWait: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      //_waitMark(wd);
      // printf("Waitmark ReadAddressWait\n");
      wd->stepState=kRVMWD177XStepWaitingMark;
      wd->marka = mark;
      //

      wd->headerI=0xff;
      wd->state=kRVMWD177XReadAddressDataFlag;
      return;

    }

    case kRVMWD177XReadAddressDataFlag: {
      wd->stepState=kRVMWD177XStepReadByte;
      wd->state=kRVMWD177XReadAddressBytes;
      return;
    }

    case kRVMWD177XReadAddressBytes: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }


      if(wd->headerI==0xff) {
        if(wd->a!=0xfe) {
          wd->state=kRVMWD177XReadAddressWait;
          _do(wd);
          return;
        }
      } else {
        //printf("Data %02x crc:%04x\n",wd->a,wd->crc);
        // if(wd->headerI == 0) {
        //   wd->header[0]=1;
        //   wd->data=1;
        // } else {
          if (wd->headerI<7)
            wd->header[wd->headerI]=wd->a;
          wd->data=wd->a;
        // }
        if(wd->control & kRVMWD177XDRQ) {
          wd->status|=kRVMWD177XStatusLostData;
        }
        wd->control|=kRVMWD177XDRQ;
      }

      wd->headerI++;

      if (wd->headerI==0x6) {

        wd->sector = wd->header[0];

        // printf("Read Adress sector: %d\n",wd->header[0]);

        //printf("Header crc: %04x\n",wd->crc);

        // if(wd->crc) {
        // wd->status|=kRVMWD177XStatusCRC;
        // }

        _end(wd);

        return;

      }

      return;
    }

    case kRVMWD177XReadHeaderBytes: {

      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      if(wd->headerI!=0xff) {
        if (wd->headerI<7)
          wd->header[wd->headerI]=wd->a;
      }

      wd->headerI++;

      if(wd->headerI==0x7) {
        wd->state=wd->next;
        _do(wd);
        return;
      }

      wd->stepState=kRVMWD177XStepReadByte;
      return;
    }

    case kRVMWD177XTypeIHeaderReaded: {
      if(!wd->retry) {
        wd->status|=kRVMWD177XStatusSeek;
        _end(wd);
        return;
      }

      if(wd->header[0]!=0xfe) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      if(wd->header[1]!=wd->track) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      //printf("Header readed, track:%d sector:%d\n",wd->header[1],wd->header[3]);

      // if(wd->crc) {
      //   wd->status|=kRVMWD177XStatusCRC;
      // } else {
        wd->status&=~kRVMWD177XStatusCRC;
        _end(wd);
        return;
      // }

    }

    case kRVMWD177XTypeIISetHead: {
      wd->next=kRVMWD177XTypeIICommand;
      wd->state=kRVMWD177XSettingHeader;
      _do(wd);
      return;
    }

    case kRVMWD177XTypeIICommand: {
#if !PICO_RP2040
      if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile || wd->disk[wd->diskS]->IsTD0File))
          wd->fdiTstates = 0;
#if FDD_PORT_TRACE
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
          Debug::log("[TD0 II] cmd=%02X trk=%d sec=%d side=%d secCnt=%d lc=%d ls=%d",
                     wd->command, wd->track, wd->sector, wd->side,
                     wd->fdiSectorCount, wd->diskLoadedCyl, wd->diskLoadedSide);
#endif
#endif

      if((wd->command & 0xc0)==0x80) { // Read or Write Sector

        if(wd->command & 0x20) { // Write Sector
          // Convert SCL to TRD on first write attempt
          if(wd->disk[wd->diskS]->IsSCLFile && !wd->disk[wd->diskS]->writeprotect)
            sclConvertToTRD(wd);
          if(wd->disk[wd->diskS]->writeprotect) {
            wd->status|=kRVMWD177XStatusProtected;
            OSD::osdCenteredMsg(OSD_DSK_WRITE_PROTECT[Config::lang], LEVEL_WARN);
            _end(wd);
            return;
          }
        } else { // Read Sector
        }
        wd->retry=5; //5 retrys

        // wd->state=kRVMWD177XReadHeader;
        // wd->next=kRVMWD177XReadSectorHeader;
        // _do(wd);

        // _waitMark(wd);
        // printf("Waitmark ReadHeader\n");
        wd->stepState=kRVMWD177XStepWaitingMark;
        wd->marka = mark;
        wd->headerI=0xff;
        wd->state=kRVMWD177XReadHeaderBytes;
        wd->next=kRVMWD177XReadSectorHeader;

      } else if((wd->command & 0xf0)==0xc0) { // Read Address

        wd->retry=5; //5 retrys
        wd->state=kRVMWD177XReadAddressWait;
        // printf("State -> ReadAddressWait\n");
        _do(wd);

      } else if((wd->command & 0xf0)==0xf0) { // Write Track
        // Convert SCL to TRD on first write attempt
        if(wd->disk[wd->diskS]->IsSCLFile && !wd->disk[wd->diskS]->writeprotect)
          sclConvertToTRD(wd);
        if(wd->disk[wd->diskS]->writeprotect) {
          wd->status|=kRVMWD177XStatusProtected;
          OSD::osdCenteredMsg(OSD_DSK_WRITE_PROTECT[Config::lang], LEVEL_WARN);
          _end(wd);
          return;
        }

        wd->state=kRVMWD177XWriteTrackStart;
        wd->stepState=kRVMWD177XStepWaitIndex;
        wd->control|=kRVMWD177XDRQ;
        wd->wtrackmark=0;
        //_do(wd);

      } else if((wd->command & 0xf0)==0xe0) { // Read Track

        wd->state=kRVMWD177XReadTrackStart;
        wd->stepState=kRVMWD177XStepWaitIndex;

      }

      return;

    }

    case kRVMWD177XReadSectorHeader: {

      if(wd->header[0]!=0xfe) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      //   printf("--------------------\n");
      //   printf(" Header track, sector and side: %d, %d, %d\n",wd->header[1],wd->header[3],wd->header[2]);
      //   printf("Desired track, sector and side: %d, %d, %d\n",wd->track,wd->sector,wd->side);
      //   printf("Disk index: %d\n",wd->disk[wd->diskS]->indx);
      //   for (int i = 0; i < 7; i++) {
      //     printf("Header[%d]: %02x\n",i,wd->header[i]);
      //   }
      //   printf("--------------------\n");

#if !PICO_RP2040 && FDD_PORT_TRACE
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
          Debug::log("[TD0 hdr] got C=%d H=%d R=%d N=%d  want trk=%d sec=%d cmdSide=%d",
                     wd->header[1], wd->header[2], wd->header[3], wd->header[4],
                     wd->track, wd->sector, (wd->command>>3) & 1);
#endif

      if(wd->header[1]!=wd->track) {
        wd->state=kRVMWD177XReadHeader;
        _do(wd);
        return;
      }

      if (!wd->fastmode || (wd->command & 0x20) || wd->sector < 1 || wd->sector > 16) {

        if(wd->header[3]!=wd->sector) {
          wd->state=kRVMWD177XReadHeader;
          _do(wd);
          return;
        }

        // Side compare: reject if header side != command side (WD1793 spec)
        if((wd->command & 0x2) && ((wd->header[2] & 1) != ((wd->command>>3) & 1))) {
#if !PICO_RP2040 && FDD_PORT_TRACE
          if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
              Debug::log("[TD0 hdr] SIDE MISMATCH hdrH=%d cmdSide=%d → reject",
                         wd->header[2] & 1, (wd->command>>3) & 1);
#endif
          wd->state=kRVMWD177XReadHeader;
          _do(wd);
          return;
        }

      } else {

          // Side compare: reject if header side != command side (WD1793 spec)
          if((wd->command & 0x2) && ((wd->header[2] & 1) != ((wd->command>>3) & 1))) {
            wd->state=kRVMWD177XReadHeader;
            _do(wd);
            return;
          }

          wd->header[3] = wd->sector;
          wd->disk[wd->diskS]->indx = sectdatapos[wd->sector - 1] + 39; //5;

      }

      // if(wd->crc) {

      //   printf("ReadSectorHeader CRC %08X\n",wd->crc);

      //   wd->status|=kRVMWD177XStatusCRC;
      //   wd->state=kRVMWD177XReadHeader;
      //   _do(wd);
      //   return;

      // } else {

        wd->status &= ~kRVMWD177XStatusCRC;

        // Sector size from address mark: 0=128, 1=256, 2=512, 3=1024
        // Use real size for UDI/FDI/MBD, hardcode 256 for TRD/SCL (Betadisk standard)
        {
            uint32_t sz = 128 << (wd->header[4] & 0x03);
#if !PICO_RP2040
            if (!wd->disk[wd->diskS]->IsUDIFile && !wd->disk[wd->diskS]->IsFDIFile && !wd->disk[wd->diskS]->IsMBDFile && !wd->disk[wd->diskS]->IsTD0File)
#endif
                sz = 0x100;
            wd->c = sz;
        }

        // _waitMark(wd);
        // printf("Waitmark ReadSectorHeader\n");
        wd->stepState=kRVMWD177XStepWaitingMark;
        wd->marka = mark;
        //
        if(wd->command & 0x20) {

          //printf("Writing Command: %02x Track:%d Sector:%d Size:%llu\n",wd->command,wd->track,wd->sector,wd->c);

          wd->state=kRVMWD177XWriteDataFlag;
          wd->control|=kRVMWD177XDRQ;

        } else {

          wd->state=kRVMWD177XReadDataFlag;

        }

        return;

      // }

    }

    case kRVMWD177XReadDataFlag: {
#if !PICO_RP2040 && FDD_PORT_TRACE
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
          Debug::log("[TD0 data] data-mark FOUND, starting read of %d bytes (sec=%d)",
                     (int)wd->c, wd->sector);
#endif
      wd->stepState=kRVMWD177XStepReadByte;
      wd->state=kRVMWD177XReadDataFlag2;
      return;
    }

    case kRVMWD177XWriteDataFlag: {
      wd->stepState=kRVMWD177XStepWriteByte;
      wd->state=kRVMWD177XWriteData;
      wd->control|=kRVMWD177XWriting;
      wd->a=(wd->command & 0x1)?0xf8:0xfb;
      // wd->crc=crc(wd->crc,wd->a);
      return;
    }

case kRVMWD177XWriteData: {

      // Verificar underrun ANTES de procesar el byte
      if(wd->control & kRVMWD177XDRQ) {
        //printf("Lost data in write - aborting command\n");
        wd->status|=kRVMWD177XStatusLostData;
        wd->control&=~kRVMWD177XWriting;
        _end(wd);
        return;
      }


      wd->a=wd->data;
      // wd->crc=crc(wd->crc,wd->a);
      //printf("Write %d data: %02x CRC: %04x\n",wd->c,wd->a,wd->crc);
      wd->data=0;

      if(--wd->c) {
        wd->control|=kRVMWD177XDRQ;
      } else {
        wd->state=kRVMWD177XWriteCRC1;
        //_do(wd);
      }
      return;
    }

    case kRVMWD177XWriteCRC1: {
      // wd->a=wd->crc>>8;
      //printf("Write CRC byte: %02x CRC: %04x\n",wd->a,wd->crc);
      wd->state=kRVMWD177XWriteCRC2;
      return;
    }

    case kRVMWD177XWriteCRC2: {
      // wd->a=wd->crc & 0xff;
      //printf("Write CRC byte: %02x CRC: %04x\n",wd->a,wd->crc);
      wd->state=kRVMWD177XWriteLast;

      return;
    }

    case kRVMWD177XWriteLast: {
      wd->state=kRVMWD177XWriteEnd;
      wd->stepState=kRVMWD177XStepLastWriteByte;
      return;
    }

    case kRVMWD177XWriteEnd: {

      wd->control&=~kRVMWD177XWriting;

#if !PICO_RP2040
      // On real WD1793, writing a sector produces valid CRC. Fix the MFM buffer
      // and cached flags so subsequent reads on this track return correct CRC.
      if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile || wd->disk[wd->diskS]->IsTD0File) && wd->diskDirty) {
          for (int n = 0; n < wd->fdiSectorCount; n++) {
              uint32_t idPos = wd->fdiSectorIdPos[n];
              if (idPos + 5 < (uint32_t)wd->diskTrackLen &&
                  wd->diskTrackBuf[idPos + 1] == wd->header[1] &&
                  wd->diskTrackBuf[idPos + 2] == wd->header[2] &&
                  wd->diskTrackBuf[idPos + 3] == wd->header[3] &&
                  wd->diskTrackBuf[idPos + 4] == wd->header[4])
              {
                  wd->fdiSectorFlags[n] &= ~1;
                  int bufLen = wd->diskTrackLen;
                  for (int i = idPos + 7; i < (int)idPos + 87 && i < bufLen; i++) {
                      if (wd->diskTrackBuf[i] == 0xFB || wd->diskTrackBuf[i] == 0xF8) {
                          int slen = 128 << (wd->header[4] & 3);
                          int crcPos = i + 1 + slen;
                          if (crcPos + 2 <= bufLen) {
                              int crcStart = i;
                              while (crcStart > 0 && wd->diskTrackBuf[crcStart - 1] == 0xA1) crcStart--;
                              uint16_t crc = 0xFFFF;
                              for (int j = crcStart; j < crcPos; j++)
                                  crc = vgCrc(crc, wd->diskTrackBuf[j]);
                              wd->diskTrackBuf[crcPos] = (uint8_t)(crc >> 8);
                              wd->diskTrackBuf[crcPos + 1] = (uint8_t)(crc & 0xFF);
                          }
                          break;
                      }
                  }
                  break;
              }
          }
      }
#endif

      // Write buffer to diskfile
      // int saveptr = ftell(wd->disk[wd->diskS]->Diskfile);
      // int seekptr = (wd->track << (11 + wd->disk[wd->diskS]->sides)) + (wd->side ? 4096 : 0) + ((wd->sector - 1) << 8);
      // fseek(wd->disk[wd->diskS]->Diskfile,seekptr,SEEK_SET);
      // fwrite(wd->disk[wd->diskS]->cursectbuf,1,0x100, wd->disk[wd->diskS]->Diskfile);
      // fseek(wd->disk[wd->diskS]->Diskfile,saveptr,SEEK_SET);

      // printf("Track:%d, Side:%d, Sector: %d\n",wd->track,wd->side,wd->sector);
      // for (int i=0; i< 0x100; i+= 0x10) {
      //   printf("Pos %04x: ",i);
      //   for (int n=0; n< 0x10; n++) {
      //     printf("%02x ",wd->disk[wd->diskS]->cursectbuf[i + n]);
      //   }
      //   printf("\n");
      // }
      // printf("==================================\n");

#if !PICO_RP2040
      // MBD (MB-02 BS-DOS): persist the modified track to SD immediately after a
      // Write Sector completes. Without this the dirty track lingers in
      // diskTrackBuf and only reaches SD on the next track switch / eject /
      // reset — so a catalog rewrite done as the LAST disk op (.p defragment,
      // .e erase) is lost on re-insert, leaving the on-SD catalog out of sync
      // with the data area ("file not found" / "Sector not found").
      // mbdFlushTrack re-reads from the buffer + f_sync; it self-clears diskDirty.
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsMBDFile && wd->diskDirty)
          mbdFlushTrack(wd);
#endif

      if(wd->command & 0x10) { // Write sector: Multiple record flag on
        wd->sector++;
        wd->state=kRVMWD177XTypeIICommand;
        _do(wd);
      } else { // Write sector: Multiple record flag off
        _end(wd);
      }
      return;
    }

    case kRVMWD177XReadDataFlag2: {
      if(wd->a==0xf8) {
        wd->status|=kRVMWD177XStatusRecordType;
      } else if(wd->a==0xfb) {
        wd->status&=~kRVMWD177XStatusRecordType;
      } else {
#if !PICO_RP2040
#if FDD_PORT_TRACE
        if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
            Debug::log("[TD0 data] DataFlag2 unexpected byte a=%02X (want F8/FB) → re-search sec=%d",
                       wd->a, wd->sector);
#endif
        // FDI: data mark not found after ID match (sector has no data area).
        // Go to ReadHeader (preserves retry) and force index pulse so retry
        // decrements — prevents infinite loop from TypeIICommand resetting retry=5.
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile || wd->disk[wd->diskS]->IsTD0File)) {
            wd->disk[wd->diskS]->indx = wd->diskTrackLen; // trigger index pulse
            wd->state=kRVMWD177XReadHeader;
            wd->next=kRVMWD177XReadSectorHeader;
            _do(wd);
            return;
        }
#endif
        wd->state=kRVMWD177XTypeIICommand;
        _do(wd);
        wd->stepState=kRVMWD177XNone;
        return;
      }
      wd->state=kRVMWD177XReadData;
      return;
    }

    case kRVMWD177XReadData: {


      wd->data = wd->a;

#if !PICO_RP2040 && FDD_PORT_TRACE
      if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File &&
          (wd->c >= 1023 || (wd->c & 0xFF) == 0 || wd->c <= 3))
          Debug::log("[TD0 data] ReadData a=%02X c=%d DRQwas=%d",
                     wd->a, (int)wd->c, (wd->control & kRVMWD177XDRQ) ? 1 : 0);
#endif
      //printf("Read %d byte: %02x CRC: %04x\n",wd->c,wd->a,wd->crc);
      if(wd->control & kRVMWD177XDRQ) {
        //printf("Lost data in read\n");
        wd->status|=kRVMWD177XStatusLostData;
      }

      wd->control|=kRVMWD177XDRQ;
      if(!--wd->c) {
        wd->state=kRVMWD177XReadCRC;
        wd->c=2; // 2 bytes CRC
        return;
      }
      return;
    }

    case kRVMWD177XReadCRC: {
      // printf("Read CRC byte: %02x CRC: %04x\n",wd->a,wd->crc);
      if(!--wd->c) { // CRC readed

#if !PICO_RP2040
#if FDD_PORT_TRACE
        if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
          Debug::log("[TD0 data] sector READ COMPLETE sec=%d cmd=%02X multi=%d → %s",
                     wd->sector, wd->command, (wd->command & 0x10) ? 1 : 0,
                     (wd->command & 0x10) ? "next sector" : "END");
#endif
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile || wd->disk[wd->diskS]->IsTD0File) && wd->fdiDataCrcError) {
          wd->status |= kRVMWD177XStatusCRC;
          wd->fdiDataCrcError = false;
        } else
#endif
          wd->status&=~kRVMWD177XStatusCRC;

        if(wd->command & 0x10) { // Read sector: Multiple record flag on

          wd->sector++; // Next sector

          if (!wd->fastmode || wd->sector > 16) {

            // printf("Fast mode Next sector: ");
            // if (wd->sector > 16)
            //   printf("Sector >  16: %d\n",wd->sector);
            // else
            //   printf("Sector <= 16: %d\n",wd->sector);

            // printf("Read Sector, multiple: sector %d\n",wd->sector);

            wd->state=kRVMWD177XTypeIICommand;
            _do(wd);

          } else {

            wd->header[3] = wd->sector;
            wd->disk[wd->diskS]->indx = sectdatapos[wd->sector - 1] + 39; // + 5;

            wd->status &= ~kRVMWD177XStatusCRC;
            wd->c = 0x100; // Esto, en Betadisk, siempre será el tamaño del sector

            // _waitMark(wd);
            wd->stepState=kRVMWD177XStepWaitingMark;
            wd->marka = mark;
            //

            wd->state=kRVMWD177XReadDataFlag;

          }

        } else { // Read sector: Multiple record flag off

          _end(wd);

        }

      }

      return;

    }

    case kRVMWD177XWriteTrackStart:{
      if(wd->control & kRVMWD177XDRQ) {
        wd->status|=kRVMWD177XStatusLostData;
        _end(wd);
        return; // Missing return
      }
      wd->control|=kRVMWD177XWriting;
      wd->state=kRVMWD177XWriteTrack;

      wd->disk[wd->diskS]->indx = 0xffffffff;
      wd->disk[wd->diskS]->cursectbufpos = 0xff;
      wd->disk[wd->diskS]->indexDelay = 0;

      _do(wd);
    }

case kRVMWD177XWriteTrack: {

      if(!wd->retry) {
        _end(wd);
        return;
      }

      // Verificar underrun ANTES de procesar
      if(wd->control & kRVMWD177XDRQ) {
        printf("Lost data in write track - aborting command\n");
        wd->status|=kRVMWD177XStatusLostData;
        wd->control&=~kRVMWD177XWriting;
        _end(wd);
        return;
      }


        switch(wd->data) {

          case 0xf5: {
            wd->wtrackmark++;

            wd->a=sectorMark;
            wd->stepState=kRVMWD177XStepWriteByte;
            wd->control|=kRVMWD177XDRQ;
            // printf("kRVMWD177XWriteTrack -> Sector Mark!! ->");
            break;
            // return;
          }

          case 0xf7: {

            wd->wtrackmark=0;

            // wd->a=wd->crc>>8;
            wd->a=0;
            wd->stepState=kRVMWD177XStepWriteByte;
            wd->state=kRVMWD177XWriteTrackCRC;
            // printf("kRVMWD177XWriteTrack -> CRC!! ->");
            break;
            // return;
          }

          default: {
            if (wd->wtrackmark == 3 && wd->data == 0xfe) {
              // printf("Write Sector Header, track %d, side %d\n",wd->track,wd->side);
              wd->wtrackmark = 0b100000000;
            } else if (wd->wtrackmark == 3 && wd->data == 0xfb) {
              // printf("Write Sector Data at sector %d\n",wd->wtracksector);
              // wd->wtrackmark=0b1000000000;
              // For raw format disks (UDI/FDI/MBD), indx runs sequentially through the track buffer;
              // sectdatapos repositioning is only valid for TRD's fixed sector layout.
#if !PICO_RP2040
              if (!wd->disk[wd->diskS]->IsUDIFile && !wd->disk[wd->diskS]->IsFDIFile && !wd->disk[wd->diskS]->IsMBDFile && !wd->disk[wd->diskS]->IsTD0File)
#endif
              wd->disk[wd->diskS]->indx = sectdatapos[wd->wtracksector - 1] + 41;
            } else if (wd->wtrackmark & 0b100000000) {
              wd->wtrackmark++;
              // printf("   Write Sector Header, byte: %02x\n",wd->data);
              if (wd->wtrackmark == 0b100000001) {
                // printf("Write track to track0 side1 sector header!\n");
                if (wd->track == 0 && wd->side == 1) wd->disk[wd->diskS]->t0s1_info = wd->data;
              } else
              if (wd->wtrackmark == 0b100000011) {
                wd->wtracksector = wd->data;
                wd->wtrackmark = 0;
              }
            } else {
              wd->wtrackmark=0;
            }

            wd->a = wd->data;
            // wd->crc=crc(wd->crc,wd->a);
            // printf("Format byte: %02x\n",wd->a);
            wd->stepState=kRVMWD177XStepWriteByte;
            wd->control|=kRVMWD177XDRQ;
            break;
            // return;
          }
        }

      return;

    }

    case kRVMWD177XWriteTrackCRC: {
      // wd->a=wd->crc & 0xff;
      wd->a=0x0;
      wd->stepState=kRVMWD177XStepWriteByte;
      wd->state=kRVMWD177XWriteTrack;
      wd->control|=kRVMWD177XDRQ;

      // wd->disk[wd->diskS]->indx -= 4;
      // wd->disk[wd->diskS]->cursectbufpos = 0xffff;

      // printf("kRVMWD177XWriteTrack -> CRC!! ->");
      // printf("Format byte: %02x\n",wd->a);
      return;
    }

    case kRVMWD177XReadTrackStart: {
      wd->stepState=kRVMWD177XStepReadByte;
      wd->state=kRVMWD177XReadTrackData;
      wd->retry=1;
      // printf("ReadTrackStart!\n");
      return;
    }

    case kRVMWD177XReadTrackData: {

      // printf("ReadTrackData!\n");

      if(!wd->retry) {
        _end(wd);
        return;
      }


      if(wd->control & kRVMWD177XDRQ) wd->status|=kRVMWD177XStatusLostData;

      wd->control|=kRVMWD177XDRQ;
      wd->data=wd->a;
      return;
    }
  }
}

IRAM_ATTR void rvmWD1793Step(rvmWD1793 *wd, uint32_t steps) {

#if !PICO_RP2040
  // Hoisted out of the per-byte loop: arch/CPM-bit/fastmode can't change
  // while the Z80 is suspended inside this call.
  const bool profi_cpm = (Ports::portDFFD & 0x20) && Config::arch == "Profi";
#endif

  for (;steps > 0; steps--) {

#if !PICO_RP2040
    // Host-paced data transfer (Profi CP/M).  When DRQ is pending the CPU has
    // not yet read/written the data register, so the byte-by-byte transfer must
    // FREEZE — disk rotation and byte production both stop until the host
    // services DRQ.  Without this, a large step burst (the frame-end
    // rvmWD1793Step in CPU.cpp fires with HLD/HLT set during a read, and the
    // first status poll after a frame wrap accumulates a huge tstates_diff)
    // blasts through the entire sector before the CPU reads a single byte: the
    // data is lost (LostData) and the command ends with the host never having
    // participated, hanging the BIOS poll loop at #BF.
    // Mirrors pentevo's per-byte ts_byte pacing (S_READ → S_WAIT → S_READ).
    // Gated to Profi CP/M + non-fastmode so TR-DOS / FDI / UDI byte loops and
    // fastmode bulk transfers are unaffected.
    if (profi_cpm && !wd->fastmode &&
        (wd->control & kRVMWD177XDRQ) &&
        (wd->stepState == kRVMWD177XStepReadByte ||
         wd->stepState == kRVMWD177XStepWriteByte)) {
      break;
    }
#endif

    uint8_t d=0x0;
    uint8_t s=0x0;
    uint8_t dd=0x0;

    if(wd->disk[wd->diskS]) { // If active disk exists ..

      uint8_t t;
      uint16_t w = 0;

      if((wd->control & kRVMWD177XWriting) && !wd->disk[wd->diskS]->writeprotect) {
        w |= kRVMwdDiskControlWrite | wd->wb;
      }

      t = rvmwdDiskStep(wd, w);

      d = t;
      dd = wd->disk[wd->diskS]->a;
      s = wd->disk[wd->diskS]->s;

    }

    uint8_t pd = s ^ wd->diskP;
    wd->diskP=s;

    // Force Interrupt condition check: bit 2 = index pulse
    if ((wd->control >> 16) & 0x4) {
      if ((pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
        wd->control &= 0xffff; // clear conditions
        wd->control |= kRVMWD177XINTRQ;
      }
    }

    // printf("wd->stepState: %d\n",wd->stepState);

    switch(wd->stepState) {

      case kRVMWD177XStepIdle:{


        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) {
            wd->control&=~(kRVMWD177XHLD | kRVMWD177XHLT);
            return;
          }
        }
        break;
      }

      case kRVMWD177XStepWaiting: {

        if ((wd->track == 0 && wd->sector == 0) || wd->track == 0xff ) {
          wd->fdd_clicks = 0;
        }

        // Profi CP/M: PRO file uses fastmode=false (real MFM emulation needed
        // for Read Sector), but Type I Seek step-delay accumulator never fills
        // when CP/M poll loop spins ~30 T-states/iter, blocking Type I forever.
        // Treat Type I steps as fastmode in this case.
#if !PICO_RP2040
        bool profi_cpm_typeI =
            (profi_cpm
             && wd->disk[wd->diskS]
             && (wd->disk[wd->diskS]->IsProFile || wd->disk[wd->diskS]->IsFDIFile
                 || wd->disk[wd->diskS]->IsTD0File)
             && (wd->command & kRVMWD177XTypeI) == 0
             && wd->state == kRVMWD177XTypeICheck);
#else
        bool profi_cpm_typeI = false;
#endif

        if (wd->fastmode || profi_cpm_typeI) {
          wd->c = 0;
          _do(wd);
        } else if(!(--wd->c)) {
            _do(wd);
        }

        break;

      }

      case kRVMWD177XStepWaitingMark: {

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

#if !PICO_RP2040
        // FDI/MBD: empty track (0 sectors) — report Record Not Found immediately
        // instead of spinning for 5 full revolutions (~5 seconds).
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile || wd->disk[wd->diskS]->IsTD0File)
            && wd->state == kRVMWD177XReadHeaderBytes
            && wd->fdiSectorCount == 0)
        {
#if !PICO_RP2040
            if (wd->disk[wd->diskS]->IsProFile)
                Debug::log("[FDC!] EmptyTrack dt=%d t=%d s=%d side=%d lc=%d ls=%d",
                           (int)wd->disk[wd->diskS]->t, wd->track, wd->sector, wd->side,
                           wd->diskLoadedCyl, wd->diskLoadedSide);
#endif
            wd->status |= kRVMWD177XStatusSeek; // bit 4 = Record Not Found (Type II)
            _end(wd);
            break;
        }

        // FDI/MBD find_marker: find nearest sector header ahead of current disk->indx.
        // Uses actual MFM buffer position (not CPU T-states) for compatibility
        // with the incremental indx++ model used in rvmwdDiskStep.
        if (wd->disk[wd->diskS] && (wd->disk[wd->diskS]->IsFDIFile || wd->disk[wd->diskS]->IsMBDFile || wd->disk[wd->diskS]->IsTD0File)
            && (wd->state == kRVMWD177XReadHeaderBytes
                || wd->state == kRVMWD177XReadAddressDataFlag)
            && wd->marka == mark
            && wd->fdiSectorCount > 0)
        {
            rvmwdDisk *fdisk = wd->disk[wd->diskS];
            // Don't intercept while index pulse is pending (indx >= trkLen or 0xffffffff).
            // Let the normal step mechanism deliver the index pulse so retry decrements.
            if (fdisk->indx >= (uint32_t)wd->diskTrackLen)
                break;
            uint32_t trkLen = wd->diskTrackLen;
            if (trkLen > 0) {
                // Use current disk->indx as position in MFM buffer
                uint32_t curPos = (fdisk->indx < trkLen) ? fdisk->indx : 0;

                // Find nearest sector ahead (circular distance).
                // For Read/Write Sector (ReadHeaderBytes), skip sectors with no data area
                // (flags & 0x40) — they have no data mark, causing infinite retry loops.
                bool skipNoData = (wd->state == kRVMWD177XReadHeaderBytes);
                uint32_t bestDist = 0xFFFFFFFF;
                int bestSec = -1;
                for (int n = 0; n < wd->fdiSectorCount; n++) {
                    if (skipNoData && (wd->fdiSectorFlags[n] & 2)) continue;
                    uint32_t idPos = wd->fdiSectorIdPos[n];
                    uint32_t dist = (idPos > curPos) ? idPos - curPos
                                                     : trkLen + idPos - curPos;
                    if (dist < bestDist) { bestDist = dist; bestSec = n; }
                }
                // If all sectors have no data, trigger index pulse
                if (bestSec < 0) {
                    fdisk->indx = trkLen;
                    break;
                }

                uint32_t idPos = wd->fdiSectorIdPos[bestSec];

                // If the nearest sector is behind us (wrapped around track end),
                // we've crossed the index hole. Set indx past end of track so that
                // rvmwdDiskStep generates an index pulse (indexDelay), which will
                // naturally decrement retry via _checkIndex on the next step.
                if (idPos < curPos) {
                    fdisk->indx = trkLen; // trigger index pulse via rvmwdDiskStep
                    break; // exit — let normal stepping handle the revolution
                }

                // Populate header[] from MFM buffer (FE, C, H, R, N, CRC1, CRC2)
                for (int i = 0; i < 7 && (idPos + i) < trkLen; i++)
                    wd->header[i] = wd->diskTrackBuf[idPos + i];
                fdisk->indx = idPos + 7; // position past header
                wd->fdiDataCrcError = (wd->fdiSectorFlags[bestSec] & 1);

                if (wd->state == kRVMWD177XReadHeaderBytes) {
                    wd->state = wd->next; // → kRVMWD177XReadSectorHeader
                    _do(wd);
                } else {
                    // Read Address: position at FE for sequential read
                    fdisk->indx = idPos - 1;
                    wd->state = kRVMWD177XReadAddressDataFlag;
                    _do(wd);
                }
                break;
            }
        }
#endif

        if((wd->marka & 0xff) == dd) {
          wd->marka >>= 8;
          if(!wd->marka) {
            _do(wd);
            if(wd->control & kRVMWD177XWriting) goto write;
          }
        } else {
          wd->marka=mark;
        }

        break;

      }

      case kRVMWD177XStepReadByte: {

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

        wd->a = dd;
        _do(wd);
        wd->a = 0;

        break;

      }

      case kRVMWD177XStepWriteByte: {

  write:

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

        wd->wb = wd->a;
        _do(wd);

        break;

      }

      case kRVMWD177XStepWriteRaw: {

        // _checkIndex(wd,pd,s);
        if(wd->retry && (pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry--;
          if(!wd->retry) _do(wd);
        }
        // end _checkIndex

        wd->wb = wd->a;
        _do(wd);

        break;
      }

      case kRVMWD177XStepLastWriteByte:
        _do(wd);
        break;

      case kRVMWD177XStepWaitIndex: {
        if((pd & kRVMwdDiskOutIndex) && (s & kRVMwdDiskOutIndex)) {
          wd->retry=1;
          _do(wd);
        }
        break;
      }
    }

  }

}

IRAM_ATTR void rvmWD1793Write(rvmWD1793 *wd,uint8_t a,uint8_t value) {
  switch(a & 0x3) {

    case 0: //Command
      // // --- WD1793 TR-DOS bug workaround ---
      // // If the command is in the write range (0xB8-0xBF, 0xF8-0xFF) and not in TR-DOS write mode, ignore!
      // if ((value & 0xF8) == 0xB8 || (value & 0xF8) == 0xF8) {
      //   // This is a garbage "write" command (write sector/track)
      //   // Do NOT perform any write operations, do not damage the disk!
      //   wd->status &= ~kRVMWD177XStatusBusy;
      //   wd->control |= kRVMWD177XINTRQ;
      //   return;
      // }
      // // --- end of patch ---

      if ((value & 0xf0) == 0xd0) {
        //Force interrupt
        if(wd->status & kRVMWD177XStatusBusy) {
            wd->status &= ~kRVMWD177XStatusBusy;
          wd->state = kRVMWD177XNone;
          wd->stepState = kRVMWD177XStepIdle;
          wd->control &= ~(kRVMWD177XWriting|kRVMWD177XDRQ);
          wd->retry = 15;

        } else {

          wd->status=kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP;
          // WD2797: preserve Head Loaded state and sync disk_t to Track register
          if (wd->wd2797_mode) {
            wd->status |= kRVMWD177XStatusSetHead;
            if (wd->disk[wd->diskS]) {
              wd->disk[wd->diskS]->t = wd->track;
              // Update Track 0 output signal
              wd->disk[wd->diskS]->s = wd->track ? 0 : kRVMwdDiskOutTrack0;
            }
          }

        }

        if((value & 0xf)==0x0) {

          wd->control&=~(kRVMWD177XINTRQ|kRVMWD177XFINTRQ);

        } else {

          if(value & 0x8) {

            //Inmediate interrupt
            wd->control|=kRVMWD177XFINTRQ;

          } else {

            wd->control = (wd->control & 0xffff) | ((value & 0xf) << 16); //Set conditions

          }

        }

        return;

      }


      if(!(wd->status & kRVMWD177XStatusBusy)) {

        wd->control &= ~(kRVMWD177XINTRQ|kRVMWD177XFINTRQ);

        wd->command = value;

#if !PICO_RP2040 && FDD_PORT_TRACE
        if (wd->disk[wd->diskS] && wd->disk[wd->diskS]->IsTD0File)
            Debug::log("[TD0 cmd] OUT cmd=%02X trk=%d sec=%d side=%d pc=%04X",
                       value, wd->track, wd->sector, wd->side, Z80::getRegPC());
#endif
#if !PICO_RP2040 && FDD_PORT_TRACE
        // FDD command trace — every accepted WD1793 command with key registers.
        // Enable via -DFDD_PORT_TRACE=ON. Decodes the (ROM14,CPM) port scheme bug
        // class: watch `side` vs the command side bit (bit3) on RDSEC/WRSEC.
        if (Config::arch == "Profi") {
            const char *cn;
            uint8_t c = wd->command;
            if (!(c & 0x80))                cn = (c & 0x10) ? "SEEK" : ((c & 0x60) ? "STEP" : "RESTORE");
            else if ((c & 0xE0) == 0x80)    cn = (c & 0x20) ? "WRSEC" : "RDSEC";
            else if ((c & 0xF0) == 0xC0)    cn = "RDADDR";
            else if ((c & 0xF0) == 0xE0)    cn = "RDTRK";
            else if ((c & 0xF0) == 0xF0)    cn = "WRTRK";
            else if ((c & 0xF0) == 0xD0)    cn = "FORCEINT";
            else                            cn = "?";
            Debug::log("[FDC CMD] %02X %-7s trk=%d sec=%d side=%d dataReg=%d "
                       "diskS=%d cpm=%d romInUse=%d fast=%d pc=%04X",
                       c, cn, wd->track, wd->sector, wd->side, wd->data,
                       wd->diskS, (int)((Ports::portDFFD & 0x20) != 0),
                       (int)MemESP::romInUse, (int)wd->fastmode, Z80::getRegPC());
        }
#endif

        if(wd->disk[wd->diskS]  && (wd->control & (kRVMWD177XPower0 << wd->diskS))) {

          //Issue command
          if(wd->command & kRVMWD177XTypeI) {


            //Type II, III, IV kRVMWD177XTypeIISetHead
            if((wd->command & 0xc0)==0x80 || (wd->command & 0xfb)==0xc0 || (wd->command & 0xfb)==0xf0 || (wd->command & 0xfb)==0xe0) {

              //TYPE
              wd->state=kRVMWD177XTypeIISetHead;
              wd->status=kRVMWD177XStatusBusy;

              // WD2797: side select from command bit 1 (SSO — Side Select Output)
              if (wd->wd2797_mode) {
                wd->side = (wd->command >> 1) & 1;
              }

              if(wd->command & kRVMWD177XVerifyBit) {

                wd->stepState = kRVMWD177XStepWaiting;

                // if (wd->fastmode)
                //   wd->c = 1;
                // else
                  wd->c = 937; // 7500 (Value for 1 bit per diskstep) / 8
                // printf("Waiting 30ms\n");

              } else {

                _do(wd);

              }

            } else {
              // Force Interrupt (0xD0) or unrecognized Type IV
              wd->control |= kRVMWD177XINTRQ;
            }

          } else {

            //Type I
            // Clear DRQ (mirrors UnrealSpeccy S_TYPE1_CMD: status &= ~WDS_DRQ, rqs=0)
            wd->control &= ~kRVMWD177XDRQ;
            wd->status = kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP | kRVMWD177XStatusBusy;
            wd->state = kRVMWD177XTypeI0;
            // Profi boot fix: SYS-ROM probe at 0x0710 and CP/M DSKKE9A at 0x40EC
            // both poll "wait for BUSY=1" right after every Type I command.
            // If the command completes fully inside the _do() chain below
            // (e.g. Seek with verify to the already-current track), BUSY would
            // drop to 0 before BIOS reads the status, causing an infinite poll.
            // Arm a one-shot BUSY=1 to guarantee the poll sees it once.
            //
            // Armed for Profi when EITHER:
            //   - CP/M mode (DFFD bit5=1) — DSKKE9A driver, OR
            //   - SYS ROM active (romInUse==0) — boot ROM FDC probe at 0x0710.
            // TR-DOS (romInUse=1, CP/M off) relies on BUSY=0 after instant
            // completion, so the oneshot stays OFF there.
            if (Config::arch == "Profi" &&
                ((Ports::portDFFD & 0x20) || MemESP::romInUse == 0))
              wd->typeI_busy_oneshot = true;

            _do(wd);

            // Complete Type I (Seek/Step/Restore) step delays immediately.
            // Copy-protected loaders issue the next command before step delays expire.
            // WD2797 (MB-02) Seek (0x10-0x1F): don't complete instantly — BS-DOS
            // calibration checks Busy flag timing to determine step rate.
            // Restore (0x00-0x0F) still completes instantly (255 steps would hang).
            //
            // Profi CP/M exception: do NOT complete instantly. The DSKKE9A driver
            // uses CALL 0x40EA which re-issues Type I commands in a loop while BUSY=1.
            // Instant completion means every re-issue is accepted (BUSY→0 before next
            // OUT), causing infinite stack growth. Keep BUSY=1 for multiple FDDStep
            // calls (like real hardware / UnrealSpeccy) so re-issues are rejected.
            bool profi_cpm = (Config::arch == "Profi" && (Ports::portDFFD & 0x20));
            if (!(wd->command & kRVMWD177XTypeI) && !profi_cpm) {
              while (wd->stepState == kRVMWD177XStepWaiting) {
                _do(wd);
              }
            }

            // If Type I completed synchronously (TypeIEnd reached inside _do chain),
            // INTRQ was set before TR-DOS ROM enters its polling loop on port 0xFF.
            // TR-DOS reads the status register first (clearing INTRQ), then polls
            // the system register — missing the INTRQ and hanging forever.
            // Fix: promote INTRQ to FINTRQ, which is not cleared by status reads
            // but is still visible on the system register (port 0xFF bit 7).
            // FINTRQ will be cleared when the next command is issued.
            if (wd->state == kRVMWD177XTypeIEnd
                && wd->stepState == kRVMWD177XStepIdle
                && (wd->control & kRVMWD177XINTRQ)) {
              wd->control |= kRVMWD177XFINTRQ;
            }

          }

        } else {

          // No disk or no power: command rejected.
          // Real WD1793 sets BUSY=1, then completes with NOT_READY + INTRQ.
          // We simulate this: set BUSY|NOT_READY, enter TypeIEnd state with a
          // brief step counter (50) so BUSY stays high for a few FDDStep calls
          // before completing — matching real hardware timing.
          // This keeps BUSY=1 long enough for Profi CP/M DSKKE9A's CALL 0x40EA
          // re-issue loop to reject further OUTs while BUSY, preventing infinite
          // stack growth from looping at CPU speed without the Z80 WAIT line.
          // FINTRQ: survives status register reads (unlike XINTRQ) so the SYS
          // port INTRQ bit remains set for the data-transfer loop at 0x418C.
          wd->status = kRVMWD177XStatusNotReady | kRVMWD177XStatusBusy;
          wd->control |= kRVMWD177XINTRQ;
          wd->state = kRVMWD177XTypeIEnd;
          wd->stepState = kRVMWD177XStepWaiting;
          wd->c = 50;
          wd->command = 0x00;
          wd->control |= kRVMWD177XFINTRQ;
#if !PICO_RP2040
          if (Config::arch == "Profi")
            Debug::log("[FDC!] CMD=0x%02X NOT_READY: disk=%p power=0x%X diskS=%d",
                       wd->command, wd->disk[wd->diskS],
                       wd->control & 0xf000, wd->diskS);
#endif
        }

      } else {
#if !PICO_RP2040
        if (Config::arch == "Profi" && wd->disk[wd->diskS] &&
            (wd->disk[wd->diskS]->IsProFile || wd->disk[wd->diskS]->IsTD0File)) {
          Debug::log("[FDCw!] CMD=0x%02X REJECTED (BUSY) t=%d s=%d prev_cmd=0x%02X st=%d ss=%d",
                     value, wd->track, wd->sector, wd->command, wd->state, wd->stepState);
        }
#endif
      }

      break;

    case 1: //Track
      //if(!(wd->status & kRVMWD177XStatusBusy)) {
        wd->track=value;
      //}
#if !PICO_RP2040 && FDD_PORT_TRACE
      if (Config::arch == "Profi")
        Debug::log("[FDC TRK] track<-%d pc=%04X", value, Z80::getRegPC());
#endif
      break;
    case 2: //Sector
      //if(!(wd->status & kRVMWD177XStatusBusy)) {
        wd->sector=value;
      //}
      break;
    case 3: //Data
      wd->data=value;
      wd->control &= ~kRVMWD177XDRQ;
      break;
  }
}

IRAM_ATTR uint8_t rvmWD1793Read(rvmWD1793 *wd,uint8_t a) {

  uint8_t r;

  switch(a & 0x3) {
    case 0: //Status
    {
      wd->control&=~kRVMWD177XINTRQ;

      r=wd->status & 0xff;
      // One-shot BUSY for instant-complete Type I (Profi CP/M boot fix).
      if (wd->typeI_busy_oneshot) {
        r |= kRVMWD177XStatusBusy;
        wd->typeI_busy_oneshot = false;
      }


      if(wd->disk[wd->diskS]) {
        if(wd->status & kRVMWD177XStatusSetWP)  {
          if(wd->disk[wd->diskS]->writeprotect) {
            r|=kRVMWD177XStatusProtected;
          } else {
            r&=~kRVMWD177XStatusProtected;
          }
        }

        if((wd->status & kRVMWD177XStatusSetTrack0) && (wd->disk[wd->diskS]->s & kRVMwdDiskOutTrack0)) {
          r|=kRVMWD177XStatusTrack0;
        }

        if((wd->status & kRVMWD177XStatusSetIndex) && (wd->disk[wd->diskS]->s & kRVMwdDiskOutIndex)) {
          r|=kRVMWD177XStatusIndex;
        } else {
          if(wd->control & kRVMWD177XDRQ) r|=kRVMWD177XStatusDataRequest;
        }

        // HeadLoaded is only visible in status if HLT bit (bit 3 of system reg) is set.
        // UnrealSpeccy: status & ((system & 8) ? 0xFF : ~WDS_HEADL)
        // WD2797: bit 5 in Type I status = Spin-up complete (NOT Head Loaded)
        // Don't set it — BS-DOS may use this bit differently
        if (wd->wd2797_mode) {
          // Don't set bit 5 for WD2797 Type I status
        } else {
          if((wd->status & kRVMWD177XStatusSetHead) && (wd->control & kRVMWD177XTest)) {
            r|=kRVMWD177XStatusHeadLoaded;
          }
        }
      } else {
        r|=kRVMWD177XStatusNotReady;
      }
      return r;
    }
    case 1: //Track
      return wd->track;
    case 2: //Sector
      return wd->sector;
    case 3: //Data
      // if(!(wd->control&kRVMWD177XDRQ)) {
      //   printf("Read data register overrunning\n");
      // }

      wd->control &= ~kRVMWD177XDRQ;

      // printf("read data: %02x\n", wd->data);
      return wd->data;

  }

  return 0;
}

static void fdiFlushTrack(rvmWD1793 *wd);
static void mbdFlushTrack(rvmWD1793 *wd);

// Fast mode (sectdatapos fast addressing) is only valid for standard-format
// disks (SCL/TRD). Raw-format images (UDI/FDI/MBD/TD0/PRO) need real MFM
// emulation and must run with fastmode=false. There is a single per-controller
// fastmode flag, but the decision is per-disk — so it must follow the
// *currently selected* drive only. Otherwise a raw image mounted in another
// slot drags a standard disk in the active drive down to slow mode (the
// multi-disk bug: A:=TRD was forced slow whenever B:/C:/D: held a UDI/FDI/etc).
static bool diskFastCapable(rvmwdDisk *d) {
  if (!d) return true;
#if !PICO_RP2040
  if (d->IsUDIFile || d->IsFDIFile || d->IsMBDFile || d->IsTD0File || d->IsProFile)
    return false;
#endif
  return true;
}

void rvmWD1793UpdateFastmode(rvmWD1793 *wd) {
  wd->fastmode = Config::trdosFastMode && diskFastCapable(wd->disk[wd->diskS]);
}

void rvmWD1793Reset(rvmWD1793 *wd) {

  wd->state = kRVMWD177XNone;
  wd->stepState = kRVMWD177XStepIdle;
  wd->next = kRVMWD177XNone;
  wd->typeI_busy_oneshot = false;
  wd->profi_busy_hold = false;

  wd->c = 0;
  wd->control &= 0xf000;
  wd->command = wd->dsr = 0x0;
  // Note: sector and data registers are NOT cleared by WD1793 hardware reset (MR pin)
  wd->status = kRVMWD177XStatusSetIndex | kRVMWD177XStatusSetTrack0 | kRVMWD177XStatusSetWP;
  wd->track = 0xff;
  wd->fdd_clicks = 0;
  wd->wtrackmark = 0;
  wd->headerI = 0;
  wd->retry = 0;
  // wd->crc=0xffff;
  wd->crc = 0; // Disable CRC. Not needed for Betadisk emulation
  wd->side = wd->diskS = 0;
  // wd2797_mode is set externally (by MB02::init), preserve across reset.
  // Fastmode follows the active drive only (reset selects drive 0 above).
  rvmWD1793UpdateFastmode(wd);
  wd->sclConverted = false;
#if !PICO_RP2040
  // Flush modified UDI/FDI track to SD before resetting (avoid data loss)
  if (wd->diskDirty && wd->diskLoadedCyl >= 0) {
    int lu = (wd->diskLoadedUnit >= 0) ? wd->diskLoadedUnit : wd->diskS;
    rvmwdDisk *disk = wd->disk[lu];
    if (disk && disk->Diskfile) {
      if (disk->IsUDIFile) {
        int trkIdx = wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide;
        if (trkIdx >= 0 && trkIdx < 168) {
          UINT bw;
          f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
          f_write(disk->Diskfile, wd->diskTrackBuf, disk->udiTrackLengths[trkIdx], &bw);
        }
      } else if (disk->IsFDIFile) {
        fdiFlushTrack(wd);
      } else if (disk->IsMBDFile) {
        mbdFlushTrack(wd);
      }
    }
  }
  wd->diskLoadedCyl = -1;
  wd->diskLoadedSide = -1;
  wd->diskLoadedUnit = -1;
  wd->diskTrackLen = 0;
  wd->diskDirty = false;
  wd->fdiTstates = 0;
  wd->fdiSectorCount = 0;
  wd->fdiDataCrcError = false;
#endif
}

bool rvmWD1793AllocTrackBuf(rvmWD1793 *wd) {
#if !PICO_RP2040
    if (wd->diskTrackBuf) return true;
    wd->diskTrackBuf = (uint8_t *)malloc(DISK_TRACK_BUF_SZ);
    return wd->diskTrackBuf != nullptr;
#else
    return true;
#endif
}

void rvmWD1793FreeTrackBuf(rvmWD1793 *wd) {
#if !PICO_RP2040
    if (wd->diskTrackBuf) { free(wd->diskTrackBuf); wd->diskTrackBuf = nullptr; }
    wd->diskLoadedCyl = -1;
    wd->diskLoadedSide = -1;
    wd->diskLoadedUnit = -1;
    wd->diskTrackLen = 0;
    wd->diskDirty = false;
#endif
}

#if !PICO_RP2040
// TR-DOS autostarts a BASIC file named "boot" on cold start. Many TRD images
// (especially ones pulled off online archives) ship without one, so they drop to
// the TR-DOS prompt instead of running. If enabled (Config::trdosAutoBoot), and the
// freshly-mounted TRD is a valid TR-DOS disk with no "boot" file, write one in:
//  - the 16-byte catalog entry goes into the first free dir slot (index = file count),
//  - the single data sector goes into track 0's reserved tail (sector 9, offset 2304),
//    which TR-DOS never allocates for normal files, so existing data in tracks 1+ is
//    left completely untouched. First-free pointer / free-sector count are left as-is.
// Runs once: a later mount finds the boot present and skips. cursectbuf (256 B, heap)
// is used as scratch to avoid blowing the 2 KB core stack.
static void trdMaybeInjectBoot(rvmwdDisk *disk) {
    if (!Config::trdosAutoBoot || !disk || !disk->Diskfile) return;

    UINT br, bw;
    uint8_t *buf = disk->cursectbuf; // 256-byte heap scratch

    // Disk-spec sector (logical sector 9 / track 0, file offset 2048).
    if (f_lseek(disk->Diskfile, 2048) != FR_OK) return;
    if (f_read(disk->Diskfile, buf, 256, &br) != FR_OK || br < 256) return;
    if (buf[0xE7] != 0x10) return;          // not a TR-DOS disk
    uint8_t fileCount = buf[0xE4];
    if (fileCount >= 128) return;           // catalog full

    // Scan the active catalog entries (dir sectors 0-7) for an existing "boot    B",
    // and verify the boot's target — logical sector 9 of track 0 (linear sector 9) —
    // is not occupied by a file. The reserved-tail assumption only holds for the
    // standard layout (first-free pointer = track 1 / sector 0, whole of track 0
    // system-reserved). Some archived TRDs are formatted with track 0's tail
    // (logical sectors 9-15) reclaimed for data, so a file starts at linear sector 9;
    // injecting there would silently clobber that file's data. Skip injection on
    // those disks (drop to the TR-DOS prompt, same as without the feature) rather
    // than corrupt them. A file occupies linear sectors [start, start+seccnt).
    int loadedSec = -1;
    for (int i = 0; i < fileCount; i++) {
        int sec = i >> 4;
        if (sec != loadedSec) {
            if (f_lseek(disk->Diskfile, sec * 256) != FR_OK) return;
            if (f_read(disk->Diskfile, buf, 256, &br) != FR_OK || br < 256) return;
            loadedSec = sec;
        }
        const uint8_t *ent = buf + ((i & 0x0F) << 4);
        if (memcmp(ent, "boot    ", 8) == 0 && ent[8] == 'B') return; // already present
        if (ent[0] == 0x00) continue;                  // empty terminator entry
        int fileStart = ent[15] * 16 + ent[14];        // linear sector (track*16 + sector)
        int fileEnd   = fileStart + ent[13];           // ent[13] = length in sectors
        if (fileStart <= 9 && 9 < fileEnd) {           // covers track 0's tail sector 9
            Debug::log("TRD: skip boot inject — track 0 tail in use (file @lin%d len%d)",
                       fileStart, (int)ent[13]);
            return;
        }
    }

    // No boot file — append the catalog entry at slot = fileCount.
    int slotSec = fileCount >> 4;
    int slotOff = (fileCount & 0x0F) << 4;
    if (f_lseek(disk->Diskfile, slotSec * 256) != FR_OK) return;
    if (f_read(disk->Diskfile, buf, 256, &br) != FR_OK || br < 256) return;
    uint8_t *ent = buf + slotOff;
    memcpy(ent, "boot    ", 8);
    ent[8]  = 'B';
    ent[9]  = TRDOS_BOOT_START  & 0xFF; ent[10] = (TRDOS_BOOT_START  >> 8) & 0xFF;
    ent[11] = TRDOS_BOOT_LENGTH & 0xFF; ent[12] = (TRDOS_BOOT_LENGTH >> 8) & 0xFF;
    ent[13] = TRDOS_BOOT_SECCNT;
    ent[14] = 9; // start sector (track 0's reserved tail)
    ent[15] = 0; // start track
    if (f_lseek(disk->Diskfile, slotSec * 256) != FR_OK) return;
    if (f_write(disk->Diskfile, buf, 256, &bw) != FR_OK || bw < 256) return;

    // Bump the file count in the disk spec (re-read: slotSec may have been sector 8).
    if (f_lseek(disk->Diskfile, 2048) != FR_OK) return;
    if (f_read(disk->Diskfile, buf, 256, &br) != FR_OK || br < 256) return;
    buf[0xE4] = fileCount + 1;
    if (f_lseek(disk->Diskfile, 2048) != FR_OK) return;
    if (f_write(disk->Diskfile, buf, 256, &bw) != FR_OK || bw < 256) return;

    // Boot data: single sector into track 0 / sector 9 (file offset 2304).
    if (f_lseek(disk->Diskfile, 2304) != FR_OK) return;
    f_write(disk->Diskfile, kTrdosBootSector, 256, &bw);
    f_sync(disk->Diskfile);
    disk->cursectbufpos = 0xff; // scratch reused — invalidate the sector cache
    Debug::log("TRD: injected boot file (had %d files)", (int)fileCount);
}
#endif

bool rvmWD1793InsertDisk(rvmWD1793 *wd, unsigned char UnitNum, const std::string& Filename) {

#if !PICO_RP2040
    // Ensure the track buffer exists before a disk can be loaded into it.
    // (The MB-02 drive's buffer is released when MB-02 is disabled.)
    if (!rvmWD1793AllocTrackBuf(wd)) { Debug::led_blink(); return false; }
#endif

    // Close any open disk in this unit
    wdDiskEject(wd,UnitNum);

    // wd->disk[UnitNum] = (rvmwdDisk *) heap_caps_calloc(1, sizeof(rvmwdDisk), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    wd->disk[UnitNum] = (rvmwdDisk *) heap_caps_calloc(1, sizeof(rvmwdDisk), MALLOC_CAP_8BIT);

    //wd->disk[UnitNum]->Diskfile = fopen(Filename.c_str(), "r+b");
    wd->disk[UnitNum]->Diskfile = fopen2(Filename.c_str(), FA_READ | FA_WRITE);
    if (wd->disk[UnitNum]->Diskfile == NULL) {
      Debug::led_blink();
      wdDiskEject(wd,UnitNum);
      return false;
    }

    uint8_t diskType;

    char magic[8];
    UINT br;
    //fread(&magic, 1, 8, wd->disk[UnitNum]->Diskfile);
    FRESULT res = f_read(wd->disk[UnitNum]->Diskfile, &magic, 8, &br);

    if (std::strncmp(magic,"SINCLAIR",8) == 0) {
        // SCL file
        printf("SCL disk loaded\n");
        wd->disk[UnitNum]->IsSCLFile=true;
#if !PICO_RP2040
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = false;
        wd->disk[UnitNum]->IsProFile = false;
#endif
        wd->disk[UnitNum]->fname = Filename;
        // writeprotect is seeded by the caller from the per-slot Config array.
        wd->disk[UnitNum]->writeprotect = 0;
        rvmWD1793UpdateFastmode(wd);
        diskType = 0x16;

#if !PICO_RP2040
    } else if (std::strncmp(magic,"UDI!",4) == 0) {
        // UDI file
        printf("UDI disk loaded\n");
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = true;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = false;
        wd->disk[UnitNum]->IsProFile = false;
        // writeprotect is seeded by the caller from the per-slot Config array.
        wd->disk[UnitNum]->writeprotect = 0;
        wd->disk[UnitNum]->sclDataOffset = 0;

        // Read UDI header
        uint8_t hdr[16];
        f_lseek(wd->disk[UnitNum]->Diskfile, 0);
        f_read(wd->disk[UnitNum]->Diskfile, hdr, 16, &br);

        uint8_t cyls = hdr[9] + 1;
        uint8_t sides = hdr[10] + 1;
        uint32_t extHdrLen = hdr[12] | (hdr[13] << 8) | (hdr[14] << 16) | (hdr[15] << 24);

        wd->disk[UnitNum]->tracks = cyls - 1; // tracks field stores max track index
        wd->disk[UnitNum]->sides = sides;

        // Parse track offsets and lengths
        uint32_t offset = 16 + extHdrLen;
        int totalTracks = cyls * sides;
        if (totalTracks > 168) totalTracks = 168;

        for (int i = 0; i < totalTracks; i++) {
            uint8_t trkHdr[3];
            f_lseek(wd->disk[UnitNum]->Diskfile, offset);
            f_read(wd->disk[UnitNum]->Diskfile, trkHdr, 3, &br);

            uint16_t tlen = trkHdr[1] | (trkHdr[2] << 8);
            uint16_t clen = (tlen + 7) / 8;

            wd->disk[UnitNum]->udiTrackOffsets[i] = offset + 3; // data starts after 3-byte track header
            wd->disk[UnitNum]->udiTrackLengths[i] = tlen;

            offset += 3 + tlen + clen;
        }

        // Skip the normal diskType switch — we already set tracks/sides
        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        rvmWD1793UpdateFastmode(wd); // fastmode uses sectdatapos, incompatible with raw MFM

        printf("UDI: %d cylinders, %d sides\n", cyls, sides);
        return true;

    } else if (std::strncmp(magic,"FDI",3) == 0) {
        // FDI file — generate MFM track images on demand (ZXMAK2 approach)
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = true;
        wd->disk[UnitNum]->IsMBDFile = false;
        wd->disk[UnitNum]->IsProFile = false;
        wd->disk[UnitNum]->sclDataOffset = 0;

        // Read FDI header (14 bytes)
        uint8_t hdr[14];
        f_lseek(wd->disk[UnitNum]->Diskfile, 0);
        f_read(wd->disk[UnitNum]->Diskfile, hdr, 14, &br);

        uint8_t wpFlag = hdr[3];
        uint16_t cyls = hdr[4] | (hdr[5] << 8);
        uint16_t sides = hdr[6] | (hdr[7] << 8);
        uint16_t dataOffset = hdr[10] | (hdr[11] << 8);
        uint16_t extraHdrLen = hdr[12] | (hdr[13] << 8);

        wd->disk[UnitNum]->tracks = cyls - 1;
        wd->disk[UnitNum]->sides = sides;
        // Honor FDI hardware WP flag; otherwise defer to the caller's per-slot seed.
        wd->disk[UnitNum]->writeprotect = wpFlag ? 1 : 0;
        wd->disk[UnitNum]->fdiDataOffset = dataOffset;

        // Parse track headers to record their file positions
        uint32_t trkHdrPos = 14 + extraHdrLen;
        int totalTracks = cyls * sides;
        if (totalTracks > 168) totalTracks = 168;

        for (int i = 0; i < totalTracks; i++) {
            wd->disk[UnitNum]->fdiTrackHdrOffsets[i] = trkHdrPos;
            uint8_t trkHdr[7];
            f_lseek(wd->disk[UnitNum]->Diskfile, trkHdrPos);
            f_read(wd->disk[UnitNum]->Diskfile, trkHdr, 7, &br);
            uint8_t sectorCount = trkHdr[6];
            trkHdrPos += 7 + sectorCount * 7;
        }

        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        rvmWD1793UpdateFastmode(wd);

        printf("FDI: %d cylinders, %d sides\n", cyls, sides);
        return true;

    } else if ((magic[0] == 'T' && magic[1] == 'D') ||
               (magic[0] == 't' && magic[1] == 'd')) {
        // TD0 (Teledisk) — streamed track-by-track from SD so the whole
        // (potentially ~1 MB) image never has to live in RAM. Packed images
        // ("td") are LZH-decompressed once into a temp file on the card;
        // unpacked images ("TD") stream directly from the original file.
        // Per-track *file* offsets into the (decompressed) stream are stored in
        // fdiTrackHdrOffsets[]. Read-only. See td0.{h,cpp}.
        rvmwdDisk *d0 = wd->disk[UnitNum];
        d0->IsSCLFile = false;
        d0->IsUDIFile = false;
        d0->IsFDIFile = false;
        d0->IsMBDFile = false;
        d0->IsProFile = false;
        d0->IsTD0File = true;
        d0->sclDataOffset = 0;
        d0->td0Stream = nullptr;
        d0->td0OwnsStream = false;

        bool packed = (magic[0] == 't');

        // Read the 12-byte image header to learn the comment-record flag.
        uint8_t imgHdr[12];
        f_lseek(d0->Diskfile, 0);
        f_read(d0->Diskfile, imgHdr, 12, &br);
        bool hasComment = (imgHdr[7] & 0x80) != 0;

        FSIZE_t fsize = f_size(d0->Diskfile);
        if (fsize <= 12) { wdDiskEject(wd, UnitNum); Debug::led_blink(); return false; }

        // The decompressed stream lives on the SD card. For packed images we
        // write it to a per-unit temp file; for unpacked images we read it
        // straight out of Diskfile (after the 12-byte header). td0Base is the
        // file offset where the post-header stream begins in td0Stream.
        FIL *strm;
        uint32_t td0Base;
        if (packed) {
            char tmpPath[24];
            snprintf(tmpPath, sizeof(tmpPath), "/tmp/.td0_u%u.tmp", (unsigned)UnitNum);
            FIL *tf = fopen2(tmpPath, FA_WRITE | FA_CREATE_ALWAYS);
            if (!tf) { wdDiskEject(wd, UnitNum); Debug::led_blink(); return false; }

            // Stream-decompress from the TD0 file directly into a temp file.
            // td0_unpack_lzh_from_file reads the packed input via a 512-byte
            // staging window (inside the static LzhState) — no malloc(rawLen).
            f_lseek(d0->Diskfile, 12);
            struct Td0Sink { FIL *f; bool failed; uint32_t written; } sinkCtx = { tf, false, 0 };
            uint32_t decLen = td0_unpack_lzh_from_file(d0->Diskfile,
                [](void *ctx, const unsigned char *buf, unsigned len) -> bool {
                    Td0Sink *s = (Td0Sink *)ctx;
                    UINT bw = 0;
                    FRESULT fr = f_write(s->f, buf, len, &bw);
                    if (fr != FR_OK || bw != len) { s->failed = true; return false; }
                    s->written += bw;
                    return true;
                }, &sinkCtx);
            f_sync(tf);
            uint32_t tfSize = (uint32_t)f_size(tf);

#if FDD_PORT_TRACE
            Debug::log("[TD0] unpack u%d: decLen=%u written=%u tempSize=%u %s",
                       (int)UnitNum, (unsigned)decLen,
                       (unsigned)sinkCtx.written, (unsigned)tfSize,
                       sinkCtx.failed ? "WRITE-FAILED!" : "ok");
#endif

            if (decLen == 0 || sinkCtx.failed || tfSize == 0) {
                fclose2(tf); f_unlink(tmpPath);
                wdDiskEject(wd, UnitNum); Debug::led_blink(); return false;
            }
            // Reopen read-only for streaming track reads.
            fclose2(tf);
            tf = fopen2(tmpPath, FA_READ);
            if (!tf) { f_unlink(tmpPath); wdDiskEject(wd, UnitNum); Debug::led_blink(); return false; }

            d0->td0Stream = tf;
            d0->td0OwnsStream = true;
            d0->td0TempPath = tmpPath;
            strm = tf;
            td0Base = 0;
        } else {
            d0->td0Stream = d0->Diskfile;
            d0->td0OwnsStream = false;
            strm = d0->Diskfile;
            td0Base = 12; // skip the 12-byte image header
        }

        uint32_t imgLen = (uint32_t)f_size(strm); // size of the stream file
#if FDD_PORT_TRACE
        Debug::log("[TD0] insert u%d %s base=%u streamSize=%u packed=%d comment=%d",
                   (int)UnitNum, packed ? "(packed->temp)" : "(in-place)",
                   (unsigned)td0Base, (unsigned)imgLen, (int)packed, (int)hasComment);
#endif

        // Sequential scan of the stream from the card. Track-record byte
        // positions are relative to td0Base; we record absolute file offsets.
        uint32_t p = td0Base;

        // Skip optional comment record: CRC(2) + len(2) + Y/M/D/H/M/S(6) + text.
        if (hasComment) {
            uint8_t ch[4];
            if (p + 4 > imgLen) { wdDiskEject(wd, UnitNum); Debug::led_blink(); return false; }
            f_lseek(strm, p); f_read(strm, ch, 4, &br);
            uint16_t clen = ch[2] | (ch[3] << 8);
            p += 10 + clen;
        }

        // Walk track records to record offsets, geometry, and the largest
        // track-record byte span (used to size the per-track scratch buffer).
        uint32_t maxCyl = 0, maxHead = 0;
        for (int i = 0; i < 168; i++) d0->fdiTrackHdrOffsets[i] = 0xFFFFFFFF;

        int trkScanned = 0;
        while (p < imgLen) {
            uint8_t thdr[4];
            br = 0;
            f_lseek(strm, p); f_read(strm, thdr, 4, &br);
            if (br < 4) {                     // short read / EOF — stop, don't spin
#if FDD_PORT_TRACE
                Debug::log("[TD0] scan stop: short hdr read p=%u br=%u", (unsigned)p, (unsigned)br);
#endif
                break;
            }
            uint8_t secs = thdr[0];
            if (secs == 0xFF) break;          // end-of-image marker
            uint8_t cyl  = thdr[1];
            uint8_t side = thdr[2];
            if (cyl > maxCyl)  maxCyl = cyl;
            if (side > maxHead) maxHead = side;

            uint32_t recPos = p;
            p += 4;                            // track header: secs, cyl, side, crc
            for (uint8_t s = 0; s < secs && p + 6 <= imgLen; s++) {
                uint8_t shdr[6];
                f_lseek(strm, p); f_read(strm, shdr, 6, &br);
                uint8_t flags = shdr[4];
                p += 6;                        // sector header
                if (flags & (TD0_SEC_NO_DATA | TD0_SEC_NO_DATA2))
                    continue;                  // no data block follows
                if (p + 2 > imgLen) break;
                uint8_t dl[2];
                f_lseek(strm, p); f_read(strm, dl, 2, &br);
                uint16_t dlen = dl[0] | (dl[1] << 8);
                p += 2 + dlen;                 // length field + encoded block
            }
            uint32_t recLen = p - recPos;

            // Temporary keyed store (cyl*2+side); remapped after we know sides.
            int tmpIdx = cyl * 2 + side;
            if (tmpIdx >= 0 && tmpIdx < 168)
                d0->fdiTrackHdrOffsets[tmpIdx] = recPos;

            // Guard against a malformed record that fails to advance p, which
            // would otherwise spin this loop forever (a likely hang source).
            if (recLen < 4) {
#if FDD_PORT_TRACE
                Debug::log("[TD0] scan stop: no progress p=%u secs=%d", (unsigned)recPos, (int)secs);
#endif
                break;
            }
            if (++trkScanned > 168) {
#if FDD_PORT_TRACE
                Debug::log("[TD0] scan stop: >168 tracks, aborting at p=%u", (unsigned)p);
#endif
                break;
            }
        }
#if FDD_PORT_TRACE
        Debug::log("[TD0] scan done: %d tracks, lastP=%u",
                   trkScanned, (unsigned)p);
#endif

        // Allocate the per-track scratch buffer sized to the largest track
        // record (typically a few KB — far below the old whole-image buffer).
        if (trkScanned == 0) { wdDiskEject(wd, UnitNum); Debug::led_blink(); return false; }

        uint8_t cyls  = (uint8_t)(maxCyl + 1);
        uint8_t sides = (uint8_t)(maxHead + 1);
        d0->tracks = cyls - 1;
        d0->sides = sides;

        // Remap offsets from the temporary (cyl*2+side) keying to the canonical
        // (cyl*sides+side) keying used by the track loaders.
        if (sides == 1) {
            // collapse: index cyl*2+0 -> cyl
            for (int c = 0; c < cyls && c < 168; c++)
                d0->fdiTrackHdrOffsets[c] =
                    (c * 2 < 168) ? d0->fdiTrackHdrOffsets[c * 2] : 0xFFFFFFFF;
            for (int c = cyls; c < 168; c++)
                d0->fdiTrackHdrOffsets[c] = 0xFFFFFFFF;
        }
        // sides==2: cyl*2+side already equals cyl*sides+side, no remap needed.

        d0->writeprotect = 1; // TD0 is read-only
        d0->t0s1_info = 0;
        d0->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        d0->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        rvmWD1793UpdateFastmode(wd);

        printf("TD0%s: %d cylinders, %d sides, %u stream bytes (%s)\n",
               packed ? " (packed)" : "", cyls, sides, (unsigned)imgLen,
               packed ? d0->td0TempPath.c_str() : "in-place");
        return true;

    } else if (Filename.length() >= 4 &&
               (Filename.substr(Filename.length() - 4) == ".mbd" ||
                Filename.substr(Filename.length() - 4) == ".MBD" ||
                Filename.substr(Filename.length() - 4) == ".Mbd")) {
        // MBD file — raw sector dump (MB-02+ BS-DOS format)
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = true;
        wd->disk[UnitNum]->IsProFile = false;
        wd->disk[UnitNum]->sclDataOffset = 0;

        // Read MBD header to determine geometry
        uint8_t hdr[16];
        f_lseek(wd->disk[UnitNum]->Diskfile, 0);
        f_read(wd->disk[UnitNum]->Diskfile, hdr, 16, &br);

        uint8_t tracks = hdr[4]; // typically 82
        uint8_t spt = hdr[6];    // sectors per track (typically 11)
        uint8_t sides = hdr[8];  // typically 2

        if (tracks == 0 || spt == 0 || sides == 0) {
            // Fallback: assume standard HD format from file size
            FSIZE_t fsize = f_size(wd->disk[UnitNum]->Diskfile);
            tracks = 82; sides = 2; spt = 11;
            uint16_t secSize = (uint16_t)(fsize / (tracks * sides * spt));
            if (secSize != 256 && secSize != 512 && secSize != 1024) secSize = 1024;
            wd->disk[UnitNum]->mbdSectorSize = secSize;
        } else {
            // Derive sector size from file size
            FSIZE_t fsize = f_size(wd->disk[UnitNum]->Diskfile);
            uint16_t secSize = (uint16_t)(fsize / (tracks * sides * spt));
            if (secSize != 256 && secSize != 512 && secSize != 1024) secSize = 1024;
            wd->disk[UnitNum]->mbdSectorSize = secSize;
        }

        wd->disk[UnitNum]->tracks = tracks - 1;
        wd->disk[UnitNum]->sides = sides;
        wd->disk[UnitNum]->mbdSectorsPerTrack = spt;
        wd->disk[UnitNum]->writeprotect = 0; // MBD: BS-DOS requires writable disk

        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        rvmWD1793UpdateFastmode(wd);

        printf("MBD: %d tracks, %d sides, %d sec/trk, %d bytes/sec\n",
               tracks, sides, spt, wd->disk[UnitNum]->mbdSectorSize);
        return true;

    } else if (Filename.length() >= 4 &&
               (Filename.substr(Filename.length() - 4) == ".pro" ||
                Filename.substr(Filename.length() - 4) == ".PRO" ||
                Filename.substr(Filename.length() - 4) == ".Pro")) {
        // PRO file — Profi CP/M raw disk image, no header.
        // Layout per UnrealSpeccy wldr_pro.cpp: 80 cyl × 2 sides × 5 sec × 1024 bytes = 800 KB.
        // First track (cyl 0 side 0) uses special sector IDs {1,2,3,4,9}; other
        // tracks use {1,2,3,4,5}. Reuses MBD reader; PRO-specific ID array applied
        // inside mbdLoadTrack when IsProFile is set.
        wd->disk[UnitNum]->IsSCLFile = false;
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = true;
        wd->disk[UnitNum]->IsProFile = true;
        wd->disk[UnitNum]->sclDataOffset = 0;

        FSIZE_t fsize = f_size(wd->disk[UnitNum]->Diskfile);
        uint8_t tracks, sides, spt;
        uint16_t secSize;
        if (fsize == 819200) {            // 800 KB standard Profi CP/M layout
            tracks = 80; sides = 2; spt = 5; secSize = 1024;
        } else if (fsize == 409600) {     // 400 KB single-sided
            tracks = 80; sides = 1; spt = 5; secSize = 1024;
        } else {
            // Fallback: assume standard 800K geometry
            tracks = 80; sides = 2; spt = 5; secSize = 1024;
        }

        wd->disk[UnitNum]->tracks = tracks - 1;
        wd->disk[UnitNum]->sides = sides;
        wd->disk[UnitNum]->mbdSectorsPerTrack = spt;
        wd->disk[UnitNum]->mbdSectorSize = secSize;
        wd->disk[UnitNum]->writeprotect = 0;

        wd->disk[UnitNum]->t0s1_info = 0;
        wd->disk[UnitNum]->cursectbufpos = 0xff;
        wd->control |= kRVMWD177XPower0 << UnitNum;
        wd->disk[UnitNum]->fname = Filename;
        wd->diskLoadedCyl = -1;
        wd->diskLoadedSide = -1;
        rvmWD1793UpdateFastmode(wd);

        printf("PRO (Profi CP/M): %d tracks, %d sides, %d sec/trk, %d bytes/sec\n",
               tracks, sides, spt, secSize);
        return true;
#endif

    } else {
        wd->disk[UnitNum]->IsSCLFile = false;
#if !PICO_RP2040
        wd->disk[UnitNum]->IsUDIFile = false;
        wd->disk[UnitNum]->IsFDIFile = false;
        wd->disk[UnitNum]->IsMBDFile = false;
        wd->disk[UnitNum]->IsProFile = false;
#endif
        // writeprotect is seeded by the caller from the per-slot Config array.
        wd->disk[UnitNum]->writeprotect = 0;
        wd->disk[UnitNum]->sclDataOffset = 0;
        rvmWD1793UpdateFastmode(wd);

        // fseek(wd->disk[UnitNum]->Diskfile,2048 + 227,SEEK_SET);
        // fread(&diskType,1,1,wd->disk[UnitNum]->Diskfile);
        f_lseek(wd->disk[UnitNum]->Diskfile,2048 + 227);
        f_read(wd->disk[UnitNum]->Diskfile, &diskType, 1, &br);
    }

    switch(diskType) {
        case 0x16:
            wd->disk[UnitNum]->tracks = 79;
            wd->disk[UnitNum]->sides = 2;
            break;
        case 0x17:
            wd->disk[UnitNum]->tracks = 39;
            wd->disk[UnitNum]->sides = 2;
            break;
        case 0x18:
            wd->disk[UnitNum]->tracks = 79;
            wd->disk[UnitNum]->sides = 1;
            break;
        case 0x19:
            wd->disk[UnitNum]->tracks = 39;
            wd->disk[UnitNum]->sides = 1;
            break;
        default:
            wdDiskEject(wd,UnitNum);
            Debug::led_blink();
            return false;
    }

    // Check if we have more tracks than on a standard disk
    if (!wd->disk[UnitNum]->IsSCLFile) {
      // Get file size
      f_lseek(wd->disk[UnitNum]->Diskfile, 0);
      long diskbytes = f_size(wd->disk[UnitNum]->Diskfile);
      if( diskbytes > wd->disk[UnitNum]->sides * wd->disk[UnitNum]->tracks * 16 * 256 ) {
        int i;
        for( int i = wd->disk[UnitNum]->tracks + 1; i < 83; i++ ) {
          if( wd->disk[UnitNum]->sides * i * 16 * 256 >= diskbytes ) {
            wd->disk[UnitNum]->tracks = i;
            break;
          }
        }
      }
    }

    //rewind(wd->disk[UnitNum]->Diskfile);
    f_rewind(wd->disk[UnitNum]->Diskfile);

#if !PICO_RP2040
    // Plain TRD only (SCL/UDI/FDI/MBD/PRO/TD0 return earlier or are handled in RAM):
    // inject a "boot" file if the image lacks one, so TR-DOS autostarts.
    if (!wd->disk[UnitNum]->IsSCLFile && !wd->disk[UnitNum]->IsUDIFile &&
        !wd->disk[UnitNum]->IsFDIFile && !wd->disk[UnitNum]->IsMBDFile &&
        !wd->disk[UnitNum]->IsProFile && !wd->disk[UnitNum]->IsTD0File) {
        trdMaybeInjectBoot(wd->disk[UnitNum]);
    }
#endif

    wd->disk[UnitNum]->t0s1_info = 0;
    wd->disk[UnitNum]->cursectbufpos = 0xff; // 0xffff;

    // Power on drive
    wd->control |= kRVMWD177XPower0 << UnitNum;

    printf("Disk %d inserted! Disktype: %d\n",UnitNum, (int) diskType);

    wd->disk[UnitNum]->fname = Filename;

    return true;

}

#if !PICO_RP2040
// g_rawTrkDataBuf (raw-format track staging) is defined near the top of the file
// so the SCL track-0 cache can alias it — see s_scl_track0.

static void udiFlushTrack(rvmWD1793 *wd) {
    // Flush targets the unit whose track is in the buffer, not the currently
    // selected drive — they differ when a drive switch triggers the flush.
    int u = (wd->diskLoadedUnit >= 0) ? wd->diskLoadedUnit : wd->diskS;
    rvmwdDisk *disk = wd->disk[u];
    if (!disk || wd->diskLoadedCyl < 0) return;
    int trkIdx = wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide;
    if (trkIdx < 0 || trkIdx >= 168) return;
    uint16_t tlen = disk->udiTrackLengths[trkIdx];
    UINT bw;
    f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
    f_write(disk->Diskfile, wd->diskTrackBuf, tlen, &bw);
    wd->diskDirty = false;
    // CRC32 at end of file not updated — our parser doesn't validate it
}

void udiLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide
        && wd->diskS == wd->diskLoadedUnit)
        return;

    // Flush modified track back to file before switching tracks
    if (wd->diskDirty)
        udiFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168) {
        wd->diskTrackLen = 0;
        return;
    }

    uint16_t tlen = disk->udiTrackLengths[trkIdx];
    if (tlen > DISK_TRACK_BUF_SZ)
        tlen = DISK_TRACK_BUF_SZ;

    UINT br;
    f_lseek(disk->Diskfile, disk->udiTrackOffsets[trkIdx]);
    f_read(disk->Diskfile, wd->diskTrackBuf, tlen, &br);

    wd->diskTrackLen = tlen;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
    wd->diskLoadedUnit = wd->diskS;
}

// VG93/WD1793 CRC-CCITT (same algorithm as ZXMAK2 CrcVg93)
static uint16_t vgCrc(uint16_t crc, uint8_t byte) {
    crc ^= byte << 8;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    return crc;
}

// Flush modified FDI track buffer back to FDI file.
// Parses MFM buffer to find sector data and writes it back at original FDI file offsets.
static void fdiFlushTrack(rvmWD1793 *wd) {
    int u = (wd->diskLoadedUnit >= 0) ? wd->diskLoadedUnit : wd->diskS;
    rvmwdDisk *disk = wd->disk[u];
    if (!disk || !disk->IsFDIFile || wd->diskLoadedCyl < 0) { wd->diskDirty = false; return; }

    int trkIdx = wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide;
    if (trkIdx < 0 || trkIdx >= 168) { wd->diskDirty = false; return; }

    // Re-read FDI track header + sector descriptors
    uint8_t trkHdr[7];
    UINT br;
    f_lseek(disk->Diskfile, disk->fdiTrackHdrOffsets[trkIdx]);
    f_read(disk->Diskfile, trkHdr, 7, &br);

    uint32_t trkDataOffset = trkHdr[0] | (trkHdr[1]<<8) | (trkHdr[2]<<16) | (trkHdr[3]<<24);
    int secCount = trkHdr[6];
    if (secCount > 32) secCount = 32;
    if (secCount > wd->fdiSectorCount) secCount = wd->fdiSectorCount;

    uint8_t secHdrs[32 * 7];
    if (secCount > 0)
        f_read(disk->Diskfile, secHdrs, secCount * 7, &br);

    uint8_t *buf = wd->diskTrackBuf;
    int bufLen = wd->diskTrackLen;

    for (int sec = 0; sec < secCount; sec++) {
        uint8_t *sh = &secHdrs[sec * 7];
        uint8_t secN = sh[3];
        uint8_t flags = sh[4];
        uint16_t secDataOff = sh[5] | (sh[6] << 8);

        if (flags & 0x40) continue; // no data area

        int slen = 128 << (secN & 3);

        // Find data mark (0xFB/0xF8) after ID field in MFM buffer
        int idPos = (sec < 32) ? wd->fdiSectorIdPos[sec] : -1;
        if (idPos < 0) continue;
        int searchStart = idPos + 7; // skip FE + CHRN + 2 CRC bytes
        int dataPos = -1;
        for (int i = searchStart; i < searchStart + 80 && i < bufLen; i++) {
            if (buf[i] == 0xFB || buf[i] == 0xF8) {
                dataPos = i + 1;
                break;
            }
        }
        if (dataPos < 0 || dataPos + slen > bufLen) continue;

        // Write sector data back to FDI file
        uint32_t filePos = disk->fdiDataOffset + trkDataOffset + secDataOff;
        UINT bw;
        f_lseek(disk->Diskfile, filePos);
        f_write(disk->Diskfile, buf + dataPos, slen, &bw);

        // Update CRC flag only for sectors that were actually written
        if (!(flags & (1 << (secN & 3))) && sec < 32 && !(wd->fdiSectorFlags[sec] & 1)) {
            uint8_t newFlags = (1 << (secN & 3));
            if (flags & 0x80) newFlags |= 0x80;
            uint32_t flagsPos = disk->fdiTrackHdrOffsets[trkIdx] + 7 + sec * 7 + 4;
            f_lseek(disk->Diskfile, flagsPos);
            f_write(disk->Diskfile, &newFlags, 1, &bw);
        }
    }

    wd->diskDirty = false;
}

// Generate MFM track image from FDI sector data (ZXMAK2 approach).
// Called on demand when cylinder/side changes, same as udiLoadTrack.
void fdiLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide
        && wd->diskS == wd->diskLoadedUnit)
        return;

    // Flush any pending writes before loading new track
    if (wd->diskDirty)
        fdiFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168) {
        wd->diskTrackLen = 0;
        return;
    }

    // Read FDI track header (4 bytes data offset + 2 reserved + 1 sector count)
    uint8_t trkHdr[7];
    UINT br;
    f_lseek(disk->Diskfile, disk->fdiTrackHdrOffsets[trkIdx]);
    f_read(disk->Diskfile, trkHdr, 7, &br);

    uint32_t trkDataOffset = trkHdr[0] | (trkHdr[1] << 8) | (trkHdr[2] << 16) | (trkHdr[3] << 24);
    int secCount = trkHdr[6];
    if (secCount > 32) secCount = 32;

    // Read sector headers (7 bytes each: C H R N Flags DataOffLo DataOffHi)
    uint8_t secHdrs[32 * 7];
    if (secCount > 0)
        f_read(disk->Diskfile, secHdrs, secCount * 7, &br);

    // Calculate total data length to determine gap sizes (ZXMAK2 algorithm).
    // Same pass also finds the [minOff, maxEnd) extent of all sector data within
    // the track-data block, so the whole block can be read in ONE SD transaction
    // (vs one f_lseek+f_read per sector — ~16 SPI transactions at ~1.4ms each).
    int imageSize = 6250;
    int trkdatalen = 0;
    uint32_t dataMinOff = 0xFFFFFFFF, dataMaxEnd = 0;
    for (int s = 0; s < secCount; s++) {
        uint8_t *sh = &secHdrs[s * 7];
        uint8_t flags = sh[4];
        int slen = 128 << (sh[3] & 3);
        trkdatalen += 2 + 6;  // A1 + FE + 6 bytes (CHRN + CRC)
        if (!(flags & 0x40)) { // has data area
            trkdatalen += 4;   // A1 + FB + 2 bytes CRC
            trkdatalen += slen;
            uint32_t off = sh[5] | (sh[6] << 8);
            if (off < dataMinOff) dataMinOff = off;
            if (off + (uint32_t)slen > dataMaxEnd) dataMaxEnd = off + (uint32_t)slen;
        } else {
            slen = 0;
        }
    }

    // Bulk-read the whole track-data block once into a scratch buffer. Sectors
    // then memcpy from RAM instead of hitting the SD card individually. Falls back
    // to per-sector f_read if the span exceeds the scratch buffer (rare).
    bool bulkOK = false;
    if (dataMaxEnd > dataMinOff && (dataMaxEnd - dataMinOff) <= sizeof(g_rawTrkDataBuf)) {
        f_lseek(disk->Diskfile, disk->fdiDataOffset + trkDataOffset + dataMinOff);
        f_read(disk->Diskfile, g_rawTrkDataBuf, dataMaxEnd - dataMinOff, &br);
        invalidateSclCacheForScratch();  // scratch shares SRAM with the SCL track-0 cache
        bulkOK = true;
    }

    // Dynamic gap sizing (ZXMAK2: distribute free space across gaps)
    int freeSpace = imageSize - (trkdatalen + secCount * (3 + 2)); // 3×4E + 2×00 per sector
    int synchroPulseLen = 1;
    int firstSpaceLen = 1;
    int secondSpaceLen = 1;
    int thirdSpaceLen = 1;
    int synchroSpaceLen = 1;
    freeSpace -= firstSpaceLen + secondSpaceLen + thirdSpaceLen + synchroSpaceLen;
    if (freeSpace < 0) {
        imageSize += -freeSpace;
        freeSpace = 0;
    }
    while (freeSpace > 0) {
        if (freeSpace >= (secCount * 2))
            if (synchroSpaceLen < 12) { synchroSpaceLen++; freeSpace -= secCount * 2; }
        if (freeSpace < secCount) break;
        if (firstSpaceLen < 10) { firstSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (secondSpaceLen < 22) { secondSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (thirdSpaceLen < 60) { thirdSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (synchroSpaceLen >= 12 && firstSpaceLen >= 10 &&
            secondSpaceLen >= 22 && thirdSpaceLen >= 60) break;
    }
    if (freeSpace > (secCount * 2) + 10) { synchroPulseLen++; freeSpace -= secCount; }
    if (freeSpace > (secCount * 2) + 9) synchroPulseLen++;
    // Ensure minimum 3 A1 sync bytes for MFM mark detection (mark = 0xa1a1a1).
    // Each A1 is written twice per sector (ID + data), so extra A1s cost 2*secCount each.
    while (synchroPulseLen < 3) {
        synchroPulseLen++;
        imageSize += secCount * 2; // expand buffer to fit extra sync bytes
    }
    if (freeSpace < 0) { imageSize += -freeSpace; freeSpace = 0; }

    // Clamp to buffer size
    int bufSize = (int)DISK_TRACK_BUF_SZ;
    if (imageSize > bufSize) imageSize = bufSize;

    uint8_t *buf = wd->diskTrackBuf;
    int pos = 0;

    for (int sec = 0; sec < secCount; sec++) {
        uint8_t *sh = &secHdrs[sec * 7];
        uint8_t secC = sh[0], secH = sh[1], secR = sh[2], secN = sh[3];
        uint8_t flags = sh[4];
        uint16_t secDataOff = sh[5] | (sh[6] << 8);

        // Gap 1 (firstSpace)
        for (int i = 0; i < firstSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync (synchroSpace)
        for (int i = 0; i < synchroSpaceLen && pos < imageSize; i++) buf[pos++] = 0x00;
        // Sync pulse (A1 bytes)
        int crcStart = pos;
        for (int i = 0; i < synchroPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
        // Address mark
        if (sec < 32) {
            wd->fdiSectorIdPos[sec] = pos; // record position of 0xFE
            wd->fdiSectorFlags[sec] = (!(flags & (1 << (secN & 3))) ? 1 : 0)
                                    | ((flags & 0x40) ? 2 : 0);
        }
        if (pos < imageSize) buf[pos++] = 0xFE;
        // ID field: C H R N
        if (pos + 4 <= imageSize) {
            buf[pos++] = secC;
            buf[pos++] = secH;
            buf[pos++] = secR;
            buf[pos++] = secN;
        }
        // ID CRC
        uint16_t crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        // Gap 2 (secondSpace)
        for (int i = 0; i < secondSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync before data
        for (int i = 0; i < synchroSpaceLen && pos < imageSize; i++) buf[pos++] = 0x00;

        // Data area (only if bit6 of flags is clear)
        if (!(flags & 0x40)) {
            crcStart = pos;
            // Sync pulse
            for (int i = 0; i < synchroPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
            // Data mark: F8=deleted, FB=normal
            uint8_t dataMark = (flags & 0x80) ? 0xF8 : 0xFB;
            if (pos < imageSize) buf[pos++] = dataMark;

            // Sector data: copy from the bulk-read scratch buffer (fast path), or
            // fall back to an individual SD read if the bulk read was skipped.
            int slen = 128 << (secN & 3);
            uint32_t filePos = disk->fdiDataOffset + trkDataOffset + secDataOff;
            int toRead = slen;
            if (pos + toRead > imageSize) toRead = imageSize - pos;
            if (bulkOK && secDataOff >= dataMinOff &&
                (uint32_t)(secDataOff - dataMinOff) + (uint32_t)toRead <= (dataMaxEnd - dataMinOff)) {
                memcpy(buf + pos, g_rawTrkDataBuf + (secDataOff - dataMinOff), toRead);
            } else {
                f_lseek(disk->Diskfile, filePos);
                f_read(disk->Diskfile, buf + pos, toRead, &br);
            }
            pos += toRead;

            // Data CRC
            crc = 0xFFFF;
            for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
            // Bad CRC if none of the low 6 bits are set
            if (!(flags & (1 << (secN & 3)))) crc ^= 0xFFFF;
            if (pos + 2 <= imageSize) {
                buf[pos++] = (uint8_t)(crc >> 8);
                buf[pos++] = (uint8_t)(crc & 0xFF);
            }
        }

        // Gap 3 (thirdSpace)
        for (int i = 0; i < thirdSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
    }

    wd->fdiSectorCount = (secCount < 32) ? secCount : 32;

    // Fill remainder with 0x4E
    while (pos < imageSize) buf[pos++] = 0x4E;

    wd->diskTrackLen = pos;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
    wd->diskLoadedUnit = wd->diskS;
}

// Generate MFM track image from a TD0 (Teledisk) track record. The record is
// streamed from the SD card (Diskfile or temp file) into td0TrackBuf on demand.
// Mirrors fdiLoadTrack's MFM layout/CRC/gap logic
// but decodes each sector's data (raw/pattern/RLE) from the TD0 stream.
// Read-only: writes are silently dropped by rvmwdDiskStep (writeprotect=1).
void td0LoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide
        && wd->diskS == wd->diskLoadedUnit)
        return;

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int trkIdx = cyl * disk->sides + side;
    if (trkIdx < 0 || trkIdx >= 168 || !disk->td0Stream
        || disk->fdiTrackHdrOffsets[trkIdx] == 0xFFFFFFFF) {
        wd->diskTrackLen = 0;
        wd->fdiSectorCount = 0;
        return;
    }

    // Per-sector staging for the fallback path (track record > 8 KB): holds
    // one sector's encoded TD0 data at a time, up to a 1 KB raw sector
    // (method byte + 1024 data bytes). Static — no heap fragmentation.
    static uint8_t g_td0_enc_sec[2048];

    uint32_t filePos = disk->fdiTrackHdrOffsets[trkIdx];
    UINT br = 0;
    f_lseek(disk->td0Stream, filePos);
#if FDD_PORT_TRACE
    Debug::log("[TD0] load trk cyl=%d side=%d idx=%d filePos=%u",
               (int)cyl, (int)side, trkIdx, (unsigned)filePos);
#endif

    // Read the 4-byte track header: secCount, cyl, side, crc
    uint8_t thdr[4];
    f_read(disk->td0Stream, thdr, 4, &br);
    if (br < 4) { wd->diskTrackLen = 0; wd->fdiSectorCount = 0; return; }
    int secCount = thdr[0];
    if (secCount > 32) secCount = 32;
    filePos += 4;

    // Speculative bulk read: TD0 sector records (header + encoded data) are
    // laid out contiguously, so stage the whole track record in one SD
    // multi-block read. Pieces that fall outside the staging buffer (track
    // record > 8 KB, rare) fall back to per-sector f_lseek+f_read below.
    uint32_t spanStart = filePos;
    UINT bulkLen = 0;
    {
        FSIZE_t rem = f_size(disk->td0Stream) - spanStart;
        UINT want = (rem > (FSIZE_t)sizeof(g_rawTrkDataBuf))
                    ? (UINT)sizeof(g_rawTrkDataBuf) : (UINT)rem;
        f_read(disk->td0Stream, g_rawTrkDataBuf, want, &bulkLen);
        invalidateSclCacheForScratch();  // scratch shares SRAM with the SCL track-0 cache
    }

    // First pass: parse sector headers (from the staging buffer when covered),
    // recording per-sector geometry and the file offset of each sector's
    // encoded data block.
    struct TD0Sec {
        uint8_t c, h, r, n, flags;
        uint16_t secSize;
        uint32_t encFileOff; // file offset of encoded data (method byte first)
        uint16_t encLen;     // bytes including method byte; 0 if no data
        bool present;        // false -> no data (NO_DATA / NO_DATA2 flag)
    } secs[32];

    int kept = 0;
    int trkdatalen = 0;
    for (int s = 0; s < secCount; s++) {
        uint8_t shdr[6];
        if (filePos - spanStart + 6 <= bulkLen) {
            memcpy(shdr, g_rawTrkDataBuf + (filePos - spanStart), 6);
        } else {
            f_lseek(disk->td0Stream, filePos);
            f_read(disk->td0Stream, shdr, 6, &br);
            if (br < 6) break;
        }
        filePos += 6;

        uint8_t sc = shdr[0], sh = shdr[1], sr = shdr[2], sn = shdr[3];
        uint8_t flags = shdr[4];
        bool hasData = !(flags & (TD0_SEC_NO_DATA | TD0_SEC_NO_DATA2));
        uint32_t encFileOff = 0;
        uint16_t encLen = 0;
        if (hasData) {
            uint8_t dl[2];
            if (filePos - spanStart + 2 <= bulkLen) {
                memcpy(dl, g_rawTrkDataBuf + (filePos - spanStart), 2);
            } else {
                f_lseek(disk->td0Stream, filePos);
                f_read(disk->td0Stream, dl, 2, &br);
                if (br < 2) break;
            }
            encLen = dl[0] | (dl[1] << 8);
            filePos += 2;
            encFileOff = filePos;
            filePos += encLen;
        }
        // Sectors with no ID field are skipped (no header to expose to the FDC).
        if (flags & TD0_SEC_NO_ID) continue;

        TD0Sec &d = secs[kept];
        d.c = sc; d.h = sh; d.r = sr; d.n = sn; d.flags = flags;
        d.secSize = (uint16_t)(128 << (sn & 3));
        d.encFileOff = encFileOff; d.encLen = encLen;
        d.present = hasData;

        trkdatalen += 2 + 6;       // A1 + FE + CHRN + CRC
        if (hasData) trkdatalen += 4 + d.secSize; // A1 + FB + data + CRC
        kept++;
        if (kept >= 32) break;
    }
    secCount = kept;

#if FDD_PORT_TRACE
    // Dump the sector IDs (CHRN) the FDC will be presented for this track, so a
    // loader that keeps re-seeking can be diagnosed (wrong R order / N / count).
    {
        char ids[160]; int n = 0;
        for (int s = 0; s < secCount && n < (int)sizeof(ids) - 16; s++)
            n += snprintf(ids + n, sizeof(ids) - n, "%d/%d/%d/%d%s ",
                          secs[s].c, secs[s].h, secs[s].r, secs[s].n,
                          secs[s].present ? "" : "(nd)");
        Debug::log("[TD0] trk cyl=%d side=%d secs=%d CHRN: %s",
                   (int)cyl, (int)side, secCount, ids);
    }
#endif

    // --- Dynamic gap sizing (identical to fdiLoadTrack) ---
    int imageSize = 6250;
    int freeSpace = imageSize - (trkdatalen + secCount * (3 + 2));
    int synchroPulseLen = 1, firstSpaceLen = 1, secondSpaceLen = 1,
        thirdSpaceLen = 1, synchroSpaceLen = 1;
    freeSpace -= firstSpaceLen + secondSpaceLen + thirdSpaceLen + synchroSpaceLen;
    if (freeSpace < 0) { imageSize += -freeSpace; freeSpace = 0; }
    while (freeSpace > 0) {
        if (freeSpace >= (secCount * 2))
            if (synchroSpaceLen < 12) { synchroSpaceLen++; freeSpace -= secCount * 2; }
        if (freeSpace < secCount) break;
        if (firstSpaceLen < 10) { firstSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (secondSpaceLen < 22) { secondSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (thirdSpaceLen < 60) { thirdSpaceLen++; freeSpace -= secCount; }
        if (freeSpace < secCount) break;
        if (synchroSpaceLen >= 12 && firstSpaceLen >= 10 &&
            secondSpaceLen >= 22 && thirdSpaceLen >= 60) break;
    }
    if (freeSpace > (secCount * 2) + 10) { synchroPulseLen++; freeSpace -= secCount; }
    if (freeSpace > (secCount * 2) + 9) synchroPulseLen++;
    while (synchroPulseLen < 3) { synchroPulseLen++; imageSize += secCount * 2; }
    if (freeSpace < 0) { imageSize += -freeSpace; freeSpace = 0; }

    int bufSize = (int)DISK_TRACK_BUF_SZ;
    if (imageSize > bufSize) imageSize = bufSize;

    uint8_t *buf = wd->diskTrackBuf;
    int pos = 0;

    for (int sec = 0; sec < secCount; sec++) {
        TD0Sec &d = secs[sec];

        for (int i = 0; i < firstSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
        for (int i = 0; i < synchroSpaceLen && pos < imageSize; i++) buf[pos++] = 0x00;
        int crcStart = pos;
        for (int i = 0; i < synchroPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;

        bool dataCrcErr = (d.flags & TD0_SEC_CRC_ERR) != 0;
        if (sec < 32) {
            wd->fdiSectorIdPos[sec] = pos; // position of 0xFE
            wd->fdiSectorFlags[sec] = (dataCrcErr ? 1 : 0) | (d.present ? 0 : 2);
        }
        if (pos < imageSize) buf[pos++] = 0xFE;
        if (pos + 4 <= imageSize) {
            buf[pos++] = d.c; buf[pos++] = d.h; buf[pos++] = d.r; buf[pos++] = d.n;
        }
        uint16_t crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        for (int i = 0; i < secondSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
        for (int i = 0; i < synchroSpaceLen && pos < imageSize; i++) buf[pos++] = 0x00;

        if (d.present) {
            crcStart = pos;
            for (int i = 0; i < synchroPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
            uint8_t dataMark = (d.flags & TD0_SEC_DELETED) ? 0xF8 : 0xFB;
            if (pos < imageSize) buf[pos++] = dataMark;

            int slen = d.secSize;
            int avail = imageSize - pos;
            int toEmit = (slen <= avail) ? slen : avail;
            if (toEmit > 0 && d.encLen > 0) {
                if (d.encFileOff - spanStart + d.encLen <= bulkLen) {
                    // Encoded data already staged by the bulk read.
                    td0_decode_sector(g_rawTrkDataBuf + (d.encFileOff - spanStart),
                                      d.encLen, slen, buf + pos);
                } else {
                    // Fetch this sector's encoded data from the file on demand.
                    uint16_t readLen = d.encLen < (uint16_t)sizeof(g_td0_enc_sec)
                                       ? d.encLen : (uint16_t)sizeof(g_td0_enc_sec);
                    UINT br2 = 0;
                    f_lseek(disk->td0Stream, d.encFileOff);
                    f_read(disk->td0Stream, g_td0_enc_sec, readLen, &br2);
                    td0_decode_sector(g_td0_enc_sec, (uint16_t)br2, slen, buf + pos);
                }
            } else if (toEmit > 0)
                memset(buf + pos, 0, toEmit);
            pos += toEmit;

            crc = 0xFFFF;
            for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
            if (dataCrcErr) crc ^= 0xFFFF; // expose source CRC error to the FDC
            if (pos + 2 <= imageSize) {
                buf[pos++] = (uint8_t)(crc >> 8);
                buf[pos++] = (uint8_t)(crc & 0xFF);
            }
        }

        for (int i = 0; i < thirdSpaceLen && pos < imageSize; i++) buf[pos++] = 0x4E;
    }

    wd->fdiSectorCount = (secCount < 32) ? secCount : 32;
    while (pos < imageSize) buf[pos++] = 0x4E;

    wd->diskTrackLen = pos;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
    wd->diskLoadedUnit = wd->diskS;
#if FDD_PORT_TRACE
    Debug::log("[TD0] built trk cyl=%d side=%d: imageSize=%d trkLen=%d bufSize=%d trkdatalen=%d gaps[1=%d s=%d 2=%d 3=%d pulse=%d]",
               (int)cyl, (int)side, imageSize, (int)pos, (int)DISK_TRACK_BUF_SZ,
               trkdatalen, firstSpaceLen, synchroSpaceLen, secondSpaceLen, thirdSpaceLen, synchroPulseLen);
#endif
}

// Generate MFM track image from MBD raw sector dump (MB-02+ BS-DOS format).
// MBD is a simple linear layout: tracks × sides × sectors × sectorSize bytes.
void mbdLoadTrack(rvmWD1793 *wd, uint32_t cyl, uint8_t side) {
    if ((int)cyl == wd->diskLoadedCyl && (int)side == wd->diskLoadedSide
        && wd->diskS == wd->diskLoadedUnit)
        return;

    // Flush any pending writes before loading new track
    if (wd->diskDirty)
        mbdFlushTrack(wd);

    rvmwdDisk *disk = wd->disk[wd->diskS];
    int spt = disk->mbdSectorsPerTrack;
    int secSize = disk->mbdSectorSize;
    uint8_t secN = (secSize == 1024) ? 3 : (secSize == 512) ? 2 : (secSize == 256) ? 1 : 0;

    // Base file offset for this track/side
    uint32_t trackOffset = ((uint32_t)cyl * disk->sides + side) * spt * secSize;

    // Build synthetic MFM track image (same approach as fdiLoadTrack)
    // Bulk-read all sector data first: one f_lseek + one f_read for the whole
    // track (spt * secSize bytes), staged at the end of diskTrackBuf.
    // This reduces N separate CMD18(2)+STOP SPI transactions to one CMD18(N*2).
    // Safety: MFM build grows from pos=0; staging occupies the high end.
    // Per-sector MFM overhead (framing, gaps, CRCs) is ~102 bytes, so:
    //   max MFM = spt*(102+secSize); raw staging = spt*secSize.
    // Only use bulk path when both fit in DISK_TRACK_BUF_SZ.
    int imageSize = (int)DISK_TRACK_BUF_SZ;
    uint8_t *buf = wd->diskTrackBuf;
    int pos = 0;

    uint32_t totalRawSize = (uint32_t)spt * (uint32_t)secSize;
    uint32_t maxMFMSize   = (uint32_t)spt * (102u + (uint32_t)secSize);
    bool bulk_ok = (maxMFMSize + totalRawSize <= (uint32_t)DISK_TRACK_BUF_SZ);
    uint8_t *rawStage = bulk_ok ? (buf + DISK_TRACK_BUF_SZ - (int)totalRawSize) : nullptr;

    UINT br;
    if (bulk_ok) {
        // One seek + one read for all sector data; FatFS issues CMD18(count) internally.
        f_lseek(disk->Diskfile, trackOffset);
        f_read(disk->Diskfile, rawStage, totalRawSize, &br);
    }

    // Gap sizes for HD MFM (approximate standard values)
    int gap1Len = 10;      // gap before each sector ID
    int syncLen = 12;      // 0x00 sync bytes
    int syncPulseLen = 3;  // 0xA1 sync pulses
    int gap2Len = 22;      // gap between ID and data
    int gap3Len = 30;      // gap after data

    for (int sec = 0; sec < spt && sec < 32; sec++) {
        // Gap 1
        for (int i = 0; i < gap1Len && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync
        for (int i = 0; i < syncLen && pos < imageSize; i++) buf[pos++] = 0x00;
        // Sync pulse (A1 bytes)
        int crcStart = pos;
        for (int i = 0; i < syncPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
        // ID address mark
        wd->fdiSectorIdPos[sec] = pos;
        wd->fdiSectorFlags[sec] = 0; // no CRC errors, has data area
        if (pos < imageSize) buf[pos++] = 0xFE;
        // ID field: C H R N
        if (pos + 4 <= imageSize) {
            buf[pos++] = (uint8_t)cyl;
            buf[pos++] = side;
            // PRO (Profi CP/M) special: first track (cyl 0 side 0) uses sector
            // IDs {1,2,3,4,9} per UnrealSpeccy wldr_pro.cpp. All other tracks
            // use standard {1,2,3,4,5}.
            uint8_t sec_id;
            if (disk->IsProFile && cyl == 0 && side == 0) {
                static const uint8_t sn0[5] = {1, 2, 3, 4, 9};
                sec_id = (sec < 5) ? sn0[sec] : (uint8_t)(sec + 1);
            } else {
                sec_id = (uint8_t)(sec + 1); // sectors numbered from 1
            }
            buf[pos++] = sec_id;
            buf[pos++] = secN;
        }
        // ID CRC
        uint16_t crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        // Gap 2
        for (int i = 0; i < gap2Len && pos < imageSize; i++) buf[pos++] = 0x4E;
        // Sync before data
        for (int i = 0; i < syncLen && pos < imageSize; i++) buf[pos++] = 0x00;

        // Data area
        crcStart = pos;
        for (int i = 0; i < syncPulseLen && pos < imageSize; i++) buf[pos++] = 0xA1;
        if (pos < imageSize) buf[pos++] = 0xFB; // data mark

        // Copy sector data from staging area (bulk-read) or read individually.
        int toRead = secSize;
        if (pos + toRead > imageSize) toRead = imageSize - pos;
        if (bulk_ok) {
            memcpy(buf + pos, rawStage + sec * secSize, (size_t)toRead);
        } else {
            uint32_t fileOffset = trackOffset + (uint32_t)sec * (uint32_t)secSize;
            f_lseek(disk->Diskfile, fileOffset);
            f_read(disk->Diskfile, buf + pos, (UINT)toRead, &br);
        }
        pos += toRead;

        // Data CRC
        crc = 0xFFFF;
        for (int i = crcStart; i < pos; i++) crc = vgCrc(crc, buf[i]);
        if (pos + 2 <= imageSize) {
            buf[pos++] = (uint8_t)(crc >> 8);
            buf[pos++] = (uint8_t)(crc & 0xFF);
        }

        // Gap 3
        for (int i = 0; i < gap3Len && pos < imageSize; i++) buf[pos++] = 0x4E;
    }

    wd->fdiSectorCount = (spt < 32) ? spt : 32;

    // Fill remainder with 0x4E
    while (pos < imageSize) buf[pos++] = 0x4E;

    wd->diskTrackLen = pos;
    wd->diskLoadedCyl = (int)cyl;
    wd->diskLoadedSide = (int)side;
    wd->diskLoadedUnit = wd->diskS;
}

// Flush modified MBD track buffer back to file
static void mbdFlushTrack(rvmWD1793 *wd) {
    if (!wd->diskDirty || wd->diskLoadedCyl < 0) return;

    int u = (wd->diskLoadedUnit >= 0) ? wd->diskLoadedUnit : wd->diskS;
    rvmwdDisk *disk = wd->disk[u];
    if (!disk || !disk->IsMBDFile || disk->writeprotect) {
        wd->diskDirty = false;
        return;
    }

    int spt = disk->mbdSectorsPerTrack;
    int secSize = disk->mbdSectorSize;

    // Extract sector data from MFM track buffer and write back
    for (int sec = 0; sec < spt && sec < wd->fdiSectorCount; sec++) {
        // Find data mark (0xFB) after the sector's ID mark
        int idPos = wd->fdiSectorIdPos[sec];
        // Skip: FE + C + H + R + N + CRC(2) + gap2 + sync + A1s + FB
        int searchStart = idPos + 7; // past ID mark and CHRN+CRC
        int dataStart = -1;
        for (int i = searchStart; i < (int)wd->diskTrackLen - 1; i++) {
            if (wd->diskTrackBuf[i] == 0xFB || wd->diskTrackBuf[i] == 0xF8) {
                dataStart = i + 1;
                break;
            }
        }
        if (dataStart < 0 || dataStart + secSize > (int)wd->diskTrackLen) continue;

        uint32_t fileOffset = ((uint32_t)wd->diskLoadedCyl * disk->sides + wd->diskLoadedSide) * spt * secSize
                            + sec * secSize;
        UINT bw;
        f_lseek(disk->Diskfile, fileOffset);
        f_write(disk->Diskfile, wd->diskTrackBuf + dataStart, secSize, &bw);
    }
    f_sync(disk->Diskfile);
    wd->diskDirty = false;
}

#endif

IRAM_ATTR uint8_t rvmwdDiskStep(rvmWD1793 *wd, uint32_t control) {

  rvmwdDisk *disk = wd->disk[wd->diskS];
  const uint32_t seek = control & 0x300;
  disk->a = 0;

  // Seek forward or backward
  if (seek) disk->t = (seek == 0x300) ? disk->t - (disk->t != 0) : disk->t + (disk->t < disk->tracks);

  disk->s = disk->t ? 0 : kRVMwdDiskOutTrack0;

  if(disk->indexDelay) {
    disk->indexDelay--;
    disk->s |= kRVMwdDiskOutIndex;
    return disk->s;
  }

  if (disk->cursectbufpos < 0xff) {

    disk->cursectbufpos++;

    disk->indx++;

    if(control & kRVMwdDiskControlWrite) {
      const uint8_t wr = control & 0xff;
      UINT bw;
      f_write(disk->Diskfile, &wr, 1, &bw);
      disk->cursectbuf[disk->cursectbufpos] = wr;
      return 0;
    }

    disk->a = disk->cursectbuf[disk->cursectbufpos];
    return disk->s;

  } else {

#if !PICO_RP2040
    if (disk->IsUDIFile) {

      // During seek (Type I), don't load track data — only update disk->t and status.
      // Track will be loaded on first actual data access (Read/Write Sector, etc.)
      if (seek)
        return disk->s;

      // Idle rotation (Type I stepping, command-start delay, motor idle): no
      // data transfer is in progress, so don't fetch track data — a long seek
      // would otherwise load EVERY intermediate cylinder from SD (~5.6 ms
      // each, 400+ ms per seek = the Profi CP/M loading FPS collapse). Keep
      // the index-pulse timing running against the last loaded track length.
      if (wd->stepState <= kRVMWD177XStepWaiting) {
          uint32_t tl = wd->diskTrackLen ? wd->diskTrackLen : 6250;
          if (disk->indx != 0xffffffff && disk->indx >= tl) {
              disk->indx = 0xffffffff;
              disk->indexDelay = 25;
              return disk->s;
          }
          disk->indx++;
          return disk->s;
      }

      // Load track before checking length.
      // Don't reload mid-stream while bytes are being transferred (ReadByte/
      // WriteByte) — but DO honor the FDC's selected side when a new sector
      // search starts (WaitingMark). Previously WaitingMark was lumped in with
      // the byte-transfer states, which only worked because the old idle path
      // pre-loaded wd->side every step; once the e1db3d5 idle-rotation early-
      // return removed that pre-load, keeping the stale side here made side 1
      // of an already-loaded cylinder unreadable (broke double-sided UDIs).
      {
        uint8_t loadSide = wd->side;
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        udiLoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->diskTrackLen)
          wd->diskTrackBuf[disk->indx] = control & 0xff;
        wd->diskDirty = true;
        return 0;
      }

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e; // gap filler

      return disk->s;

    }

    if (disk->IsTD0File) {

      if (seek)
        return disk->s;

      // Idle rotation (Type I stepping, command-start delay, motor idle): no
      // data transfer is in progress, so don't fetch track data — a long seek
      // would otherwise load EVERY intermediate cylinder from SD (~5.6 ms
      // each, 400+ ms per seek = the Profi CP/M loading FPS collapse). Keep
      // the index-pulse timing running against the last loaded track length.
      if (wd->stepState <= kRVMWD177XStepWaiting) {
          uint32_t tl = wd->diskTrackLen ? wd->diskTrackLen : 6250;
          if (disk->indx != 0xffffffff && disk->indx >= tl) {
              disk->indx = 0xffffffff;
              disk->indexDelay = 25;
              return disk->s;
          }
          disk->indx++;
          return disk->s;
      }

      {
        uint8_t loadSide = wd->side;
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        td0LoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      // TD0 is read-only — drop any write (writeprotect set at insert).
      if(control & kRVMwdDiskControlWrite)
        return 0;

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e;

      return disk->s;

    }

    if (disk->IsFDIFile) {

      if (seek)
        return disk->s;

      // Idle rotation (Type I stepping, command-start delay, motor idle): no
      // data transfer is in progress, so don't fetch track data — a long seek
      // would otherwise load EVERY intermediate cylinder from SD (~5.6 ms
      // each, 400+ ms per seek = the Profi CP/M loading FPS collapse). Keep
      // the index-pulse timing running against the last loaded track length.
      if (wd->stepState <= kRVMWD177XStepWaiting) {
          uint32_t tl = wd->diskTrackLen ? wd->diskTrackLen : 6250;
          if (disk->indx != 0xffffffff && disk->indx >= tl) {
              disk->indx = 0xffffffff;
              disk->indexDelay = 25;
              return disk->s;
          }
          disk->indx++;
          return disk->s;
      }

      {
        uint8_t loadSide = wd->side;
        // Preserve the loaded track buffer only during actual data transfer
        // (ReadByte/WriteByte). During WaitingMark (mark search), always use
        // wd->side so a side change is detected and the correct track is loaded.
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        fdiLoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->diskTrackLen)
          wd->diskTrackBuf[disk->indx] = control & 0xff;
        wd->diskDirty = true;
        return 0;
      }

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e;

      return disk->s;

    }

    if (disk->IsMBDFile) {

      if (seek)
        return disk->s;

      // Idle rotation (Type I stepping, command-start delay, motor idle): no
      // data transfer is in progress, so don't fetch track data — a long seek
      // would otherwise load EVERY intermediate cylinder from SD (~5.6 ms
      // each, 400+ ms per seek = the Profi CP/M loading FPS collapse). Keep
      // the index-pulse timing running against the last loaded track length.
      if (wd->stepState <= kRVMWD177XStepWaiting) {
          uint32_t tl = wd->diskTrackLen ? wd->diskTrackLen : 6250;
          if (disk->indx != 0xffffffff && disk->indx >= tl) {
              disk->indx = 0xffffffff;
              disk->indexDelay = 25;
              return disk->s;
          }
          disk->indx++;
          return disk->s;
      }

      {
        uint8_t loadSide = wd->side;
        bool activeCmd = (wd->stepState == kRVMWD177XStepReadByte
                       || wd->stepState == kRVMWD177XStepWriteByte);
        if (activeCmd && wd->diskLoadedCyl == (int)disk->t && wd->diskLoadedSide >= 0)
            loadSide = (uint8_t)wd->diskLoadedSide;
        mbdLoadTrack(wd, disk->t, loadSide);
      }

      if(disk->indx != 0xffffffff && disk->indx >= wd->diskTrackLen) {
        disk->indx = 0xffffffff;
        disk->indexDelay = 25;
        return disk->s;
      }

      disk->indx++;

      if(control & kRVMwdDiskControlWrite) {
        if (disk->indx < wd->diskTrackLen)
          wd->diskTrackBuf[disk->indx] = control & 0xff;
        wd->diskDirty = true;
        return 0;
      }

      if (disk->indx < wd->diskTrackLen)
        disk->a = wd->diskTrackBuf[disk->indx];
      else
        disk->a = 0x4e;

      return disk->s;

    }

#endif

    if(disk->indx != 0xffffffff && disk->indx >= /*6417*/ 6663) {
      disk->indx = 0xffffffff;
      disk->cursectbufpos = 0xff;
      disk->indexDelay = 25;
      return disk->s;
    }

    disk->indx++;

    const uint32_t cursect = (disk->indx - 146) / 392;

    // const uint32_t cursect = System34_track_info[disk->indx];

    if (cursect < 16) {

      if (disk->indx == sectdatapos[cursect]) // Track in sector header
        disk->a = (!disk->t && wd->side) ? disk->t0s1_info : disk->t;
      else if (disk->indx == sectdatapos[cursect] + 44) { // Sector data

        // const uint32_t side = (control & 0x800) << 1;

        if ((disk->IsSCLFile) && (!disk->t) && (!wd->side)) {

          // Create track0 from SCL file if not already done
          unsigned char* t0 = claim_scl_track0(wd);
          if (!wd->sclConverted) {
              SCLtoTRD(disk, t0);
              wd->sclConverted = true;
          }

          // SCL disk -> Read sector to cache from created Track0
          if (cursect < 9)
            memcpy(disk->cursectbuf, t0 + (cursect << 8), 0x100);
#if !PICO_RP2040
          else if (cursect == 9 && disk->bootInjected)
            memcpy(disk->cursectbuf, kTrdosBootSector, 0x100); // injected boot data
#endif
          else
            memset(disk->cursectbuf, 0, 0x100);

          disk->a = disk->cursectbuf[0];

        } else {

          const int seekptr = (disk->t << (11 + disk->sides)) + (wd->side << 12) + (cursect << 8) + disk->sclDataOffset;

          UINT br;
          f_lseek(disk->Diskfile,seekptr);
          f_read(disk->Diskfile, disk->cursectbuf, 0x100, &br);

          if(control & kRVMwdDiskControlWrite) {
            uint8_t wr = control & 0xff;
            UINT bw;
            f_lseek(disk->Diskfile,seekptr);
            f_write(disk->Diskfile, &wr,1,&bw);
            disk->cursectbuf[0] = wr;
          } else {
            disk->a = disk->cursectbuf[0];
          }

        }

        disk->cursectbufpos = 0;

      } else disk->a = System34_track[disk->indx];

    } else disk->a = System34_track[disk->indx];

    if(control & kRVMwdDiskControlWrite) {
      disk->a = 0;
      return 0;
    }

    return disk->s;

  }

}

void wdDiskEject(rvmWD1793 *wd, unsigned char UnitNum) {

  if(wd->disk[UnitNum] != NULL) {

    printf("Ejecting disk\n");

    if (wd->disk[UnitNum]->Diskfile != NULL) {
#if !PICO_RP2040
        // Flush only if the buffer actually holds this unit's dirty track.
        // (diskLoadedUnit, not diskS — they differ after a drive switch.)
        if (wd->diskDirty && wd->diskLoadedUnit == (int)UnitNum) {
            if (wd->disk[UnitNum]->IsUDIFile) udiFlushTrack(wd);
            else if (wd->disk[UnitNum]->IsFDIFile) fdiFlushTrack(wd);
            else if (wd->disk[UnitNum]->IsMBDFile) mbdFlushTrack(wd);
        }
#endif
        fclose2(wd->disk[UnitNum]->Diskfile);
        wd->disk[UnitNum]->Diskfile = NULL;
    }

#if !PICO_RP2040
    // TD0 streaming cleanup: close+unlink the temp file if we own it (packed
    // images), and free the per-track scratch buffer. The unpacked case shares
    // Diskfile, which was already closed above.
    if (wd->disk[UnitNum]->IsTD0File) {
        if (wd->disk[UnitNum]->td0OwnsStream && wd->disk[UnitNum]->td0Stream) {
            fclose2(wd->disk[UnitNum]->td0Stream);
            if (!wd->disk[UnitNum]->td0TempPath.empty())
                f_unlink(wd->disk[UnitNum]->td0TempPath.c_str());
        }
        wd->disk[UnitNum]->td0Stream = NULL;
        wd->disk[UnitNum]->td0OwnsStream = false;
    }
#endif

    free(wd->disk[UnitNum]);
    wd->disk[UnitNum] = NULL;

    if (wd->diskS == UnitNum) {

      _end(wd);

      wd->side = 0;

      wd->sclConverted = false;

    }

    // Power off drive
    wd->control &= ~(kRVMWD177XPower0 << UnitNum);

  } else printf("No disk to eject\n");

}

void SCLtoTRD(rvmwdDisk *d, unsigned char* track0) {

    uint8_t numberOfFiles;

    // Reset track 0 info
    memset(track0,0,2304);

    // fseek(d->Diskfile,8,SEEK_SET);
    // fread(&numberOfFiles,1,1,d->Diskfile);
    UINT br;
    f_lseek(d->Diskfile,8);
    f_read(d->Diskfile, &numberOfFiles,1,&br);

    // printf("Number of files: %d\n",(int)numberOfFiles);

    char diskNameArray[9]="SCL_DISK";

    // Populate FAT.
    int startSector = 0;
    int startTrack = 1; // Since Track 0 is reserved for FAT and Disk Specification.

    uint8_t data;
    for (int i = 0; i < numberOfFiles; i++) {

        int n = i << 4;

        UINT bw;
        for (int j = 0; j < 13; j++) {
            // fread(&data,1,1,d->Diskfile);
            f_read(d->Diskfile, &data,1,&br);
            track0[n + j] = data;
        }

        // fread(&data,1,1,d->Diskfile); // Filelenght
        f_read(d->Diskfile, &data,1,&br);
        track0[n + 13] = data;

        // printf("File #: %d, Filelenght: %d\n",i + 1,(int)data);

        track0[n + 14] = (uint8_t)startSector;
        track0[n + 15] = (uint8_t)startTrack;

        int newStartTrack = (startTrack * 16 + startSector + data) / 16;
        startSector = (startTrack * 16 + startSector + data) - 16 * newStartTrack;
        startTrack = newStartTrack;

    }

    // Populate Disk Specification.
    track0[2273] = (uint8_t)startSector;
    track0[2274] = (uint8_t)startTrack;
    track0[2275] = 22; // Disk Type
    track0[2276] = (uint8_t)numberOfFiles; // File Count
    uint16_t freeSectors = 2560 - (startTrack << 4) + startSector;
    // printf("Free Sectors: %d\n",freeSectors);
    track0[2277] = freeSectors & 0x00ff;
    track0[2278] = freeSectors >> 8;
    track0[2279] = 0x10; // TR-DOS ID

    for (int i = 0; i < 9; i++) track0[2282 + i] = 0x20;

    // Store the image file name in the disk label section of the Disk Specification.
    for (int i = 0; i < 8; i++) track0[2293 + i] = diskNameArray[i];

    d->sclDataOffset =  (9 + (numberOfFiles * 14)) - 4096;

#if !PICO_RP2040
    // Auto-boot: if the SCL has no "boot" file, synthesise one into the in-RAM
    // catalog. Its single data sector is served from flash (kTrdosBootSector) for
    // track 0 / sector 9 in the read path — see rvmwdDiskStep. Track 0's tail is
    // never used by TR-DOS for normal files, so the streamed file data (tracks 1+)
    // is untouched. The free-sector count stays as computed above (that area isn't
    // counted), only the file count is bumped.
    d->bootInjected = false;
    if (Config::trdosAutoBoot && numberOfFiles < 128) {
        bool hasBoot = false;
        for (int i = 0; i < numberOfFiles; i++) {
            const unsigned char *e = track0 + (i << 4);
            if (memcmp(e, "boot    ", 8) == 0 && e[8] == 'B') { hasBoot = true; break; }
        }
        if (!hasBoot) {
            unsigned char *e = track0 + (numberOfFiles << 4);
            memcpy(e, "boot    ", 8);
            e[8]  = 'B';
            e[9]  = TRDOS_BOOT_START  & 0xFF; e[10] = (TRDOS_BOOT_START  >> 8) & 0xFF;
            e[11] = TRDOS_BOOT_LENGTH & 0xFF; e[12] = (TRDOS_BOOT_LENGTH >> 8) & 0xFF;
            e[13] = TRDOS_BOOT_SECCNT;
            e[14] = 9; // start sector (track 0's reserved tail)
            e[15] = 0; // start track
            track0[2276] = (uint8_t)(numberOfFiles + 1); // bump file count in disk spec
            d->bootInjected = true;
        }
    }
#endif
}

// Create an empty formatted TRD disk image at the given path.
// 80 tracks, 2 sides, 16 sectors/track, 256 bytes/sector = 655360 bytes.
// Track 0: 9 catalog sectors (0xFF-filled) + service sector + 6 zero sectors.
bool rvmWD1793CreateEmptyTRD(const char *path) {
    // FIL (~560 B with FF_FS_TINY=0) + buf must NOT live on the stack: this runs
    // deep in the OSD file-browser call chain on core0's 2 KB stack (PICO_STACK_SIZE),
    // and ~820 B of locals overflowed it → stack corruption → MSTKERR fault on the
    // next ISR entry. Static, like FileInfo.cpp. Not reentrant — one-shot creation.
    static FIL f;
    static uint8_t buf[256];
    if (f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return false;
    UINT bw;

    // Sectors 0-7: catalog (empty = zeros, 8 sectors = 128 directory entries)
    memset(buf, 0, 256);
    for (int s = 0; s < 8; s++) f_write(&f, buf, 256, &bw);

    // Sector 9 (0-indexed: 8): service sector (disk info at offsets 0xE1-0xE7)
    // Reader expects this at file offset 2048+227 = 8*256+0xE3
    memset(buf, 0, 256);
    buf[0xe1] = 0x00; // first free sector on free track
    buf[0xe2] = 0x01; // first free track (track 1)
    buf[0xe3] = 0x16; // disk type: 80T 2DS
    buf[0xe4] = 0x00; // number of files
    buf[0xe5] = 0xF0; // free sectors low  (0x09F0 = 2544)
    buf[0xe6] = 0x09; // free sectors high
    buf[0xe7] = 0x10; // TR-DOS ID byte
    // 0xEA-0xF2: 9 spaces (password field)
    memset(buf + 0xea, 0x20, 9);
    // 0xF5-0xFC: disk name (8 chars)
    memcpy(buf + 0xf5, "NEW DISK", 8);
    f_write(&f, buf, 256, &bw);

    // Sectors 9-15: rest of track 0, zeros
    memset(buf, 0, 256);
    for (int s = 9; s < 16; s++) f_write(&f, buf, 256, &bw);

    // Remaining 2544 sectors (tracks 1-79, both sides): zeros
    for (int s = 0; s < 2544; s++) f_write(&f, buf, 256, &bw);

    f_close(&f);
    return true;
}

// Convert SCL disk to TRD file on first write attempt.
// Creates a .trd file alongside the .scl, copies all data, and switches the disk handle.
static bool sclConvertToTRD(rvmWD1793 *wd) {
    rvmwdDisk *disk = wd->disk[wd->diskS];
    if (!disk || !disk->IsSCLFile || !disk->Diskfile) return false;

    // Ensure Track0 is populated
    unsigned char* t0 = claim_scl_track0(wd);
    if (!wd->sclConverted) {
        SCLtoTRD(disk, t0);
        wd->sclConverted = true;
    }

    // Build .trd filename from .scl filename
    std::string trdName = disk->fname;
    size_t dotPos = trdName.rfind('.');
    if (dotPos != std::string::npos)
        trdName = trdName.substr(0, dotPos);
    trdName += ".trd";

    // Create TRD file
    Debug::log("SCL->TRD: creating %s", trdName.c_str());
    FIL *trdFile = fopen2(trdName.c_str(), FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
    if (!trdFile) {
        Debug::log("SCL->TRD: fopen2 failed");
        return false;
    }

    UINT bw;
    uint8_t zeroBuf[256];
    memset(zeroBuf, 0, 256);

    // Write track 0 side 0: 16 sectors from Track0 (first 4096 bytes = 16 * 256)
    f_write(trdFile, t0, 2304, &bw);
    // Pad remaining sectors of track 0 (sectors 9..15 are already in Track0 as zeros,
    // but Track0 is only 2304 bytes = 9 sectors; pad to 16 sectors = 4096 bytes).
    // If we injected a boot file, sector 9 holds its data (served from flash before
    // conversion) — write it into the TRD so the catalog entry still resolves.
    for (int s = 9; s < 16; s++) {
#if !PICO_RP2040
        if (s == 9 && disk->bootInjected)
            f_write(trdFile, kTrdosBootSector, 256, &bw);
        else
#endif
            f_write(trdFile, zeroBuf, 256, &bw);
    }

    // Copy remaining data from SCL using same seek formula as rvmwdDiskStep
    // TRD layout: [T0S0: 16*256][T0S1: 16*256][T1S0: 16*256][T1S1: 16*256]...
    uint8_t secBuf[256];
    int totalTracks = disk->tracks + 1;
    int sclFileSize = (int)f_size(disk->Diskfile);
    for (int trk = 0; trk < totalTracks; trk++) {
        for (int side = 0; side < disk->sides; side++) {
            for (int sec = 0; sec < 16; sec++) {
                // Skip track 0 side 0 — already written from Track0
                if (trk == 0 && side == 0) continue;
                int seekptr = (trk << (11 + disk->sides)) + (side << 12) + (sec << 8) + disk->sclDataOffset;
                if (seekptr >= 0 && seekptr < sclFileSize) {
                    UINT br;
                    f_lseek(disk->Diskfile, seekptr);
                    f_read(disk->Diskfile, secBuf, 256, &br);
                    if (br < 256) memset(secBuf + br, 0, 256 - br);
                } else {
                    memset(secBuf, 0, 256);
                }
                // TRD file offset for this sector
                uint32_t trdOffset = (trk * disk->sides + side) * 16 * 256 + sec * 256;
                f_lseek(trdFile, trdOffset);
                f_write(trdFile, secBuf, 256, &bw);
            }
        }
    }

    // Flush TRD to SD card
    f_sync(trdFile);

    // Close SCL, switch to TRD
    fclose2(disk->Diskfile);
    disk->Diskfile = trdFile;
    disk->IsSCLFile = false;
    disk->sclDataOffset = 0;
    disk->fname = trdName;

    Debug::log("SCL->TRD: done, size=%d", (int)f_size(trdFile));
    return true;
}