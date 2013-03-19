/* AudioUtil.h

Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of The Linux Foundation nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#ifndef ALSA_SOUND_AUDIO_UTIL_H
#define ALSA_SOUND_AUDIO_UTIL_H

#define BIT(nr)     (1UL << (nr))
#define MAX_EDID_BLOCKS 10
#define MAX_SHORT_AUDIO_DESC_CNT        30
#define MIN_AUDIO_DESC_LENGTH           3
#define MIN_SPKR_ALLOCATION_DATA_LENGTH 3

typedef enum EDID_AUDIO_FORMAT_ID {
    LPCM = 1,
    AC3,
    MPEG1,
    MP3,
    MPEG2_MULTI_CHANNEL,
    AAC,
    DTS,
    ATRAC,
    SACD,
    DOLBY_DIGITAL_PLUS,
    DTS_HD,
    MAT,
    DST,
    WMA_PRO
} EDID_AUDIO_FORMAT_ID;

typedef struct EDID_AUDIO_BLOCK_INFO {
    EDID_AUDIO_FORMAT_ID nFormatId;
    int nSamplingFreq;
    int nBitsPerSample;
    int nChannels;
} EDID_AUDIO_BLOCK_INFO;

typedef struct EDID_AUDIO_INFO {
    int nAudioBlocks;
    unsigned char nSpeakerAllocation[MIN_SPKR_ALLOCATION_DATA_LENGTH];
    EDID_AUDIO_BLOCK_INFO AudioBlocksArray[MAX_EDID_BLOCKS];
} EDID_AUDIO_INFO;

class AudioUtil {
public:

    //Parses EDID audio block when if HDMI is connected to determine audio sink capabilities.
    static bool getHDMIAudioSinkCaps(EDID_AUDIO_INFO*);

private:
    static int printFormatFromEDID(unsigned char format);
    static int getSamplingFrequencyFromEDID(unsigned char byte);
    static int getBitsPerSampleFromEDID(unsigned char byte,
        unsigned char format);
    static bool getSpeakerAllocation(EDID_AUDIO_INFO* pInfo);
};

#endif /* ALSA_SOUND_AUDIO_UTIL_H */
