/* AudioHardwareALSA.h
 **
 ** Copyright 2008-2010, Wind River Systems
 ** Copyright (c) 2011-2013, The Linux Foundation. All rights reserved
 ** Not a Contribution.
 **
 ** Licensed under the Apache License, Version 2.0 (the "License");
 ** you may not use this file except in compliance with the License.
 ** You may obtain a copy of the License at
 **
 **     http://www.apache.org/licenses/LICENSE-2.0
 **
 ** Unless required by applicable law or agreed to in writing, software
 ** distributed under the License is distributed on an "AS IS" BASIS,
 ** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 ** See the License for the specific language governing permissions and
 ** limitations under the License.
 */

#ifndef ANDROID_AUDIO_HARDWARE_ALSA_H
#define ANDROID_AUDIO_HARDWARE_ALSA_H

#include <utils/List.h>
#include <hardware_legacy/AudioHardwareBase.h>

#include <hardware_legacy/AudioHardwareInterface.h>
#include <hardware_legacy/AudioSystemLegacy.h>
#include <system/audio.h>
#include <hardware/audio.h>
#include <utils/threads.h>
#include <sys/poll.h>
#include <sys/eventfd.h>
#include "AudioUtil.h"

extern "C" {
   #include <sound/asound.h>
   #include <sound/compress_params.h>
   #include <sound/compress_offload.h>
   #include "alsa_audio.h"
   // ToDo: For now we are depending on 8960, it should be made generic
   #include "msm8960_use_cases.h"
   #include "audio_parsers.h"
   void set_bits(unsigned char *input, unsigned char num_bits_reqd,
       unsigned char value, unsigned char *hdr_bit_index);
}

#include <hardware/hardware.h>
#include "SoftMS11.h"

namespace android_audio_legacy
{
using android::List;
using android::Mutex;
using android::Condition;
class ALSADevice;
class ALSAControl;
class AudioHardwareALSA;
class SoftMS11;
class AudioBitstreamSM;
/*******************************************************************************
ID - ALSA MODULE
*******************************************************************************/
#define ALSA_HARDWARE_MODULE_ID "alsa"
#define ALSA_HARDWARE_NAME      "alsa"

/*******************************************************************************
PLAYBACK, RECORD AND VOICE RELATED
*******************************************************************************/
#define DEFAULT_SAMPLING_RATE 48000
#define MAX_SUPPORTED_SAMPLING_RATE 192000
#define DEFAULT_CHANNEL_MODE  2
#define MAX_SUPPORTED_CHANNELS  8
#define VOICE_SAMPLING_RATE   8000
#define VOICE_CHANNEL_MODE    1
#define PLAYBACK_LATENCY      24000
#define RECORD_LATENCY        96000
#define VOICE_LATENCY         85333
#define DEFAULT_BUFFER_SIZE   2048
#define DEFAULT_IN_BUFFER_SIZE   320
#define FM_BUFFER_SIZE        1024
#define MULTI_CHANNEL_MIN_PERIOD_SIZE 256
#define MULTI_CHANNEL_MAX_PERIOD_SIZE 4032
#define MULTI_CHANNEL_PERIOD_COUNT 8
#define PCM_BUFFER_DURATION 10000
#define DEFAULT_OUT_BUFFER_SIZE_PER_CHANNEL   1536
#define DEFAULT_IN_BUFFER_SIZE_PCM_PER_CHANNEL   1536
#define DEFAULT_IN_BUFFER_SIZE_BROADCAST_COMPRESSED   6208

#define VOIP_SAMPLING_RATE_8K 8000
#define VOIP_SAMPLING_RATE_16K 16000
#define VOIP_DEFAULT_CHANNEL_MODE  1
#define VOIP_BUFFER_SIZE_8K    320
#define VOIP_BUFFER_SIZE_16K   640
#define VOIP_BUFFER_MAX_SIZE   VOIP_BUFFER_SIZE_16K
#define VOIP_PLAYBACK_LATENCY      6400
#define VOIP_RECORD_LATENCY        6400
/*******************************************************************************
KEYVALUE PAIR FOR SET/GET PARAMETER
*******************************************************************************/
#define DUALMIC_KEY         "dualmic_enabled"
#define FLUENCE_KEY         "fluence"
#define ANC_KEY             "anc_enabled"
#define TTY_MODE_KEY        "tty_mode"
#define BT_SAMPLERATE_KEY   "bt_samplerate"
#define BTHEADSET_VGS       "bt_headset_vgs"
#define WIDEVOICE_KEY       "wide_voice_enable"
#define FENS_KEY            "fens_enable"
#define SPDIF_FORMAT_KEY    "spdif_format"
#define HDMI_FORMAT_KEY     "hdmi_format"
#define SPDIF_MUTE_KEY      "spdif_mute"
#define HDMI_MUTE_KEY       "hdmi_mute"
#define SPDIF_OCHANNELS_KEY "spdif_output_channels"
#define HDMI_OCHANNELS_KEY  "hdmi_output_channels"
#define SPDIF_DELAY_KEY     "spdif_delay"
#define HDMI_DELAY_KEY      "hdmi_delay"
#define STANDBY_DEVICES_KEY "standby_devices"
#define RESUME_DEVICES_KEY  "resume_devices"
#define COMPR_STANDBY_DEVICES_KEY   "compr_standby_devices"

#define ANC_FLAG        0x00000001
#define DMIC_FLAG       0x00000002
#define QMIC_FLAG       0x00000004
#define TTY_OFF         0x00000010
#define TTY_FULL        0x00000020
#define TTY_VCO         0x00000040
#define TTY_HCO         0x00000080
#define TTY_CLEAR       0xFFFFFF0F
/*******************************************************************************
BIT STREAM STATEMACHINE
*****************************************************************************/
#define SAMPLES_PER_CHANNEL             1536*2
#define MAX_INPUT_CHANNELS_SUPPORTED    8
#define FACTOR_FOR_BUFFERING            2
#define STEREO_CHANNELS                 2
#define MAX_OUTPUT_CHANNELS_SUPPORTED   6
#define PCM_BLOCK_PER_CHANNEL_MS11      1536*2
#define AAC_BLOCK_PER_CHANNEL_MS11      768
#define NUMBER_BITS_IN_A_BYTE           8
#define AC3_BUFFER_SIZE                 1920*2

#define PCM_2CH_OUT                 0
#define PCM_OUT                     0 // should be same as PCM_OUT
#define PCM_MCH_OUT                 1
#define SPDIF_OUT                   2
#define COMPRESSED_OUT              2 // should be same as SPDIF_OUT
#define TRANSCODE_OUT               3

#ifndef ALSA_DEFAULT_SAMPLE_RATE
#define ALSA_DEFAULT_SAMPLE_RATE 44100 // in Hz
#endif
/*******************************************************************************
SESSION ID
*******************************************************************************/
#define NORMAL_PLAYBACK_SESSION_ID 0
#define LPA_SESSION_ID 1
#define TUNNEL_SESSION_ID 2
#define MPQ_SESSION_ID 3
/*******************************************************************************
TUNNEL MODE
*******************************************************************************/
//Required for Tunnel
#define TUNNEL_DECODER_BUFFER_SIZE 4800
#define TUNNEL_DECODER_BUFFER_COUNT 256
// To accommodate worst size frame of  AC3 and DTS and meta data
#define TUNNEL_DECODER_BUFFER_SIZE_BROADCAST  9600
#define TUNNEL_DECODER_BUFFER_COUNT_BROADCAST 128
#define SIGNAL_EVENT_THREAD 2
#define SIGNAL_PLAYBACK_THREAD 2
//Values to exit poll via eventfd
#define KILL_EVENT_THREAD 1
#define POLL_FD_MODIFIED 3
#define KILL_PLAYBACK_THREAD 1
#define KILL_CAPTURE_THREAD 1
#define NUM_FDS 2
#define NUM_AUDIOSESSION_FDS 3
#define AFE_PROXY_SAMPLE_RATE 48000
#define AFE_PROXY_CHANNEL_COUNT 2
/*******************************************************************************
A2DP STATES
*******************************************************************************/
#define A2DP_RENDER_SETUP   0
#define A2DP_RENDER_START   1
#define A2DP_RENDER_STOP    2
#define A2DP_RENDER_SUSPEND 3
/*******************************************************************************
CHANNEL MAP
******************************************************************************/
#define PCM_CHANNEL_FL    1 /* Front left channel */
#define PCM_CHANNEL_FR    2 /* Front right channel. */
#define PCM_CHANNEL_FC    3 /* Front center channel. */
#define PCM_CHANNEL_LS    4 /* Left surround channel.*/
#define PCM_CHANNEL_RS    5 /* Right surround channel.*/
#define PCM_CHANNEL_LFE   6 /* Low frequency effect channel. */
#define PCM_CHANNEL_CS    7 /* Center surround channel; Rear center channel. */
#define PCM_CHANNEL_LB    8 /* Left back channel; Rear left channel. */
#define PCM_CHANNEL_RB    9 /* Right back channel; Rear right channel. */
#define PCM_CHANNEL_TS   10 /* Top surround channel. */
#define PCM_CHANNEL_CVH  11 /* Center vertical height channel.*/
#define PCM_CHANNEL_MS   12 /* Mono surround channel.*/
#define PCM_CHANNEL_FLC  13 /* Front left of center. */
#define PCM_CHANNEL_FRC  14 /* Front right of center. */
#define PCM_CHANNEL_RLC  15 /* Rear left of center. */
#define PCM_CHANNEL_RRC  16 /* Rear right of center. */
/*******************************************************************************
ADTS HEADER PARSING
*******************************************************************************/
//Required for ADTS Header Parsing
#define ADTS_HEADER_SYNC_RESULT 0xfff0
#define ADTS_HEADER_SYNC_MASK 0xfff6
/*******************************************************************************
HDMI and SPDIF Device Output format control
*******************************************************************************/
#define NUM_DEVICES_WITH_PP_PARAMS 2
#define HDMI_RX 0x8001
#define SECONDARY_I2S_RX 0x8002
//Param ID's
#define ADM_PP_PARAM_MUTE_ID 0
#define ADM_PP_PARAM_MUTE_LENGTH 3
#define ADM_PP_PARAM_LATENCY_ID 1
#define ADM_PP_PARAM_LATENCY_LENGTH 3
/*******************************************************************************
USECASES AND THE CORRESPONDING DEVICE FORMATS THAT WE SUPPORT IN HAL
*******************************************************************************/
/*
In general max of 2 for pass through. Additional 1 for handling transcode
as the existence of transcode is with a PCM handle followed by transcode handle
So, a (AC3/EAC3) pass through + trancode require - 1 for pas through, 1 - pcm and
1 - transcode
*/
#define NUM_DEVICES_SUPPORT_COMPR_DATA 2+1
#define NUM_SUPPORTED_CODECS           14
#define NUM_DECODE_PATH                6
#define NUM_COLUMN_FOR_INDEXING        2
#define NUM_STATES_FOR_EACH_DEVICE_FMT 3
#define DECODER_TYPE_IDX               0
#define ROUTE_FORMAT_IDX               1

#define MIN_SIZE_FOR_METADATA    64
#define NUM_OF_PERIODS           8
/*Period size to be a multiple of chanels * bitwidth,
So min period size = LCM (1,2...8) * 4*/
#define PERIOD_SIZE_COMPR        3360
#define MS11_INPUT_BUFFER_SIZE   1536

#define COMPR_INPUT_BUFFER_SIZE  (PERIOD_SIZE_COMPR - MIN_SIZE_FOR_METADATA)
#define PCM_16_BITS_PER_SAMPLE   2
#define PCM_24_BITS_PER_SAMPLE   3
#define AC3_PERIOD_SIZE          1536 * PCM_16_BITS_PER_SAMPLE
#define TIME_PER_BUFFER          20 //Time duration in ms
/*
List of indexes of the supported formats
Redundant formats such as (AAC-LC, HEAAC) are removed from the indexes as they
are treated with the AAC format
*/
enum {
    PCM_IDX = 0,
    AAC_IDX,
    AC3_IDX,
    EAC3_IDX,
    DTS_IDX,
    DTS_LBR_IDX,
    MP3_IDX,
    WMA_IDX,
    WMA_PRO_IDX,
    MP2_IDX,
    ALL_FORMATS_IDX
};
/*
List of pass through's supported in the current usecases
*/
enum {
    NO_PASSTHROUGH = 0,
    AC3_PASSTHR,
    EAC3_PASSTHR,
    DTS_PASSTHR
};
/*
List of transcoder's supported in the current usecases
*/
enum {
    NO_TRANSCODER = 0,
    AC3_TRANSCODER,
    DTS_TRANSCODER
};
/*
Requested end device format by user/app through set parameters
*/
enum {
    UNCOMPRESSED = 0,
    COMPRESSED,
    COMPRESSED_CONVERT_EAC3_AC3,
    COMPRESSED_CONVERT_ANY_AC3,
    COMPRESSED_CONVERT_ANY_DTS,
    ALL_DEVICE_FORMATS
};
/*
List of type of data routed on end device
*/
enum {
    ROUTE_NONE = 0,
    ROUTE_UNCOMPRESSED = 1,
    ROUTE_COMPRESSED = 2,
    ROUTE_SW_TRANSCODED_COMPRESSED = 4,
    ROUTE_DSP_TRANSCODED_COMPRESSED = 8
};
/*
List of end device formats
*/
enum {
    FORMAT_INVALID = -1,
    FORMAT_PCM,
    FORMAT_COMPR
};
/*
Below are the only different types of decode that we perform
*/
enum {
    DSP_DECODE = 1,      // render uncompressed
    DSP_PASSTHROUGH = 2, // render compressed
    DSP_TRANSCODE = 4,   // render as compressed
    SW_DECODE = 8,       // render as uncompressed
    SW_PASSTHROUGH = 16, // render compressed
    SW_TRANSCODE = 32    // render compressed
};
/*
Modes of buffering that we can support
As of now, we only support input buffering to an extent specified by usecase
*/
enum {
    NO_BUFFERING_MODE = 0,
    INPUT_BUFFERING_MODE,
    OUTPUT_BUFFEING_MODE
};
/*
playback controls
*/
enum {
    PLAY = 1,
    PAUSE = 1<<1,
    RESUME = 1<<2,
    SEEK = 1<<3,
    EOS = 1<<4,
    STOP = 1<<5,
    STANDBY = 1<<6,
    DEROUTE = 1<<7
};
/*
Multiple instance of use case
*/
enum {
    STEREO_DRIVER = 0,
    MULTI_CHANNEL_DRIVER,
    COMRPESSED_DRIVER,
};
/*
Instance bits
*/
enum {
    MULTI_CHANNEL_1_BIT = 1<<4,
    MULTI_CHANNEL_2_BIT = 1<<5,
    MULTI_CHANNEL_3_BIT = 1<<6,
    COMPRESSED_1_BIT    = 1<<12,
    COMPRESSED_2_BIT    = 1<<13,
    COMPRESSED_3_BIT    = 1<<14,
    COMPRESSED_4_BIT    = 1<<15,
    COMPRESSED_5_BIT    = 1<<16,
    COMPRESSED_6_BIT    = 1<<17
};
/*
List of support formats configured from frameworks
*/
const int supportedFormats[NUM_SUPPORTED_CODECS] = {
    AUDIO_FORMAT_PCM_16_BIT,
    AUDIO_FORMAT_PCM_24_BIT,
    AUDIO_FORMAT_AAC,
    AUDIO_FORMAT_HE_AAC_V1,
    AUDIO_FORMAT_HE_AAC_V2,
    AUDIO_FORMAT_AAC_ADIF,
    AUDIO_FORMAT_AC3,
    AUDIO_FORMAT_EAC3,
    AUDIO_FORMAT_DTS,
    AUDIO_FORMAT_DTS_LBR,
    AUDIO_FORMAT_MP3,
    AUDIO_FORMAT_WMA,
    AUDIO_FORMAT_WMA_PRO,
    AUDIO_FORMAT_MP2
};
/*
we can only have 6 types of decoder type stored with bit masks.
*/
const int routeToDriver[NUM_DECODE_PATH][NUM_COLUMN_FOR_INDEXING] = {
    {DSP_DECODE,      ROUTE_UNCOMPRESSED},
    {DSP_PASSTHROUGH, ROUTE_COMPRESSED},
    {DSP_TRANSCODE,   ROUTE_DSP_TRANSCODED_COMPRESSED},
    {SW_DECODE,       ROUTE_UNCOMPRESSED},
    {SW_PASSTHROUGH,  ROUTE_COMPRESSED},
    {SW_TRANSCODE,    ROUTE_SW_TRANSCODED_COMPRESSED}
};
/*
table to query index based on the format
*/
const int formatIndex[NUM_SUPPORTED_CODECS][NUM_COLUMN_FOR_INDEXING] = {
/*---------------------------------------------
|    FORMAT                  | INDEX           |
----------------------------------------------*/
    {AUDIO_FORMAT_PCM_16_BIT,  PCM_IDX},
    {AUDIO_FORMAT_PCM_24_BIT,  PCM_IDX},
    {AUDIO_FORMAT_AAC,         AAC_IDX},
    {AUDIO_FORMAT_HE_AAC_V1,   AAC_IDX},
    {AUDIO_FORMAT_HE_AAC_V2,   AAC_IDX},
    {AUDIO_FORMAT_AAC_ADIF,    AAC_IDX},
    {AUDIO_FORMAT_AC3,         AC3_IDX},
    {AUDIO_FORMAT_EAC3,        EAC3_IDX},
    {AUDIO_FORMAT_DTS,         DTS_IDX},
    {AUDIO_FORMAT_DTS_LBR,     DTS_LBR_IDX},
    {AUDIO_FORMAT_MP3,         MP3_IDX},
    {AUDIO_FORMAT_WMA,         WMA_IDX},
    {AUDIO_FORMAT_WMA_PRO,     WMA_PRO_IDX},
    {AUDIO_FORMAT_MP2,         MP2_IDX}
};
/*
Table to query non HDMI and SPDIF devices and their states such as type of
decode, type of data routed to end device and type of transcoding needed
*/
const int usecaseDecodeFormat[ALL_FORMATS_IDX*NUM_STATES_FOR_EACH_DEVICE_FMT] = {
/*-----------------
|    UNCOMPR      |
-----------------*/
/*      PCM    */
    DSP_DECODE,   //PCM_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    SW_DECODE,    // AAC_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    SW_DECODE,    //AC3_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    SW_DECODE,    //EAC3_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    DSP_DECODE,   //DTS_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    DSP_DECODE,   //DTS_LBR_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    DSP_DECODE,   //MP3_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    DSP_DECODE,   //WMA_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    DSP_DECODE,   //WMA_PRO_IDX
    FORMAT_PCM,   //ROUTE_FORMAT
    NO_TRANSCODER,//TRANSCODE_FORMAT
/*      PCM    */
    DSP_DECODE,  //MP2_IDX
    FORMAT_PCM,  //ROUTE_FORMAT
    NO_TRANSCODER//TRANSCODE_FORMAT
};
/*
Table to query HDMI and SPDIF devices and their states such as type of
decode, type of data routed to end device and type of transcoding needed
*/
const int usecaseDecodeHdmiSpdif[ALL_FORMATS_IDX*NUM_STATES_FOR_EACH_DEVICE_FMT]
                                [ALL_DEVICE_FORMATS] = {
/*--------------------------------------------------------------------------------
|   UNCOMPRESSED   |     COMPR      |  COMPR_CONV  | COMPR_CONV    |   COMPR_CONV    |
|                  |                |   EAC3_AC3   |   ANY_AC3     |     ANY_DTS     |
---------------------------------------------------------------------------------*/
/*   PCM            PCM              PCM             PCM             PCM       */
    {DSP_DECODE,    DSP_DECODE,      DSP_DECODE,     DSP_DECODE,     DSP_DECODE|DSP_TRANSCODE},     //PCM_IDX
    {FORMAT_PCM,    FORMAT_PCM,      FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, NO_TRANSCODER,   NO_TRANSCODER,  NO_TRANSCODER,  DTS_TRANSCODER}, //TRANSCODE_FMT
/*   PCM            PCM              PCM             AC3             PCM       */
    {SW_DECODE,     SW_DECODE,       SW_DECODE,      SW_TRANSCODE,   DSP_DECODE},     //AAC_IDX
    {FORMAT_PCM,    FORMAT_PCM,      FORMAT_PCM,     FORMAT_COMPR,   FORMAT_PCM},     //ROUTE_FORMAT
    {NO_TRANSCODER, NO_TRANSCODER,   NO_TRANSCODER,  AC3_TRANSCODER,  NO_TRANSCODER}, //TRANSCODE_FMT
/*   PCM            AC3              AC3             AC3             PCM       */
    {SW_DECODE,     SW_PASSTHROUGH,  SW_PASSTHROUGH, SW_PASSTHROUGH, DSP_DECODE},     //AC3_IDX
    {FORMAT_PCM,    FORMAT_COMPR,    FORMAT_COMPR,   FORMAT_COMPR,   FORMAT_PCM},     //ROUTE_FORMAT
    {NO_TRANSCODER, AC3_PASSTHR,     AC3_PASSTHR,    AC3_PASSTHR,    NO_TRANSCODER},  //TRANSCODE_FMT
/*   PCM            EAC3             AC3             AC3             PCM       */
    {SW_DECODE,     SW_PASSTHROUGH,  SW_TRANSCODE,   SW_TRANSCODE,   DSP_DECODE},     //EAC3_IDX
    {FORMAT_PCM,    FORMAT_COMPR,    FORMAT_COMPR,   FORMAT_COMPR,   FORMAT_PCM},     //ROUTE_FORMAT
    {NO_TRANSCODER, EAC3_PASSTHR,    AC3_TRANSCODER, AC3_TRANSCODER, NO_TRANSCODER},  //TRANSCODE_FMT
/*   PCM            DTS              PCM             PCM             DTS       */
    {DSP_DECODE,    DSP_PASSTHROUGH, DSP_DECODE,     DSP_DECODE,     DSP_PASSTHROUGH},//DTS_IDX
    {FORMAT_PCM,    FORMAT_COMPR,    FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, DTS_PASSTHR,     NO_TRANSCODER,  NO_TRANSCODER,  DTS_PASSTHR},    //TRANSCODE_FMT
/*   PCM            DTS_LBR          PCM             PCM             DTS       */
    {DSP_DECODE,    DSP_PASSTHROUGH, DSP_DECODE,     DSP_DECODE,     DSP_PASSTHROUGH},//DTS_LBR_IDX
    {FORMAT_PCM,    FORMAT_COMPR,    FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, DTS_PASSTHR,     NO_TRANSCODER,  NO_TRANSCODER,  DTS_PASSTHR},    //TRANSCODE_FMT
/*   PCM            PCM              PCM             PCM             DTS       */
    {DSP_DECODE,    DSP_DECODE,      DSP_DECODE,     DSP_DECODE,     DSP_DECODE|DSP_TRANSCODE},  //MP3_IDX
    {FORMAT_PCM,    FORMAT_PCM,      FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, NO_TRANSCODER,   NO_TRANSCODER,  NO_TRANSCODER,  DTS_TRANSCODER}, //TRANSCODE_FMT
/*   PCM            PCM              PCM             PCM             DTS       */
    {DSP_DECODE,    DSP_DECODE,      DSP_DECODE,     DSP_DECODE,     DSP_DECODE|DSP_TRANSCODE},  //WMA_IDX
    {FORMAT_PCM,    FORMAT_PCM,      FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, NO_TRANSCODER,   NO_TRANSCODER,  NO_TRANSCODER,  DTS_TRANSCODER}, //TRANSCODE_FMT
/*   PCM            PCM              PCM             PCM             DTS       */
    {DSP_DECODE,    DSP_DECODE,      DSP_DECODE,     DSP_DECODE,     DSP_DECODE|DSP_TRANSCODE},  //WMA_PRO_IDX
    {FORMAT_PCM,    FORMAT_PCM,      FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, NO_TRANSCODER,   NO_TRANSCODER,  NO_TRANSCODER,  DTS_TRANSCODER}, //TRANSCODE_FMT
/*   PCM            PCM              PCM             PCM             DTS       */
    {DSP_DECODE,    DSP_DECODE,      DSP_DECODE,     DSP_DECODE,     DSP_DECODE|DSP_TRANSCODE},  //MP2_IDX
    {FORMAT_PCM,    FORMAT_PCM,      FORMAT_PCM,     FORMAT_PCM,     FORMAT_COMPR},   //ROUTE_FORMAT
    {NO_TRANSCODER, NO_TRANSCODER,   NO_TRANSCODER,  NO_TRANSCODER,  DTS_TRANSCODER}  //TRANSCODE_FMT
};
/*
List of decoders which require config as part of first buffer
*/
const int decodersRequireConfig[] = {
    AUDIO_FORMAT_AAC,
    AUDIO_FORMAT_HE_AAC_V1,
    AUDIO_FORMAT_HE_AAC_V2,
    AUDIO_FORMAT_AAC_ADIF,
    AUDIO_FORMAT_WMA,
    AUDIO_FORMAT_WMA_PRO
};
/*
List of enum that are used in Broadcast.
NOTE: Need to be removed once broadcast is moved with updated states as above
*/
enum {
    INVALID_FORMAT               = -1,
    PCM_FORMAT                   = 0,
    COMPRESSED_FORMAT            = 1,
    COMPRESSED_FORCED_PCM_FORMAT = 2,
    COMPRESSED_PASSTHROUGH_FORMAT = 3
};

enum {
    STANDBY_LPCM_SPDIF  = 1,
    STANDBY_LPCM_HDMI   = 1<<1,
    STANDBY_COMPR_SPDIF = 1<<3,
    STANDBY_COMPR_HDMI  = 1<<4,
    STANDBY_LPCM_ALL    = STANDBY_LPCM_SPDIF | STANDBY_LPCM_HDMI,
    STANDBY_COMPR_ALL   = STANDBY_COMPR_SPDIF | STANDBY_COMPR_HDMI
};

/******************************************************************************
FLUENCE
*******************************************************************************/
static uint32_t FLUENCE_MODE_ENDFIRE   = 0;
static uint32_t FLUENCE_MODE_BROADSIDE = 1;
/******************************************************************************
STRUCTURES
*******************************************************************************/
/*
Alsa handle. Holds all the required states about the stream, device it is
mapped to and the correponding driver opened
*/
struct alsa_handle_t {
    ALSADevice*         module;
    uint32_t            devices;
    char                useCase[MAX_STR_LEN];
    uint8_t             type;            // Device Node Type, i.e. PCM/COMPRESSED/PASSTHROUGH
    struct pcm *        handle;
    snd_pcm_format_t    format;
    uint16_t            channels;
    uint16_t            timeStampMode;
    uint16_t            metaDataMode;
    uint32_t            sampleRate;
    int                 mode;            // Phone state i.e. incall/normal/incommunication
    unsigned int        latency;         // Delay in usec
    unsigned int        bufferSize;      // Size of sample buffer
    unsigned int        periodSize;
    struct pcm *        rxHandle;
    uint32_t            activeDevice;
    uint8_t             playbackMode;
    uint8_t             hdmiFormat;
    uint8_t             spdifFormat;
    snd_use_case_mgr_t  *ucMgr;
};
typedef List<alsa_handle_t> ALSAHandleList;
/*
Meta data structure for handling compressed read and input path
*/
struct compressed_read_metadata_t {
    uint32_t            streamId;
    uint32_t            formatId;
    uint32_t            dataOffset;
    uint32_t            frameSize;
    uint32_t            commandOffset;
    uint32_t            commandSize;
    uint32_t            encodedPcmSamples;
    uint32_t            timestampMsw;
    uint32_t            timestampLsw;
    uint32_t            flags;
};
struct input_metadata_handle_t {
    uint64_t            timestamp;
    uint32_t            bufferLength;
    uint32_t            consumedLength;
};
typedef List<input_metadata_handle_t> inputMetadataList;
/*
Meta data structure for handling compressed output
*/
struct output_metadata_handle_t {
    uint32_t            metadataLength;
    uint32_t            bufferLength;
    uint64_t            timestamp;
    uint32_t            reserved[12];
};
/*
use case string list
*/
struct use_case_t {
    char                useCase[MAX_STR_LEN];
};
typedef List<use_case_t> ALSAUseCaseList;


//returns the smallest number such that it is
//greater than n and is a multiple of m
inline int nextMultiple(int n, int m) {
    return ((n/m) + 1) * m;
}

/******************************************************************************
CLASS
*******************************************************************************/
// ----------------------------------------------------------------------------

class ALSADevice
{
public:
    ALSADevice();
    virtual                ~ALSADevice();

    //status_t    init(ALSADevice *module, ALSAHandleList &list);
    status_t    open(alsa_handle_t *handle);
    status_t    startVoipCall(alsa_handle_t *handle);
    status_t    startVoiceCall(alsa_handle_t *handle);
    status_t    startFm(alsa_handle_t *handle);
    status_t    startLoopback(alsa_handle_t *handle);
    status_t    setFmVolume(int value);
    status_t    setLpaVolume(int value);
    status_t    start(alsa_handle_t *handle);
    status_t    close(alsa_handle_t *handle);
    status_t    standby(alsa_handle_t *handle);
    status_t    route(uint32_t devices, int mode);
    status_t    setChannelStatus(unsigned char *channelStatus);
    void        disableDevice(alsa_handle_t *handle);
    char*       getUCMDevice(uint32_t devices, int input);
    void        setVoiceVolume(int vol);
    void        setVoipVolume(int vol);
    void        setMicMute(int state);
    void        setVoipMicMute(int state);
    void        setBtscoRate(int rate);
    void        enableWideVoice(bool flag);
    void        enableFENS(bool flag);
    void        setFlags(uint32_t flags);
    int         getUseCaseType(const char *useCase);
    int32_t     get_linearpcm_channel_status(uint32_t sampleRate,
                                             unsigned char *channel_status);
    int32_t     get_compressed_channel_status(void *audio_stream_data,
                                              uint32_t audio_frame_size,
                                              unsigned char *channel_status,
                                              enum audio_parser_code_type codec_type);

    status_t    setPlaybackVolume(alsa_handle_t *handle, int volume);
    status_t    setPlaybackFormat(int device, bool isCompressed);
    status_t    setCaptureFormat(const char *value);
    status_t    setChannelMap(alsa_handle_t *handle, int maxChannels,
                              char *channelMap);
    status_t    setDeviceMute(int device, int value);
    void        setChannelAlloc(int channelAlloc);
    status_t    setWMAParams(int[], int);
    int         getALSABufferSize(alsa_handle_t *handle);
    status_t    setHDMIChannelCount(int channels);
    void        switchDeviceUseCase(alsa_handle_t *handle, uint32_t devices,
                                            uint32_t mode);
    void        setDeviceList(ALSAHandleList *mDeviceList);
    int         setUseCase(alsa_handle_t *handle, bool bIsUseCaseSet);
    void        setUseCase(alsa_handle_t *handle, bool bIsUseCaseSet, char *device);
    void        removeUseCase(alsa_handle_t *handle, char *device);
    status_t    openCapture(alsa_handle_t *handle, bool isMmapMode,
                            bool isCompressed);
    status_t    openPlayback(alsa_handle_t *handle, bool isMmapMode);
    status_t    configureTranscode(alsa_handle_t *handle);
    void        updateHDMIEDIDInfo();
    int         getFormatHDMIIndexEDIDInfo(EDID_AUDIO_FORMAT_ID formatId);
    bool        isTunnelPlaybackUseCase(const char *useCase);
    bool        isMultiChannelPlaybackUseCase(const char *useCase);
    bool        isTunnelPseudoPlaybackUseCase(const char *useCase);
    char*       getPlaybackUseCase(int type, bool isModifier);
    void        freePlaybackUseCase(const char *useCase);
    void        setHdmiOutputProperties(int type);
    int         mSpdifFormat;
    int         mHdmiFormat;
    int         mSpdifOutputChannels;
    int         mHdmiOutputChannels;
    EDID_AUDIO_INFO mEDIDInfo;
    unsigned int mDriverInstancesUsed;
    status_t setPlaybackOutputDelay(int outputDevice, unsigned int delay);

protected:
    friend class AudioHardwareALSA;

private:
    status_t   openProxyDevice();
    status_t   closeProxyDevice();
    bool       isProxyDeviceOpened();
    bool       isProxyDeviceSuspended();
    bool       suspendProxy();
    bool       resumeProxy();
    void       resetProxyVariables();
    ssize_t    readFromProxy(void **captureBuffer , ssize_t *bufferSize);
    status_t   exitReadFromProxy();
    void       initProxyParams();
    status_t   startProxy();
    int        mapDeviceToPort(int device);

private:
    int         deviceName(alsa_handle_t *handle, unsigned flags, char **value);
    status_t    setHardwareParams(alsa_handle_t *handle);
    status_t    setSoftwareParams(alsa_handle_t *handle);
    void        switchDevice(uint32_t devices, uint32_t mode);
    status_t    getMixerControl(const char *name, unsigned int &value, int index);
    status_t    setMixerControl(const char *name, unsigned int value, int index);
    status_t    setMixerControl(const char *name, const char *value);
    status_t    setMixerControlExt(const char *name, int count, char **setValues);
    int         getDevices(uint32_t devices, uint32_t mode,
                                    char **rxDevice, char **txDevice);
    int         getDeviceType(uint32_t devices, uint32_t mode);
    void        enableDevice(alsa_handle_t *handle, bool bIsUseCaseSet);
    status_t    setCaptureHardwareParams(alsa_handle_t *handle, bool isCompressed);
    status_t    setCaptureSoftwareParams(alsa_handle_t *handle, bool isCompressed);
    status_t    setPlaybackHardwareParams(alsa_handle_t *handle);
    status_t    setPlaybackSoftwareParams(alsa_handle_t *handle);

    char        mic_type[128];
    char        curRxUCMDevice[50];
    char        curTxUCMDevice[50];
    int         fluence_mode;
    int         fmVolume;
    uint32_t    mDevSettingsFlag;
    int         btsco_samplerate;
    int         callMode;
    int         mWMA_params[8];

    struct mixer*  mMixer;
    ALSAUseCaseList mUseCaseList;
    ALSAHandleList  *mDeviceList;

    struct proxy_params {
        bool                mExitRead;
        struct pcm          *mProxyPcmHandle;
        uint32_t            mCaptureBufferSize;
        void                *mCaptureBuffer;
        enum {
            EProxyClosed    = 0,
            EProxyOpened    = 1,
            EProxySuspended = 2,
            EProxyCapture   = 3,
        };

        uint32_t mProxyState;
        unsigned mAvail;
        struct pollfd mPfdProxy[NUM_FDS];
        long mFrames;
        long mBufferTime;
    };
    struct proxy_params mProxyParams;
};
// ----------------------------------------------------------------------------
class ALSAControl
{
public:
    ALSAControl(const char *device = "/dev/snd/controlC0");
    virtual                ~ALSAControl();

    status_t                get(const char *name, unsigned int &value, int index = 0);
    status_t                set(const char *name, unsigned int value, int index = -1);
    status_t                set(const char *name, const char *);

private:
    struct mixer*             mHandle;
};

class ALSAStreamOps
{
public:
    ALSAStreamOps(AudioHardwareALSA *parent, alsa_handle_t *handle);
    virtual            ~ALSAStreamOps();

    status_t            set(int *format, uint32_t *channels, uint32_t *rate, uint32_t device);

    status_t            setParameters(const String8& keyValuePairs);
    String8             getParameters(const String8& keys);

    uint32_t            sampleRate() const;
    size_t              bufferSize() const;
    int                 format() const;
    uint32_t            channels() const;

    status_t            open(int mode);
    void                close();

protected:
    friend class AudioHardwareALSA;

    AudioHardwareALSA *     mParent;
    alsa_handle_t *         mHandle;
    uint32_t                mDevices;

    bool                    mPowerLock;
};
// ----------------------------------------------------------------------------
class AudioStreamOutALSA : public AudioStreamOut, public ALSAStreamOps
{
public:
    AudioStreamOutALSA(AudioHardwareALSA *parent, alsa_handle_t *handle);
    virtual            ~AudioStreamOutALSA();

    virtual uint32_t    sampleRate() const
    {
        return ALSAStreamOps::sampleRate();
    }

    virtual size_t      bufferSize() const
    {
        return ALSAStreamOps::bufferSize();
    }

    virtual uint32_t    channels() const;

    virtual int         format() const
    {
        return ALSAStreamOps::format();
    }

    virtual uint32_t    latency() const;

    virtual ssize_t     write(const void *buffer, size_t bytes);

    virtual status_t    dump(int fd, const Vector<String16>& args);

    status_t            setVolume(float left, float right);

    virtual status_t    standby();

    virtual status_t    setParameters(const String8& keyValuePairs);

    virtual String8     getParameters(const String8& keys);

    // return the number of audio frames written by the audio dsp to DAC since
    // the output has exited standby
    virtual status_t    getRenderPosition(uint32_t *dspFrames);

    status_t            open(int mode);
    status_t            close();

private:
    uint32_t            mFrameCount;
    alsa_handle_t *     mPcmRxHandle;
    alsa_handle_t *     mSpdifRxHandle;
    alsa_handle_t *     mCompreRxHandle;
    uint32_t            mA2dpUseCase;

protected:
    AudioHardwareALSA *     mParent;
};
// ----------------------------------------------------------------------------
class AudioSessionOutALSA : public AudioStreamOut
{
public:
    AudioSessionOutALSA(AudioHardwareALSA *parent, 
                        uint32_t   devices,
                        int        format,
                        uint32_t   channels,
                        uint32_t   samplingRate,
                        int        sessionId,
                        status_t   *status);
    virtual            ~AudioSessionOutALSA();

    virtual uint32_t    sampleRate() const
    {
        return mSampleRate;
    }

    virtual size_t      bufferSize() const
    {
        return mBufferSize;
    }

    virtual uint32_t    channels() const 
    {
        return mChannels;
    }

    virtual int         format() const
    {
        return mFormat;
    }

    virtual uint32_t    latency() const;

    virtual ssize_t     write(const void *buffer, size_t bytes);

    virtual status_t    start();
    virtual status_t    pause();
    virtual status_t    flush();
    virtual status_t    stop();

    virtual status_t    dump(int fd, const Vector<String16>& args);

    status_t            setVolume(float left, float right);

    virtual status_t    standby();

    virtual status_t    setParameters(const String8& keyValuePairs);

    virtual String8     getParameters(const String8& keys);

    // return the number of audio frames written by the audio dsp to DAC since
    // the output has exited standby
    virtual status_t    getRenderPosition(uint32_t *dspFrames);

    virtual status_t    getNextWriteTimestamp(int64_t *timeStamp);

    virtual status_t    setObserver(void *observer);

private:
    // mutex
    Mutex               mLock;
    Mutex               mControlLock;
    Mutex               mRoutingLock;
    Mutex               mFrameCountMutex;
    Mutex               mBitStreamMutex;
    Mutex               mFlushLock;
    Condition           mEOSCv;

    AudioHardwareALSA  *mParent;
    ALSADevice         *mALSADevice;
    snd_use_case_mgr_t *mUcMgr;

    // states
    int                 mDevices;
    int                 mFormat;
    uint32_t            mChannels;
    uint32_t            mSampleRate;
    uint32_t            mSessionId;
    uint32_t            mBufferSize;

    // device specifics
    int                 mSpdifOutputFormat;
    int                 mHdmiOutputFormat;
    int                 mSpdifFormat;
    int                 mHdmiFormat;
    int                 mStandByDevices;
    int                 mStandByFormats;
    bool                mConfiguringSessions;
    // decoder specifics
    int                 mDecoderType;
    bool                mDecoderConfigSet;
    bool                mUseMS11Decoder;
    SoftMS11            *mMS11Decoder;
    AudioBitstreamSM    *mBitstreamSM;
    uint32_t            mMinBytesReqToDecode;
    bool                mPaused;
    int                 mStreamVol;
    bool                mIsMS11FilePlaybackMode;
    void                *mDecoderConfigBuffer;
    int32_t             mDecoderConfigBufferLength;
    bool                mFirstBitstreamBuffer;
    // rendering
    int                 mFrameCount;
    // misc
    bool                mPowerLock;
    // routing
    uint32_t            mA2dpUseCase;
    bool                mRouteAudioToA2dp;
    bool                mOpenDecodeRoute;
    int                 mDecodeFormatDevices;
    bool                mOpenPassthroughRoute;
    int                 mPassthroughFormatDevices;
    bool                mOpenTranscodeRoute;
    int                 mTranscodeFormatDevices;
    int                 mRouteDecodeFormat;
    int                 mRoutePassthroughFormat;
    int                 mRouteTrancodeFormat;
    bool                mChannelStatusSet;
    unsigned char       mChannelStatus[24];
    char                *mWriteTempBuffer;
    AudioEventObserver  *mObserver;
        /*
        At any time, we can only route
        1. PCM on all devices
        2. PCM on few and Compressed(either DECODE/PASSTHROUGH or TRANSCODE) on others
        3. Compressed on all devices
        4. Compressed DECODE/PASSTHROUGH or TRANSCODE
        Hence atmost 2 handles are required
        so track states of the handles with corresponding format and devices
        */
    int                 mNumRxHandlesActive;
    alsa_handle_t       *mRxHandle[NUM_DEVICES_SUPPORT_COMPR_DATA];
    int                 mRxHandleRouteFormat[NUM_DEVICES_SUPPORT_COMPR_DATA];
    int                 mRxHandleDevices[NUM_DEVICES_SUPPORT_COMPR_DATA];
    int                 mRxHandleRouteFormatType[NUM_DEVICES_SUPPORT_COMPR_DATA];
    output_metadata_handle_t  mOutputMetadata;

    uint32_t            channelMapToChannels(uint32_t channelMap);
    bool                isSupportedFormat(int format);
    bool                isMS11SupportedFormats(int format);
    bool                canAc3PassThroughWithoutMS11(int format);
    int                 getFormatIndex();
    void                fixUpDevicesForA2DPPlayback();
    int                 getIndexHandleBasedOnHandleFormat(int handleFormat);
    int                 getIndexHandleBasedOnHandleDevices(int handleDevices);
    void                updateDeviceSupportedFormats();
    void                fixUpHdmiFormatBasedOnEDID();
    void                initialize();
    status_t            flush_l();
    void                reset();
    void                reinitialize();
    void                fixupSampleRateChannelModeMS11Formats();
    void                updateDecodeTypeAndRoutingStates();
    void                updateRxHandleStates();
    int                 getDeviceFormat(int devices);
    bool                isDecoderConfigRequired();
    bool                isInputBufferingModeReqd();
    int                 getBufferingFactor();
    status_t            routingSetup();
    status_t            openMS11Instance();
    status_t            openPlaybackDevice(int index, int devices, int deviceFormat);
    status_t            openDevice(const char *useCase, bool bIsUseCase,
                                   int devices, int deviceFormat);
    void                getPeriodSizeCountAndFormat(int routeFormat, int *periodSize,
                                                    int *periodCount, int *format);
    int                 getBufferLength();
    status_t            openTempBufForMetadataModeRendering();
    status_t            closeDevice(alsa_handle_t *pHandle);
    status_t            a2dpRenderingControl(int state);
    void                setMS11ChannelMap(alsa_handle_t *handle);
    void                setPCMChannelMap(alsa_handle_t *handle);
    int                 setDecodeConfig(char *buffer, size_t bytes);
    void                copyBitstreamInternalBuffer(char *buffer, size_t bytes);
    bool                decode(char *buffer, size_t bytes);
    bool                swDecode(char *buffer, size_t bytes);
    bool                dspDecode(char *buffer, size_t bytes);
    void                setSpdifChannelStatus(char *buffer, size_t bytes,
                                              audio_parser_code_type codec_type);
    uint32_t            render(bool continueDecode);
    void                eosHandling();
    status_t            doRouting(int device);
    void                adjustRxHandleAndStates();
    void                resetRxHandleState(int index);
    void                handleSwitchAndOpenForDeviceSwitch(int devices, int format);
    void                handleCloseForDeviceSwitch(int format);
    void                updateSessionDevices(int device);
    void                updateDevicesInSessionList(int devices, int state);
    void                updateStandByDevices(int device, int enable);
    bool                isDeviceinStandByFormats(int devices);
#ifdef DEBUG
    enum {
        INPUT = 0,
        OUTPUT
    };
    FILE                *mFpDumpInput;
    FILE                *mFpDumpPCMOutput;
    void                updateDumpWithPlaybackControls(int controlType);
    void                dumpInputOutput(int type, char *buffer, size_t bytes);
#endif
};
// ----------------------------------------------------------------------------
class AudioBroadcastStreamALSA : public AudioBroadcastStream
{
public:
    AudioBroadcastStreamALSA(AudioHardwareALSA *parent,
                             uint32_t  devices,
                             int      format,
                             uint32_t channels,
                             uint32_t sampleRate,
                             uint32_t audioSource,
                             status_t *status);

    virtual            ~AudioBroadcastStreamALSA();

    virtual uint32_t    sampleRate() const
    {
        return mSampleRate;
    }

    virtual size_t      bufferSize() const
    {
        return mBufferSize;
    }

    virtual uint32_t    channels() const
    {
        return mChannels;
    }

    virtual int         format() const
    {
        return mFormat;
    }

    virtual uint32_t    latency() const;

    virtual ssize_t     write(const void *buffer, size_t bytes, int64_t timestamp, int audiotype);

    virtual status_t    dump(int fd, const Vector<String16>& args);

    virtual status_t    start(int64_t absTimeToStart);

    virtual status_t    mute(bool mute);

    status_t            setVolume(float left, float right);

    virtual status_t    standby();

    virtual status_t    setParameters(const String8& keyValuePairs);

    virtual String8     getParameters(const String8& keys);

    // return the number of audio frames written by the audio dsp to DAC since
    // the output has exited standby
    virtual status_t    getRenderPosition(uint32_t *dspFrames);

private:
    Mutex               mLock;
    Mutex               mSyncLock;
    Mutex               mControlLock;
    uint32_t            mFrameCount;
    uint32_t            mSampleRate;
    uint32_t            mChannels;
    int                 mInputFormat;
    int                 mFormat;
    int                 mDevices;
    uint32_t            mAudioSource;
    uint32_t            mStreamVol;
    size_t              mBufferSize;
    int                 mCurDevice;
    int                 mTranscodeDevices;
    // Capture and Routing Flags
    bool                mCapturePCMFromDSP;
    bool                mCaptureCompressedFromDSP;
    bool                mRoutePCMStereoToDSP;
    bool                mRoutePCMMChToDSP;
    bool                mUseMS11Decoder;
    bool                mUseTunnelDecoder;
    bool                mRoutePcmAudio;
    bool                mRoutingSetupDone;
    bool                mSignalToSetupRoutingPath;
    int                 mInputBufferSize;
    int                 mInputBufferCount;
    int32_t             mMinBytesReqToDecode;
    int64_t             hw_ptr;
    bool                mDtsTranscode;
    compressed_read_metadata_t mReadMetaData;
    // HDMI and SPDIF specifics
    int32_t             mSpdifFormat;
    int32_t             mHdmiFormat;
    char                mSpdifOutputFormat[128];
    char                mHdmiOutputFormat[128];
    unsigned char       mChannelStatus[24];
    bool                mChannelStatusSet;
    // Decoder Specifics
    bool                mAacConfigDataSet;
    bool                mWMAConfigDataSet;
    bool                mDDFirstFrameBuffered;
    bool                mPlaybackReachedEOS;
    bool                isSessionPaused;

    bool                mRouteAudioToA2dp;
    bool                mCaptureFromProxy;
    // Avsync Specifics
    bool                mTimeStampModeSet;
    uint32_t            mCompleteBufferTimePcm;
    uint32_t            mCompleteBufferTimeCompre;
    uint32_t            mPartialBufferTimePcm;
    uint32_t            mPartialBufferTimeCompre;
    uint32_t            mOutputMetadataLength;
    inputMetadataList   mInputMetadataListPcm;
    inputMetadataList   mInputMetadataListCompre;
    output_metadata_handle_t mOutputMetadataPcm;
    output_metadata_handle_t mOutputMetadataCompre;
    char                *mPcmWriteTempBuffer;
    char                *mCompreWriteTempBuffer;
    uint32_t            mA2dpUseCase;

    // ALSA device handle to route PCM 2.0 playback
    alsa_handle_t      *mPcmRxHandle;
    // ALSA device handle to Compressed audio playback
    alsa_handle_t      *mCompreRxHandle;
    // ALSA device handle to PCM 2.0 capture from DSP
    alsa_handle_t      *mPcmTxHandle;
    // ALSA device handle to Compressed audio capture from DSP
    alsa_handle_t      *mCompreTxHandle;
    // Common handle to handle the Capture thread
    alsa_handle_t      *mCaptureHandle;
    //ALSA device handle to route the DTS transcoded stream to selected devices
    alsa_handle_t *     mTranscodeHandle;
    // Open the MS11 decoder for AAC, AC3 and EC3
    SoftMS11           *mMS11Decoder;
    // Bitstream state machine to handle the buffering on input
    // and output
    AudioBitstreamSM   *mBitstreamSM;

    ALSADevice *        mALSADevice;
    snd_use_case_mgr_t *mUcMgr;
    AudioEventObserver *mObserver;


    //Declare all the threads
    //Capture
    pthread_t           mCaptureThread;
    Mutex               mCaptureMutex;
    Condition           mCaptureCv;
    struct              pollfd mCapturePfd[NUM_FDS];
    unsigned            mAvail;
    long                mFrames;
    int                 mCapturefd;
    bool                mKillCaptureThread;
    bool                mCaptureThreadAlive;
    bool                mExitReadCapturePath;

    //Playback
    pthread_t           mPlaybackThread;
    Mutex               mPlaybackMutex;
    Condition           mPlaybackCv;
    struct              pollfd mPlaybackPfd[NUM_FDS];
    Mutex               mInputMemMutex;
    Mutex               mWriteCvMutex;
    Mutex               mRoutingLock;
    Condition           mWriteCv;
    int                 mPlaybackfd;
    bool                mKillPlaybackThread;
    bool                mPlaybackThreadAlive;
    bool                mSkipWrite;


    void                updateSampleRateChannelMode();
    void                initialization();
    void                updateOutputFormat();
    status_t            openCapturingAndRoutingDevices();
    status_t            closeDevice(alsa_handle_t *pHandle);
    status_t            doRouting(int devices);
    status_t            pause_l();
    status_t            resume_l();
    //Capture
    void                setCaptureFlagsBasedOnConfig();
    status_t            openPCMCapturePath();
    status_t            openCompressedCapturePath();
    status_t            createCaptureThread();
    void                captureThreadEntry();
    static void *       captureThreadWrapper(void *me);
    void                allocateCapturePollFd();
    status_t            startCapturePath();
    status_t            doRoutingSetup();
    ssize_t             readFromCapturePath(char *buffer);
    void                resetCapturePathVariables();
    void                exitFromCaptureThread();
    uint32_t            read4BytesFromBuffer(char *buf);
    void                updateCaptureMetaData(char *buf);
    void                updatePCMCaptureMetaData(char *buf);
    // Playback
    void                setRoutingFlagsBasedOnConfig();
    void                setSpdifHdmiRoutingFlags(int devices);
    status_t            openRoutingDevice(char *useCase, bool bIsUseCase,
                                          int devices);
    status_t            openPcmDevice(int devices);
    status_t            openTunnelDevice(int devices);
    void                bufferAlloc(alsa_handle_t *handle);
    void                bufferDeAlloc();
    status_t            createPlaybackThread();
    void                playbackThreadEntry();
    static void *       playbackThreadWrapper(void *me);
    void                allocatePlaybackPollFd();
    status_t            openMS11Instance();
    ssize_t             write_l(char *buffer, size_t bytes);
    void                copyBitstreamInternalBuffer(char *buffer, size_t bytes);
    uint32_t            setDecoderConfig(char *buffer, size_t bytes);
    bool                decode(char *buffer, size_t bytes);
    uint32_t            render(bool continueDecode);
    int32_t             writeToCompressedDriver(char *buffer, int bytes);
    void                resetPlaybackPathVariables();
    void                exitFromPlaybackThread();
    // Avsync Specifics
    void                update_input_meta_data_list_pre_decode(uint32_t type);
    void                update_input_meta_data_list_post_decode(uint32_t type,
                            uint32_t bytesConsumedInDecode);
    void                update_time_stamp_pre_write_to_driver(uint32_t type);
    void                update_time_stamp_post_write_to_driver(uint32_t type,
                           uint32_t remainingSamples,
                           uint32_t requiredBufferSize);
    void                update_input_meta_data_list_post_write(uint32_t type);

    //Structure to hold mem buffer information
    class BuffersAllocated {
        public:
            BuffersAllocated(void *buf1, int32_t nSize) :
                 memBuf(buf1), memBufsize(nSize), bytesToWrite(0) {}
            void* memBuf;
            int32_t memBufsize;
            uint32_t bytesToWrite;
    };
    List<BuffersAllocated> mInputMemEmptyQueue;
    List<BuffersAllocated> mInputMemFilledQueue;
    List<BuffersAllocated> mInputBufPool;

protected:
    AudioHardwareALSA  *mParent;
    bool                mPowerLock;
};

class AudioStreamInALSA : public AudioStreamIn, public ALSAStreamOps
{
public:
    AudioStreamInALSA(AudioHardwareALSA *parent,
            alsa_handle_t *handle,
            AudioSystem::audio_in_acoustics audio_acoustics);
    virtual            ~AudioStreamInALSA();

    virtual uint32_t    sampleRate() const
    {
        return ALSAStreamOps::sampleRate();
    }

    virtual size_t      bufferSize() const
    {
        return ALSAStreamOps::bufferSize();
    }

    virtual uint32_t    channels() const
    {
        return ALSAStreamOps::channels();
    }

    virtual int         format() const
    {
        return ALSAStreamOps::format();
    }

    virtual ssize_t     read(void* buffer, ssize_t bytes);
    virtual status_t    dump(int fd, const Vector<String16>& args);

    virtual status_t    setGain(float gain);

    virtual status_t    standby();

    virtual status_t    setParameters(const String8& keyValuePairs)
    {
        return ALSAStreamOps::setParameters(keyValuePairs);
    }

    virtual String8     getParameters(const String8& keys)
    {
        return ALSAStreamOps::getParameters(keys);
    }

    // Return the amount of input frames lost in the audio driver since the last call of this function.
    // Audio driver is expected to reset the value to 0 and restart counting upon returning the current value by this function call.
    // Such loss typically occurs when the user space process is blocked longer than the capacity of audio driver buffers.
    // Unit: the number of input audio frames
    virtual unsigned int  getInputFramesLost() const;

    virtual status_t addAudioEffect(effect_handle_t effect)
    {
        return BAD_VALUE;
    }

    virtual status_t removeAudioEffect(effect_handle_t effect)
    {
        return BAD_VALUE;
    }
    status_t            setAcousticParams(void* params);

    status_t            open(int mode);
    status_t            close();

private:
    void                resetFramesLost();

    unsigned int        mFramesLost;
    AudioSystem::audio_in_acoustics mAcoustics;

protected:
    AudioHardwareALSA *     mParent;
};
// ----------------------------------------------------------------------------
class AudioHardwareALSA : public AudioHardwareBase
{
public:
    AudioHardwareALSA();
    virtual            ~AudioHardwareALSA();

    /**
     * check to see if the audio hardware interface has been initialized.
     * return status based on values defined in include/utils/Errors.h
     */
    virtual status_t    initCheck();

    /** set the audio volume of a voice call. Range is between 0.0 and 1.0 */
    virtual status_t    setVoiceVolume(float volume);

    /**
     * set the audio volume for all audio activities other than voice call.
     * Range between 0.0 and 1.0. If any value other than NO_ERROR is returned,
     * the software mixer will emulate this capability.
     */
    virtual status_t    setMasterVolume(float volume);
    virtual status_t    setFmVolume(float volume);

    /**
     * setMode is called when the audio mode changes. NORMAL mode is for
     * standard audio playback, RINGTONE when a ringtone is playing, and IN_CALL
     * when a call is in progress.
     */
    virtual status_t    setMode(int mode);

    // mic mute
    virtual status_t    setMicMute(bool state);
    virtual status_t    getMicMute(bool* state);

    // set/get global audio parameters
    virtual status_t    setParameters(const String8& keyValuePairs);
    virtual String8     getParameters(const String8& keys);

    // Returns audio input buffer size according to parameters passed or 0 if one of the
    // parameters is not supported
    virtual size_t    getInputBufferSize(uint32_t sampleRate, int format, int channels);
#ifdef QCOM_TUNNEL_LPA_ENABLED
    /** This method creates and opens the audio hardware output
      *  session for LPA */
    virtual AudioStreamOut* openOutputSession(
            uint32_t devices,
            int      *format,
            status_t *status,
            int      sessionId,
            uint32_t samplingRate,
            uint32_t channels);
    virtual void closeOutputSession(AudioStreamOut* out);
#endif
    /** This method creates and opens the audio hardware output stream */
    virtual AudioStreamOut* openOutputStream(
            uint32_t devices,
            int      *format=0,
            uint32_t *channels=0,
            uint32_t *sampleRate=0,
            status_t *status=0);
    virtual    void        closeOutputStream(AudioStreamOut* out);

    /** This method creates and opens the audio hardware output stream */
    virtual AudioBroadcastStream* openBroadcastStream(
            uint32_t  devices,
            int      format=0,
            uint32_t channels=0,
            uint32_t sampleRate=0,
            uint32_t audioSource=0,
            status_t *status=0);
    virtual    void        closeBroadcastStream(AudioBroadcastStream* out);

    /** This method creates and opens the audio hardware input stream */
    virtual AudioStreamIn* openInputStream(
            uint32_t devices,
            int      *format,
            uint32_t *channels,
            uint32_t *sampleRate,
            status_t *status,
            AudioSystem::audio_in_acoustics acoustics);
    virtual    void        closeInputStream(AudioStreamIn* in);

    status_t    startA2dpPlayback(uint32_t activeUsecase);
    status_t    stopA2dpPlayback(uint32_t activeUsecase);
    bool        suspendA2dpPlayback(uint32_t activeUsecase);

    status_t    startA2dpPlayback_l(uint32_t activeUsecase);
    status_t    stopA2dpPlayback_l(uint32_t activeUsecase);
    bool        suspendA2dpPlayback_l(uint32_t activeUsecase);
    int         buffer_data(struct pcm *pcm, void *data, unsigned count);
    int         is_buffer_available(struct pcm *pcm, void *data, int count, int format);
    int         hw_pcm_write(struct pcm *pcm, void *data, unsigned count);
    void        updateDevicesOfOtherSessions(int device, int state);
    int         getUnComprDeviceInCurrDevices(int devices);

    /**This method dumps the state of the audio hardware */
    //virtual status_t dumpState(int fd, const Vector<String16>& args);

    static AudioHardwareInterface* create();

    int                 mode()
    {
        return mMode;
    }

private:
    status_t     openA2dpOutput();
    status_t     closeA2dpOutput();
    status_t     stopA2dpThread();
    void       a2dpThreadFunc();
    static void*        a2dpThreadWrapper(void *context);
    void        setA2DPActiveUseCases_l(uint32_t activeUsecase);
    uint32_t    getA2DPActiveUseCases_l();
    void        clearA2DPActiveUseCases_l(uint32_t activeUsecase);
    uint32_t    useCaseStringToEnum(const char *usecase);
    void        standbySessionDevice(int device);
protected:
    virtual status_t    dump(int fd, const Vector<String16>& args);
    status_t            doRouting(int device);

    friend class AudioBroadcastStreamALSA;
    friend class AudioSessionOutALSA;
    friend class AudioStreamOutALSA;
    friend class AudioStreamInALSA;
    friend class ALSAStreamOps;

    ALSADevice*         mALSADevice;
    snd_use_case_mgr_t *mUcMgr;
    ALSAHandleList      mDeviceList;
    Mutex               mLock;
    Mutex               mDeviceStateLock;

    uint32_t            mCurDevice;
    uint32_t            mCurMode;
    /* The flag holds all the audio related device settings from
     * Settings and Qualcomm Settings applications */
    uint32_t            mDevSettingsFlag;
    uint32_t            mVoipStreamCount;
    bool                mVoipMicMute;
    uint32_t            mIncallMode;

    bool                mMicMute;
    int                 mIsVoiceCallActive;
    int                 mIsFmActive;
    bool                mBluetoothVGS;
    int                 mSpdifOutputFormat;
    int                 mHdmiOutputFormat;
    bool                mSpdifMuteOn;
    bool                mHdmiMuteOn;
    int                 mSpdifOutputChannels;
    int                 mHdmiOutputChannels;
    int                 mSpdifRenderFormat;
    int                 mHdmiRenderFormat;

    //A2DP variables
    audio_stream_out   *mA2dpStream;
    audio_hw_device_t  *mA2dpDevice;

    volatile bool       mKillA2DPThread;
    volatile bool       mA2dpThreadAlive;
    pthread_t           mA2dpThread;
    Mutex               mA2dpMutex;
    Condition           mA2dpCv;
    volatile bool       mIsA2DPEnabled;
    volatile bool       mIsA2DPSuspended;

    enum {
      USECASE_NONE = 0x00,
      USECASE_HIFI = 0x01,
      USECASE_HIFI2 = 0x0100,
      USECASE_HIFI3 = 0x0200,
      USECASE_HIFI4 = 0x0400,
      USECASE_HIFI_TUNNEL1 = 0x001000,
      USECASE_HIFI_TUNNEL2 = 0x002000,
      USECASE_HIFI_TUNNEL3 = 0x004000,
      USECASE_HIFI_TUNNEL4 = 0x008000,
      USECASE_HIFI_TUNNEL5 = 0x010000,
      USECASE_HIFI_TUNNEL6 = 0x020000,
      USECASE_FM = 0x01000000,
      USECASE_HWOUTPUT = 0x11,
    };
    uint32_t mA2DPActiveUseCases;
    List <AudioStreamOut *> mSessions;
public:
    bool mRouteAudioToA2dp;
};
// ----------------------------------------------------------------------------
class AudioBitstreamSM
{
public:
    AudioBitstreamSM();
    ~AudioBitstreamSM();
    bool    initBitstreamPtr();
    bool    initBitstreamPtr(int bufferingFactor);
    void    resetBitstreamPtr();
    void    copyBitstreamToInternalBuffer(char *bufPtr, size_t bytes);
    bool    sufficientBitstreamToDecode(size_t minThreshBytesToDecode);
    bool    sufficientSamplesToRender(int format, int minSizeReqdToRender);
    char*   getInputBufferPtr();
    char*   getInputBufferWritePtr();
    char*   getOutputBufferPtr (int format);
    size_t  bitStreamBufSize();
    void    copyResidueBitstreamToStart(size_t bytesConsumedInDecode);
    void    copyResidueOutputToStart(int format, size_t samplesRendered);
    char*   getOutputBufferWritePtr(int format);
    void    setOutputBufferWritePtr(int format, size_t outputPCMSample);
    void    appendSilenceToBitstreamInternalBuffer(size_t bytes, unsigned char);
    void    resetOutputBitstreamPtr();
    void    startInputBufferingMode();
    void    stopInputBufferingMode();
private:
    int                mBufferingFactor;
    int                mBufferingFactorCnt;
    // Buffer pointers for input and output
    char               *mInputBuffer;
    char               *mInputBufferCurrPtr;
    char               *mInputBufferWritePtr;

    char               *mEncOutputBuffer;
    char               *mEncOutputBufferWritePtr;

    char               *mPCM2ChOutputBuffer;
    char               *mPCM2ChOutputBufferWritePtr;

    char               *mPCMMChOutputBuffer;
    char               *mPCMMChOutputBufferWritePtr;

    char               *mPassthroughOutputBuffer;
    char               *mPassthroughOutputBufferWritePtr;
};
// ----------------------------------------------------------------------------

};        // namespace android_audio_legacy
#endif    // ANDROID_AUDIO_HARDWARE_ALSA_H
