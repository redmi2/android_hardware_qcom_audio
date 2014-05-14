/* ALSADevice.cpp
 **
 ** Copyright (c) 2011-2013, The Linux Foundation. All rights reserved
 ** Not a Contribution.
 **
 ** Copyright 2009 Wind River Systems
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

#define LOG_TAG "ALSADevice"
#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0
#define LOG_NIDEBUG 0
#include <utils/Log.h>
#include <cutils/properties.h>
#include <linux/ioctl.h>
#include "AudioUtil.h"
#include "AudioHardwareALSA.h"
#include <media/AudioRecord.h>

#define BTSCO_RATE_16KHZ 16000
#define USECASE_TYPE_RX 1
#define USECASE_TYPE_TX 2
#define DEVICE_TYPE_RX 1
#define DEVICE_TYPE_TX 2
#define MAX_HDMI_CHANNEL_CNT 8
#define MAX_NO_BITS 63

#define AFE_PROXY_PERIOD_SIZE 3072
#define KILL_A2DP_THREAD 1
#define SIGNAL_A2DP_THREAD 2
#define PROXY_CAPTURE_DEVICE_NAME (const char *)("hw:0,8")

#define MAX_AUDIO_OUTPUT_DELAY 100 /* Max audio delay (ms) supported by ADSP */
#define MIN_AUDIO_OUTPUT_DELAY 0   /* Min audio delay (ms) supported by ADSP */


namespace sys_close {
    ssize_t lib_close(int fd) {
        return close(fd);
    }
};


namespace android_audio_legacy
{

const int deviceToPortID[][NUM_DEVICES_WITH_PP_PARAMS] = {
        {AudioSystem::DEVICE_OUT_AUX_DIGITAL, HDMI_RX},
        {AudioSystem::DEVICE_OUT_SPDIF, SECONDARY_I2S_RX}
};

ALSADevice::ALSADevice() {
    mDevSettingsFlag = TTY_OFF;
    btsco_samplerate = 8000;
    int callMode = AudioSystem::MODE_NORMAL;

    char value[128];
    char platform[10];
    FILE *fp;
    property_get("persist.audio.handset.mic",value,"0");
    strlcpy(mic_type, value, sizeof(mic_type));
    property_get("persist.audio.fluence.mode",value,"0");
    if (!strncmp("broadside", value,9)) {
        fluence_mode = FLUENCE_MODE_BROADSIDE;
    } else {
        fluence_mode = FLUENCE_MODE_ENDFIRE;
    }
    strlcpy(curRxUCMDevice, "None", sizeof(curRxUCMDevice));
    strlcpy(curTxUCMDevice, "None", sizeof(curTxUCMDevice));

    if ((fp = fopen("/sys/devices/system/soc/soc0/hw_platform", "r")) != NULL) {
        fscanf(fp, "%s", platform);
        fclose(fp);
    }
    if(!strncmp(platform, "DTV", sizeof(platform)))
        mHardwarePlatform = DTV_PLATFORM;
    else
       mHardwarePlatform = DEFAULT_PLATFORM;

    mMixer = mixer_open("/dev/snd/controlC0");
    mProxyParams.mExitRead = false;
    mProxyParams.mAvail = 0;
    mProxyParams.mFrames = 0;
    mProxyParams.mPfdProxy[1].fd = -1;
    mProxyParams.mCaptureBufferSize = AFE_PROXY_PERIOD_SIZE;
    mProxyParams.mCaptureBuffer = NULL;
    mProxyParams.mProxyState = proxy_params::EProxyClosed;
    mProxyParams.mProxyPcmHandle = NULL;
    memset(&mEDIDInfo, 0, sizeof(struct EDID_AUDIO_INFO));
    ALOGD("ALSA Device opened");
};

ALSADevice::~ALSADevice()
{
    if (mMixer) mixer_close(mMixer);
    if(mProxyParams.mCaptureBuffer != NULL) {
        free(mProxyParams.mCaptureBuffer);
        mProxyParams.mCaptureBuffer = NULL;
    }
    mProxyParams.mProxyState = proxy_params::EProxyClosed;
}

// ----------------------------------------------------------------------------

int ALSADevice::deviceName(alsa_handle_t *handle, unsigned flags, char **value)
{
    ALOGV("deviceName");
    int ret = 0;
    char ident[70];
    char *rxDevice, useCase[70];

    if (flags & PCM_IN) {
        strlcpy(ident, "CapturePCM/", sizeof(ident));
    } else {
        strlcpy(ident, "PlaybackPCM/", sizeof(ident));
    }
    strlcat(ident, handle->useCase, sizeof(ident));
    ret = snd_use_case_get(handle->ucMgr, ident, (const char **)value);
    ALOGD("Device value returned is %s", (*value));
    return ret;
}

status_t ALSADevice::setHardwareParams(alsa_handle_t *handle)
{
    struct snd_pcm_hw_params *params;
    struct snd_compr_caps compr_cap;
    struct snd_compr_params compr_params;

    int32_t minPeroid, maxPeroid;
    unsigned long bufferSize, reqBuffSize;
    unsigned int periodTime, bufferTime;

    int status = 0;
    status_t err = NO_ERROR;
    unsigned int requestedRate = handle->sampleRate;
    int format = handle->format;

    char dtsModelId[128];
    int hdmiChannels = 8;
    char dtsPPparam[20];
    int dtsPPval = -1;

    if (handle->activeDevice & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
            int channel_count = 0;
            updateHDMIEDIDInfo();
            for (int i = 0; i < mEDIDInfo.nAudioBlocks && i < MAX_EDID_BLOCKS; i++) {
                if (mEDIDInfo.AudioBlocksArray[i].nFormatId == LPCM &&
                    mEDIDInfo.AudioBlocksArray[i].nChannels > channel_count &&
                    mEDIDInfo.AudioBlocksArray[i].nChannels <= MAX_HDMI_CHANNEL_CNT) {
                    channel_count = mEDIDInfo.AudioBlocksArray[i].nChannels;
                }
            }
            hdmiChannels = channel_count;
            pcm_set_channel_map(NULL, mMixer, MAX_HDMI_CHANNEL_CNT, mEDIDInfo.channelMap);
            if(hdmiChannels == 2)
                setChannelAlloc(0x0);
            else
                setChannelAlloc(mEDIDInfo.channelAllocation);
    }

    params = (snd_pcm_hw_params*) calloc(1, sizeof(struct snd_pcm_hw_params));
    if (!params) {
        return NO_INIT;
    }
    param_init(params);

    reqBuffSize = handle->bufferSize;
    ALOGD("Handle type %x", (int)handle->type);
    if (handle->type & COMPRESSED_FORMAT) {
        if (ioctl(handle->handle->fd, SNDRV_COMPRESS_GET_CAPS, &compr_cap)) {
            ALOGE("SNDRV_COMPRESS_GET_CAPS, failed Error no %d \n", errno);
            err = -errno;
            if (params)
                free(params);
            return err;
        }

        param_set_int(params, SNDRV_PCM_HW_PARAM_PERIODS,
                      (handle->bufferSize/handle->periodSize));
        minPeroid = compr_cap.min_fragment_size;
        maxPeroid = compr_cap.max_fragment_size;
        ALOGV("Min peroid size = %d , Maximum Peroid size = %d format = %d",\
            minPeroid, maxPeroid, format);
        //TODO: what if codec not supported or the array has wrong codec!!!!
        if (format == AUDIO_FORMAT_WMA || format == AUDIO_FORMAT_WMA_PRO) {
            ALOGV("WMA CODEC");
            if (format == AUDIO_FORMAT_WMA_PRO) {
                compr_params.codec.id = compr_cap.codecs[4];
            }
            else {
                compr_params.codec.id = compr_cap.codecs[3];
            }
            if (mWMA_params == NULL) {
                ALOGV("WMA param config missing.");
                if (params)
                    free(params);
                return BAD_VALUE;
            }
            compr_params.codec.bit_rate = mWMA_params[0];
            compr_params.codec.align = mWMA_params[1];
            compr_params.codec.options.wma.encodeopt = mWMA_params[2];
            compr_params.codec.format = mWMA_params[3];
            compr_params.codec.options.wma.bits_per_sample = mWMA_params[4];
            compr_params.codec.options.wma.channelmask = mWMA_params[5];
            compr_params.codec.options.wma.encodeopt1 = mWMA_params[6];
            compr_params.codec.options.wma.encodeopt2 = mWMA_params[7];
            compr_params.codec.sample_rate = handle->sampleRate;
            compr_params.codec.ch_in = handle->channels;
        } else if(format == AUDIO_FORMAT_AAC || format == AUDIO_FORMAT_HE_AAC_V1 ||
           format == AUDIO_FORMAT_HE_AAC_V2 || format == AUDIO_FORMAT_AAC_ADIF) {
            ALOGV("AAC CODEC");
            compr_params.codec.id = compr_cap.codecs[2];
            hdmiChannels = 2;
        } else if(format == AUDIO_FORMAT_AC3) {
            ALOGV("AC3 CODEC");
            compr_params.codec.id = compr_cap.codecs[2];
            hdmiChannels = 2;
        } else if(format == AUDIO_FORMAT_EAC3) {
            ALOGV("EAC3 CODEC");
            compr_params.codec.id = compr_cap.codecs[14];
            // EAC3 pass through needs to be confogured with 4 time the sample rate
            handle->sampleRate = handle->sampleRate*4;
            hdmiChannels = 2;
        } else if(format == AUDIO_FORMAT_MP3) {
             ALOGV("MP3 CODEC");
             compr_params.codec.id = compr_cap.codecs[0];
        } else if((format & AUDIO_FORMAT_MAIN_MASK) == AUDIO_FORMAT_DTS
                  && !(handle->type & PASSTHROUGH_FORMAT)) {
             ALOGV("DTS CODEC");
             property_get("ro.build.modelid",dtsModelId,"0");
             ALOGV("from property modelId=%s,length=%d\n",
                dtsModelId, strlen(dtsModelId));
             compr_params.codec.dts.modelIdLength = strlen(dtsModelId);
             compr_params.codec.dts.modelId = (__u8 *)dtsModelId;
             ALOGV("passing to driver modelId=%s,length=%d\n",
                compr_params.codec.dts.modelId,
                compr_params.codec.dts.modelIdLength);
             compr_params.codec.id = compr_cap.codecs[5];

             property_get("dts.pp.mix_lfe_to_front", dtsPPparam, "-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval == 0 || dtsPPval == 1))
                 dtsPPval = -1;
             compr_params.codec.options.dts.mix_lfe_to_front = dtsPPval;
             property_get("dts.pp.drc_ratio",dtsPPparam,"-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval >= 0 && dtsPPval <= 100))
                 dtsPPval = -1;
             compr_params.codec.options.dts.drc_ratio = dtsPPval;
             property_get("dts.pp.enable_dialnorm",dtsPPparam,"-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval == 0 || dtsPPval == 1))
                 dtsPPval = -1;
             compr_params.codec.options.dts.enable_dialnorm = dtsPPval;
             property_get("dts.pp.parse_rev2aux",dtsPPparam,"-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval == 0 || dtsPPval == 1))
                 dtsPPval = -1;
             compr_params.codec.options.dts.parse_rev2aux = dtsPPval;
        } else if((format & AUDIO_FORMAT_MAIN_MASK) == AUDIO_FORMAT_DTS_LBR
                  && !(handle->type & PASSTHROUGH_FORMAT)) {
             ALOGV("DTS LBR CODEC");
             property_get("ro.build.modelid", dtsModelId, "0");
             ALOGV("from property modelId=%s,length=%d\n",
                 dtsModelId, strlen(dtsModelId));
             compr_params.codec.dts.modelIdLength = strlen(dtsModelId);
             compr_params.codec.dts.modelId = (__u8 *)dtsModelId;
             ALOGV("passing to driver modelId=%s,length=%d\n",
                compr_params.codec.dts.modelId,
                compr_params.codec.dts.modelIdLength);
             compr_params.codec.id = compr_cap.codecs[6];
             property_get("dts.pp.mix_lfe_to_front",dtsPPparam,"-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval == 0 || dtsPPval == 1))
                 dtsPPval = -1;
             compr_params.codec.options.dts.mix_lfe_to_front = dtsPPval;
             compr_params.codec.options.dts.drc_ratio = -1;
             property_get("dts.pp.enable_dialnorm",dtsPPparam,"-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval == 0 || dtsPPval == 1))
                 dtsPPval = -1;
             compr_params.codec.options.dts.enable_dialnorm = dtsPPval;
             property_get("dts.pp.parse_rev2aux",dtsPPparam,"-1");
             dtsPPval = atoi(dtsPPparam);
             if(!(dtsPPval == 0 || dtsPPval == 1))
                 dtsPPval = -1;
             compr_params.codec.options.dts.parse_rev2aux = dtsPPval;
        } else if((format & AUDIO_FORMAT_MAIN_MASK) == AUDIO_FORMAT_DTS
                 && (handle->type & PASSTHROUGH_FORMAT)) {
             ALOGV("DTS PASSTHROUGH CODEC");
             compr_params.codec.id = compr_cap.codecs[7];
             hdmiChannels = 2;
        } else if((format & AUDIO_FORMAT_MAIN_MASK) == AUDIO_FORMAT_DTS_LBR
                 && (handle->type & PASSTHROUGH_FORMAT)) {
             ALOGV("DTS LBR PASSTHROUGH CODEC");
             compr_params.codec.id = compr_cap.codecs[13];
             hdmiChannels = 2;
        }  else if(format == AUDIO_FORMAT_MP2) {
             ALOGV("MP2 CODEC");
             compr_params.codec.id = compr_cap.codecs[12];
        } else if(format == SNDRV_PCM_FORMAT_S16_LE) {
             ALOGV("PCM FORMAT");
             compr_params.codec.id = compr_cap.codecs[11];
        } else {
             ALOGE("format not supported to open tunnel device");
             if (params)
                 free(params);
             return BAD_VALUE;
        }
        if (handle->type & TRANSCODE_FORMAT) {
             property_get("ro.build.modelid",dtsModelId,"0");
             ALOGD("from property modelId=%s,length=%d\n",
                dtsModelId, strlen(dtsModelId));
             compr_params.codec.dts.modelIdLength = strlen(dtsModelId);
             compr_params.codec.dts.modelId = (__u8 *)dtsModelId;
             ALOGD("passing to driver modelId=%s,length=%d\n",
             compr_params.codec.dts.modelId,
             compr_params.codec.dts.modelIdLength);
             compr_params.codec.transcode_dts = 1;
        } else {
             compr_params.codec.transcode_dts = 0;
        }
        if (ioctl(handle->handle->fd, SNDRV_COMPRESS_SET_PARAMS, &compr_params)) {
            ALOGE("SNDRV_COMPRESS_SET_PARAMS,failed Error no %d \n", errno);
            err = -errno;
            if (params)
                free(params);
            return err;
        }

        if(format != SNDRV_PCM_FORMAT_S16_LE){
            handle->handle->flags &= ~(PCM_STEREO | PCM_MONO | PCM_QUAD | PCM_5POINT1 | PCM_TRIPLE | PCM_PENTA | PCM_7POINT );
            handle->handle->flags |= PCM_7POINT1;
        }
        
        if (handle->type == COMPRESSED_FORMAT &&
            ((handle->devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL && hdmiChannels == 2) ||
            !(handle->devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL))) {
            handle->handle->flags &= ~PCM_7POINT1;
            handle->handle->flags |= PCM_STEREO;
            handle->channels = 2;
        } else if(format != SNDRV_PCM_FORMAT_S16_LE){
            handle->channels = 8;
        }
        handle->handle->flags |= PCM_TUNNEL;

    } else if(handle->type & TRANSCODE_FORMAT) {
        struct snd_pcm_transcode_param transcode_param;
        property_get("ro.build.modelid",dtsModelId,"0");
        ALOGD("from property modelId=%s,length=%d\n",
            dtsModelId, strlen(dtsModelId));
        transcode_param.modelIdLength = strlen(dtsModelId);
        transcode_param.modelId = (__u8 *)dtsModelId;
        ALOGD("passing to driver modelId=%s,length=%d\n",
             transcode_param.modelId,
             transcode_param.modelIdLength);
        transcode_param.transcode_dts = 1;
        transcode_param.session_type = INVALID_SESSION;
        transcode_param.operation = INVALID_STREAM;
        if (ioctl(handle->handle->fd, SNDRV_PCM_CONFIGURE_TRANSCODE, &transcode_param)) {
            ALOGE("SNDRV_COMPRESS_SET_PARAMS,failed Error no %d \n", errno);
            err = -errno;
            return err;
        }
    }
    if(handle->sampleRate > 192000) {
        ALOGE("Sample rate >192000, opening the driver with 192000Hz");
        handle->sampleRate = 192000;
    }
    if(handle->channels > 8)
        handle->channels = 8;
    if (handle->devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
        err = setHDMIChannelCount(hdmiChannels);
        if(err != OK) {
            ALOGE("setHDMIChannelCount err = %d", err);
            if (params)
                free(params);
            return err;
        }
    }

    ALOGD("setHardwareParams: reqBuffSize %d, periodSize %d, channels %d, sampleRate %d.",
         (int) reqBuffSize, handle->periodSize, handle->channels, handle->sampleRate);

    param_set_mask(params, SNDRV_PCM_HW_PARAM_ACCESS,
        (handle->handle->flags & PCM_MMAP) ? SNDRV_PCM_ACCESS_MMAP_INTERLEAVED
        : SNDRV_PCM_ACCESS_RW_INTERLEAVED);
    if ((handle->handle->flags & PCM_TUNNEL) || handle->format == SNDRV_PCM_FORMAT_S24_LE) {
        param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
                       SNDRV_PCM_FORMAT_S24_LE);
        param_set_int(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 32);
        param_set_int(params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
                       handle->channels * 32);
        handle->handle->format = SNDRV_PCM_FORMAT_S24_LE;
        handle->handle->bytes_per_sample = 4;
    } else {
        param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
                       SNDRV_PCM_FORMAT_S16_LE);
        param_set_int(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 16);
        param_set_int(params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
                       handle->channels * 16);
        handle->handle->format = SNDRV_PCM_FORMAT_S16_LE;
        handle->handle->bytes_per_sample = 2;
    }
    param_set_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT,
                   SNDRV_PCM_SUBFORMAT_STD);
    param_set_int(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, handle->periodSize);

    param_set_int(params, SNDRV_PCM_HW_PARAM_CHANNELS,
                  handle->channels);
    param_set_int(params, SNDRV_PCM_HW_PARAM_RATE, handle->sampleRate);
    param_set_hw_refine(handle->handle, params);

    if (param_set_hw_params(handle->handle, params)) {
        ALOGE("cannot set hw params");
        if (params)
            free(params);
        return NO_INIT;
    }
    param_dump(params);

    handle->handle->buffer_size = pcm_buffer_size(params);
    handle->handle->period_size = pcm_period_size(params);
    handle->handle->period_cnt = handle->handle->buffer_size/handle->handle->period_size;
    ALOGD("setHardwareParams: buffer_size %d, period_size %d, period_cnt %d, format %d",
        handle->handle->buffer_size, handle->handle->period_size,
        handle->handle->period_cnt, handle->handle->format);
    handle->handle->rate = handle->sampleRate;
    handle->handle->channels = handle->channels;
    handle->periodSize = handle->handle->period_size;
    handle->bufferSize = handle->handle->period_size;

    if((!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI,
                              strlen(SND_USE_CASE_VERB_HIFI))) ||
       (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC,
                             strlen(SND_USE_CASE_MOD_PLAY_MUSIC)))) {

        handle->latency = PLAYBACK_LATENCY + (handle->handle->period_cnt * PCM_BUFFER_DURATION);
    }else if (handle->type == PCM_FORMAT)
        handle->latency += (handle->handle->period_cnt * PCM_BUFFER_DURATION);

    return NO_ERROR;
}

status_t ALSADevice::setSoftwareParams(alsa_handle_t *handle)
{
    struct snd_pcm_sw_params* params;
    struct pcm* pcm = handle->handle;

    unsigned long periodSize = pcm->period_size;
    int channels;

    if(pcm->flags & PCM_MONO)
        channels = 1;
    else if(pcm->flags & PCM_TRIPLE)
        channels = 3;
    else if(pcm->flags & PCM_QUAD)
        channels = 4;
    else if(pcm->flags & PCM_PENTA)
        channels = 5;
    else if(pcm->flags & PCM_5POINT1)
        channels = 6;
    else if(pcm->flags & PCM_7POINT)
        channels = 7;
    else if(pcm->flags & PCM_7POINT1)
        channels = 8;
    else
        channels = 2;

    params = (snd_pcm_sw_params*) calloc(1, sizeof(struct snd_pcm_sw_params));
    if (!params) {
        LOG_ALWAYS_FATAL("Failed to allocate ALSA software parameters!");
        return NO_INIT;
    }

    // Get the current software parameters
    if(handle->timeStampMode == SNDRV_PCM_TSTAMP_ENABLE)
        params->tstamp_mode = SNDRV_PCM_TSTAMP_ENABLE;
    else
        params->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
    params->period_step = 1;
    if(((!strncmp(handle->useCase,SND_USE_CASE_MOD_PLAY_VOIP,
                            strlen(SND_USE_CASE_MOD_PLAY_VOIP))) ||
        (!strncmp(handle->useCase,SND_USE_CASE_VERB_IP_VOICECALL,
                            strlen(SND_USE_CASE_VERB_IP_VOICECALL))))){
          ALOGV("setparam:  start & stop threshold for Voip ");
          params->avail_min = periodSize/(2*channels);
          params->start_threshold = periodSize/2;
          params->stop_threshold = INT_MAX;
     } else {
         params->avail_min =  periodSize/(2*handle->channels);
         params->start_threshold = periodSize/(handle->channels);
         //Data required in packets for WMA which could be upto 16K.
         if (handle->format == AUDIO_FORMAT_WMA ||
              handle->format == AUDIO_FORMAT_WMA_PRO)
             params->start_threshold = params->start_threshold * 2;
         params->stop_threshold = INT_MAX;
     }
    if (handle->type & COMPRESSED_FORMAT) {
        params->period_step = 1;
        params->avail_min = handle->channels - 1 ? periodSize/2 : periodSize/4;
        if (handle->handle->flags & PCM_TUNNEL) {
            params->start_threshold = periodSize/(4*(handle->channels));
            params->xfer_align = handle->handle->period_size/(4*channels);
        } else {
            params->start_threshold = (periodSize/(handle->channels)) * 2;
            params->xfer_align = handle->handle->period_size/(2*channels);
        }
    }
    params->silence_threshold = 0;
    params->silence_size = 0;

    if (param_set_sw_params(handle->handle, params)) {
        ALOGE("cannot set sw params");
        if (params)
            free(params);
        return NO_INIT;
    }
    return NO_ERROR;
}

int ALSADevice::getDeviceType(uint64_t devices, uint32_t mode)
{
     int ret = 0;

     devices = getDevices(devices, NULL, NULL);
     if(devices & AudioSystem::DEVICE_OUT_ALL)
        ret = DEVICE_TYPE_RX;
     if(devices & VIRTUAL_DEVICE_MASK)
        ret |= DEVICE_TYPE_RX;
     if(devices & AudioSystem::DEVICE_IN_ALL)
        ret |= DEVICE_TYPE_TX;

     return ret;

}
void ALSADevice::switchDevice(uint32_t devices, uint32_t mode)
{
    ALOGV("switchDevice devices = %x, mode = %d", devices,mode);
    for(ALSAHandleList::iterator it = mDeviceList->begin(); it != mDeviceList->end(); ++it) {
        if((strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                          strlen(SND_USE_CASE_VERB_HIFI_TUNNEL))) &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL1,
                          strlen(SND_USE_CASE_MOD_PLAY_TUNNEL1))) &&
           (strncmp(it->useCase, SND_USE_CASE_VERB_HIFI2,
                          strlen(SND_USE_CASE_VERB_HIFI2))) &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC2,
                          strlen(SND_USE_CASE_MOD_PLAY_MUSIC2))) &&
           (strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
                          strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2))) &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
                          strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2))) &&
           (strncmp(it->useCase, SND_USE_CASE_VERB_HIFI3,
                          strlen(SND_USE_CASE_VERB_HIFI3))) &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC3,
                          strlen(SND_USE_CASE_MOD_PLAY_MUSIC3)))  &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
                          strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3))) &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC4,
                          strlen(SND_USE_CASE_MOD_PLAY_MUSIC4)))  &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC5,
                          strlen(SND_USE_CASE_MOD_PLAY_MUSIC5)))  &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC6,
                          strlen(SND_USE_CASE_MOD_PLAY_MUSIC6)))  &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC7,
                          strlen(SND_USE_CASE_MOD_PLAY_MUSIC7)))  &&
           (strncmp(it->useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED,
                          strlen(SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED))) &&
           (strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_REC_COMPRESSED,
                          strlen(SND_USE_CASE_VERB_HIFI_REC_COMPRESSED)))) {
            if(getUseCaseType(it->useCase) ==
               (getUseCaseType(it->useCase) & getDeviceType(devices, mode)))
                switchDeviceUseCase(&(*it), devices, mode);
        }
    }
}

void ALSADevice::switchDeviceUseCase(alsa_handle_t *handle,
                                              uint32_t devices, uint32_t mode)
{
    char *use_case = NULL;
    uint64_t activeDevices = handle->activeDevice;
    uint64_t switchTodevices;
    bool bIsUseCaseSet = false;

    switchTodevices = updateDevices(handle->useCase, devices);
    ALOGV("switchDeviceUseCase curdevices = %llx usecase %s devices = %llx, mode = %d",
           handle->activeDevice, handle->useCase, switchTodevices, mode);

    //Update the active devices to the device list which needs to be derouted
    handle->activeDevice = handle->activeDevice & (~switchTodevices);

    disableDevice(handle);

    //List of the devices to be enabled
    handle->activeDevice = switchTodevices & (~activeDevices);

    snd_use_case_get(handle->ucMgr, "_verb", (const char **)&use_case);
    bIsUseCaseSet = ((use_case == NULL) ||
        (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
            strlen(SND_USE_CASE_VERB_INACTIVE))) ||
        (!strncmp(use_case, handle->useCase, MAX_STR_LEN)));

    enableDevice(handle, bIsUseCaseSet);

    handle->devices = handle->activeDevice = switchTodevices;

    if (use_case != NULL) {
        free(use_case);
        use_case = NULL;
    }
}


void ALSADevice::updateHDMIEDIDInfo(){
    ALOGV("Update EDID info");
    AudioUtil::getHDMIAudioSinkCaps(&mEDIDInfo);
}

int ALSADevice::getFormatHDMIIndexEDIDInfo(EDID_AUDIO_FORMAT_ID formatId){

    int i;
    for (i = 0; i < mEDIDInfo.nAudioBlocks && i < MAX_EDID_BLOCKS; i++) {
        if(mEDIDInfo.AudioBlocksArray[i].nFormatId == formatId)
            return i;
    }
    return -1;
}

void ALSADevice::getDevicesBasedOnOutputChannels(int devices, int* stereoDevices, int* multiChDevices){

    int channels=0;
    *stereoDevices=*multiChDevices=0;

    if((devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)){
                  for (int i = 0; i < mEDIDInfo.nAudioBlocks && i < MAX_EDID_BLOCKS; i++) {
                         if (mEDIDInfo.AudioBlocksArray[i].nFormatId == LPCM &&
                               mEDIDInfo.AudioBlocksArray[i].nChannels > channels &&
                               mEDIDInfo.AudioBlocksArray[i].nChannels <= MAX_HDMI_CHANNEL_CNT) {
                                      channels = mEDIDInfo.AudioBlocksArray[i].nChannels;
                         }
                  }

            if (channels > 2){
                  *multiChDevices = AudioSystem::DEVICE_OUT_AUX_DIGITAL;
            }
    }
    *stereoDevices = devices & ~(*multiChDevices);
}


// ----------------------------------------------------------------------------

status_t ALSADevice::open(alsa_handle_t *handle)
{
    ALOGV("open");
    char *devName;
    unsigned flags = 0;
    int err = NO_ERROR;
    /* No need to call s_close for LPA as pcm device open and close is handled by LPAPlayer in stagefright */
    if((!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_LOW_POWER,
                           strlen(SND_USE_CASE_VERB_HIFI_LOW_POWER))) ||
       (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_LPA,
                           strlen(SND_USE_CASE_VERB_HIFI_LOW_POWER)))) {
        ALOGD("s_open: Opening LPA playback");
        return NO_ERROR;
    }

    close(handle);
    ALOGD("s_open: handle %p", handle);

    // ASoC multicomponent requires a valid path (frontend/backend) for
    // the device to be opened

    // The PCM stream is opened in blocking mode, per ALSA defaults.  The
    // AudioFlinger seems to assume blocking mode too, so asynchronous mode
    // should not be used.
    // ToDo: Add a condition check for HIFI2 use cases also
    if ((!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI,
                            strlen(SND_USE_CASE_VERB_HIFI))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI2,
                            strlen(SND_USE_CASE_VERB_HIFI2))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC2,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC2))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                            strlen(SND_USE_CASE_VERB_HIFI_TUNNEL))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL1,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL1))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI3,
                            strlen(SND_USE_CASE_VERB_HIFI3))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC3,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC3))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC4,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC4))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC5,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC5))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC6,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC6))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC7,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC7))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
                            strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3)))) {
        flags = PCM_OUT;
    } else {
        flags = PCM_IN;
    }

    if (handle->channels == 1) {
        flags |= PCM_MONO;
    } else if (handle->channels == 3) {
        flags |= PCM_TRIPLE;
    } else if (handle->channels == 4) {
        flags |= PCM_QUAD;
    } else if (handle->channels == 5) {
        flags |= PCM_PENTA;
    } else if (handle->channels == 6) {
        flags |= PCM_5POINT1;
    } else if (handle->channels == 7) {
        flags |= PCM_7POINT;
    } else if (handle->channels == 8) {
        flags |= PCM_7POINT1;
    } else {
        flags |= PCM_STEREO;
    }
    ALOGD("s_open: handle %p", handle);
    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node: %s", devName);
        return NO_INIT;
    }
    if (!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                           strlen(SND_USE_CASE_VERB_HIFI_TUNNEL)) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL1,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL1))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
                            strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2))) ||
        (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3)))) {
        flags |= DEBUG_ON | PCM_MMAP;
    }
    handle->handle = pcm_open(flags, (char*)devName);
    ALOGE("s_open: opening ALSA device '%s'", devName);
    if (handle->handle->fd < 0) {
        ALOGE("s_open: Failed to initialize ALSA device '%s'", devName);
        free(devName);
        return NO_INIT;
    }
    handle->handle->flags = flags;
    ALOGD("setting hardware parameters");

    err = setHardwareParams(handle);
    if (err == NO_ERROR) {
        ALOGD("setting software parameters");
        err = setSoftwareParams(handle);
    }
    if(err != NO_ERROR) {
        ALOGE("Set HW/SW params failed: Closing the pcm stream");
        standby(handle);
        if(devName) {
            free(devName);
            devName = NULL;
        }
        return err;
    }
    // if handle type is PCM_FORMAT and spdif or hdmi is present as active device
    // set playbackformat for them with LPCM (they are not set from AudioHardwareALSA)
    if (handle->type == PCM_FORMAT) {
        ALOGV("setting spdif/hdmi format to lpcm: devices %x", (int)handle->activeDevice);
        if (handle->activeDevice & AudioSystem::DEVICE_OUT_SPDIF)
            setPlaybackFormat("LPCM", AudioSystem::DEVICE_OUT_SPDIF);
        if (handle->activeDevice & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            setPlaybackFormat("LPCM", AudioSystem::DEVICE_OUT_AUX_DIGITAL);
    }

    free(devName);
    return NO_ERROR;
}
status_t ALSADevice::configureTranscode(alsa_handle_t *handle) {
    int flags = PCM_STEREO;
    char *devName;
    int err = NO_ERROR;

    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node: %s", devName);
        return NO_INIT;
    }
    ALOGE("s_open: opening ALSA device '%s'", devName);
    handle->handle = pcm_open(PCM_STEREO,  (char*)devName);
    struct snd_pcm_hw_params *params;
    params = (snd_pcm_hw_params*) calloc(1, sizeof(struct snd_pcm_hw_params));
    if (!params) {
        return NO_INIT;
    }
    param_init(params);
    param_set_int(params, SNDRV_PCM_HW_PARAM_RATE, handle->sampleRate);
    param_set_int(params, SNDRV_PCM_HW_PARAM_CHANNELS,
                          handle->channels);
    if (handle->activeDevice & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
        err = setHDMIChannelCount(2);
        if(err != OK) {
            ALOGE("setHDMIChannelCount err = %d", err);
            return err;
        }
    }
    param_set_hw_refine(handle->handle, params);

    if (param_set_hw_params(handle->handle, params)) {
        ALOGE("cannot set local hw_params");
        return NO_INIT;
    }
    pcm_prepare(handle->handle);
    return NO_ERROR;
}
status_t ALSADevice::startVoipCall(alsa_handle_t *handle)
{

    char* devName;
    char* devName1;
    unsigned flags = 0;
    int err = NO_ERROR;
    uint8_t voc_pkt[VOIP_BUFFER_MAX_SIZE];

    close(handle);
    flags = PCM_OUT;
    flags |= PCM_MONO;
    ALOGV("s_open:s_start_voip_call  handle %p", handle);

    if (deviceName(handle, flags, &devName) < 0) {
         ALOGE("Failed to get pcm device node");
         return NO_INIT;
    }

     handle->handle = pcm_open(flags, (char*)devName);

     if (!handle->handle) {
          free(devName);
          ALOGE("s_open: Failed to initialize ALSA device '%s'", devName);
          return NO_INIT;
     }

     if (!pcm_ready(handle->handle)) {
         ALOGE(" pcm ready failed");
     }

     handle->handle->flags = flags;
     err = setHardwareParams(handle);

     if (err == NO_ERROR) {
         err = setSoftwareParams(handle);
     }

     err = pcm_prepare(handle->handle);
     if(err != NO_ERROR) {
         ALOGE("DEVICE_OUT_DIRECTOUTPUT: pcm_prepare failed");
     }

     /* first write required start dsp */
     memset(&voc_pkt,0,sizeof(voc_pkt));
     handle->rxHandle = handle->handle;
     if(devName) {
         free(devName);
         devName = NULL;
     }
     ALOGV("s_open: DEVICE_IN_COMMUNICATION ");
     flags = PCM_IN;
     flags |= PCM_MONO;
     handle->handle = 0;

     if (deviceName(handle, flags, &devName1) < 0) {
        ALOGE("Failed to get pcm device node");
        return NO_INIT;
     }
     handle->handle = pcm_open(flags, (char*)devName1);

     if (!handle->handle) {
         if(devName) {
             free(devName);
             devName = NULL;
         }
         if(devName1) {
             free(devName1);
             devName1 = NULL;
         }
         ALOGE("s_open: Failed to initialize ALSA device '%s'", devName);
         return NO_INIT;
     }

     if (!pcm_ready(handle->handle)) {
        ALOGE(" pcm ready in failed");
     }

     handle->handle->flags = flags;
     err = setHardwareParams(handle);

     if (err == NO_ERROR) {
         err = setSoftwareParams(handle);
     }


     err = pcm_prepare(handle->handle);
     if(err != NO_ERROR) {
         ALOGE("DEVICE_IN_COMMUNICATION: pcm_prepare failed");
     }

     /* first read required start dsp */
     memset(&voc_pkt,0,sizeof(voc_pkt));

     if(devName) {
         free(devName);
         devName = NULL;
     }
     if(devName1) {
         free(devName1);
         devName1 = NULL;
     }
     return NO_ERROR;
}

status_t ALSADevice::startVoiceCall(alsa_handle_t *handle)
{
    char* devName;
    unsigned flags = 0;
    int err = NO_ERROR;

    ALOGD("startVoiceCall: handle %p", handle);
    // ASoC multicomponent requires a valid path (frontend/backend) for
    // the device to be opened

    flags = PCM_OUT | PCM_MONO;
    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node");
        return NO_INIT;
    }
    handle->handle = pcm_open(flags, (char*)devName);
    if (!handle->handle) {
        ALOGE("startVoiceCall: could not open PCM device");
        goto Error;
    }

    handle->handle->flags = flags;
    err = setHardwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("startVoiceCall: setHardwareParams failed");
        goto Error;
    }

    err = setSoftwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("startVoiceCall: setSoftwareParams failed");
        goto Error;
    }

    err = pcm_prepare(handle->handle);
    if(err != NO_ERROR) {
        ALOGE("startVoiceCall: pcm_prepare failed");
        goto Error;
    }

    if (ioctl(handle->handle->fd, SNDRV_PCM_IOCTL_START)) {
        ALOGE("startVoiceCall:SNDRV_PCM_IOCTL_START failed\n");
        goto Error;
    }

    // Store the PCM playback device pointer in rxHandle
    handle->rxHandle = handle->handle;
    free(devName);

    // Open PCM capture device
    flags = PCM_IN | PCM_MONO;
    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node");
        goto Error;
    }
    handle->handle = pcm_open(flags, (char*)devName);
    if (!handle->handle) {
        free(devName);
        goto Error;
    }

    handle->handle->flags = flags;
    err = setHardwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("startVoiceCall: setHardwareParams failed");
        goto Error;
    }

    err = setSoftwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("startVoiceCall: setSoftwareParams failed");
        goto Error;
    }

    err = pcm_prepare(handle->handle);
    if(err != NO_ERROR) {
        ALOGE("startVoiceCall: pcm_prepare failed");
        goto Error;
    }

    if (ioctl(handle->handle->fd, SNDRV_PCM_IOCTL_START)) {
        ALOGE("startVoiceCall:SNDRV_PCM_IOCTL_START failed\n");
        goto Error;
    }
    if(devName) {
        free(devName);
        devName = NULL;
    }
    return NO_ERROR;

Error:
    ALOGE("startVoiceCall: Failed to initialize ALSA device '%s'", devName);
    if(devName) {
        free(devName);
        devName = NULL;
    }
    close(handle);
    return NO_INIT;
}

status_t ALSADevice::startFm(alsa_handle_t *handle)
{
    int err = NO_ERROR;

    err = startLoopback(handle);

    if(err == NO_ERROR)
        setFmVolume(fmVolume);

    return err;
}

status_t ALSADevice::startLoopback(alsa_handle_t *handle)
{
    char *devName;
    unsigned flags = 0;
    int err = NO_ERROR;

    ALOGE("s_start_fm: handle %p", handle);

    // ASoC multicomponent requires a valid path (frontend/backend) for
    // the device to be opened

    flags = PCM_OUT | PCM_STEREO;
    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node");
        goto Error;
    }
    handle->handle = pcm_open(flags, (char*)devName);
    if (!handle->handle) {
        ALOGE("s_start_fm: could not open PCM device");
        goto Error;
    }

    handle->handle->flags = flags;
    err = setHardwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("s_start_fm: setHardwareParams failed");
        goto Error;
    }

    err = setSoftwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("s_start_fm: setSoftwareParams failed");
        goto Error;
    }

    err = pcm_prepare(handle->handle);
    if(err != NO_ERROR) {
        ALOGE("s_start_fm: setSoftwareParams failed");
        goto Error;
    }

    if (ioctl(handle->handle->fd, SNDRV_PCM_IOCTL_START)) {
        ALOGE("s_start_fm: SNDRV_PCM_IOCTL_START failed\n");
        goto Error;
    }

    // Store the PCM playback device pointer in rxHandle
    handle->rxHandle = handle->handle;
    free(devName);

    // Open PCM capture device
    flags = PCM_IN | PCM_STEREO;
    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node");
        goto Error;
    }
    handle->handle = pcm_open(flags, (char*)devName);
    if (!handle->handle) {
        goto Error;
    }

    handle->handle->flags = flags;
    err = setHardwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("s_start_fm: setHardwareParams failed");
        goto Error;
    }

    err = setSoftwareParams(handle);
    if(err != NO_ERROR) {
        ALOGE("s_start_fm: setSoftwareParams failed");
        goto Error;
    }

    err = pcm_prepare(handle->handle);
    if(err != NO_ERROR) {
        ALOGE("s_start_fm: pcm_prepare failed");
        goto Error;
    }

    if (ioctl(handle->handle->fd, SNDRV_PCM_IOCTL_START)) {
        ALOGE("s_start_fm: SNDRV_PCM_IOCTL_START failed\n");
        goto Error;
    }
    if(devName) {
        free(devName);
        devName = NULL;
    }
    return NO_ERROR;

Error:
    if(devName) {
        free(devName);
        devName = NULL;
    }
    close(handle);
    return NO_INIT;
}

status_t ALSADevice::setChannelStatus(unsigned char *channelStatus)
{
    struct snd_aes_iec958 iec958;
    status_t err = NO_ERROR;
    memcpy(iec958.status, channelStatus,24);
    unsigned int ptr = (unsigned int)&iec958;
    setMixerControl("IEC958 Playback PCM Stream",ptr,0);

    return err;
}

status_t ALSADevice::setFmVolume(int value)
{
    status_t err = NO_ERROR;
    setMixerControl("Internal FM RX Volume",value,0);
    fmVolume = value;

    return err;
}

status_t ALSADevice::setLpaVolume(int value)
{
    status_t err = NO_ERROR;
    setMixerControl("LPA RX Volume",value,0);

    return err;
}

void ALSADevice::setDeviceList(ALSAHandleList *mParentDeviceList)
{
    mDeviceList = mParentDeviceList;
}

status_t ALSADevice::start(alsa_handle_t *handle)
{
    status_t err = NO_ERROR;

    if(!handle->handle) {
        ALOGE("No active PCM driver to start");
        return err;
    }

    err = pcm_prepare(handle->handle);

    return err;
}

status_t ALSADevice::close(alsa_handle_t *handle)
{
    int ret;
    status_t err = NO_ERROR;
    struct pcm *h = handle->rxHandle;
    char * use_case = NULL;
    bool bIsUseCaseSet = false;

    snd_use_case_get(handle->ucMgr, "_verb", (const char **)&use_case);
    bIsUseCaseSet = ((use_case != NULL) && (handle->handle != NULL) &&
                        (!strncmp(use_case, handle->useCase, MAX_STR_LEN)));

    handle->rxHandle = 0;
    ALOGD("close: handle %p h %p", handle, h);
    if (h) {
        ALOGV("close rxHandle\n");
        err = pcm_close(h);
        if(err != NO_ERROR) {
            ALOGE("close: pcm_close failed for rxHandle with err %d", err);
        }
    }

    h = handle->handle;
    handle->handle = 0;

    if (h) {
          ALOGV("close handle h %p\n", h);
        err = pcm_close(h);
        if(err != NO_ERROR) {
            ALOGE("close: pcm_close failed for handle with err %d", err);
        }
        disableDevice(handle);
    } else if((!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_LOW_POWER,
                                  strlen(SND_USE_CASE_VERB_HIFI_LOW_POWER))) ||
              (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_LPA,
                                  strlen(SND_USE_CASE_MOD_PLAY_LPA)))) {
        disableDevice(handle);
    }

    if (bIsUseCaseSet)
        snd_use_case_set(handle->ucMgr, "_verb", SND_USE_CASE_VERB_INACTIVE);

    return err;
}

/*
    this is same as close, but don't discard
    the device/mode info. This way we can still
    close the device, hit idle and power-save, reopen the pcm
    for the same device/mode after resuming
*/
status_t ALSADevice::standby(alsa_handle_t *handle)
{
    int ret;
    status_t err = NO_ERROR;
    struct pcm *h = handle->rxHandle;
    handle->rxHandle = 0;
    ALOGD("s_standby: handle %p h %p", handle, h);
    if (h) {
        ALOGE("s_standby  rxHandle\n");
        err = pcm_close(h);
        if(err != NO_ERROR) {
            ALOGE("s_standby: pcm_close failed for rxHandle with err %d", err);
        }
    }

    h = handle->handle;
    handle->handle = 0;

    if (h) {
          ALOGE("s_standby handle h %p\n", h);
        err = pcm_close(h);
        if(err != NO_ERROR) {
            ALOGE("s_standby: pcm_close failed for handle with err %d", err);
        }
        disableDevice(handle);
    } else if((!strncmp(handle->useCase, SND_USE_CASE_VERB_HIFI_LOW_POWER,
                                  strlen(SND_USE_CASE_VERB_HIFI_LOW_POWER))) ||
              (!strncmp(handle->useCase, SND_USE_CASE_MOD_PLAY_LPA,
                                  strlen(SND_USE_CASE_MOD_PLAY_LPA)))) {
        disableDevice(handle);
    }

    return err;
}

status_t ALSADevice::route(uint32_t devices, int mode)
{
    status_t status = NO_ERROR;

    ALOGD("s_route: devices 0x%x in mode %d", devices, mode);
    callMode = mode;
    switchDevice(devices, mode);
    return status;
}

int  ALSADevice::getUseCaseType(const char *useCase)
{
    ALOGE("use case is %s\n", useCase);
    if (!strncmp(useCase, SND_USE_CASE_VERB_HIFI,
           strlen(SND_USE_CASE_VERB_HIFI)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI_LOW_POWER,
           strlen(SND_USE_CASE_VERB_HIFI_LOW_POWER)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
           strlen(SND_USE_CASE_VERB_HIFI_TUNNEL)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI2,
           strlen(SND_USE_CASE_VERB_HIFI2)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
           strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI3,
           strlen(SND_USE_CASE_VERB_HIFI3)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_DIGITAL_RADIO,
           strlen(SND_USE_CASE_VERB_DIGITAL_RADIO)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_LPA,
           strlen(SND_USE_CASE_MOD_PLAY_LPA)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL1,
           strlen(SND_USE_CASE_MOD_PLAY_TUNNEL1)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC2,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC2)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
           strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC3,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC3)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
           strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC4,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC4)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC5,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC5)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC6,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC6)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC7,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC7)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_FM,
           strlen(SND_USE_CASE_MOD_PLAY_FM)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI_PSEUDO_TUNNEL1,
           strlen(SND_USE_CASE_VERB_HIFI_PSEUDO_TUNNEL1)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PSEUDO_TUNNEL1,
           strlen(SND_USE_CASE_MOD_PSEUDO_TUNNEL1)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PSEUDO_TUNNEL2,
           strlen(SND_USE_CASE_MOD_PSEUDO_TUNNEL2)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PSEUDO_TUNNEL3,
           strlen(SND_USE_CASE_MOD_PSEUDO_TUNNEL3)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PSEUDO_MUSIC2,
           strlen(SND_USE_CASE_MOD_PSEUDO_MUSIC2)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PSEUDO_MUSIC3,
           strlen(SND_USE_CASE_MOD_PSEUDO_MUSIC3)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PSEUDO_MUSIC4,
           strlen(SND_USE_CASE_MOD_PSEUDO_MUSIC4))) {
        return USECASE_TYPE_RX;
    } else if (!strncmp(useCase, SND_USE_CASE_VERB_HIFI_REC,
           strlen(SND_USE_CASE_VERB_HIFI_REC)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI_REC2,
           strlen(SND_USE_CASE_VERB_HIFI_REC2)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_HIFI_REC_COMPRESSED,
           strlen(SND_USE_CASE_VERB_HIFI_REC_COMPRESSED)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_FM_REC,
           strlen(SND_USE_CASE_VERB_FM_REC)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_FM_A2DP_REC,
           strlen(SND_USE_CASE_VERB_FM_A2DP_REC)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC,
           strlen(SND_USE_CASE_MOD_CAPTURE_MUSIC)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC2,
           strlen(SND_USE_CASE_MOD_CAPTURE_MUSIC2)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED,
           strlen(SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_FM,
           strlen(SND_USE_CASE_MOD_CAPTURE_FM)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_A2DP_FM,
           strlen(SND_USE_CASE_MOD_CAPTURE_A2DP_FM))) {
        return USECASE_TYPE_TX;
    } else if (!strncmp(useCase, SND_USE_CASE_VERB_VOICECALL,
           strlen(SND_USE_CASE_VERB_VOICECALL)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_IP_VOICECALL,
           strlen(SND_USE_CASE_VERB_IP_VOICECALL)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_DL_REC,
           strlen(SND_USE_CASE_VERB_DL_REC)) ||
        !strncmp(useCase, SND_USE_CASE_VERB_UL_DL_REC,
           strlen(SND_USE_CASE_VERB_UL_DL_REC)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_VOICE,
           strlen(SND_USE_CASE_MOD_PLAY_VOICE)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_PLAY_VOIP,
           strlen(SND_USE_CASE_MOD_PLAY_VOIP)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_VOICE_DL,
           strlen(SND_USE_CASE_MOD_CAPTURE_VOICE_DL)) ||
        !strncmp(useCase, SND_USE_CASE_MOD_CAPTURE_VOICE_UL_DL,
           strlen(SND_USE_CASE_MOD_CAPTURE_VOICE_UL_DL)) ) {
        return (USECASE_TYPE_RX | USECASE_TYPE_TX);
  } else {
       ALOGE("unknown use case %s\n", useCase);
        return 0;
  }
}

void ALSADevice::disableDevice(alsa_handle_t *handle)
{
    char *rxDevice = NULL, *txDevice = NULL;
    uint64_t devices = handle->activeDevice;
    bool disableRxDevice = true, disableTxDevice = true;
    char *use_case = NULL;
    unsigned usecase_type = 0;
    int64_t deviceToDisable,actualDevices;
    int bitNo=MAX_NO_BITS;
    bool bIsUseCaseSet = false;

    snd_use_case_get(handle->ucMgr, "_verb", (const char **)&use_case);
    bIsUseCaseSet = ((use_case != NULL) && (!strncmp(use_case, handle->useCase, MAX_STR_LEN)));
    ALOGD("disableDevice device = %llx verb  %s mode %d use case %s",
          devices, (use_case == NULL) ? "NULL" : use_case, handle->mode, handle->useCase);

    while (devices != 0) {
        // Disable devices from leftmost bit, so that if speaker is present,Slimbus virtual port
        // is disabled first than the speaker.

        for(;bitNo >= 0;bitNo--){
            if(deviceToDisable = (devices & (0x1<<bitNo)))
            break;
        }

        actualDevices = getDevices(deviceToDisable, &rxDevice, &txDevice);


        for (ALSAHandleList::iterator it = mDeviceList->begin(); it != mDeviceList->end(); ++it) {
            if (it->useCase != NULL) {
                if (strcmp(it->useCase, handle->useCase)) {
                    if ((&(*it)) != handle && handle->activeDevice && it->activeDevice && (getDevices(it->activeDevice, NULL, NULL) & actualDevices)) {
                        ALOGD("disableRxDevice - false use case %s active Device %llx deviceToDisable %llx",
                                  it->useCase, it->activeDevice, deviceToDisable);
                        if(getDeviceType(getDevices(it->activeDevice, NULL, NULL) & actualDevices, 0) & getUseCaseType(it->useCase) & DEVICE_TYPE_RX)
                            disableRxDevice = false;
                        if(getDeviceType(getDevices(it->activeDevice, NULL, NULL) & actualDevices, 0) & getUseCaseType(it->useCase) & DEVICE_TYPE_TX)
                            disableTxDevice = false;
                    }
                }
            }
        }



        if(rxDevice) {
            usecase_type = getUseCaseType(handle->useCase);
            if (usecase_type & DEVICE_TYPE_RX) {
                if(disableRxDevice)
                    snd_use_case_set_case(handle->ucMgr, "_disdev", rxDevice, handle->useCase);
                else if(bIsUseCaseSet)
                    snd_use_case_set_case(handle->ucMgr, "_disverb", handle->useCase, rxDevice);
                else
                    snd_use_case_set_case(handle->ucMgr, "_dismod", handle->useCase, rxDevice);
            }
            free(rxDevice);
            rxDevice = NULL;
        }

        if(txDevice) {
            usecase_type = getUseCaseType(handle->useCase);
            if (usecase_type & USECASE_TYPE_TX) {
                if(disableTxDevice)
                    snd_use_case_set_case(handle->ucMgr, "_disdev", txDevice, handle->useCase);
                else if(bIsUseCaseSet)
                    snd_use_case_set_case(handle->ucMgr, "_disverb", handle->useCase, txDevice);
                else
                    snd_use_case_set_case(handle->ucMgr, "_dismod", handle->useCase, txDevice);
            }
            free(txDevice);
            txDevice = NULL;
        }
        devices = devices & (~deviceToDisable);
        disableRxDevice = true;
        disableTxDevice = true;
    }
    handle->activeDevice = 0;
}

void ALSADevice::enableDevice(alsa_handle_t *handle, bool bIsUseCaseSet)
{
    char *rxDevice = NULL, *txDevice = NULL, *tempRxDevice;
    uint64_t devices = handle->activeDevice;
    int64_t deviceToEnable;
    unsigned usecase_type = 0;
    int bitNo= MAX_NO_BITS;

    ALOGD("enableDevice %llx bIsUseCaseSet %d", handle->activeDevice, bIsUseCaseSet);
    while (devices != 0) {

        // Enable devices from leftmost bit, so that if speaker is present,Slimbus virtual port
        // is enabled first than the speaker.
        for(;bitNo >= 0;bitNo--){
            if(deviceToEnable = (devices & (0x1<<bitNo)))
            break;
        }

        ALOGV("%s: device %llx, to enable %llx", __FUNCTION__, devices, deviceToEnable);
        getDevices(deviceToEnable, &rxDevice, &txDevice);
        tempRxDevice = strdup(rxDevice);

        if(rxDevice != NULL) {
            usecase_type = getUseCaseType(handle->useCase);
            if (usecase_type & USECASE_TYPE_RX) {
                if(bIsUseCaseSet) {
                    snd_use_case_set_case(handle->ucMgr, "_verb", handle->useCase, rxDevice);
                } else {
                    snd_use_case_set_case(handle->ucMgr, "_enamod", handle->useCase, rxDevice);
                }
            }
            free(rxDevice);
            rxDevice = NULL;
        }
        if(txDevice != NULL) {
            usecase_type = getUseCaseType(handle->useCase);
            if (usecase_type & USECASE_TYPE_TX) {
                 ALOGE("tempRxDevice %s", tempRxDevice);
                 if(!(strncmp(tempRxDevice, SND_USE_CASE_DEV_SPDIF, sizeof(SND_USE_CASE_DEV_SPDIF))))
                    setMixerControl("EC_REF_RX", "SEC_RX");
                 if(!(strncmp(tempRxDevice, SND_USE_CASE_DEV_SPEAKER, sizeof(SND_USE_CASE_DEV_SPEAKER))))
                    setMixerControl("EC_REF_RX", "SLIM_RX");
                 if(!(strncmp(tempRxDevice, SND_USE_CASE_DEV_HDMI, sizeof(SND_USE_CASE_DEV_HDMI))))
                    setMixerControl("EC_REF_RX", "HDMI_RX");

                if(bIsUseCaseSet) {
                    snd_use_case_set_case(handle->ucMgr, "_verb", handle->useCase, txDevice);
                } else {
                    snd_use_case_set_case(handle->ucMgr, "_enamod", handle->useCase, txDevice);
                }
            }
            free(txDevice);
            txDevice = NULL;
        }
        free(tempRxDevice);
        devices = devices & (~deviceToEnable);
    }
}

char* ALSADevice::getUCMDevice(uint64_t devices, int input)
{
    ALOGD("getUCMDevice:: devices %llx", devices);
    if (!input) {
        if (!(mDevSettingsFlag & TTY_OFF) &&
            (callMode == AudioSystem::MODE_IN_CALL) &&
            ((devices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) ||
             (devices & AudioSystem::DEVICE_OUT_WIRED_HEADPHONE) ||
             (devices & AudioSystem::DEVICE_OUT_ANC_HEADSET) ||
             (devices & AudioSystem::DEVICE_OUT_ANC_HEADPHONE))) {
             if (mDevSettingsFlag & TTY_VCO) {
                 return strdup(SND_USE_CASE_DEV_TTY_HEADSET_RX);
             } else if (mDevSettingsFlag & TTY_FULL) {
                 return strdup(SND_USE_CASE_DEV_TTY_FULL_RX);
             } else if (mDevSettingsFlag & TTY_HCO) {
                 return strdup(SND_USE_CASE_DEV_EARPIECE); /* HANDSET RX */
             }
        } else if (devices & AudioSystem::DEVICE_OUT_EARPIECE) {
                return strdup(SND_USE_CASE_DEV_EARPIECE); /* HANDSET RX */
        } else if (devices & AudioSystem::DEVICE_OUT_WIRED_HEADSET ||
                   devices & AudioSystem::DEVICE_OUT_WIRED_HEADPHONE) {
                if (mDevSettingsFlag & ANC_FLAG)
                    return strdup(SND_USE_CASE_DEV_ANC_HEADSET); /* ANC HEADSET RX */
                else
                    return strdup(SND_USE_CASE_DEV_HEADPHONES); /* HEADSET RX */
#ifdef QCOM_USBAUDIO_ENABLED
        } else if ((devices & AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET) ||
                  (devices & AudioSystem::DEVICE_OUT_DGTL_DOCK_HEADSET)) {
                return strdup(SND_USE_CASE_DEV_USB_PROXY_RX); /* PROXY RX */
#endif
        } else if (devices & AudioSystem::DEVICE_OUT_SPEAKER) {
                return strdup(SND_USE_CASE_DEV_SPEAKER); /* SPEAKER RX */
        } else if (devices & AudioSystem::DEVICE_OUT_ANC_HEADSET ||
                   devices & AudioSystem::DEVICE_OUT_ANC_HEADPHONE) {
                return strdup(SND_USE_CASE_DEV_ANC_HEADSET); /* ANC HEADSET RX */
        } else if (devices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO ||
                   devices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_HEADSET ||
                   devices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_CARKIT) {
                if (btsco_samplerate == BTSCO_RATE_16KHZ)
                    return strdup(SND_USE_CASE_DEV_BTSCO_WB_RX); /* BTSCO RX*/
                else
                    return strdup(SND_USE_CASE_DEV_BTSCO_NB_RX); /* BTSCO RX*/
        } else if (devices & AudioSystem::DEVICE_OUT_BLUETOOTH_A2DP ||
            devices & AudioSystem::DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES ||
            devices &  AudioSystem::DEVICE_OUT_DIRECTOUTPUT ||
            devices & AudioSystem::DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER) {
                /* Nothing to be done, use current active device */
                return strdup(curRxUCMDevice);
        } else if (devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
                return strdup(SND_USE_CASE_DEV_HDMI); /* HDMI RX */
        } else if (devices & AudioSystem::DEVICE_OUT_PROXY) {
                return strdup(SND_USE_CASE_DEV_PROXY_RX); /* PROXY RX */
        } else if (devices & AudioSystem::DEVICE_OUT_FM_TX) {
                return strdup(SND_USE_CASE_DEV_FM_TX); /* FM Tx */
        } else if (devices & AudioSystem::DEVICE_OUT_SPDIF) {
                return strdup(SND_USE_CASE_DEV_SPDIF);
        } else if (devices & VIRTUAL_DEVICE_SLIMBUS_VIRTUAL_PORT) {
                return strdup(SND_USE_CASE_DEV_SLIMBUS_VIRTUAL_PORT);
        }
        ALOGD("No valid output device: %llx", devices);
    } else {
        if (!(mDevSettingsFlag & TTY_OFF) &&
            (callMode == AudioSystem::MODE_IN_CALL) &&
            ((devices & AudioSystem::DEVICE_IN_WIRED_HEADSET) ||
             (devices & AudioSystem::DEVICE_IN_ANC_HEADSET))) {
             if (mDevSettingsFlag & TTY_HCO) {
                 return strdup(SND_USE_CASE_DEV_TTY_HEADSET_TX);
             } else if (mDevSettingsFlag & TTY_FULL) {
                 return strdup(SND_USE_CASE_DEV_TTY_FULL_TX);
             } else if (mDevSettingsFlag & TTY_VCO) {
                 if (!strncmp(mic_type, "analog", 6)) {
                     return strdup(SND_USE_CASE_DEV_HANDSET); /* HANDSET TX */
                 } else {
                     return strdup(SND_USE_CASE_DEV_LINE); /* BUILTIN-MIC TX */
                 }
             }
        } else if (devices & AudioSystem::DEVICE_IN_BUILTIN_MIC) {
            if (!strncmp(mic_type, "analog", 6)) {
                return strdup(SND_USE_CASE_DEV_HANDSET); /* HANDSET TX */
            } else {
                if (mDevSettingsFlag & DMIC_FLAG) {
                    if (fluence_mode == FLUENCE_MODE_ENDFIRE) {
                        return strdup(SND_USE_CASE_DEV_DUAL_MIC_ENDFIRE); /* DUALMIC EF TX */
                    } else if (fluence_mode == FLUENCE_MODE_BROADSIDE) {
                        return strdup(SND_USE_CASE_DEV_DUAL_MIC_BROADSIDE); /* DUALMIC BS TX */
                    }
                } else if (mDevSettingsFlag & QMIC_FLAG){
                    return strdup(SND_USE_CASE_DEV_QUAD_MIC);
                } else {
                    return strdup(SND_USE_CASE_DEV_LINE); /* BUILTIN-MIC TX */
                }
            }
        } else if (devices & AudioSystem::DEVICE_IN_AUX_DIGITAL) {
            return strdup(SND_USE_CASE_DEV_HDMI_TX); /* HDMI TX */
        } else if ((devices & AudioSystem::DEVICE_IN_WIRED_HEADSET) ||
                   (devices & AudioSystem::DEVICE_IN_ANC_HEADSET)) {
            return strdup(SND_USE_CASE_DEV_HEADSET); /* HEADSET TX */
        } else if (devices & AudioSystem::DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
             if (btsco_samplerate == BTSCO_RATE_16KHZ)
                 return strdup(SND_USE_CASE_DEV_BTSCO_WB_TX); /* BTSCO TX*/
             else
                 return strdup(SND_USE_CASE_DEV_BTSCO_NB_TX); /* BTSCO TX*/
#ifdef QCOM_USBAUDIO_ENABLED
        } else if (devices & AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET) {
                if(mHardwarePlatform == DTV_PLATFORM)
                    return strdup(SND_USE_CASE_DEV_SPEAKER_USB_PROXY_TX);
                else
                    return strdup(SND_USE_CASE_DEV_USB_PROXY_TX); /* USB PROXY TX */
#endif
        } else if (devices & AudioSystem::DEVICE_IN_DEFAULT) {
            if (!strncmp(mic_type, "analog", 6)) {
                return strdup(SND_USE_CASE_DEV_HANDSET); /* HANDSET TX */
            } else {
                if (mDevSettingsFlag & DMIC_FLAG) {
                    if (fluence_mode == FLUENCE_MODE_ENDFIRE) {
                        return strdup(SND_USE_CASE_DEV_SPEAKER_DUAL_MIC_ENDFIRE); /* DUALMIC EF TX */
                    } else if (fluence_mode == FLUENCE_MODE_BROADSIDE) {
                        return strdup(SND_USE_CASE_DEV_SPEAKER_DUAL_MIC_BROADSIDE); /* DUALMIC BS TX */
                    }
                } else if (mDevSettingsFlag & QMIC_FLAG){
                    return strdup(SND_USE_CASE_DEV_QUAD_MIC);
                } else {
                    return strdup(SND_USE_CASE_DEV_LINE); /* BUILTIN-MIC TX */
                }
            }
        } else if (devices & AudioSystem::DEVICE_IN_PROXY) {
            return strdup(SND_USE_CASE_DEV_PROXY_TX); /* PROXY TX */
        } else if ((devices & AudioSystem::DEVICE_IN_COMMUNICATION) ||
                   (devices & AudioSystem::DEVICE_IN_FM_RX) ||
                   (devices & AudioSystem::DEVICE_IN_FM_RX_A2DP) ||
                   (devices & AudioSystem::DEVICE_IN_VOICE_CALL)) {
            /* Nothing to be done, use current active device */
            return strdup(curTxUCMDevice);
        } else if ((devices & AudioSystem::DEVICE_IN_COMMUNICATION) ||
                   (devices & AudioSystem::DEVICE_IN_AMBIENT) ||
                   (devices & AudioSystem::DEVICE_IN_BACK_MIC) ||
                   (devices & AudioSystem::DEVICE_IN_AUX_DIGITAL)) {
            ALOGI("No proper mapping found with UCM device list, setting default");
            if (!strncmp(mic_type, "analog", 6)) {
                return strdup(SND_USE_CASE_DEV_HANDSET); /* HANDSET TX */
            } else {
                return strdup(SND_USE_CASE_DEV_LINE); /* BUILTIN-MIC TX */
            }
        } else {
            ALOGD("No valid input device: %llx", devices);
        }
    }
    return NULL;
}

void ALSADevice::setVoiceVolume(int vol)
{
    ALOGD("setVoiceVolume: volume %d", vol);
    setMixerControl("Voice Rx Volume", vol, 0);
}

void ALSADevice::setVoipVolume(int vol)
{
    ALOGD("setVoipVolume: volume %d", vol);
    setMixerControl("Voip Rx Volume", vol, 0);
}
void ALSADevice::setMicMute(int state)
{
    ALOGD("setMicMute: state %d", state);
    setMixerControl("Voice Tx Mute", state, 0);
}

void ALSADevice::setVoipMicMute(int state)
{
    ALOGD("setVoipMicMute: state %d", state);
    setMixerControl("Voip Tx Mute", state, 0);
}

void ALSADevice::setBtscoRate(int rate)
{
    btsco_samplerate = rate;
}

void ALSADevice::enableWideVoice(bool flag)
{
    ALOGD("enableWideVoice: flag %d", flag);
    if(flag == true) {
        setMixerControl("Widevoice Enable", 1, 0);
    } else {
        setMixerControl("Widevoice Enable", 0, 0);
    }
}

void ALSADevice::enableFENS(bool flag)
{
    ALOGD("enableFENS: flag %d", flag);
    if(flag == true) {
        setMixerControl("FENS Enable", 1, 0);
    } else {
        setMixerControl("FENS Enable", 0, 0);
    }
}

void ALSADevice::setFlags(uint32_t flags)
{
    ALOGV("setFlags: flags %d", flags);
    mDevSettingsFlag = flags;
}

status_t ALSADevice::getMixerControl(const char *name, unsigned int &value, int index)
{
    struct mixer_ctl *ctl;

    if (!mMixer) {
        ALOGE("Control not initialized");
        return NO_INIT;
    }

    ctl =  mixer_get_control(mMixer, name, index);
    if (!ctl)
        return BAD_VALUE;

    mixer_ctl_get(ctl, &value);
    return NO_ERROR;
}

status_t ALSADevice::setMixerControl(const char *name, unsigned int value, int index)
{
    struct mixer_ctl *ctl;
    int ret = 0;
    ALOGD("set:: name %s value %d index %d", name, value, index);
    if (!mMixer) {
        ALOGE("Control not initialized");
        return NO_INIT;
    }

    // ToDo: Do we need to send index here? Right now it works with 0
    ctl = mixer_get_control(mMixer, name, 0);
    if(ctl == NULL) {
        ALOGE("Could not get the mixer control");
        return BAD_VALUE;
    }
    ret = mixer_ctl_set(ctl, value);
    return (ret < 0) ? BAD_VALUE : NO_ERROR;
}

status_t ALSADevice::setMixerControl(const char *name, const char *value)
{
    struct mixer_ctl *ctl;
    int ret = 0;
    ALOGD("set:: name %s value %s", name, value);

    if (!mMixer) {
        ALOGE("Control not initialized");
        return NO_INIT;
    }

    ctl = mixer_get_control(mMixer, name, 0);
    if(ctl == NULL) {
        ALOGE("Could not get the mixer control");
        return BAD_VALUE;
    }
    ret = mixer_ctl_select(ctl, value);
    return (ret < 0) ? BAD_VALUE : NO_ERROR;
}

status_t ALSADevice::setMixerControlExt(const char *name, int count, char **setValues)
{
    struct mixer_ctl *ctl;
    int ret = 0;
    ALOGD("setMixerControl:: name %s count %d", name, count);
    if (!mMixer) {
        ALOGE("Control not initialized");
        return NO_INIT;
    }

    // ToDo: Do we need to send index here? Right now it works with 0
    ctl = mixer_get_control(mMixer, name, 0);
    if(ctl == NULL) {
        ALOGE("Could not get the mixer control");
        return BAD_VALUE;
    }
    ret = mixer_ctl_set_value(ctl, count, setValues);
    return (ret < 0) ? BAD_VALUE : NO_ERROR;
}

int32_t ALSADevice::get_linearpcm_channel_status(uint32_t sampleRate,
                                                 unsigned char *channel_status)
{
    int32_t status = 0;
    unsigned char bit_index;
    memset(channel_status,0,24);
    bit_index = 0;
    /* block start bit in preamble bit 3 */
    set_bits(channel_status, 1, 1, &bit_index);

    //linear pcm
    bit_index = 1;
    set_bits(channel_status, 1, 0, &bit_index);

    bit_index = 24;
    switch (sampleRate) {
        case 8000:
           set_bits(channel_status, 4, 0x09, &bit_index);
           break;
        case 11025:
           set_bits(channel_status, 4, 0x0A, &bit_index);
           break;
        case 12000:
           set_bits(channel_status, 4, 0x0B, &bit_index);
           break;
        case 16000:
           set_bits(channel_status, 4, 0x0E, &bit_index);
           break;
        case 22050:
           set_bits(channel_status, 4, 0x02, &bit_index);
           break;
        case 24000:
           set_bits(channel_status, 4, 0x06, &bit_index);
           break;
        case 32000: // 1100 in 24..27
           set_bits(channel_status, 4, 0x0C, &bit_index);
           break;
        case 44100: // 0000 in 24..27
           break;
        case 48000: // 0100 in 24..27
            set_bits(channel_status, 4, 0x04, &bit_index);
            break;
        case 88200: // 0001 in 24..27
           set_bits(channel_status, 4, 0x01, &bit_index);
           break;
        case 96000: // 0101 in 24..27
            set_bits(channel_status, 4, 0x05, &bit_index);
            break;
        case 176400: // 0011 in 24..27
            set_bits(channel_status, 4, 0x03, &bit_index);
            break;
        case 192000: // 0111 in 24..27
            set_bits(channel_status, 4, 0x07, &bit_index);
            break;
        default:
            ALOGV("Invalid sample_rate %u\n", sampleRate);
            status = -1;
            break;
    }
    return status;
}

void ALSADevice::get_compressed_channel_status(
                                 unsigned char *channel_status)
{
    int32_t status = 0;
    unsigned char bit_index;
    memset(channel_status,0,24);
    bit_index = 0;
    /* block start bit in preamble bit 3 */
    set_bits(channel_status, 1, 1, &bit_index);

    //compre out
    bit_index = 1;
    set_bits(channel_status, 1, 1, &bit_index);

    // sample rate; fixed 48K for default/transcode
    bit_index = 24;
    set_bits(channel_status, 4, 0x04, &bit_index);

}

int32_t ALSADevice::get_compressed_channel_status(void *audio_stream_data,
                                                  uint32_t audio_frame_size,
                                                  unsigned char *channel_status,
                                                  enum audio_parser_code_type codec_type)
                                                  // codec_type - AUDIO_PARSER_CODEC_AC3
                                                  //            - AUDIO_PARSER_CODEC_DTS
{
    unsigned char *streamPtr;
    int ret = 0;
    streamPtr = (unsigned char *)audio_stream_data;

    if (audio_stream_data == NULL || audio_frame_size == 0) {
        ALOGW("no buffer to get channel status, return default for compress");
        get_compressed_channel_status(channel_status);
        return ret;
    }

    memset(channel_status,0,24);
    if(init_audio_parser(streamPtr, audio_frame_size, codec_type) == -1)
    {
        ALOGE("init audio parser failed");
        return -1;
    }
    ret = get_channel_status(channel_status, codec_type);
    return ret;
}

status_t ALSADevice::setPlaybackVolume(int value, char *useCase)
{
    status_t err = NO_ERROR;
    char volMixerCtrlStr[128];

    /* Add new use case checks at correct order/place, strncmp 3rd
     * argument should be longest strings length otherwise
     * strncmp will not give desired results.
     */
    if((!strncmp(useCase, SND_USE_CASE_VERB_HIFI2,
           strlen(SND_USE_CASE_VERB_HIFI2))) ||
       (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC2,
           strlen(SND_USE_CASE_MOD_PLAY_MUSIC2))))
        strlcpy(volMixerCtrlStr, "HIFI2 RX Volume", sizeof(volMixerCtrlStr));
    else if((!strncmp(useCase, SND_USE_CASE_VERB_HIFI3,
                strlen(SND_USE_CASE_VERB_HIFI3))) ||
            (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC3,
                strlen(SND_USE_CASE_MOD_PLAY_MUSIC3))))
        strlcpy(volMixerCtrlStr, "HIFI3 RX Volume", sizeof(volMixerCtrlStr));
    else if(!strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC4,
                strlen(SND_USE_CASE_MOD_PLAY_MUSIC4)))
        strlcpy(volMixerCtrlStr, "HIFI4 RX Volume", sizeof(volMixerCtrlStr));
    else if(!strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC5,
                strlen(SND_USE_CASE_MOD_PLAY_MUSIC5)))
        strlcpy(volMixerCtrlStr, "HIFI5 RX Volume", sizeof(volMixerCtrlStr));
    else if(!strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC6,
                strlen(SND_USE_CASE_MOD_PLAY_MUSIC6)))
        strlcpy(volMixerCtrlStr, "HIFI6 RX Volume", sizeof(volMixerCtrlStr));
    else if(!strncmp(useCase, SND_USE_CASE_MOD_PLAY_MUSIC7,
                strlen(SND_USE_CASE_MOD_PLAY_MUSIC7)))
        strlcpy(volMixerCtrlStr, "HIFI7 RX Volume", sizeof(volMixerCtrlStr));
    else if((!strncmp(useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
                strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2))) ||
            (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
                strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2))))
        strlcpy(volMixerCtrlStr, "COMPRESSED2 RX Volume", sizeof(volMixerCtrlStr));
    else if((!strncmp(useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                strlen(SND_USE_CASE_VERB_HIFI_TUNNEL))) ||
            (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL1,
                strlen(SND_USE_CASE_MOD_PLAY_TUNNEL1))))
        strlcpy(volMixerCtrlStr, "COMPRESSED RX Volume", sizeof(volMixerCtrlStr));
    else if(!strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
                strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3)))
        strlcpy(volMixerCtrlStr, "COMPRESSED3 RX Volume", sizeof(volMixerCtrlStr));

    err = setMixerControl(volMixerCtrlStr,value,0);
    if(err) {
        ALOGE("setPlaybackVolume - error = %d",err);
    }
    return err;
}

status_t ALSADevice::setPlaybackFormat(const char *value, int device)
{
    status_t err = NO_ERROR;
    if (device == AudioSystem::DEVICE_OUT_SPDIF) {
        err = setMixerControl("SEC RX Format",value);
        if (!strncmp(value, "Compr", sizeof(value)))
            err = setMixerControl("SEC RX Rate", "Variable");
        else
            err = setMixerControl("SEC RX Rate", "Default");
    }
    else if(device == AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
        err = setMixerControl("HDMI RX Format",value);
        if (!strncmp(value, "Compr", sizeof(value)))
            err = setMixerControl("HDMI RX Rate", "Variable");
        else
            err = setMixerControl("HDMI RX Rate", "Default");
    }
    if(err) {
        ALOGE("setPlaybackFormat error = %d",err);
    }

    return NO_ERROR;
}

int ALSADevice::mapDeviceToPort(int device)
{
    for(int i=0; i<NUM_DEVICES_WITH_PP_PARAMS; i++)
        if(deviceToPortID[i][0] == device)
            return deviceToPortID[i][1];
    return 0;
}


status_t ALSADevice::setPlaybackOutputDelay(int device, unsigned int delay)
{

    status_t status = NO_ERROR;

    ALOGD("device - 0x%x, delay = %dms", device, delay);
    if ((delay < MIN_AUDIO_OUTPUT_DELAY) ||
        (delay > MAX_AUDIO_OUTPUT_DELAY) ||
        ((device != AudioSystem::DEVICE_OUT_SPDIF) &&
         (device != AudioSystem::DEVICE_OUT_AUX_DIGITAL))) {

        ALOGE("%s: Invalid input parameters: delay (%d) output device (%d)",
              __FUNCTION__, delay, device);
        return BAD_VALUE;
    }

    /*Convert the delay value from miliseconds to microseconds*/
    delay *= 1000;

    char** setValues;
    int port_id;
    bool isMemAvailable = true;
    int memAllocIdx = 0;
    setValues = (char**)malloc(ADM_PP_PARAM_LATENCY_LENGTH*sizeof(char*));
    if(setValues) {
        for(int i=0; i<ADM_PP_PARAM_LATENCY_LENGTH; i++) {
            setValues[i] = (char*)malloc(STRING_LENGTH_OF_INTEGER*sizeof(char));
            if(!setValues[i]) {
                ALOGE("memory allocation for seending device mute failed");
                isMemAvailable = false;
                memAllocIdx = i;
                break;
            }
        }
    } else {
        ALOGE("memory allocation for set device mute failed");
        isMemAvailable = false;
    }
    if (isMemAvailable) {
        port_id = mapDeviceToPort(device);
        sprintf(setValues[0], "%d", ADM_PP_PARAM_LATENCY_ID);
        sprintf(setValues[1], "%d", port_id);
        sprintf(setValues[2], "%d", delay);

        setMixerControlExt("Device PP Params", ADM_PP_PARAM_LATENCY_LENGTH, setValues);
    }
    for(int i=0; i<memAllocIdx; i++)
        if(setValues[i])
            free(setValues[i]);
    if(setValues)
        free(setValues);
    return NO_ERROR;

}


status_t ALSADevice::setChannelMap(alsa_handle_t *handle, int maxChannels,
                                       char *channelMap)
{
    status_t status = NO_ERROR;

    if(handle)
        status = pcm_set_channel_map(handle->handle, mMixer,
                                     maxChannels, channelMap);
    return status;
}

void ALSADevice::setChannelAlloc(int channelAlloc)
{
    ALOGD("channel allocation = 0x%x", channelAlloc);
    char** setValues;
    setValues = (char**)malloc(sizeof(char*));
    if (setValues == NULL) {
          return;
    }
    setValues[0] = (char*)malloc(4*sizeof(char));
    if (setValues[0] == NULL) {
          free(setValues);
          return;
    }
    sprintf(setValues[0], "%d", channelAlloc);
    setMixerControlExt("HDMI RX CA", 1, setValues);
    free(setValues[0]);
    free(setValues);
    return;
}

status_t ALSADevice::setCaptureFormat(const char *value)
{
    status_t err = NO_ERROR;

    err = setMixerControl("MI2S TX Format",value);

    if(err) {
        ALOGE("setPlaybackFormat error = %d",err);
    }

    return err;
}

status_t ALSADevice::setWMAParams(alsa_handle_t *handle, int params[], int size)
{
    status_t err = NO_ERROR;
    if (size > sizeof(mWMA_params)/sizeof(mWMA_params[0])) {
        ALOGE("setWMAParams too many params error");
        return BAD_VALUE;
    }
    for (int i = 0; i < size; i++)
        mWMA_params[i] = params[i];
    return err;
}

int ALSADevice::getALSABufferSize(alsa_handle_t *handle) {

    int format = 2;

    switch (handle->format) {
        case SNDRV_PCM_FORMAT_S8:
            format = 1;
        break;
        case SNDRV_PCM_FORMAT_S16_LE:
            format = 2;
        break;
        case SNDRV_PCM_FORMAT_S24_LE:
            format = 3;
        break;
        default:
           format = 2;
        break;
    }
    ALOGD("getALSABufferSize - handle->channels = %d,  handle->sampleRate = %d,\
            format = %d",handle->channels, handle->sampleRate,format);

    ALOGD("buff size is %d",((PCM_BUFFER_DURATION *  handle->channels\
            *  handle->sampleRate * format) / 1000000));
    int bufferSize = ((PCM_BUFFER_DURATION *  handle->channels
            *  handle->sampleRate * format) / 1000000);


    //Check for power of 2
    if (bufferSize & (bufferSize-1)) {

        bufferSize -= 1;
        for (uint32_t i=1; i<sizeof(bufferSize -1)*CHAR_BIT; i<<=1)
                bufferSize = bufferSize | bufferSize >> i;
        bufferSize += 1;
        ALOGV("Not power of 2 - buff size is = %d",bufferSize);
    }
    else {
        ALOGV("power of 2");
    }
    if(bufferSize < MULTI_CHANNEL_MIN_PERIOD_SIZE)
        bufferSize = MULTI_CHANNEL_MIN_PERIOD_SIZE;
    if(bufferSize >  MULTI_CHANNEL_MAX_PERIOD_SIZE)
        bufferSize = MULTI_CHANNEL_MAX_PERIOD_SIZE;

    return bufferSize;
}

status_t ALSADevice::setHDMIChannelCount(int channels)
{
    status_t err = NO_ERROR;
    const char *channel_cnt_str = NULL;
    ALOGE("dddd: channels in sethdmichannelcount = %d", channels);
    switch (channels) {
    case 8: channel_cnt_str = "Eight"; break;
    case 7: channel_cnt_str = "Seven"; break;
    case 6: channel_cnt_str = "Six"; break;
    case 5: channel_cnt_str = "Five"; break;
    case 4: channel_cnt_str = "Four"; break;
    case 3: channel_cnt_str = "Three"; break;
    default: channel_cnt_str = "Two"; break;
    }
    ALOGD("HDMI channel count: %s", channel_cnt_str);
    setMixerControl("HDMI_RX Channels",channel_cnt_str);
    return err;
}

uint64_t ALSADevice::updateDevices(const char * use_case, uint64_t devices) {
    if (strncmp(use_case, SND_USE_CASE_MOD_PLAY_VOIP,
               strlen(SND_USE_CASE_MOD_PLAY_VOIP)) &&
        strncmp(use_case, SND_USE_CASE_VERB_IP_VOICECALL,
               strlen(SND_USE_CASE_VERB_IP_VOICECALL))) {
        ALOGD("devices = %llx mHardwarePlatform = %d", devices, mHardwarePlatform);
        if ((devices & AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET) ||
             (devices & AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET)) {
           devices &= ~(AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET);
           devices |= (AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET);
           if(mHardwarePlatform == DTV_PLATFORM)
               devices |= AudioSystem::DEVICE_OUT_SPEAKER;
           else
               devices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
           if (getUseCaseType(use_case) == USECASE_TYPE_TX)
               devices = devices & AudioSystem::DEVICE_IN_ALL;
           else
               devices = devices & AudioSystem::DEVICE_OUT_ALL;
        }
    }

    if ((devices & AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET) ||
        (devices & AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET)) {
        devices &= ~(AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET);
        devices |= (AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET);
    }

    if ((devices & AudioSystem::DEVICE_OUT_SPEAKER) ||
        (devices & AudioSystem::DEVICE_OUT_WIRED_HEADSET)) {
        devices |= VIRTUAL_DEVICE_SLIMBUS_VIRTUAL_PORT;
        ALOGD("Adding virtual device, devices = %llx ", devices);
    }

    return devices;
}

int64_t ALSADevice::getDevices(uint64_t devices, char **rxDevice, char **txDevice)
{
    ALOGV("%s: device %llx", __FUNCTION__, devices);

    if ((devices & AudioSystem::DEVICE_OUT_WIRED_HEADSET) ||
        (devices & AudioSystem::DEVICE_IN_WIRED_HEADSET)) {
        devices = devices | (AudioSystem::DEVICE_OUT_WIRED_HEADSET |
                  AudioSystem::DEVICE_IN_WIRED_HEADSET);
    } else if (devices & AudioSystem::DEVICE_OUT_WIRED_HEADPHONE) {
        devices = devices | (AudioSystem::DEVICE_OUT_WIRED_HEADPHONE |
                  AudioSystem::DEVICE_IN_BUILTIN_MIC);
    } else if ((devices & AudioSystem::DEVICE_OUT_EARPIECE) ||
              (devices & AudioSystem::DEVICE_IN_BUILTIN_MIC)) {
        devices = devices | (AudioSystem::DEVICE_IN_BUILTIN_MIC |
                  AudioSystem::DEVICE_OUT_EARPIECE);
    } else if (devices & AudioSystem::DEVICE_OUT_SPEAKER &&
                         (mHardwarePlatform != DTV_PLATFORM)) {
        devices = devices | (AudioSystem::DEVICE_IN_DEFAULT |
                   AudioSystem::DEVICE_OUT_SPEAKER);
    } else if ((devices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO) ||
               (devices & AudioSystem::DEVICE_OUT_BLUETOOTH_SCO_HEADSET) ||
               (devices & AudioSystem::DEVICE_IN_BLUETOOTH_SCO_HEADSET)) {
        devices = devices | (AudioSystem::DEVICE_IN_BLUETOOTH_SCO_HEADSET |
                  AudioSystem::DEVICE_OUT_BLUETOOTH_SCO);
    } else if ((devices & AudioSystem::DEVICE_OUT_ANC_HEADSET) ||
               (devices & AudioSystem::DEVICE_IN_ANC_HEADSET)) {
        devices = devices | (AudioSystem::DEVICE_OUT_ANC_HEADSET |
                  AudioSystem::DEVICE_IN_ANC_HEADSET);
    } else if (devices & AudioSystem::DEVICE_OUT_ANC_HEADPHONE) {
        devices = devices | (AudioSystem::DEVICE_OUT_ANC_HEADPHONE |
                  AudioSystem::DEVICE_IN_BUILTIN_MIC);
#ifdef QCOM_USBAUDIO_ENABLED
    } else if ((devices & AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET) ||
             (devices & AudioSystem::DEVICE_OUT_ANLG_DOCK_HEADSET)) {
        devices |= AudioSystem::DEVICE_IN_ANLG_DOCK_HEADSET;
        if(mHardwarePlatform == DTV_PLATFORM)
            devices |= AudioSystem::DEVICE_OUT_SPEAKER;
        else
            devices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
#endif
    } else if (devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
        devices = devices | (AudioSystem::DEVICE_OUT_AUX_DIGITAL |
                  AudioSystem::DEVICE_IN_AUX_DIGITAL);
    } else if ((devices & AudioSystem::DEVICE_OUT_PROXY) ||
              (devices & AudioSystem::DEVICE_IN_PROXY)) {
        devices = devices | (AudioSystem::DEVICE_OUT_PROXY |
                  AudioSystem::DEVICE_IN_PROXY);
    } else if (devices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        ALOGE("getDevices:: Invalid A2DP Combination");
    }
    if(rxDevice != NULL)
        *rxDevice = getUCMDevice((uint64_t)(devices & AudioSystem::DEVICE_OUT_ALL)
                                 | (devices & VIRTUAL_DEVICE_MASK), 0);
    if(txDevice != NULL)
        *txDevice = getUCMDevice(devices & AudioSystem::DEVICE_IN_ALL, 1);

    return devices;
}

/**
 * Compares two usecase to check if they are same or can be replaced
 * with each other
 * Returns true if replaceable, false otherwise.
 *
 * Two usecases can be replaced by each other if
 * 1. They are same
 * 2. They differ only by a number(not more than two digit) at the end
 * 3. Both usecases should have number as suffix at the end.
 *
 * E.g. "Play Music1" and "Play Music2" are replaceable
 *      "Play Music11" and "Play Music9" are replaceable
 *    but "Play Music" and "Play Music2" are NOT REPLACEABLE
 *
 *    The last case is included to distinguise standard Usecases from
 *    Our Usecase definitions.
 */
bool ALSADevice::isUsecaseMatching(const char *usecase, const char *requsecase)
{
    int len1 = 0, len2 = 0, lc = 0;

    while (usecase[lc] != '\0' && requsecase[lc] != '\0'
           && usecase[lc] == requsecase[lc])
        lc++;

    len1 = strlen(usecase);
    len2 = strlen(requsecase);

    if (len1 == lc && len2 == lc)
        return true; /* Both Usecases are same */
    if (lc+2 < len1 || lc+2 < len2 || len1 == lc || len2 == lc)
        return false; /* Usecases differ by more than two characters*/

    while (len1-- > lc) {
        if (usecase[len1] < '0' || usecase[len1] > '9')
            return false; /* Characters at end the end are not numerals */
    }
    while (len2-- > lc) {
        if (requsecase[len2] < '0' || requsecase[len2] > '9')
            return false; /* Characters at end the end are not numerals */
    }

    return true; /* Replaceable Usecases */
}

/*
 * Checks whether the requested Usecase is already opened or conflicts otherwise
 * Returns int:
 *     0, if no conflicts detected
 *     1, if conflicts but alternative usecase is available
 *     NEGATIVE, Error values if usecase invalid or cannot be used
 */
int ALSADevice::checkAndGetAvailableUseCase(alsa_handle_t *handle, char altUsecase[])
{
    char const **list = NULL;
    int ret = 0, index = 0, listCount = 0;
    bool usecaseConflicts = false;
    ALSAHandleList::iterator it;

    ALOGD("checkAndGetAvailableUseCase usecase req: %s", handle->useCase);
    for (it = handle->module->mDeviceList->begin();
         it != handle->module->mDeviceList->end(); ++it) {
         if (handle != &(*it) && strncmp(it->useCase, handle->useCase, sizeof(handle->useCase))
             == 0 && it->handle && it->handle->fd > 0) {
             ALOGV("requsecase in conflict");
             usecaseConflicts = true;
             break;
         }
    }
    if (usecaseConflicts == false) {
       strlcpy(altUsecase, handle->useCase, sizeof(handle->useCase));
       return ret;
    }

    listCount = snd_use_case_get_list(handle->ucMgr, "_modifiers", &list);
    while(index < listCount) {
        if (isUsecaseMatching(list[index], handle->useCase) == true) {
            usecaseConflicts = false;
            for (it = handle->module->mDeviceList->begin();
                it != handle->module->mDeviceList->end(); ++it) {
                if (handle != &(*it) && strncmp(it->useCase, list[index], sizeof(handle->useCase))
                    == 0 && it->handle && it->handle->fd > 0) {
                    usecaseConflicts = true;
                    break;
                }
            }
            if (usecaseConflicts == false) {
                ALOGV("Alternative usecase suggested %s.", list[index]);
                strlcpy(altUsecase, list[index], MAX_STR_LEN);
                ret = 1;
                break;
            }
        }
        index++;
    }
    if (listCount < 0 || index == listCount)
        ret = -1;
    if (listCount > 0)
        snd_use_case_free_list(list, listCount);
    return ret;
}

int ALSADevice::setUseCase(alsa_handle_t *handle, bool bIsUseCaseSet)
{
    char altUseCase[MAX_STR_LEN] = "";
    int ret = 0;
    uint32_t devices;

    // check for Conflicts if usecase is already set
    ALOGD("setUseCase device = %llx", handle->activeDevice);
    if (!bIsUseCaseSet) {
        ret = checkAndGetAvailableUseCase(handle, altUseCase);
        if (ret > 0 && altUseCase[0] != '\0') {
            strlcpy(handle->useCase, altUseCase, sizeof(handle->useCase));
        } else if (ret < 0) {
            ALOGV("no valid match for usecase found,req usecase %s", handle->useCase);
            return -1;
        }
    }
    handle->devices = handle->activeDevice = updateDevices(handle->useCase, handle->activeDevice);
    enableDevice(handle, bIsUseCaseSet);

   return 0;
}

status_t ALSADevice::exitReadFromProxy()
{
    ALOGV("exitReadFromProxy");
    mProxyParams.mExitRead = true;
    if(mProxyParams.mPfdProxy[1].fd != -1) {
        uint64_t writeValue = KILL_A2DP_THREAD;
        ALOGD("Writing to mPfdProxy[1].fd %d",mProxyParams.mPfdProxy[1].fd);
        write(mProxyParams.mPfdProxy[1].fd, &writeValue, sizeof(uint64_t));
    }
    return NO_ERROR;
}

void ALSADevice::resetProxyVariables() {

    mProxyParams.mAvail = 0;
    mProxyParams.mFrames = 0;
    if(mProxyParams.mPfdProxy[1].fd != -1) {
        sys_close::lib_close(mProxyParams.mPfdProxy[1].fd);
        mProxyParams.mPfdProxy[1].fd = -1;
    }
}

ssize_t  ALSADevice::readFromProxy(void **captureBuffer , ssize_t *bufferSize) {

    status_t err = NO_ERROR;
    int err_poll = 0;
    initProxyParams();
    err = startProxy();
    if(err) {
        ALOGE("ReadFromProxy-startProxy returned err = %d", err);
        *captureBuffer = NULL;
        *bufferSize = 0;
        return err;
    }
    struct pcm * capture_handle = (struct pcm *)mProxyParams.mProxyPcmHandle;

    while(!mProxyParams.mExitRead) {
        err = sync_ptr(capture_handle);
        if(err == EPIPE) {
               ALOGE("Failed in sync_ptr \n");
               /* we failed to make our window -- try to restart */
               capture_handle->underruns++;
               capture_handle->running = 0;
               capture_handle->start = 0;
               continue;
        }else if(err != NO_ERROR){
                ALOGE("Error: Sync ptr returned %d", err);
                break;
        }

        mProxyParams.mAvail = pcm_avail(capture_handle);
        if (mProxyParams.mAvail < capture_handle->sw_p->avail_min) {
            err_poll = poll(mProxyParams.mPfdProxy, NUM_FDS, TIMEOUT_INFINITE);
            if(mProxyParams.mPfdProxy[0].revents) {
                struct snd_timer_tread rbuf[4];
                read(mProxyParams.mProxyPcmHandle->timer_fd, rbuf, sizeof(struct snd_timer_tread) * 4 );
            }
            if (mProxyParams.mPfdProxy[1].revents & POLLIN) {
                ALOGV("Event on userspace fd");
                mProxyParams.mPfdProxy[1].revents = 0;
            }
            if ((mProxyParams.mPfdProxy[1].revents & POLLERR) ||
                    (mProxyParams.mPfdProxy[1].revents & POLLNVAL)) {
                ALOGV("POLLERR or INVALID POLL");
                err = BAD_VALUE;
                break;
            }
            if((mProxyParams.mPfdProxy[0].revents & POLLERR) ||
                    (mProxyParams.mPfdProxy[0].revents & POLLNVAL)) {
                ALOGV("POLLERR or INVALID POLL on zero");
                err = BAD_VALUE;
                break;
            }
            if (mProxyParams.mPfdProxy[0].revents & POLLIN) {
                ALOGV("POLLIN on zero");
                mProxyParams.mPfdProxy[0].revents = 0;
            }
            continue;
        }
        break;
    }
    if(err != NO_ERROR) {
        ALOGE("Reading from proxy failed = err = %d", err);
        *captureBuffer = NULL;
        *bufferSize = 0;
        return err;
    }
    void *data  = dst_address(capture_handle);
    //TODO: Return a pointer to AudioHardware
    if(mProxyParams.mCaptureBuffer == NULL)
        mProxyParams.mCaptureBuffer =  malloc(mProxyParams.mCaptureBufferSize);
    memcpy(mProxyParams.mCaptureBuffer, (char *)data,
             mProxyParams.mCaptureBufferSize);
    capture_handle->sync_ptr->c.control.appl_ptr += mProxyParams.mFrames;
    capture_handle->sync_ptr->flags = 0;
    ALOGV("Calling sync_ptr for proxy after sync");
    err = sync_ptr(capture_handle);
    if(err == EPIPE) {
        ALOGV("Failed in sync_ptr \n");
        capture_handle->running = 0;
        err = sync_ptr(capture_handle);
    }
    if(err != NO_ERROR ) {
        ALOGE("Error: Sync ptr end returned %d", err);
        *captureBuffer = NULL;
        *bufferSize = 0;
        return err;
    }
    *captureBuffer = mProxyParams.mCaptureBuffer;
    *bufferSize = mProxyParams.mCaptureBufferSize;
    return err;
}

void ALSADevice::initProxyParams() {
    if(mProxyParams.mPfdProxy[1].fd == -1) {
        ALOGV("Allocating A2Dp poll fd");
        int channels = mProxyParams.mProxyPcmHandle->channels;
        mProxyParams.mPfdProxy[0].fd = mProxyParams.mProxyPcmHandle->timer_fd;
        mProxyParams.mPfdProxy[0].events = (POLLIN | POLLERR | POLLNVAL);
        mProxyParams.mPfdProxy[0].revents = 0;
        ALOGV("Allocated A2DP poll fd");
        mProxyParams.mPfdProxy[1].fd = eventfd(0,0);
        mProxyParams.mPfdProxy[1].events = (POLLIN | POLLERR | POLLNVAL);
        mProxyParams.mPfdProxy[1].revents = 0;
        mProxyParams.mFrames =
            (mProxyParams.mProxyPcmHandle->period_size / (2*channels));
    }
}

status_t ALSADevice::startProxy() {

    status_t err = NO_ERROR;
    struct pcm * capture_handle = (struct pcm *)mProxyParams.mProxyPcmHandle;
    while(1) {
        if (!capture_handle->start) {
            if(ioctl(capture_handle->fd, SNDRV_PCM_IOCTL_START)) {
                err = -errno;
                if (errno == EPIPE) {
                   ALOGV("Failed in SNDRV_PCM_IOCTL_START\n");
                   /* we failed to make our window -- try to restart */
                   capture_handle->underruns++;
                   capture_handle->running = 0;
                   capture_handle->start = 0;
                   continue;
                } else {
                   ALOGE("IGNORE - IOCTL_START failed for proxy err: %d \n", errno);
                   err = NO_ERROR;
                   break;
                }
           } else {
               ALOGD(" Proxy Driver started(IOCTL_START Success)\n");
               break;
           }
       }
       else {
           ALOGV("Proxy Already started break out of condition");
           break;
       }
   }
   ALOGV("startProxy - Proxy started");
   capture_handle->start = 1;
   capture_handle->sync_ptr->flags = SNDRV_PCM_SYNC_PTR_APPL |
               SNDRV_PCM_SYNC_PTR_AVAIL_MIN;
   return err;
}

status_t ALSADevice::openProxyDevice()
{
    struct snd_pcm_hw_params *params = NULL;
    struct snd_pcm_sw_params *sparams = NULL;
    int flags = (DEBUG_ON | PCM_MMAP| PCM_STEREO | PCM_IN);
    int channels;

    ALOGV("openProxyDevice");
    mProxyParams.mProxyPcmHandle = pcm_open(flags, PROXY_CAPTURE_DEVICE_NAME);
    if (!pcm_ready(mProxyParams.mProxyPcmHandle)) {
        ALOGE("Opening proxy device failed");
        goto bail;
    }
    ALOGV("Proxy device opened successfully: mProxyPcmHandle %p", mProxyParams.mProxyPcmHandle);
    mProxyParams.mProxyPcmHandle->channels = AFE_PROXY_CHANNEL_COUNT;
    mProxyParams.mProxyPcmHandle->rate     = AFE_PROXY_SAMPLE_RATE;
    mProxyParams.mProxyPcmHandle->flags    = flags;
    mProxyParams.mProxyPcmHandle->period_size = AFE_PROXY_PERIOD_SIZE;
    mProxyParams.mBufferTime = (AFE_PROXY_PERIOD_SIZE*1000)/(AFE_PROXY_CHANNEL_COUNT*AFE_PROXY_SAMPLE_RATE*2);

    params = (struct snd_pcm_hw_params*) calloc(1,sizeof(struct snd_pcm_hw_params));
    if (!params) {
         goto bail;
    }


    channels = mProxyParams.mProxyPcmHandle->channels;
    param_init(params);

    param_set_mask(params, SNDRV_PCM_HW_PARAM_ACCESS,
            (mProxyParams.mProxyPcmHandle->flags & PCM_MMAP)?
            SNDRV_PCM_ACCESS_MMAP_INTERLEAVED
            : SNDRV_PCM_ACCESS_RW_INTERLEAVED);
    param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
            SNDRV_PCM_FORMAT_S16_LE);
    param_set_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT,
            SNDRV_PCM_SUBFORMAT_STD);
    param_set_min(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
            mProxyParams.mProxyPcmHandle->period_size);
    param_set_int(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 16);
    param_set_int(params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
            mProxyParams.mProxyPcmHandle->channels * 16);
    param_set_int(params, SNDRV_PCM_HW_PARAM_CHANNELS,
            mProxyParams.mProxyPcmHandle->channels);
    param_set_int(params, SNDRV_PCM_HW_PARAM_RATE,
            mProxyParams.mProxyPcmHandle->rate);

    //default the format to SNDRV_PCM_FORMAT_S16_LE
    mProxyParams.mProxyPcmHandle->format = SNDRV_PCM_FORMAT_S16_LE;
    mProxyParams.mProxyPcmHandle->bytes_per_sample = 2;
    param_set_hw_refine(mProxyParams.mProxyPcmHandle, params);

    if (param_set_hw_params(mProxyParams.mProxyPcmHandle, params)) {
        ALOGE("Failed to set hardware params on Proxy device");
        goto bail;
    }

    mProxyParams.mProxyPcmHandle->buffer_size = pcm_buffer_size(params);
    mProxyParams.mProxyPcmHandle->period_size = pcm_period_size(params);
    mProxyParams.mProxyPcmHandle->period_cnt  =
            mProxyParams.mProxyPcmHandle->buffer_size /
            mProxyParams.mProxyPcmHandle->period_size;
    ALOGV("Capture - period_size (%d)",\
            mProxyParams.mProxyPcmHandle->period_size);
    ALOGV("Capture - buffer_size (%d)",\
            mProxyParams.mProxyPcmHandle->buffer_size);
    ALOGV("Capture - period_cnt  (%d)\n",\
            mProxyParams.mProxyPcmHandle->period_cnt);
    sparams = (struct snd_pcm_sw_params*) calloc(1,sizeof(struct snd_pcm_sw_params));
    if (!sparams) {
        ALOGE("Failed to allocated software params for Proxy device");
        goto bail;
    }

   sparams->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
   sparams->period_step = 1;
   sparams->avail_min =
           mProxyParams.mProxyPcmHandle->period_size/(2*channels);
   sparams->start_threshold = 1;
   sparams->stop_threshold = INT_MAX;
   sparams->xfer_align =
           mProxyParams.mProxyPcmHandle->period_size/(2*channels);
   sparams->silence_size = 0;
   sparams->silence_threshold = 0;

   if (param_set_sw_params(mProxyParams.mProxyPcmHandle, sparams)) {
        ALOGE("Failed to set software params on Proxy device");
        goto bail;
   }
   mmap_buffer(mProxyParams.mProxyPcmHandle);

   if (pcm_prepare(mProxyParams.mProxyPcmHandle)) {
       ALOGE("Failed to pcm_prepare on Proxy device");
       goto bail;
   }
   mProxyParams.mProxyState = proxy_params::EProxyOpened;
   return NO_ERROR;

bail:
   if(mProxyParams.mProxyPcmHandle)  {
       pcm_close(mProxyParams.mProxyPcmHandle);
       mProxyParams.mProxyPcmHandle = NULL;
   }
   mProxyParams.mProxyState = proxy_params::EProxyClosed;
   return NO_INIT;
}

status_t ALSADevice::closeProxyDevice() {
    status_t err = NO_ERROR;
    if(mProxyParams.mProxyPcmHandle) {
        pcm_close(mProxyParams.mProxyPcmHandle);
        mProxyParams.mProxyPcmHandle = NULL;
    }
    resetProxyVariables();
    mProxyParams.mProxyState = proxy_params::EProxyClosed;
    mProxyParams.mExitRead = false;
    return err;
}

bool ALSADevice::isProxyDeviceOpened() {

   //TODO : Add some intelligence to return appropriate value
   if(mProxyParams.mProxyState == proxy_params::EProxyOpened ||
           mProxyParams.mProxyState == proxy_params::EProxyCapture ||
           mProxyParams.mProxyState == proxy_params::EProxySuspended)
       return true;
   return false;
}

bool ALSADevice::isProxyDeviceSuspended() {

   if(mProxyParams.mProxyState == proxy_params::EProxySuspended)
        return true;
   return false;
}

bool ALSADevice::suspendProxy() {

   status_t err = NO_ERROR;
   if(mProxyParams.mProxyState == proxy_params::EProxyOpened ||
           mProxyParams.mProxyState == proxy_params::EProxyCapture) {
       mProxyParams.mProxyState = proxy_params::EProxySuspended;
   }
   else {
       ALOGE("Proxy already suspend or closed, in state = %d",\
                mProxyParams.mProxyState);
   }
   return err;
}

bool ALSADevice::resumeProxy() {

   status_t err = NO_ERROR;
   struct pcm *capture_handle= mProxyParams.mProxyPcmHandle;
   ALOGD("resumeProxy mProxyParams.mProxyState = %d, capture_handle =%p",\
           mProxyParams.mProxyState, capture_handle);
   if((mProxyParams.mProxyState == proxy_params::EProxyOpened ||
           mProxyParams.mProxyState == proxy_params::EProxySuspended) &&
           capture_handle != NULL) {
       ALOGV("pcm_prepare from Resume");
       capture_handle->start = 0;
       err = pcm_prepare(capture_handle);
       if(err != OK) {
           ALOGE("IGNORE: PCM Prepare - capture failed err = %d", err);
       }
       err = startProxy();
       if(err) {
           ALOGE("IGNORE:startProxy returned error = %d", err);
       }
       mProxyParams.mProxyState = proxy_params::EProxyCapture;
       err = sync_ptr(capture_handle);
       if (err) {
           ALOGE("IGNORE: sync ptr from resumeProxy returned error = %d", err);
       }
       ALOGV("appl_ptr= %d", (int)capture_handle->sync_ptr->c.control.appl_ptr);
   }
   else {
        ALOGE("resume Proxy ignored in invalid state - ignore");
        if(mProxyParams.mProxyState == proxy_params::EProxyClosed ||
                capture_handle == NULL) {
            ALOGE("resumeProxy = BAD_VALUE");
            err = BAD_VALUE;
            return err;
        }
   }
   return NO_ERROR;
}

void ALSADevice::setUseCase(alsa_handle_t *handle, bool bIsUseCaseSet, char *device)
{
    if(bIsUseCaseSet)
        snd_use_case_set_case(handle->ucMgr, "_verb", handle->useCase, device);
    else
        snd_use_case_set_case(handle->ucMgr, "_enamod", handle->useCase, device);
}

void ALSADevice::removeUseCase(alsa_handle_t *handle, char *device)
{
    snd_use_case_set_case(handle->ucMgr, "_disdev", device, handle->useCase);
}

status_t ALSADevice::openCapture(alsa_handle_t *handle,
                                 bool isMmapMode,
                                 bool isCompressed)
{
    char *devName = NULL;
    unsigned flags = PCM_IN | DEBUG_ON;
    int err = NO_ERROR;

    close(handle);

    ALOGD("openCapture: handle %p", handle);

    if (handle->channels == 1)
        flags |= PCM_MONO;
    else if (handle->channels == 3)
        flags |= PCM_TRIPLE;
    else if (handle->channels == 4)
        flags |= PCM_QUAD;
    else if (handle->channels == 5)
        flags |= PCM_PENTA;
    else if (handle->channels == 6)
        flags |= PCM_5POINT1;
    else if (handle->channels == 7)
        flags |= PCM_7POINT;
    else if (handle->channels == 8)
        flags |= PCM_7POINT1;
    else
        flags |= PCM_STEREO;

    if(isMmapMode)
        flags |= PCM_MMAP;
    else
        flags |= PCM_NMMAP;

    if (deviceName(handle, flags, &devName) < 0) {
        ALOGE("Failed to get pcm device node: %s", devName);
        return NO_INIT;
    }
    if(devName != NULL)
        handle->handle = pcm_open(flags, (char*)devName);
    ALOGE("s_open: opening ALSA device '%s'", devName);

    if(devName != NULL)
        free(devName);

    if (!handle->handle) {
        ALOGE("s_open: Failed to initialize ALSA device '%s'", devName);
        return NO_INIT;
    }

    handle->handle->flags = flags;

    ALOGD("setting hardware parameters");
    err = setCaptureHardwareParams(handle, isCompressed);
    if (err == NO_ERROR) {
        ALOGD("setting software parameters");
        err = setCaptureSoftwareParams(handle, isCompressed);
    }
    if(err != NO_ERROR) {
        ALOGE("Set HW/SW params failed: Closing the pcm stream");
        standby(handle);
        return err;
    }

    if(mmap_buffer(handle->handle)) {
        ALOGE("Failed to mmap the buffer");
        standby(handle);
        return NO_INIT;
    }

    if (pcm_prepare(handle->handle)) {
        ALOGE("Failed to pcm_prepare on caoture stereo device");
        standby(handle);
        return NO_INIT;
    }

    return NO_ERROR;
}


status_t ALSADevice::setCaptureHardwareParams(alsa_handle_t *handle, bool isCompressed)
{
    struct snd_pcm_hw_params *params;
    struct snd_compr_caps compr_cap;
    struct snd_compr_params compr_params;

    int32_t minPeroid, maxPeroid;
    unsigned long bufferSize, reqBuffSize;
    unsigned int periodTime, bufferTime;

    int status = 0;
    status_t err = NO_ERROR;
    unsigned int requestedRate = handle->sampleRate;
    int format = handle->format;

    reqBuffSize = handle->bufferSize;
    if (isCompressed) {
        if (ioctl(handle->handle->fd, SNDRV_COMPRESS_GET_CAPS, &compr_cap)) {
            ALOGE("SNDRV_COMPRESS_GET_CAPS, failed Error no %d \n", errno);
            err = -errno;
            return err;
        }

        minPeroid = compr_cap.min_fragment_size;
        maxPeroid = compr_cap.max_fragment_size;
        compr_params.codec.id = compr_cap.codecs[10];
        handle->channels = 2;
//NOTE:
// channels = 2 would set 1 MI2S line, greater than 2 will set more than
// 1 MI2S lines
        ALOGV("Min peroid size = %d , Maximum Peroid size = %d",\
            minPeroid, maxPeroid);
        if (ioctl(handle->handle->fd, SNDRV_COMPRESS_SET_PARAMS,
                  &compr_params)) {
            ALOGE("SNDRV_COMPRESS_SET_PARAMS,failed Error no %d \n", errno);
            err = -errno;
            return err;
        }
    } else {
        if (ioctl(handle->handle->fd, SNDRV_COMPRESS_GET_CAPS, &compr_cap)) {
            ALOGE("SNDRV_COMPRESS_GET_CAPS, failed Error no %d \n", errno);
            err = -errno;
            return err;
        }

        minPeroid = compr_cap.min_fragment_size;
        maxPeroid = compr_cap.max_fragment_size;
        compr_params.codec.id = compr_cap.codecs[11];
        ALOGV("Min peroid size = %d , Maximum Peroid size = %d",\
            minPeroid, maxPeroid);
        if (ioctl(handle->handle->fd, SNDRV_COMPRESS_SET_PARAMS,
                  &compr_params)) {
            ALOGE("SNDRV_COMPRESS_SET_PARAMS,failed Error no %d \n", errno);
            err = -errno;
            return err;
        }
    }

    params = (snd_pcm_hw_params*) calloc(1, sizeof(struct snd_pcm_hw_params));
    if (!params) {
        ALOGE("Failed to allocate ALSA hardware parameters!");
        return NO_INIT;
    }

    ALOGD("setHardwareParamsCapture: reqBuffSize %d channels %d sampleRate %d",
         (int) reqBuffSize, handle->channels, handle->sampleRate);

    param_init(params);
    param_set_mask(params, SNDRV_PCM_HW_PARAM_ACCESS,
                   (handle->handle->flags & PCM_MMAP) ?
                       SNDRV_PCM_ACCESS_MMAP_INTERLEAVED :
                       SNDRV_PCM_ACCESS_RW_INTERLEAVED);
    param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
                   SNDRV_PCM_FORMAT_S16_LE);
    param_set_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT,
                   SNDRV_PCM_SUBFORMAT_STD);
    param_set_int(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
                  handle->periodSize);
    param_set_int(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 16);
    param_set_int(params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
                  handle->channels* 16);
    param_set_int(params, SNDRV_PCM_HW_PARAM_CHANNELS,
                  handle->channels);
    param_set_int(params, SNDRV_PCM_HW_PARAM_RATE,
                  handle->sampleRate);
    param_set_hw_refine(handle->handle, params);
    if (param_set_hw_params(handle->handle, params)) {
        ALOGE("Failed to set hardware params on stereo capture device");
        return NO_INIT;
    }
    handle->handle->buffer_size = pcm_buffer_size(params);
    handle->handle->period_size = pcm_period_size(params);
    handle->handle->period_cnt  = handle->handle->buffer_size /
                                     handle->handle->period_size;
    ALOGV("period_size %d, period_cnt %d", handle->handle->period_size,
              handle->handle->period_cnt);
    handle->handle->rate = handle->sampleRate;
    handle->handle->channels = handle->channels;
    handle->periodSize = handle->handle->period_size;
    handle->bufferSize = handle->handle->period_size;
    if (handle->handle->format == SNDRV_PCM_FORMAT_S24_LE)
             handle->handle->bytes_per_sample = 4;
    else
             handle->handle->bytes_per_sample = 2;

    return NO_ERROR;
}

status_t ALSADevice::setCaptureSoftwareParams(alsa_handle_t *handle,
                                              bool isCompressed)
{
    struct snd_pcm_sw_params* params;
    struct pcm* pcm = handle->handle;

    unsigned long periodSize = pcm->period_size;
    unsigned flags = pcm->flags;

    params = (snd_pcm_sw_params*) calloc(1, sizeof(struct snd_pcm_sw_params));
    if (!params) {
        ALOGE("Failed to allocate ALSA software parameters!");
        return NO_INIT;
    }

    if(handle->timeStampMode == SNDRV_PCM_TSTAMP_ENABLE)
	    params->tstamp_mode = SNDRV_PCM_TSTAMP_ENABLE;
    else
        params->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
    params->period_step = 1;
    if ((pcm->flags & PCM_MONO)||(pcm->flags & PCM_TRIPLE)||
        (pcm->flags & PCM_QUAD)||(pcm->flags & PCM_PENTA)||
        (pcm->flags & PCM_5POINT1) ||(pcm->flags & PCM_7POINT)||
        (pcm->flags & PCM_7POINT1)){
          params->avail_min = pcm->period_size/(pcm->bytes_per_sample * pcm-> channels);
          params->xfer_align = pcm->period_size/(pcm->bytes_per_sample * pcm-> channels);
    } else {
          params->avail_min = pcm->period_size/(2*pcm->bytes_per_sample);
          params->xfer_align = pcm->period_size/(2*pcm->bytes_per_sample);
    }

    params->start_threshold = 1;
    params->stop_threshold = INT_MAX;
    params->silence_threshold = 0;
    params->silence_size = 0;

    if (param_set_sw_params(handle->handle, params)) {
        ALOGE("cannot set sw params");
        return NO_INIT;
    }
    return NO_ERROR;
}

}