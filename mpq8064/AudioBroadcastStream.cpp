/* AudioBroadcastStreamALSA.cpp
 **
 ** Copyright 2008-2009 Wind River Systems
 ** Copyright (c) 2011-2013, The Linux Foundation. All rights reserved
 ** Not a Contribution, Apache license notifications and license are retained
 ** for attribution purposes only.
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

#include <errno.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>

#define LOG_TAG "AudioBroadcastStreamALSA"
//#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0
#include <utils/Log.h>
#include <utils/String8.h>

#include <cutils/properties.h>
#include <media/AudioRecord.h>
#include <hardware_legacy/power.h>

#include <linux/ioctl.h>

#include "AudioHardwareALSA.h"
#include <sys/prctl.h>
#include <sys/resource.h>
#include <pthread.h>
#include <linux/unistd.h>

#define COMPRE_CAPTURE_HEADER_SIZE (sizeof(struct snd_compr_audio_info))

namespace sys_broadcast {
    ssize_t lib_write(int fd, const void *buf, size_t count) {
        return write(fd, buf, count);
    }
    ssize_t lib_close(int fd) {
        return close(fd);
    }
};

namespace android_audio_legacy
{

// ----------------------------------------------------------------------------

AudioBroadcastStreamALSA::AudioBroadcastStreamALSA(AudioHardwareALSA *parent,
                                                   uint32_t  devices,
                                                   int      format,
                                                   uint32_t channels,
                                                   uint32_t sampleRate,
                                                   uint32_t audioSource,
                                                   status_t *status)
{
    /* ------------------------------------------------------------------------
    Description: Initialize Flags and device handles
    -------------------------------------------------------------------------*/
    initialization();

    /* ------------------------------------------------------------------------
    Description: set config parameters from input arguments and error handling
    -------------------------------------------------------------------------*/
    mParent         = parent;
    mDevices        = devices;
    mInputFormat    = format;
    mSampleRate     = sampleRate;
    mChannels       = channels;
    mAudioSource    = audioSource;
    mALSADevice     = mParent->mALSADevice;
    mUcMgr          = mParent->mUcMgr;
    mA2dpUseCase    = AudioHardwareALSA::USECASE_NONE;

    ALOGD("devices:%d, format:%d, channels:%d, sampleRate:%d, audioSource:%d",
        devices, format, channels, sampleRate, audioSource);

    if(!(devices & AudioSystem::DEVICE_OUT_ALL) ||
       (mSampleRate == 0) || ((mChannels < 1) && (mChannels > 8)) ||
       ((audioSource != QCOM_AUDIO_SOURCE_DIGITAL_BROADCAST_MAIN_AD) &&
        (audioSource != QCOM_AUDIO_SOURCE_DIGITAL_BROADCAST_MAIN_ONLY) &&
        (audioSource != QCOM_AUDIO_SOURCE_ANALOG_BROADCAST) &&
        (audioSource != QCOM_AUDIO_SOURCE_HDMI_IN ))) {
        ALOGE("invalid config");
        *status = BAD_VALUE;
        return;
    }
    if(audioSource == QCOM_AUDIO_SOURCE_HDMI_IN) {
        if((format != QCOM_BROADCAST_AUDIO_FORMAT_LPCM) &&
           (format != QCOM_BROADCAST_AUDIO_FORMAT_COMPRESSED) &&
           (format != QCOM_BROADCAST_AUDIO_FORMAT_COMPRESSED_HBR)) {
            ALOGE("invalid config");
            *status = BAD_VALUE;
            return;
        }
        mChannels = mChannels%2?mChannels+1:mChannels;
    } else if(audioSource == QCOM_AUDIO_SOURCE_ANALOG_BROADCAST) {
        if((format != QCOM_BROADCAST_AUDIO_FORMAT_LPCM) &&
           (mChannels != 2)) {
            ALOGE("invalid config");
            *status = BAD_VALUE;
            return;
        }
        if(((format == AudioSystem::AAC) ||
            (format == AudioSystem::HE_AAC_V1) ||
            (format == AudioSystem::HE_AAC_V2) ||
            (format == AudioSystem::AC3) ||
            (format == AudioSystem::AC3_PLUS) ||
            (format == AudioSystem::EAC3)) &&
            (!(dlopen ("libms11.so", RTLD_NOW)))) {
            ALOGE("MS11 Decoder not available");
            *status = BAD_VALUE;
            return;
        }
    } else {
        if(!(format & AudioSystem::MAIN_FORMAT_MASK)) {
            ALOGE("invalid config");
            *status = BAD_VALUE;
            return;
        }
        mFormat             = format;
        mAacConfigDataSet   = true;
    }
    /* ------------------------------------------------------------------------
    Description: set the output device format
    -------------------------------------------------------------------------*/
    updateOutputFormat();

    /* ------------------------------------------------------------------------
    Description: Set appropriate flags based on the configuration parameters
    -------------------------------------------------------------------------*/
    *status = openCapturingAndRoutingDevices();

    if(*status != NO_ERROR) {
        ALOGE("Could not open the capture and routing devices");
        *status = BAD_VALUE;
         return;
    }
    return;
}

AudioBroadcastStreamALSA::~AudioBroadcastStreamALSA()
{
    ALOGD("Destructor ++");

    mSkipWrite = true;
    mWriteCv.signal();

    exitFromCaptureThread();

    alsa_handle_t *compreRxHandle_dup = mCompreRxHandle;
    alsa_handle_t *pcmRxHandle_dup = mPcmRxHandle;
    alsa_handle_t *compreTxHandle_dup = mCompreTxHandle;
    alsa_handle_t *pcmTxHandle_dup = mPcmTxHandle;

    if(mPcmTxHandle) {
        closeDevice(mPcmTxHandle);
        mALSADevice->removeUseCase(mPcmTxHandle, "MI2S");
    }

    if(mCompreTxHandle) {
        closeDevice(mCompreTxHandle);
        mALSADevice->removeUseCase(mCompreTxHandle, "MI2S");
    }

    if(mTranscodeHandle) {
        closeDevice(mTranscodeHandle);
        free(mTranscodeHandle);
        mTranscodeHandle = NULL;
    }

    exitFromPlaybackThread();

    if(mPcmRxHandle) {
        closeDevice(mPcmRxHandle);
        if(mPcmWriteTempBuffer)
            free(mPcmWriteTempBuffer);
    }

    if(mCompreRxHandle) {
        closeDevice(mCompreRxHandle);
        if(mCompreWriteTempBuffer)
            free(mCompreWriteTempBuffer);
    }

    if (mRouteAudioToA2dp) {
        ALOGD("stopA2dpPlayback_l for broadcast usecase %x", mA2dpUseCase);
        status_t status = mParent->stopA2dpPlayback_l(mA2dpUseCase);
        if (status)
            ALOGW("stopA2dpPlayback_l for broadcast returned %d", status);
        mRouteAudioToA2dp = false;
    }

    for(ALSAHandleList::iterator it = mParent->mDeviceList.begin();
        it != mParent->mDeviceList.end(); ++it) {
        alsa_handle_t *it_dup = &(*it);
        if(compreRxHandle_dup == it_dup || pcmRxHandle_dup == it_dup ||
           compreTxHandle_dup == it_dup || pcmTxHandle_dup == it_dup) {
            mParent->mDeviceList.erase(it);
            it_dup = NULL;
        }
    }

    if(mMS11Decoder)
        delete mMS11Decoder;

    if(mBitstreamSM)
        delete mBitstreamSM;

    initialization();

    for(ALSAHandleList::iterator it = mParent->mDeviceList.begin();
            it != mParent->mDeviceList.end(); ++it) {
        if((!strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_REC_COMPRESSED,
                strlen(SND_USE_CASE_VERB_HIFI_REC_COMPRESSED))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED,
                strlen(SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED))) ||
           (!strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_REC2,
                 strlen(SND_USE_CASE_VERB_HIFI_REC2))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC2,
                 strlen(SND_USE_CASE_MOD_CAPTURE_MUSIC2))) ||
           (!strncmp(it->useCase, SND_USE_CASE_VERB_HIFI3,
                 strlen(SND_USE_CASE_VERB_HIFI3))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC3,
                 strlen(SND_USE_CASE_MOD_PLAY_MUSIC3))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC4,
                 strlen(SND_USE_CASE_MOD_PLAY_MUSIC4)))||
           (!strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
                 strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
                 strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
                 strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3)))) {
            mParent->mDeviceList.erase(it);
        }
    }
    //clear avsync metadatalist
    for(inputMetadataList::iterator it = mInputMetadataListPcm.begin();
            it != mInputMetadataListPcm.end(); ++it)
        mInputMetadataListPcm.erase(it);
    for(inputMetadataList::iterator it = mInputMetadataListCompre.begin();
            it != mInputMetadataListCompre.end(); ++it)
        mInputMetadataListCompre.erase(it);
    ALOGD("Destructor --");
}

status_t AudioBroadcastStreamALSA::setParameters(const String8& keyValuePairs)
{
    ALOGV("setParameters");
    Mutex::Autolock autoLock(mControlLock);
    status_t status = NO_ERROR;
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key = String8(AudioParameter::keyRouting);
    int device;
    if (param.getInt(key, device) == NO_ERROR) {
        // Ignore routing if device is 0.
        ALOGD("setParameters(): keyRouting with device %d", device);
        if(device) {
            //ToDo: Call device setting UCM API here
            doRouting(device);
        }
        param.remove(key);
    } else {
        mControlLock.unlock();
        mParent->setParameters(keyValuePairs);
    }

    return status;
}

String8 AudioBroadcastStreamALSA::getParameters(const String8& keys)
{
    ALOGV("getParameters");
    Mutex::Autolock autoLock(mControlLock);
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        param.addInt(key, (int)mDevices);
    }

    ALOGV("getParameters() %s", param.toString().string());
    return param.toString();
}

status_t AudioBroadcastStreamALSA::start(int64_t absTimeToStart)
{
    ALOGV("start");
    Mutex::Autolock autoLock(mLock);
    status_t status = NO_ERROR;
    // 1. Set the absolute time stamp
    // ToDo: We need the ioctl from driver to set the time stamp

    // 2. Signal the driver to start rendering data
    if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_START)) {
        ALOGE("start:SNDRV_PCM_IOCTL_START failed\n");
        status = BAD_VALUE;
    }
    return status;
}

status_t AudioBroadcastStreamALSA::mute(bool mute)
{
    ALOGV("mute");
    Mutex::Autolock autoLock(mControlLock);
    status_t status = NO_ERROR;
    uint32_t volume;
    if(mute) {
        // Set the volume to 0 to mute the stream
        volume = 0;
    } else {
        // Set the volume back to current volume
        volume = mStreamVol;
    }
    if(mPcmRxHandle) {
        if(mPcmRxHandle->type == PCM_FORMAT) {
            status = mPcmRxHandle->module->setPlaybackVolume(volume,
                                               mPcmRxHandle->useCase);
        }
    } else if(mCompreRxHandle) {
        if(mSpdifFormat != COMPRESSED_FORMAT && mHdmiFormat != COMPRESSED_FORMAT) {
            status = mCompreRxHandle->module->setPlaybackVolume(volume,
                                                 mCompreRxHandle->useCase);
        }
    }
    return OK;
}

status_t AudioBroadcastStreamALSA::setVolume(float left, float right)
{
    ALOGV("setVolume");
    Mutex::Autolock autoLock(mControlLock);
    float volume;
    status_t status = NO_ERROR;

    volume = (left + right) / 2;
    if (volume < 0.0) {
        ALOGD("setVolume(%f) under 0.0, assuming 0.0\n", volume);
        volume = 0.0;
    } else if (volume > 1.0) {
        ALOGD("setVolume(%f) over 1.0, assuming 1.0\n", volume);
        volume = 1.0;
    }
    mStreamVol = lrint((volume * 0x2000)+0.5);

    ALOGD("Setting broadcast stream volume to %d \
                 (available range is 0 to 100)\n", mStreamVol);
    if(mPcmRxHandle) {
        if(mPcmRxHandle->type == PCM_FORMAT) {
            status = mPcmRxHandle->module->setPlaybackVolume(mStreamVol,
                                               mPcmRxHandle->useCase);
        }
    } else if(mCompreRxHandle) {
        if(mSpdifFormat != COMPRESSED_FORMAT && mHdmiFormat != COMPRESSED_FORMAT)
            status = mCompreRxHandle->module->setPlaybackVolume(mStreamVol,
                                                 mCompreRxHandle->useCase);
    }
    return status;
}

//NOTE:
// This will be used for Digital Broadcast usecase. This function will serve
// as a wrapper to handle the buffering of both Main and Associated data in
// case of dual decode use case.
ssize_t AudioBroadcastStreamALSA::write(const void *buffer, size_t bytes,
                                        int64_t timestamp, int audiotype)
{
    ALOGV("write");
    int period_size;
    char *use_case;

    ALOGV("write:: buffer %p, bytes %d", buffer, bytes);
    if (!mPowerLock) {
        acquire_wake_lock (PARTIAL_WAKE_LOCK, "AudioOutLock");
        mPowerLock = true;
    }

    snd_pcm_sframes_t n;
    size_t            sent = 0;
    status_t          err;

#if 0
    // 1. Check if MS11 decoder instance is present and if present we need to
    //    preserve the data and supply it to MS 11 decoder.
    if(mMS11Decoder != NULL) {
    }

    // 2. Get the output data from MS11 decoder and write to PCM driver
    if(mPcmRxHandle && mRoutePcmAudio) {
        int write_pending = bytes;
        period_size = mPcmRxHandle->periodSize;
        do {
            if (write_pending < period_size) {
                ALOGE("write:: We should not be here !!!");
                write_pending = period_size;
            }
            n = pcm_write(mPcmRxHandle->handle,
                     (char *)buffer + sent,
                      period_size);
            if (n == -EBADFD) {
                // Somehow the stream is in a bad state. The driver probably
                // has a bug and snd_pcm_recover() doesn't seem to handle this.
                mPcmRxHandle->module->open(mPcmRxHandle);
            }
            else if (n < 0) {
                // Recovery is part of pcm_write. TODO split is later.
                ALOGE("pcm_write returned n < 0");
                return static_cast<ssize_t>(n);
            }
            else {
                mFrameCount += n;
                sent += static_cast<ssize_t>((period_size));
                write_pending -= period_size;
            }
        } while ((mPcmRxHandle->handle) && (sent < bytes));
    }
#endif

    return write_l((char*)buffer,bytes);
}

status_t AudioBroadcastStreamALSA::dump(int fd, const Vector<String16>& args)
{
    return NO_ERROR;
}

status_t AudioBroadcastStreamALSA::standby()
{
    ALOGD("standby");
    Mutex::Autolock autoLock(mLock);

    if(mPcmRxHandle) {
        mPcmRxHandle->module->standby(mPcmRxHandle);
    }

    if(mCompreRxHandle) {
        mCompreRxHandle->module->standby(mCompreRxHandle);
    }

    if(mPcmTxHandle) {
        mPcmTxHandle->module->standby(mPcmTxHandle);
        mALSADevice->removeUseCase(mPcmTxHandle, "MI2S");
    }
    if(mCompreTxHandle) {
        mCompreTxHandle->module->standby(mCompreTxHandle);
        mALSADevice->removeUseCase(mCompreTxHandle, "MI2S");
    }
    if (mRouteAudioToA2dp) {
        ALOGD("stopA2dpPlayback_l from standby - usecase %x", mA2dpUseCase);
        status_t status = mParent->stopA2dpPlayback_l(mA2dpUseCase);
        if(status) {
            ALOGW("stopA2dpPlayback_l from standby returned = %d", status);
        }
        mRouteAudioToA2dp = false;
    }

    if (mPowerLock) {
        release_wake_lock ("AudioBroadcastLock");
        mPowerLock = false;
    }

    mFrameCount = 0;

    return NO_ERROR;
}

#define USEC_TO_MSEC(x) ((x + 999) / 1000)

uint32_t AudioBroadcastStreamALSA::latency() const
{
    // Android wants latency in milliseconds.
    if(mPcmRxHandle)
        return USEC_TO_MSEC (mPcmRxHandle->latency);
    else
        return 0;
}

// return the number of audio frames written by the audio dsp to DAC since
// the output has exited standby
status_t AudioBroadcastStreamALSA::getRenderPosition(uint32_t *dspFrames)
{
    *dspFrames = mFrameCount;
    return NO_ERROR;
}

/*******************************************************************************
Description: update sample rate and channel info based on format
*******************************************************************************/
void AudioBroadcastStreamALSA::updateSampleRateChannelMode()
{
//NOTE: For AAC, the output of MS11 is 48000 for the sample rates greater than
//      24000. The samples rates <= 24000 will be at their native sample rate
//      and channel mode
//      For AC3, the PCM output is at its native sample rate if the decoding is
//      single decode usecase for MS11.
//      Since, SPDIF is limited to PCM stereo for LPCM format, output is
//      stereo always.
    if(mFormat == AudioSystem::AAC || mFormat == AudioSystem::HE_AAC_V1 ||
       mFormat == AudioSystem::HE_AAC_V2) {
        if(mSampleRate > 24000) {
            mSampleRate = DEFAULT_SAMPLING_RATE;
        }
        mChannels = 6;
    } else if(mFormat == AudioSystem::AC3 || mFormat == AudioSystem::AC3_PLUS) {
        mChannels = 6;
    }
}

/*******************************************************************************
Description: Initialize Flags and device handles
*******************************************************************************/
void AudioBroadcastStreamALSA::initialization()
{
    mFrameCount        = 0;
    mFormat            = AudioSystem::INVALID_FORMAT;
    mSampleRate        = DEFAULT_SAMPLING_RATE;
    mChannels          = DEFAULT_CHANNEL_MODE;
    mBufferSize        = DEFAULT_BUFFER_SIZE;
    mStreamVol         = 0x2000;
    mPowerLock         = false;
    hw_ptr             = 0;
    // device handles
    mPcmRxHandle       = NULL;
    mCompreRxHandle    = NULL;
    mPcmTxHandle       = NULL;
    mCompreTxHandle    = NULL;
    mCaptureHandle     = NULL;
    mTranscodeHandle   = NULL;

    // MS11
    mMS11Decoder       = NULL;
    mBitstreamSM       = NULL;

    // capture and routing Flags
    mCapturePCMFromDSP       = false;
    mCaptureCompressedFromDSP= false;
    mRoutePCMStereoToDSP     = false;
    mRoutePCMMChToDSP        = false;
    mUseMS11Decoder          = false;
    mUseTunnelDecoder        = false;
    mDtsTranscode            = false;
    mRoutePcmAudio           = false;
    mRoutingSetupDone        = false;
    mSignalToSetupRoutingPath = false;
    mInputBufferSize         = 0;
    mInputBufferCount        = 0;
    mMinBytesReqToDecode     = 0;
    mTranscodeDevices        = 0;
    mChannelStatusSet        = false;
    mSpdifFormat            = INVALID_FORMAT;
    mHdmiFormat             = INVALID_FORMAT;

    mAacConfigDataSet        = false;
    mWMAConfigDataSet        = false;
    mDDFirstFrameBuffered    = false;
    memset(&mReadMetaData, 0, sizeof(mReadMetaData));

    // Thread
    mCaptureThread           = NULL;
    mKillCaptureThread       = false;
    mCaptureThreadAlive      = false;
    mExitReadCapturePath     = false;
    mCapturefd               = -1;
    mAvail                   = 0;
    mFrames                  = 0;

    mPlaybackThread          = false;
    mKillPlaybackThread      = false;
    mPlaybackThreadAlive     = false;
    mPlaybackfd              = -1;
    // playback controls
    mSkipWrite               = false;
    mPlaybackReachedEOS      = false;
    isSessionPaused          = false;
    mObserver                = NULL;


    // Proxy
    mRouteAudioToA2dp        = false;
    mCaptureFromProxy        = false;

    // AVSync
    mTimeStampModeSet         = false;
    mCompleteBufferTimePcm    = 0;
    mCompleteBufferTimeCompre = 0;
    mPartialBufferTimePcm     = 0;
    mPartialBufferTimeCompre  = 0;
    mOutputMetadataLength     = 0;
    mPcmWriteTempBuffer       = NULL;
    mCompreWriteTempBuffer    = NULL;

    return;
}


/*******************************************************************************
Description: set the output device format
*******************************************************************************/
void AudioBroadcastStreamALSA::updateOutputFormat()
{
    char value[128];
    property_get("mpq.audio.spdif.format",value,"0");
    if (!strncmp(value,"lpcm",sizeof(value)) ||
        !strncmp(value,"ac3",sizeof(value)) ||
        !strncmp(value,"dts",sizeof(value)))
        strlcpy(mSpdifOutputFormat, value, sizeof(mSpdifOutputFormat));
    else
        strlcpy(mSpdifOutputFormat, "lpcm", sizeof(mSpdifOutputFormat));

    property_get("mpq.audio.hdmi.format",value,"0");
    if (!strncmp(value,"lpcm",sizeof(value)) ||
        !strncmp(value,"ac3",sizeof(value)) ||
	!strncmp(value,"dts",sizeof(value)))
        strlcpy(mHdmiOutputFormat, value, sizeof(mHdmiOutputFormat));
    else
        strlcpy(mHdmiOutputFormat, "lpcm", sizeof(mHdmiOutputFormat));

    ALOGV("mSpdifOutputFormat: %s", mSpdifOutputFormat);
    ALOGV("mHdmiOutputFormat: %s", mHdmiOutputFormat);
    return;
}

/*******************************************************************************
Description: Set appropriate flags based on the configuration parameters
*******************************************************************************/
void AudioBroadcastStreamALSA::setCaptureFlagsBasedOnConfig()
{
    /*-------------------------------------------------------------------------
                    Set the capture and playback flags
    -------------------------------------------------------------------------*/
    // 1. Validate the audio source type
    if(mAudioSource == QCOM_AUDIO_SOURCE_ANALOG_BROADCAST) {
        mCapturePCMFromDSP = true;
    } else if(mAudioSource == QCOM_AUDIO_SOURCE_HDMI_IN) {
//NOTE:
// The format will be changed from decoder format to the format specified by
// ADV driver through TVPlayer
        if( mInputFormat == QCOM_BROADCAST_AUDIO_FORMAT_LPCM) {
            mCapturePCMFromDSP = true;
        } else {
            mCaptureCompressedFromDSP = true;
            mTimeStampModeSet = false;
           // ToDo: What about mixing main and AD in this case?
           // Should we pull back both the main and AD decoded data and mix
           // using MS11 decoder?
        }
    }
    if(mCapturePCMFromDSP || mCaptureCompressedFromDSP)
        mSignalToSetupRoutingPath = true;
}

void AudioBroadcastStreamALSA::setRoutingFlagsBasedOnConfig()
{
    mCaptureFromProxy = false;
    if(mAudioSource == QCOM_AUDIO_SOURCE_ANALOG_BROADCAST) {
        mRoutePCMStereoToDSP = true;
    } else if(mAudioSource == QCOM_AUDIO_SOURCE_HDMI_IN ||
              mAudioSource == QCOM_AUDIO_SOURCE_DIGITAL_BROADCAST_MAIN_ONLY ||
              mAudioSource == QCOM_AUDIO_SOURCE_DIGITAL_BROADCAST_MAIN_AD) {
//NOTE:
// The format will be changed from decoder format to the format specified by
// DSP
        if((mFormat == AudioSystem::PCM_16_BIT) ||
           (mInputFormat == QCOM_BROADCAST_AUDIO_FORMAT_LPCM)) {
            if(mChannels <= 2) {
                mRoutePCMStereoToDSP = true;
            } else {
                mRoutePCMMChToDSP = true;
//                mUseMS11Decoder = true;
// NOTE: enable this is compressed AC3 data is required over SPDIF/HDMI 
            }
        } else {
            if(mFormat == AudioSystem::AC3 || mFormat == AudioSystem::AC3_PLUS ||
               mFormat == AudioSystem::AAC || mFormat == AudioSystem::HE_AAC_V1 ||
               mFormat == AudioSystem::HE_AAC_V2 || mFormat == AudioSystem::EAC3) {
                mUseMS11Decoder = true;
                mRoutePCMMChToDSP = true;
                if(mAudioSource == QCOM_AUDIO_SOURCE_HDMI_IN)
                    mAacConfigDataSet = false;
                else
                    mAacConfigDataSet = true;
            } else {
                mUseTunnelDecoder = true;
            }
        }
    }
    /*-------------------------------------------------------------------------
                    Set the device routing flags
    -------------------------------------------------------------------------*/
    if(mDevices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        ALOGE("Set Capture from proxy true");
        mRouteAudioToA2dp = true;
        mDevices  &= ~AudioSystem::DEVICE_OUT_ALL_A2DP;
        mDevices  &= ~AudioSystem::DEVICE_OUT_SPDIF;
        mDevices |=  AudioSystem::DEVICE_OUT_PROXY;
    }

    if(mDevices & AudioSystem::DEVICE_OUT_PROXY)
        mCaptureFromProxy = true;

    setSpdifHdmiRoutingFlags(mDevices);

    if((((mSpdifFormat == PCM_FORMAT) ||
       (mSpdifFormat == COMPRESSED_FORCED_PCM_FORMAT)) &&
       (mDevices & (AudioSystem::DEVICE_OUT_SPDIF))) ||
       (((mHdmiFormat == PCM_FORMAT) ||
       (mHdmiFormat == COMPRESSED_FORCED_PCM_FORMAT)) &&
       (mDevices & (AudioSystem::DEVICE_OUT_AUX_DIGITAL))) ||
       (((mRoutePCMStereoToDSP) || (mRoutePCMMChToDSP)) &&
       (mDevices & ~(AudioSystem::DEVICE_OUT_SPDIF |
             AudioSystem::DEVICE_OUT_AUX_DIGITAL)))) {
        mRoutePcmAudio = true;
    }
    if(mUseTunnelDecoder)
        mRoutePcmAudio = false;

    return;
}

void AudioBroadcastStreamALSA::setSpdifHdmiRoutingFlags(int devices)
{
    if(!(devices & AudioSystem::DEVICE_OUT_SPDIF))
        mSpdifFormat = INVALID_FORMAT;
    if(!(devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL))
        mHdmiFormat = INVALID_FORMAT;

    mDtsTranscode = false;
    if(!strncmp(mSpdifOutputFormat,"lpcm",sizeof(mSpdifOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_SPDIF)
            mSpdifFormat = PCM_FORMAT;
    }
    if(!strncmp(mHdmiOutputFormat,"lpcm",sizeof(mHdmiOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            mHdmiFormat = PCM_FORMAT;
    }
    if(!strncmp(mSpdifOutputFormat,"ac3",sizeof(mSpdifOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_SPDIF) {
            if(mSampleRate > 24000 && mUseMS11Decoder)
                mSpdifFormat = COMPRESSED_FORMAT;
            else
                mSpdifFormat = PCM_FORMAT;
        // 44.1, 22.05 and 11.025K are not supported on Spdif for Passthrough
            if (mSampleRate == 44100)
                mSpdifFormat = COMPRESSED_FORCED_PCM_FORMAT;
        }
    }
    if(!strncmp(mHdmiOutputFormat,"ac3",sizeof(mHdmiOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            if(mSampleRate > 24000 && mUseMS11Decoder)
                mHdmiFormat = COMPRESSED_FORMAT;
            else
                mHdmiFormat = PCM_FORMAT;
    }
    if(!strncmp(mSpdifOutputFormat,"dts",sizeof(mSpdifOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_SPDIF) {
            if(mFormat != AUDIO_FORMAT_PCM_16_BIT && mUseTunnelDecoder == true) {
                mSpdifFormat = COMPRESSED_FORMAT;
                if(mFormat != AUDIO_FORMAT_DTS && mFormat != AUDIO_FORMAT_DTS_LBR) {
                    mTranscodeDevices |= AudioSystem::DEVICE_OUT_SPDIF;
                    mDtsTranscode = true;
                }
            }
            else
                mSpdifFormat = COMPRESSED_FORCED_PCM_FORMAT;
        // 44.1, 22.05 and 11.025K are not supported on Spdif for Passthrough
            if (mSampleRate == 44100 || mSampleRate == 22050 ||
                mSampleRate == 11025)
                mSpdifFormat = COMPRESSED_FORCED_PCM_FORMAT;
        }
    }
    if(!strncmp(mHdmiOutputFormat,"dts",sizeof(mHdmiOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            if(mFormat != AUDIO_FORMAT_PCM_16_BIT && mUseTunnelDecoder == true) {
                mHdmiFormat = COMPRESSED_FORMAT;
                if(mFormat != AUDIO_FORMAT_DTS && mFormat != AUDIO_FORMAT_DTS_LBR) {
                    mTranscodeDevices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    mDtsTranscode = true;
                }
            }
            else
                mHdmiFormat = COMPRESSED_FORCED_PCM_FORMAT;
    }
    ALOGV("mSpdifFormat- %d, mHdmiFormat- %d", mSpdifFormat, mHdmiFormat);

    return;
}

/*******************************************************************************
Description: open appropriate capture and routing devices
*******************************************************************************/
status_t AudioBroadcastStreamALSA::openCapturingAndRoutingDevices()
{
    int32_t devices = mDevices;
    status_t status = NO_ERROR;
    bool bIsUseCaseSet = false;
    mCaptureHandle = NULL;
    /* ------------------------------------------------------------------------
    Description: Set appropriate Capture flags based on the configuration
                 parameters
    -------------------------------------------------------------------------*/
    setCaptureFlagsBasedOnConfig();

    /*-------------------------------------------------------------------------
                           open capture device
    -------------------------------------------------------------------------*/
    if(mCapturePCMFromDSP) {
        status = openPCMCapturePath();
        if(status != NO_ERROR) {
            ALOGE("open capture path for pcm stereo failed");
            return status;
        }
        mCaptureHandle = (alsa_handle_t *) mPcmTxHandle;
    } else if(mCaptureCompressedFromDSP) {
        status = openCompressedCapturePath();
        if(status != NO_ERROR) {
            ALOGE("open capture path for compressed failed ");
            return status;
        }
        mCaptureHandle = (alsa_handle_t *) mCompreTxHandle;
    } else {
        ALOGD("Capture path not enabled");
    }
    if(mCaptureHandle)
        status = createCaptureThread();

    /*A call to DoRoutingSetup() is required below if the audio
    source is digital broadcast, as capture thread is not created
    in case of digital broadcast and in current design
    doRoutingSetup is called from capture thread*/

    if(mAudioSource == QCOM_AUDIO_SOURCE_DIGITAL_BROADCAST_MAIN_ONLY)
        status = doRoutingSetup();

    return status;
}


status_t AudioBroadcastStreamALSA::openPCMCapturePath()
{
    ALOGV("openPCMStereoCapturePath");
    bool bIsUseCaseSet = false;
    status_t status = NO_ERROR;
    alsa_handle_t alsa_handle;
    char *use_case;

    alsa_handle.module = mParent->mALSADevice;
    alsa_handle.periodSize = DEFAULT_IN_BUFFER_SIZE_BROADCAST_COMPRESSED;
    alsa_handle.devices = 0;
    alsa_handle.activeDevice = 0;
    alsa_handle.handle = 0;
    alsa_handle.type = PCM_FORMAT;
    alsa_handle.format = SNDRV_PCM_FORMAT_S16_LE;
    alsa_handle.channels = mChannels;
    alsa_handle.sampleRate = mSampleRate;
    alsa_handle.latency = RECORD_LATENCY;
    alsa_handle.rxHandle = 0;
    alsa_handle.mode = mParent->mode();
    alsa_handle.ucMgr = mParent->mUcMgr;
    alsa_handle.timeStampMode = SNDRV_PCM_TSTAMP_NONE;
    snd_use_case_get(mParent->mUcMgr, "_verb", (const char **)&use_case);
    if ((use_case == NULL) ||
        (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
             strlen(SND_USE_CASE_VERB_INACTIVE)))) {
        strlcpy(alsa_handle.useCase, SND_USE_CASE_VERB_HIFI_REC_COMPRESSED,
                    sizeof(alsa_handle.useCase));
        bIsUseCaseSet = true;
    } else {
        strlcpy(alsa_handle.useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED,
                    sizeof(alsa_handle.useCase));
    }
    if(use_case)
        free(use_case);
    mParent->mDeviceList.push_back(alsa_handle);
    ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
    ALOGD("useCase %s", it->useCase);
    mPcmTxHandle = &(*it);

    mALSADevice->setUseCase(mPcmTxHandle, bIsUseCaseSet,"MI2S");
    status = mALSADevice->setCaptureFormat("LPCM");
    if(status != NO_ERROR) {
        ALOGE("set Capture format failed");
        return BAD_VALUE;
    }

    status = mALSADevice->openCapture(mPcmTxHandle, true, false);
    if(status != NO_ERROR)
        ALOGE("Open capture PCM stereo driver failed");
    return status;
}

status_t AudioBroadcastStreamALSA::openCompressedCapturePath()
{
    ALOGV("openCompressedCapturePath");
    bool bIsUseCaseSet = false;
    alsa_handle_t alsa_handle;
    char *use_case;
    status_t status = NO_ERROR;

    alsa_handle.module = mParent->mALSADevice;
    alsa_handle.periodSize = DEFAULT_IN_BUFFER_SIZE_BROADCAST_COMPRESSED;
    alsa_handle.devices = 0;
    alsa_handle.activeDevice = 0;
    alsa_handle.handle = 0;
    alsa_handle.type = COMPRESSED_FORMAT;
    alsa_handle.format = mInputFormat;
    if(mInputFormat == QCOM_BROADCAST_AUDIO_FORMAT_COMPRESSED)
        alsa_handle.channels = 2; //This is to set one MI2S line to read data
    else
        alsa_handle.channels = 8; //This is to set four MI2S lines to read data
    alsa_handle.sampleRate = mSampleRate;
    alsa_handle.latency = RECORD_LATENCY;
    alsa_handle.rxHandle = 0;
    alsa_handle.mode = mParent->mode();
    alsa_handle.ucMgr = mParent->mUcMgr;
    alsa_handle.timeStampMode = SNDRV_PCM_TSTAMP_NONE;

    snd_use_case_get(mParent->mUcMgr, "_verb", (const char **)&use_case);
    if ((use_case == NULL) ||
        (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
             strlen(SND_USE_CASE_VERB_INACTIVE)))) {
        strlcpy(alsa_handle.useCase, SND_USE_CASE_VERB_HIFI_REC_COMPRESSED,
                    sizeof(alsa_handle.useCase));
        bIsUseCaseSet = true;
    } else {
        strlcpy(alsa_handle.useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC_COMPRESSED,
                    sizeof(alsa_handle.useCase));
    }
    if(use_case) {
        free(use_case);
        use_case = NULL;
    }
    mParent->mDeviceList.push_back(alsa_handle);
    ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
    ALOGD("useCase %s", it->useCase);
    mCompreTxHandle = &(*it);

    mALSADevice->setUseCase(mCompreTxHandle, bIsUseCaseSet, "MI2S");
    status = mALSADevice->setCaptureFormat("Compr");
    if(status != NO_ERROR) {
        ALOGE("set Capture format failed");
        return BAD_VALUE;
    }
    status =  mALSADevice->openCapture(mCompreTxHandle, true, true);
    if(status != NO_ERROR) {
        ALOGE("Open compressed driver failed");
    }
    return status;
}

status_t AudioBroadcastStreamALSA::openPcmDevice(int devices)
{
    char *use_case;
    status_t status = NO_ERROR;

    if(mSpdifFormat == COMPRESSED_FORMAT) {
        devices = devices & ~AudioSystem::DEVICE_OUT_SPDIF;
    }
    if(mHdmiFormat == COMPRESSED_FORMAT) {
        devices = devices & ~AudioSystem::DEVICE_OUT_AUX_DIGITAL;
    }
    if(mCaptureFromProxy) {
        devices = AudioSystem::DEVICE_OUT_PROXY;
    }
    status = setPlaybackFormat();
    if(status != NO_ERROR) {
        ALOGE("setPlaybackFormat Failed");
        return BAD_VALUE;
    }
    mInputBufferSize = DEFAULT_OUT_BUFFER_SIZE_PER_CHANNEL * mChannels;
    mInputBufferCount = MULTI_CHANNEL_PERIOD_COUNT;
    snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
    if ((use_case == NULL) ||
        (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
            strlen(SND_USE_CASE_VERB_INACTIVE)))) {
        status = openRoutingDevice(SND_USE_CASE_VERB_HIFI3, true,
                     devices);
    } else {
        status = openRoutingDevice(SND_USE_CASE_MOD_PLAY_MUSIC3, false,
                     devices);
    }
    if(use_case) {
        free(use_case);
        use_case = NULL;
    }
    if(status != NO_ERROR) {
        return status;
    }
    ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
    mPcmRxHandle = &(*it);
    mBufferSize = mPcmRxHandle->periodSize;

    if(mPcmRxHandle) {
         if(mPcmRxHandle->type == PCM_FORMAT) {
             status = mPcmRxHandle->module->setPlaybackVolume(mStreamVol,
                                                mPcmRxHandle->useCase);
         }
    }

    if(mUseMS11Decoder && (mPcmRxHandle->channels > 2))
        setChannelMap(mPcmRxHandle);
    else if(mPcmRxHandle->channels > 2)
        setPCMChannelMap(mPcmRxHandle);

    mPcmWriteTempBuffer = (char *) malloc(mBufferSize);
    if(mPcmWriteTempBuffer == NULL) {
        ALOGE("Memory allocation of temp buffer to write pcm to driver failed");
        return BAD_VALUE;
    }

    return NO_ERROR;
}

void AudioBroadcastStreamALSA::setChannelMap(alsa_handle_t *handle)
{
    char channelMap[8];
    status_t status = NO_ERROR;

    memset(channelMap, 0, sizeof(channelMap));
    switch (handle->channels) {
    case 3:
    case 4:
    case 5:
        ALOGE("TODO: Investigate and add appropriate channel map appropriately");
        break;
    case 6:
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_FC;
        channelMap[3] = PCM_CHANNEL_LFE;
        channelMap[4] = PCM_CHANNEL_LB;
        channelMap[5] = PCM_CHANNEL_RB;
        break;
    case 7:
    case 8:
        ALOGE("TODO: Investigate and add appropriate channel map appropriately");
        break;
    default:
        ALOGE("un supported channels for setting channel map");
        return;
    }

    status = mALSADevice->setChannelMap(handle, sizeof(channelMap), channelMap);
    if(status)
        ALOGE("set channel map failed. Default channel map is used instead");

    return;
}

void AudioBroadcastStreamALSA::setPCMChannelMap(alsa_handle_t *handle)
{
    char channelMap[8];
    status_t status = NO_ERROR;

    memset(channelMap, 0, sizeof(channelMap));
    switch (handle->channels) {
    case 3:
    case 4:
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_LB;
        channelMap[3] = PCM_CHANNEL_RB;
    case 5:
        ALOGE("TODO: Investigate and add appropriate channel map appropriately");
        break;
    case 6:
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_FC;
        channelMap[3] = PCM_CHANNEL_LFE;
        channelMap[4] = PCM_CHANNEL_LB;
        channelMap[5] = PCM_CHANNEL_RB;
        break;
    case 7:
        ALOGE("TODO: Investigate and add appropriate channel map appropriately");
        break;
    case 8:
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_FC;
        channelMap[3] = PCM_CHANNEL_LFE;
        channelMap[4] = PCM_CHANNEL_LB;
        channelMap[5] = PCM_CHANNEL_RB;
        channelMap[6] = PCM_CHANNEL_FLC;
        channelMap[7] = PCM_CHANNEL_FRC;
        break;
    default:
        ALOGE("un supported channels for setting channel map");
        return;
    }

    status = mALSADevice->setChannelMap(handle, sizeof(channelMap), channelMap);
    if(status)
        ALOGE("set channel map failed. Default channel map is used instead");

    return;
}


status_t AudioBroadcastStreamALSA::openTunnelDevice(int devices)
{
    ALOGV("openTunnelDevice");
    char *use_case;
    status_t status = NO_ERROR;
    if(mCaptureFromProxy) {
        devices = AudioSystem::DEVICE_OUT_PROXY;
    }
    mInputBufferSize    = TUNNEL_DECODER_BUFFER_SIZE_BROADCAST;
    mInputBufferCount   = TUNNEL_DECODER_BUFFER_COUNT_BROADCAST;
    status = setPlaybackFormat();
    if(status != NO_ERROR) {
        ALOGE("setPlaybackFormat Failed");
        return BAD_VALUE;
    }
    if (devices & ~mTranscodeDevices) {
        hw_ptr = 0;
        snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
        if ((use_case == NULL) || (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                strlen(SND_USE_CASE_VERB_INACTIVE)))) {
            status = openRoutingDevice(SND_USE_CASE_VERB_HIFI_TUNNEL2, true,
                         devices & ~mTranscodeDevices);
        } else {
            status = openRoutingDevice(SND_USE_CASE_MOD_PLAY_TUNNEL2, false,
                         devices & ~mTranscodeDevices);
        }
        ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
        mCompreRxHandle = &(*it);

        if(mCompreRxHandle) {
           if(mSpdifFormat != COMPRESSED_FORMAT && mHdmiFormat != COMPRESSED_FORMAT)
                status = mCompreRxHandle->module->setPlaybackVolume(mStreamVol,
                                                  mCompreRxHandle->useCase);
        }

        //mmap the buffers for playback
        status = mmap_buffer(mCompreRxHandle->handle);
        if(status) {
            ALOGE("MMAP buffer failed - playback err = %d", status);
            return status;
        }
    }
    if (mDtsTranscode) {
        mTranscodeHandle = (alsa_handle_t *) calloc(1, sizeof(alsa_handle_t));
        mTranscodeHandle->devices = mTranscodeDevices;
        mTranscodeHandle->activeDevice= mTranscodeDevices;
        mTranscodeHandle->mode = mParent->mode();
        mTranscodeHandle->ucMgr = mUcMgr;
        mTranscodeHandle->module = mALSADevice;
        //Passthrough to be configured with 2 channels
        mTranscodeHandle->channels = 2;
        mTranscodeHandle->sampleRate = mSampleRate > 48000 ? 48000: mSampleRate;
        ALOGV("Transcode devices = %d", mTranscodeDevices);
        strlcpy(mTranscodeHandle->useCase, mCompreRxHandle->useCase, sizeof(mTranscodeHandle->useCase));
        strncat(mTranscodeHandle->useCase, SND_USE_CASE_PSEUDO_TUNNEL_SUFFIX, sizeof(mTranscodeHandle->useCase));
        mALSADevice->setUseCase(mTranscodeHandle, false);
        mALSADevice->configureTranscode(mTranscodeHandle);
    }
    if(use_case) {
        free(use_case);
        use_case = NULL;
    }
    if(status != NO_ERROR) {
        return status;
    }
    if (devices & ~mTranscodeDevices) {
        //prepare the driver for playback
        status = pcm_prepare(mCompreRxHandle->handle);
        if (status) {
            ALOGE("PCM Prepare failed - playback err = %d", status);
            return status;
        }
        bufferAlloc(mCompreRxHandle);
        mBufferSize = mCompreRxHandle->periodSize;
        if (mCompreWriteTempBuffer == NULL) {
            mCompreWriteTempBuffer = (char *) malloc(mBufferSize);
            if (mCompreWriteTempBuffer == NULL) {
                ALOGE("Memory allocation of temp buffer to write pcm to driver failed");
                return BAD_VALUE;
            }
        }
    }

    return NO_ERROR;
}

status_t AudioBroadcastStreamALSA::setPlaybackFormat()
{
    status_t status = NO_ERROR;

    if((mSpdifFormat == PCM_FORMAT) ||
              (mSpdifFormat == COMPRESSED_FORCED_PCM_FORMAT)) {
        status = mALSADevice->setPlaybackFormat("LPCM",
                           AudioSystem::DEVICE_OUT_SPDIF, mDtsTranscode);
    } else if(mSpdifFormat == COMPRESSED_FORMAT) {
        status = mALSADevice->setPlaybackFormat("Compr",
                           AudioSystem::DEVICE_OUT_SPDIF, mDtsTranscode);
    }
    if((mHdmiFormat == PCM_FORMAT) ||
               (mHdmiFormat == COMPRESSED_FORCED_PCM_FORMAT)) {
        status = mALSADevice->setPlaybackFormat("LPCM",
                           AudioSystem::DEVICE_OUT_AUX_DIGITAL, mDtsTranscode);
    } else if (mHdmiFormat == COMPRESSED_FORMAT) {
        status = mALSADevice->setPlaybackFormat("Compr",
                           AudioSystem::DEVICE_OUT_AUX_DIGITAL, mDtsTranscode);
    }
    return status;
}

status_t AudioBroadcastStreamALSA::openRoutingDevice(char *useCase,
                                  bool bIsUseCase, int devices)
{
    alsa_handle_t alsa_handle;
    status_t status = NO_ERROR;
    ALOGD("openRoutingDevice: E");
    alsa_handle.module      = mALSADevice;
    alsa_handle.bufferSize  = mInputBufferSize * mInputBufferCount;
    alsa_handle.periodSize  = mInputBufferSize;
    alsa_handle.devices     = devices;
    alsa_handle.activeDevice= devices;
    alsa_handle.handle      = 0;
    alsa_handle.format      = (mFormat == AUDIO_FORMAT_PCM_16_BIT ?
                                  SNDRV_PCM_FORMAT_S16_LE : mFormat);
    alsa_handle.channels    = mChannels;
    alsa_handle.sampleRate  = mSampleRate;
    alsa_handle.mode        = mParent->mode();
    alsa_handle.latency     = PLAYBACK_LATENCY;
    alsa_handle.rxHandle    = 0;
    alsa_handle.ucMgr       = mUcMgr;
    if(mTimeStampModeSet)
        alsa_handle.timeStampMode = SNDRV_PCM_TSTAMP_ENABLE;
    else
        alsa_handle.timeStampMode = SNDRV_PCM_TSTAMP_NONE;

    if ((!strncmp(useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                          strlen(SND_USE_CASE_VERB_HIFI_TUNNEL)) ||
        (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL1,
                          strlen(SND_USE_CASE_MOD_PLAY_TUNNEL1)))) ||
        (!strncmp(useCase, SND_USE_CASE_VERB_HIFI_TUNNEL2,
                          strlen(SND_USE_CASE_VERB_HIFI_TUNNEL2)) ||
        (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL2,
                          strlen(SND_USE_CASE_MOD_PLAY_TUNNEL2))) ||
        (!strncmp(useCase, SND_USE_CASE_MOD_PLAY_TUNNEL3,
                          strlen(SND_USE_CASE_MOD_PLAY_TUNNEL3))))) {
        if (mUseMS11Decoder == true)
            alsa_handle.type = COMPRESSED_PASSTHROUGH_FORMAT;
        else if (mFormat == AUDIO_FORMAT_DTS || mFormat == AUDIO_FORMAT_DTS_LBR) {
            if (((mSpdifFormat == COMPRESSED_FORMAT) &&
                (mDevices & (AudioSystem::DEVICE_OUT_SPDIF))) ||
                ((mHdmiFormat == COMPRESSED_FORMAT) &&
                (mDevices & (AudioSystem::DEVICE_OUT_AUX_DIGITAL))))
                alsa_handle.type = COMPRESSED_PASSTHROUGH_FORMAT;
            else
                alsa_handle.type = COMPRESSED_FORMAT;
        } else
            alsa_handle.type = COMPRESSED_FORMAT;
    }
    else
       alsa_handle.type = PCM_FORMAT;
    strlcpy(alsa_handle.useCase, useCase, sizeof(alsa_handle.useCase));

    if (mALSADevice->setUseCase(&alsa_handle, bIsUseCase))
        return NO_INIT;
    status = mALSADevice->open(&alsa_handle);
    if(status != NO_ERROR) {
        ALOGE("Could not open the ALSA device for use case %s",
                alsa_handle.useCase);
        mALSADevice->close(&alsa_handle);
    } else{
        mParent->mDeviceList.push_back(alsa_handle);
    }
    ALOGD("openRoutingDevice: X");
    return status;
}

status_t AudioBroadcastStreamALSA::openMS11Instance()
{
    int32_t formatMS11;
    mMS11Decoder = new SoftMS11;
    if(mMS11Decoder->initializeMS11FunctionPointers() == false) {
        ALOGE("Could not resolve all symbols Required for MS11");
        delete mMS11Decoder;
        return BAD_VALUE;
    }
    if(mFormat == AUDIO_FORMAT_AAC || mFormat == AUDIO_FORMAT_HE_AAC_V1 ||
       mFormat == AUDIO_FORMAT_HE_AAC_V2 || mFormat == AUDIO_FORMAT_AAC_ADIF) {
        if(mFormat == AUDIO_FORMAT_AAC_ADIF)
            mMinBytesReqToDecode = AAC_BLOCK_PER_CHANNEL_MS11*mChannels-1;
        else
            mMinBytesReqToDecode = 0;
        formatMS11 = FORMAT_DOLBY_PULSE_MAIN;
    } else if(mFormat == AUDIO_FORMAT_AC3 || mFormat == AUDIO_FORMAT_AC3_PLUS) {
        formatMS11 = FORMAT_DOLBY_DIGITAL_PLUS_MAIN;
        mMinBytesReqToDecode = 0;
    } else {
        formatMS11 = FORMAT_EXTERNAL_PCM;
    }
    if(mMS11Decoder->setUseCaseAndOpenStream(formatMS11,mChannels,mSampleRate,
                                             false /* file_playback_mode */)) {
        ALOGE("SetUseCaseAndOpen MS11 failed");
        delete mMS11Decoder;
        return BAD_VALUE;
    }
    return NO_ERROR;
}

/******************************************************************************
                                 THREADS
******************************************************************************/

status_t AudioBroadcastStreamALSA::createCaptureThread()
{
    ALOGV("createCaptureThread");
    status_t status = NO_ERROR;

    mKillCaptureThread = false;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    status = pthread_create(&mCaptureThread, (const pthread_attr_t *) NULL,
                            captureThreadWrapper, this);
    if(status) {
        ALOGE("create capture thread failed = %d", status);
    } else {
        ALOGD("Capture Thread created");
        mCaptureThreadAlive = true;
    }
    return status;
}

status_t AudioBroadcastStreamALSA::createPlaybackThread()
{
    ALOGV("createPlaybackThread");
    status_t status = NO_ERROR;

    mKillPlaybackThread = false;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    status = pthread_create(&mPlaybackThread, (const pthread_attr_t *) NULL,
                            playbackThreadWrapper, this);
    if(status) {
        ALOGE("create playback thread failed = %d", status);
    } else {
        ALOGD("Playback Thread created");
        mPlaybackThreadAlive = true;
    }
    return status;
}

void * AudioBroadcastStreamALSA::captureThreadWrapper(void *me)
{
    static_cast<AudioBroadcastStreamALSA *>(me)->captureThreadEntry();
    return NULL;
}

void * AudioBroadcastStreamALSA::playbackThreadWrapper(void *me)
{
    static_cast<AudioBroadcastStreamALSA *>(me)->playbackThreadEntry();
    return NULL;
}

void AudioBroadcastStreamALSA::allocateCapturePollFd()
{
    ALOGV("allocateCapturePollFd");
    unsigned flags  = mCaptureHandle->handle->flags;
    struct pcm* pcm = mCaptureHandle->handle;

    if(mCapturefd == -1) {
        mCapturePfd[0].fd     = mCaptureHandle->handle->timer_fd;
        mCapturePfd[0].events = POLLIN | POLLERR | POLLNVAL;
        mCapturefd            = eventfd(0,0);
        mCapturePfd[1].fd     = mCapturefd;
        mCapturePfd[1].events = POLLIN | POLLERR | POLLNVAL;

        if (flags & PCM_MONO)
            mFrames   = pcm->period_size/2;
        else if (flags & PCM_QUAD)
            mFrames   = pcm->period_size/8;
        else if (flags & PCM_5POINT1)
            mFrames   = pcm->period_size/12;
        else if (flags & PCM_7POINT1)
            mFrames   = pcm->period_size/16;
        else
            mFrames   = pcm->period_size/4;
    }
}

void AudioBroadcastStreamALSA::allocatePlaybackPollFd()
{
    ALOGV("allocatePlaybackPollFd");
    if(mPlaybackfd == -1) {
        mPlaybackPfd[0].fd     = mCompreRxHandle->handle->timer_fd;
        mPlaybackPfd[0].events = POLLIN | POLLERR | POLLNVAL;
        mPlaybackfd            = eventfd(0,0);
        mPlaybackPfd[1].fd     = mPlaybackfd;
        mPlaybackPfd[1].events = POLLIN | POLLERR | POLLNVAL;
        ALOGD("mPlaybackPfd[0].fd %d mPlaybackPfd[1].fd %d", mPlaybackPfd[0].fd, mPlaybackPfd[1].fd);
    }
}

status_t AudioBroadcastStreamALSA::startCapturePath()
{
    ALOGV("startCapturePath");
    status_t status = NO_ERROR;
    struct pcm * capture_handle = (struct pcm *)mCaptureHandle->handle;

    while(1) {
        if(!capture_handle->start) {
            if(ioctl(capture_handle->fd, SNDRV_PCM_IOCTL_START)) {
                status = -errno;
                if (errno == EPIPE) {
                    ALOGV("Failed in SNDRV_PCM_IOCTL_START\n");
                    /* we failed to make our window -- try to restart */
                    capture_handle->underruns++;
                    capture_handle->running = 0;
                    capture_handle->start = 0;
                    continue;
                } else {
                    ALOGE("IOCTL_START failed for Capture, err: %d \n", errno);
                    return status;
                }
           } else {
               ALOGD(" Capture Driver started(IOCTL_START Success)\n");
               capture_handle->start = 1;
               break;
           }
       } else {
           ALOGV("Capture driver Already started break out of condition");
           break;
       }
   }
   capture_handle->sync_ptr->flags = SNDRV_PCM_SYNC_PTR_APPL |
                                     SNDRV_PCM_SYNC_PTR_AVAIL_MIN;
   return status;
}

void AudioBroadcastStreamALSA::captureThreadEntry()
{
    ALOGV("captureThreadEntry");
    androidSetThreadPriority(gettid(), ANDROID_PRIORITY_AUDIO);
    prctl(PR_SET_NAME, (unsigned long)"Broadcast Capture Thread", 0, 0, 0);

    uint32_t frameSize = 0;
    status_t status = NO_ERROR;
    ssize_t size = 0;
    resetCapturePathVariables();
    char tempBuffer[mCaptureHandle->handle->period_size];

    ALOGV("mKillCaptureThread = %d", mKillCaptureThread);
    while(!mKillCaptureThread && (mCaptureHandle != NULL)) {
        {
            Mutex::Autolock autolock(mCaptureMutex);

            size = readFromCapturePath(tempBuffer);
            if(size <= 0) {
                ALOGE("capturePath returned size = %d", size);
                if(mCaptureHandle) {
                    status = pcm_prepare(mCaptureHandle->handle);
                    if(status != NO_ERROR)
                        ALOGE("DEVICE_OUT_DIRECTOUTPUT: pcm_prepare failed");
                }
                continue;
            } else if (mSignalToSetupRoutingPath ==  true) {
                ALOGD("Setup the Routing path based on the format from DSP");
                if(mInputFormat == QCOM_BROADCAST_AUDIO_FORMAT_LPCM){
                    mFormat = AudioSystem::PCM_16_BIT;
                }
                else{
                    updateCaptureMetaData(tempBuffer);
                    switch(mReadMetaData.formatId) {
                       case Q6_AC3_DECODER:
                            mFormat = AudioSystem::AC3;
                            break;
                       case Q6_EAC3_DECODER:
                            mFormat = AudioSystem::AC3_PLUS;
                            break;
                       case Q6_DTS:
                            mFormat = AudioSystem::DTS;
                            break;
                       case Q6_DTS_LBR:
                            mFormat = AUDIO_FORMAT_DTS_LBR;
                            break;
                       default:
                            ALOGE("Invalid supported format");
                             /* If the first buffer does not contain any valid format then
                             this buffer is not correct and need to ignored so continu
                              the while loop and check for next buffer.*/
                            continue;
                            break;
                     }
                     ALOGD("format: %d, frameSize: %d", mReadMetaData.formatId,
                                         frameSize);
                }
                if(NO_ERROR != doRoutingSetup()){
                  /*If there is some failure while doing the routing setup then break the while loop
                    and kill the capture thread. This can not be communicated to TvPlayer as
                    OpenBroadCast call has returned after creating the capture thread*/
                    break;
                 }
                mSignalToSetupRoutingPath = false;
            } else if(mRoutingSetupDone == false) {
                ALOGD("Routing Setup is not ready. Dropping the samples");
                continue;
            } else {
                char *bufPtr = tempBuffer;
//NOTE: Each buffer contains valid data in length based on framesize which
//      can be less than the buffer size frm DSP to kernel. The valid length
//      prefixed to the buffer from driver to HAL. Hence, frame length is
//      extracted to read valid data from the buffer. Framelength is of 32 bit.
                if (mCaptureCompressedFromDSP) {
                    for(int i=0; i<MAX_NUM_FRAMES_PER_BUFFER; i++) {
                        updateCaptureMetaData(tempBuffer +
                                              i * META_DATA_LEN_BYTES);
                        bufPtr = tempBuffer + mReadMetaData.dataOffset;
                        frameSize = mReadMetaData.frameSize;
                        ALOGV("format: %d, frameSize: %d",
                                  mReadMetaData.formatId, frameSize);
                        write_l(bufPtr, frameSize);
                    }
                } else {
                    updatePCMCaptureMetaData(tempBuffer);
                    bufPtr = tempBuffer + COMPRE_CAPTURE_HEADER_SIZE +
                                 mReadMetaData.dataOffset;
                    frameSize = mReadMetaData.frameSize;
                    ALOGV("frameSize: %d", frameSize);
                    write_l(bufPtr, frameSize);
                }
            }
        }
    }

    resetCapturePathVariables();
    mCaptureThreadAlive = false;
    ALOGD("Capture Thread is dying");
}
status_t AudioBroadcastStreamALSA::doRoutingSetup()
{
    int32_t devices = mDevices;
    status_t status = NO_ERROR;
    bool bIsUseCaseSet = false;
    ALOGD("doRoutingSetup");
    if(mCapturePCMFromDSP || mCaptureCompressedFromDSP){

        if(((mFormat == AudioSystem::AAC) ||
            (mFormat == AudioSystem::HE_AAC_V1) ||
            (mFormat == AudioSystem::HE_AAC_V2) ||
            (mFormat == AudioSystem::AC3) ||
            (mFormat == AudioSystem::AC3_PLUS) ||
            (mFormat == AudioSystem::EAC3)) &&
            (!(dlopen ("libms11.so", RTLD_NOW)))) {
            ALOGE("MS11 Decoder not available");
            status = BAD_VALUE;
            return status;
        }
    }
    mSignalToSetupRoutingPath = false;
    setRoutingFlagsBasedOnConfig();
    updateSampleRateChannelMode();
    if(mTimeStampModeSet)
        mOutputMetadataLength = sizeof(output_metadata_handle_t);
    if(mRoutePcmAudio) {
        status = openPcmDevice(mDevices);
        if(status != NO_ERROR) {
            ALOGE("PCM device open failure");
            return status;
        }
    }
    if((mUseTunnelDecoder) ||
       ((mSpdifFormat == COMPRESSED_FORMAT) && (mDevices & (AudioSystem::DEVICE_OUT_SPDIF))) ||
       ((mHdmiFormat == COMPRESSED_FORMAT) && (mDevices & (AudioSystem::DEVICE_OUT_AUX_DIGITAL)))) {
        if((mSpdifFormat == COMPRESSED_FORMAT) &&
           (mHdmiFormat == COMPRESSED_FORMAT))
            devices = AudioSystem::DEVICE_OUT_SPDIF |
                         AudioSystem::DEVICE_OUT_AUX_DIGITAL;
        else if(mSpdifFormat == COMPRESSED_FORMAT)
            devices = AudioSystem::DEVICE_OUT_SPDIF;
        else if(mHdmiFormat == COMPRESSED_FORMAT)
            devices = AudioSystem::DEVICE_OUT_AUX_DIGITAL;
        else
            devices = mDevices;
        if(mCaptureFromProxy)
            devices |= AudioSystem::DEVICE_OUT_PROXY;
        if(mFormat != AUDIO_FORMAT_WMA && mFormat != AUDIO_FORMAT_WMA_PRO) {
            status = openTunnelDevice(devices);
            if(status != NO_ERROR) {
                ALOGE("Tunnel device open failure");
                return status;
            }
        }
        createPlaybackThread();
    }
    mBitstreamSM = new AudioBitstreamSM;
    if(false == mBitstreamSM->initBitstreamPtr()) {
        ALOGE("Unable to allocate Memory for i/p and o/p buffering for MS11");
        delete mBitstreamSM;
        return BAD_VALUE;
    }
    if(mUseMS11Decoder) {
        status = openMS11Instance();
        if(status != NO_ERROR) {
            ALOGE("Unable to open MS11 instance succesfully- exiting");
            mUseMS11Decoder = false;
            delete mBitstreamSM;
            return status;
        }
    }
    ALOGV("mRouteAudioToA2dp = %d", mRouteAudioToA2dp);
    if (mRouteAudioToA2dp) {
        alsa_handle_t *handle = NULL;
        if (mPcmRxHandle && (mPcmRxHandle->devices & AudioSystem::DEVICE_OUT_PROXY))
            handle = mPcmRxHandle;
        else if (mCompreRxHandle && (mCompreRxHandle->devices & AudioSystem::DEVICE_OUT_PROXY))
            handle = mCompreRxHandle;
        if (handle) {
            mA2dpUseCase = mParent->useCaseStringToEnum(handle->useCase);
            ALOGD("startA2dpPlayback_l - usecase %x", mA2dpUseCase);
            status = mParent->startA2dpPlayback_l(mA2dpUseCase);
            if(status != NO_ERROR) {
                ALOGW("startA2dpPlayback_l for broadcast returned = %d", status);
                status = BAD_VALUE;
            }
        } else
            status = BAD_VALUE;
    }
    mRoutingSetupDone = true;

    ALOGV("Routing setup done returning");
    return status;
}
ssize_t AudioBroadcastStreamALSA::readFromCapturePath(char *buffer)
{
    ALOGV("readFromCapturePath");
    status_t status = NO_ERROR;
    int err_poll = 0;

    allocateCapturePollFd();
    status = startCapturePath();
    if(status != NO_ERROR) {
        ALOGE("start capture path fail");
        return status;
    }
    struct pcm * capture_handle = (struct pcm *)mCaptureHandle->handle;

    while(!mExitReadCapturePath) {
        status = sync_ptr(capture_handle);
        if(status == EPIPE) {
            ALOGE("Failed in sync_ptr \n");
            /* we failed to make our window -- try to restart */
            capture_handle->underruns++;
            capture_handle->running = 0;
            capture_handle->start = 0;
            continue;
        } else if(status != NO_ERROR){
            ALOGE("Error: Sync ptr returned %d", status);
            break;
        }
        mAvail = pcm_avail(capture_handle);
        ALOGV("avail is = %d frames = %ld, avai_min = %d\n",\
              mAvail, mFrames,(int)capture_handle->sw_p->avail_min);

        if (mAvail < capture_handle->sw_p->avail_min) {
            err_poll = poll(mCapturePfd, NUM_FDS, TIMEOUT_INFINITE);
            if(mCapturePfd[0].revents) {
                struct snd_timer_tread rbuf[4];
                read(capture_handle->timer_fd, rbuf, sizeof(struct snd_timer_tread) * 4 );
            }
            if (mCapturePfd[1].revents & POLLIN) {
                ALOGD("Event on userspace fd");
                mCapturePfd[1].revents  = 0;
            }

            if ((mCapturePfd[1].revents & POLLERR) ||
                (mCapturePfd[1].revents & POLLNVAL) ||
                (mCapturePfd[0].revents & POLLERR) ||
                (mCapturePfd[0].revents & POLLNVAL)) {
                ALOGV("POLLERR or INVALID POLL");
                mCapturePfd[0].revents = 0;
                mCapturePfd[1].revents = 0;

                status = BAD_VALUE;
                break;
            }

            if (mCapturePfd[0].revents & POLLIN) {
                ALOGV("POLLIN on zero");
                mCapturePfd[0].revents = 0;
            }

            ALOGV("err_poll = %d",err_poll);
            continue;
        }
        break;
    }
    if(status != NO_ERROR) {
        ALOGE("Reading from Capture path failed = err = %d", status);
        return status;
    }

    void *data  = dst_address(capture_handle);

    if(data != NULL)
        memcpy(buffer, (char *)data, capture_handle->period_size);

    capture_handle->sync_ptr->c.control.appl_ptr += mFrames;
    capture_handle->sync_ptr->flags = 0;

    status = sync_ptr(capture_handle);
    if(status == EPIPE) {
        if(status != NO_ERROR ) {
            ALOGE("Error: Sync ptr end returned %d", status);
            return status;
        }
    }
    return capture_handle->period_size;
}

void  AudioBroadcastStreamALSA::playbackThreadEntry()
{
    int err_poll = 0;
    int avail = 0;
    androidSetThreadPriority(gettid(), ANDROID_PRIORITY_AUDIO);
    prctl(PR_SET_NAME, (unsigned long)"HAL Audio EventThread", 0, 0, 0);

    mPlaybackMutex.lock();
    if(!mKillPlaybackThread){
        ALOGV("PlaybackThreadEntry wait for signal \n");
        mPlaybackCv.wait(mPlaybackMutex);
        ALOGV("PlaybackThreadEntry ready to work \n");
    }
    mPlaybackMutex.unlock();

    if(!mKillPlaybackThread)
        allocatePlaybackPollFd();

    while(!mKillPlaybackThread && ((err_poll = poll(mPlaybackPfd, NUM_FDS, 3000)) >=0)) {
        ALOGD("pfd[0].revents = %d", mPlaybackPfd[0].revents);
        ALOGD("pfd[1].revents = %d", mPlaybackPfd[1].revents);
        if (err_poll == EINTR)
            ALOGE("Timer is intrrupted");

        if (mPlaybackPfd[1].revents & POLLIN) {
            uint64_t u;
            read(mPlaybackfd, &u, sizeof(uint64_t));
            ALOGV("POLLIN event occured on the d, value written to %llu",
                    (unsigned long long)u);
            mPlaybackPfd[1].revents = 0;
            if (u == SIGNAL_PLAYBACK_THREAD) {
                mPlaybackReachedEOS = true;
                continue;
            }
        }
        if ((mPlaybackPfd[1].revents & POLLERR) ||
            (mPlaybackPfd[1].revents & POLLNVAL)) {
            ALOGE("POLLERR or INVALID POLL");
            mPlaybackPfd[1].revents = 0;
        }

        while(!mKillPlaybackThread && mRoutingLock.tryLock())
             usleep(100);

        if(mKillPlaybackThread) {
            mRoutingLock.unlock();
            continue;
        }

        struct snd_timer_tread rbuf[4];
        read(mPlaybackPfd[0].fd, rbuf, sizeof(struct snd_timer_tread) * 4 );
        if((mPlaybackPfd[0].revents & POLLERR) ||
           (mPlaybackPfd[0].revents & POLLNVAL)) {
            mPlaybackPfd[0].revents = 0;
            mPlaybackPfd[1].revents = 0;
            ALOGD("pfd 0 poll err");
            mRoutingLock.unlock();
            continue;
        }

        if (mPlaybackPfd[0].revents & POLLIN && !mKillPlaybackThread) {
            mPlaybackPfd[0].revents = 0;
            mPlaybackPfd[1].revents = 0;
            if (isSessionPaused) {
                mRoutingLock.unlock();
                continue;
            }
            ALOGV("After an event occurs");
            {
                Mutex::Autolock autoLock(mSyncLock);
                mCompreRxHandle->handle->sync_ptr->flags = (SNDRV_PCM_SYNC_PTR_AVAIL_MIN |
                                                            SNDRV_PCM_SYNC_PTR_HWSYNC | SNDRV_PCM_SYNC_PTR_APPL);
                sync_ptr(mCompreRxHandle->handle);
            }
            while(hw_ptr < mCompreRxHandle->handle->sync_ptr->s.status.hw_ptr) {
                if (mInputMemFilledQueue.empty()) {
                    ALOGD("Filled queue is empty");
                    break;
                }
                mInputMemMutex.lock();
                BuffersAllocated buf = *(mInputMemFilledQueue.begin());
                mInputMemFilledQueue.erase(mInputMemFilledQueue.begin());

                if (mInputMemFilledQueue.empty() && mPlaybackReachedEOS) {
                    ALOGE("Queue Empty");
                    ALOGD("Audio Drain DONE ++");
                    if ( ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_DRAIN) < 0 ) {
                        ALOGE("Audio Drain failed");
                    }
                    ALOGD("Audio Drain DONE --");
                    //post the EOS To Player
//NOTE: In Broadcast stream, EOS is not available yet. This can be for
//      furture use
                    if (mObserver)
                        mObserver->postEOS(0);
                }

                mInputMemEmptyQueue.push_back(buf);

                ALOGV("playback thread :Empty Queue size() = %d, Filled Queue size() = %d ",
                     mInputMemEmptyQueue.size(), mInputMemFilledQueue.size());
                mInputMemMutex.unlock();
                mWriteCv.signal();
                hw_ptr += mCompreRxHandle->bufferSize/(2*mCompreRxHandle->channels);
                {
                    Mutex::Autolock autoLock(mSyncLock);
                    mCompreRxHandle->handle->sync_ptr->flags = (SNDRV_PCM_SYNC_PTR_AVAIL_MIN |
                                                                SNDRV_PCM_SYNC_PTR_HWSYNC | SNDRV_PCM_SYNC_PTR_APPL);
                    sync_ptr(mCompreRxHandle->handle);
                }
                ALOGD("hw_ptr = %lld status.hw_ptr = %ld appl_ptr = %ld", hw_ptr, mCompreRxHandle->handle->sync_ptr->s.status.hw_ptr,
                       mCompreRxHandle->handle->sync_ptr->c.control.appl_ptr);
           }
        } else {
            ALOGD("No event");
            mPlaybackPfd[0].revents = 0;
            mPlaybackPfd[1].revents = 0;
        }
        mRoutingLock.unlock();
    }
    mPlaybackThreadAlive = false;
    resetPlaybackPathVariables();
    ALOGD("Playback event Thread is dying.");
    return;
}

void AudioBroadcastStreamALSA::exitFromCaptureThread()
{
    ALOGV("exitFromCapturePath");
    if (!mCaptureThreadAlive)
        return;

    mExitReadCapturePath = true;
    mKillCaptureThread = true;
    ALOGD("exitFromCaptureThread mCapturefd = %d", mCapturefd);
    if(mCapturefd != -1) {
        uint64_t writeValue = KILL_CAPTURE_THREAD;
        ALOGD("Writing to mCapturefd %d",mCapturefd);
        sys_broadcast::lib_write(mCapturefd, &writeValue, sizeof(uint64_t));
    }
    mCaptureCv.signal();
    pthread_join(mCaptureThread,NULL);
    ALOGD("Capture thread killed");

    resetCapturePathVariables();
    return;
}

void AudioBroadcastStreamALSA::exitFromPlaybackThread()
{
    ALOGD("exitFromPlaybackPath");
    if (!mPlaybackThreadAlive)
        return;

    mPlaybackMutex.lock();
    mKillPlaybackThread = true;
    if(mPlaybackfd != -1) {
        ALOGD("Writing to mPlaybackfd %d",mPlaybackfd);
        uint64_t writeValue = KILL_PLAYBACK_THREAD;
        sys_broadcast::lib_write(mPlaybackfd, &writeValue, sizeof(uint64_t));
    }
    mPlaybackCv.signal();
    mPlaybackMutex.unlock();
    pthread_join(mPlaybackThread,NULL);
    ALOGD("Playback thread killed");

    resetPlaybackPathVariables();
    return;
}

void AudioBroadcastStreamALSA::resetCapturePathVariables()
{
    ALOGV("resetCapturePathVariables");
    mAvail = 0;
    mFrames = 0;
    if(mCapturefd != -1) {
        sys_broadcast::lib_close(mCapturefd);
        mCapturefd = -1;
    }
}

void AudioBroadcastStreamALSA::resetPlaybackPathVariables()
{
    ALOGD("resetPlaybackPathVariables");
    if(mPlaybackfd != -1) {
        sys_broadcast::lib_close(mPlaybackfd);
        mPlaybackfd = -1;
    }
}
/******************************************************************************\
                      End - Thread
******************************************************************************/


/******************************************************************************\
                      Close capture and routing devices
******************************************************************************/
status_t AudioBroadcastStreamALSA::closeDevice(alsa_handle_t *pHandle)
{
    status_t status = NO_ERROR;
    ALOGV("closeDevice");
    if(pHandle) {
        ALOGV("useCase %s", pHandle->useCase);
        status = mALSADevice->close(pHandle);
    }
    return status;
}

void AudioBroadcastStreamALSA::bufferAlloc(alsa_handle_t *handle)
{
    void  *mem_buf = NULL;
    int i = 0;

    Mutex::Autolock autoLock(mInputMemMutex);
    struct pcm * local_handle = (struct pcm *)handle->handle;
    int32_t nSize = local_handle->period_size;
    ALOGV("number of input buffers = %d", mInputBufferCount);
    ALOGV("memBufferAlloc calling with required size %d", nSize);
    mInputBufPool.clear();
    mInputMemEmptyQueue.clear();
    mInputMemFilledQueue.clear();
    for (i = 0; i < mInputBufferCount; i++) {
        mem_buf = (int32_t *)local_handle->addr + (nSize * i/sizeof(int));
        BuffersAllocated buf(mem_buf, nSize);
        memset(buf.memBuf, 0x0, nSize);
        mInputMemEmptyQueue.push_back(buf);
        mInputBufPool.push_back(buf);
        ALOGD("MEM that is allocated - buffer is %x", (unsigned int)mem_buf);
    }
}

void AudioBroadcastStreamALSA::bufferDeAlloc()
{
    while (!mInputBufPool.empty()) {
        List<BuffersAllocated>::iterator it = mInputBufPool.begin();
        BuffersAllocated &memBuffer = *it;
        ALOGD("Removing input buffer from Buffer Pool ");
        mInputBufPool.erase(it);
    }
}

/******************************************************************************
                               route output
******************************************************************************/
ssize_t AudioBroadcastStreamALSA::write_l(char *buffer, size_t bytes)
{
    size_t   sent = 0;
    bool     continueDecode;

    // set decoder configuration data if any
    if(setDecoderConfig(buffer, bytes)) {
        ALOGD("decoder configuration set");
        return bytes;
    }

    copyBitstreamInternalBuffer(buffer, bytes);

    do {
        continueDecode = decode(buffer, bytes);

        sent += render(continueDecode);

    } while(continueDecode == true);

    return sent;
}

int32_t AudioBroadcastStreamALSA::writeToCompressedDriver(char *buffer, int bytes)
{
    ALOGV("writeToCompressedDriver");
    int n = 0;
    mPlaybackCv.signal();
    mInputMemMutex.lock();

    ALOGV("write Empty Queue size() = %d, Filled Queue size() = %d ",
              mInputMemEmptyQueue.size(), mInputMemFilledQueue.size());
    if (mInputMemEmptyQueue.empty()) {
        ALOGD("Write: waiting on mWriteCv");
        ALOGD("hw_ptr = %lld status.hw_ptr = %ld appl_ptr = %ld", hw_ptr, mCompreRxHandle->handle->sync_ptr->s.status.hw_ptr,
                  mCompreRxHandle->handle->sync_ptr->c.control.appl_ptr);
        mLock.unlock();
        mWriteCvMutex.lock();
        mInputMemMutex.unlock();
        mWriteCv.wait(mWriteCvMutex);
        mInputMemMutex.lock();
        mWriteCvMutex.unlock();
        mLock.lock();
        if (mSkipWrite) {
            mSkipWrite = false;
            mInputMemMutex.unlock();
            return 0;
        }
        ALOGD("Write: received a signal to wake up");
    }

    List<BuffersAllocated>::iterator it = mInputMemEmptyQueue.begin();
    BuffersAllocated buf = *it;
    if(bytes)
        mInputMemEmptyQueue.erase(it);


    memcpy(buf.memBuf, buffer, bytes);

    buf.bytesToWrite = bytes;
    if(bytes)
        mInputMemFilledQueue.push_back(buf);
    mInputMemMutex.unlock();

    pcm * local_handle = (struct pcm *)mCompreRxHandle->handle;
    ALOGV("PCM write start");
    {
        Mutex::Autolock autoLock(mSyncLock);
        n = pcm_write(local_handle, buf.memBuf, local_handle->period_size);
    }
    ALOGV("PCM write complete");
    if (bytes < local_handle->period_size) {
        ALOGD("Last buffer case");
        uint64_t writeValue = SIGNAL_PLAYBACK_THREAD;
        sys_broadcast::lib_write(mPlaybackfd, &writeValue, sizeof(uint64_t));

        //TODO : Is this code reqd - start seems to fail?
        if (ioctl(local_handle->fd, SNDRV_PCM_IOCTL_START) < 0)
            ALOGE("AUDIO Start failed");
        else
            local_handle->start = 1;
    }

    return n;
}

uint32_t AudioBroadcastStreamALSA::read4BytesFromBuffer(char *buf)
{
   uint32_t value = 0;
   for(int i=0; i<4; i++)
      value +=  (uint32_t)(buf[i] << NUMBER_BITS_IN_A_BYTE*(i));

   return value;
}

void AudioBroadcastStreamALSA::updateCaptureMetaData(char *buf)
{
   mReadMetaData.streamId          = read4BytesFromBuffer(buf+0);
   mReadMetaData.formatId          = read4BytesFromBuffer(buf+4);
   mReadMetaData.dataOffset        = read4BytesFromBuffer(buf+8);
   mReadMetaData.frameSize         = read4BytesFromBuffer(buf+12);
   mReadMetaData.commandOffset     = read4BytesFromBuffer(buf+16);
   mReadMetaData.commandSize       = read4BytesFromBuffer(buf+20);
   mReadMetaData.encodedPcmSamples = read4BytesFromBuffer(buf+24);
   mReadMetaData.timestampMsw      = read4BytesFromBuffer(buf+28);
   mReadMetaData.timestampLsw      = read4BytesFromBuffer(buf+32);
   mReadMetaData.flags             = read4BytesFromBuffer(buf+36);
   ALOGV("streamId: %d, formatId: %d, dataOffset: %d, frameSize: %d",
           mReadMetaData.streamId, mReadMetaData.formatId,
           mReadMetaData.dataOffset, mReadMetaData.frameSize);

    ALOGV("timestampMsw: %d timestampLsw: %d", mReadMetaData.timestampMsw,
            mReadMetaData.timestampLsw);
    return;
}

void AudioBroadcastStreamALSA::updatePCMCaptureMetaData(char *buf)
{
    mReadMetaData.streamId          = 0;
    mReadMetaData.formatId          = 0;
    mReadMetaData.frameSize         = read4BytesFromBuffer(buf+0);
    mReadMetaData.dataOffset        = read4BytesFromBuffer(buf+4);
    mReadMetaData.commandOffset     = 0;
    mReadMetaData.commandSize       = 0;
    mReadMetaData.encodedPcmSamples = mReadMetaData.frameSize;
    mReadMetaData.timestampMsw      = read4BytesFromBuffer(buf+8);
    mReadMetaData.timestampLsw      = read4BytesFromBuffer(buf+12);
    mReadMetaData.flags             = read4BytesFromBuffer(buf+16);
    ALOGV("streamId: %d, formatId: %d, dataOffset: %d, frameSize: %d",
           mReadMetaData.streamId, mReadMetaData.formatId,
           mReadMetaData.dataOffset, mReadMetaData.frameSize);

    ALOGV("timestampMsw: %d timestampLsw: %d", mReadMetaData.timestampMsw,
            mReadMetaData.timestampLsw);
    return;
}

status_t AudioBroadcastStreamALSA::doRouting(int devices)
{
    status_t status = NO_ERROR;
    char *use_case;
    Mutex::Autolock autoLock(mParent->mLock);

    if(devices == 0)
        return status;
    bool stopA2DP = false;
    ALOGV("doRouting: devices 0x%x, mDevices = 0x%x", devices,mDevices);
    if(devices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        ALOGV("doRouting - Capture from Proxy");
        devices &= ~AudioSystem::DEVICE_OUT_ALL_A2DP;
        devices &= ~AudioSystem::DEVICE_OUT_SPDIF;
//NOTE: TODO - donot remove SPDIF device for A2DP device switch
        devices |=  AudioSystem::DEVICE_OUT_PROXY;
        mRouteAudioToA2dp = true;

    } else if(!(devices & AudioSystem::DEVICE_OUT_ALL_A2DP)) {
        if(mRouteAudioToA2dp) {
            mRouteAudioToA2dp = false;
            stopA2DP = true;
        }
        if(devices & AudioSystem::DEVICE_OUT_PROXY) {
            devices &= ~AudioSystem::DEVICE_OUT_SPDIF;
        }
    }
    if(devices == mDevices) {
        ALOGW("Return same device ");
        if (stopA2DP == true) {
            ALOGD("doRouting-stopA2dpPlayback_l-usecase %x", mA2dpUseCase);
            status = mParent->stopA2dpPlayback_l(mA2dpUseCase);
        }
        return status;
    }

    int prevSpdifFormat = mSpdifFormat;
    int prevHdmiFormat = mHdmiFormat;

    mDevices = devices;
    setRoutingFlagsBasedOnConfig();

    if(mUseTunnelDecoder) {
        if(mCompreRxHandle) {
            status = setPlaybackFormat();
            if(status != NO_ERROR)
               return BAD_VALUE;
            if(mFormat == AUDIO_FORMAT_DTS || mFormat == AUDIO_FORMAT_DTS_LBR) {
                if((prevSpdifFormat == COMPRESSED_FORMAT && mHdmiFormat == COMPRESSED_FORMAT)
                    || (mSpdifFormat == COMPRESSED_FORMAT && prevHdmiFormat == COMPRESSED_FORMAT)) {
                    ALOGD("Rerouting the compressed to compressed stream");
                    struct snd_compr_routing params;
                    params.session_type = PASSTHROUGH_SESSION;
                    params.operation = DISCONNECT_STREAM;
                    if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_SET_ROUTING, &params) < 0) {
                        ALOGE("disconnect stream failed");
                    }

                    mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices,
                                mParent->mode());

                    params.session_type = PASSTHROUGH_SESSION;
                    params.operation = CONNECT_STREAM;
                    if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_SET_ROUTING, &params) < 0) {
                        ALOGE("Connect stream failed");
                    }
                } else if (prevSpdifFormat == COMPRESSED_FORMAT || mHdmiFormat == COMPRESSED_FORMAT
                    || mSpdifFormat == COMPRESSED_FORMAT || prevHdmiFormat == COMPRESSED_FORMAT) {
                    ALOGD("Reopening the tunnel session");
                    Mutex::Autolock autoLock(mRoutingLock);
                    Mutex::Autolock autoLock1(mLock);
                    uint64_t writeValue = POLL_FD_MODIFIED;
                    sys_broadcast::lib_write(mPlaybackfd, &writeValue, sizeof(uint64_t));
                    closeDevice(mCompreRxHandle);
                    status = openTunnelDevice(devices);
                    if(status != NO_ERROR) {
                        ALOGE("Tunnel device open failure in do routing");
                        return BAD_VALUE;
                    }
                    mPlaybackPfd[0].fd = mCompreRxHandle->handle->timer_fd;
                    mPlaybackPfd[0].events = (POLLIN | POLLERR | POLLNVAL);
                    mPlaybackPfd[0].revents = 0;
                } else {
                    ALOGD("Reroute the stream");
                    mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices,
                            mParent->mode());
                }
            } else {
                /*Device handling for the DTS transcode*/
                if (!mDtsTranscode && mTranscodeHandle) {
                    Mutex::Autolock autoLock(mLock);
                    closeDevice(mTranscodeHandle);
                    free(mTranscodeHandle);
                    mTranscodeHandle = NULL;
                    struct snd_compr_routing params;
                    params.session_type = TRANSCODE_SESSION;
                    params.operation = DISCONNECT_STREAM;
                    if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_SET_ROUTING, &params) < 0) {
                        ALOGE("disconnect stream failed");
                    }
                }
                mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices & ~ mTranscodeDevices,
                                 mParent->mode());
                if ((mDtsTranscode && mTranscodeHandle == NULL)) {
                    Mutex::Autolock autoLock(mLock);
                    status = openTunnelDevice(mTranscodeDevices);
                    if(status != NO_ERROR) {
                        ALOGE("Error opening Transocde device in doRouting");
                        return BAD_VALUE;
                    }
                    struct snd_compr_routing params;
                    params.session_type = TRANSCODE_SESSION;
                    params.operation = CONNECT_STREAM;
                    if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_SET_ROUTING, &params) < 0) {
                        ALOGE("Connect stream failed");
                    }
               }
            }
        }
    } else {
        /*
           Handles the following
           device switch    -- pcm out --> pcm out
           open pcm device  -- compressed out --> pcm out
           close pcm device -- pcm out --> compressed out
        */
        int pcmDevices = devices;
        if(mSpdifFormat == COMPRESSED_FORMAT)
            pcmDevices = pcmDevices & ~AudioSystem::DEVICE_OUT_SPDIF;

        if(mHdmiFormat == COMPRESSED_FORMAT)
            pcmDevices = pcmDevices & ~AudioSystem::DEVICE_OUT_AUX_DIGITAL;

        bool closePcmDevice = false;

        status = setPlaybackFormat();
        if(status != NO_ERROR)
            return BAD_VALUE;

        if(pcmDevices != 0) {
            if(mPcmRxHandle == NULL) {
                Mutex::Autolock autoLock(mLock);
                mRoutePcmAudio = true;
                status = openPcmDevice(pcmDevices);
                if(status != NO_ERROR) {
                    ALOGE("Error opening PCM device in doRouting");
                    return BAD_VALUE;
                }
            }
        } else {
            closePcmDevice = true;
        }
        /*
           Handles the following
           device switch       -- compressed out --> compressed out
           create thread and
           open tunnel device  -- pcm out --> compressed out
           exit thread and
           close tunnel device -- compressed out --> pcm out
        */
        if((mSpdifFormat == COMPRESSED_FORMAT) ||
           (mHdmiFormat == COMPRESSED_FORMAT)) {
            int comprDevices = devices & ~pcmDevices;
            if((prevSpdifFormat == COMPRESSED_FORMAT && mHdmiFormat == COMPRESSED_FORMAT)
                || (mSpdifFormat == COMPRESSED_FORMAT && prevHdmiFormat == COMPRESSED_FORMAT)) {
                ALOGD("Rerouting the AC3 compressed to compressed stream");
                status = setPlaybackFormat();
                if(status != NO_ERROR)
                    return BAD_VALUE;
                struct snd_compr_routing params;
                params.session_type = PASSTHROUGH_SESSION;
                params.operation = DISCONNECT_STREAM;
                if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_SET_ROUTING, &params) < 0) {
                    ALOGE("disconnect stream failed");
                }
                mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices,
                            mParent->mode());

                params.session_type = PASSTHROUGH_SESSION;
                params.operation = CONNECT_STREAM;
                if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_SET_ROUTING, &params) < 0) {
                    ALOGE("Connect stream failed");
                }
            } else if (mCompreRxHandle == NULL) {
                ALOGD("Opening the tunnel session");
                Mutex::Autolock autoLock(mLock);
                if(closePcmDevice && mPcmRxHandle) {
                    status_t status = closeDevice(mPcmRxHandle);
                    if(status != NO_ERROR) {
                        ALOGE("Error closing the pcm device in doRouting");
                        return BAD_VALUE;
                    }
                    mPcmRxHandle = NULL;
                    mRoutePcmAudio = false;
                }
                status= openTunnelDevice(comprDevices);
                if(status != NO_ERROR) {
                    ALOGE("Tunnel device open failure in do routing");
                    return BAD_VALUE;
                }
                createPlaybackThread();
            }
        } else {
            exitFromPlaybackThread();
            Mutex::Autolock autoLock(mLock);
            status = closeDevice(mCompreRxHandle);
            if(status != NO_ERROR) {
                ALOGE("Error closing compr device in doRouting");
                return BAD_VALUE;
            }
            mCompreRxHandle = NULL;
            bufferDeAlloc();
            mInputMemFilledQueue.clear();
            mInputMemEmptyQueue.clear();
        }
        if(mPcmRxHandle && pcmDevices)
            mALSADevice->switchDeviceUseCase(mPcmRxHandle, pcmDevices,
                                 mParent->mode());
    }
    if(stopA2DP) {
        ALOGD("doRouting-stopA2dpPlayback_l-usecase %x", mA2dpUseCase);
        status = mParent->stopA2dpPlayback_l(mA2dpUseCase);
    } else if(mRouteAudioToA2dp ) {
        alsa_handle_t *handle = NULL;
        if (mPcmRxHandle && (mPcmRxHandle->devices & AudioSystem::DEVICE_OUT_PROXY))
            handle = mPcmRxHandle;
        else if (mCompreRxHandle && (mCompreRxHandle->devices & AudioSystem::DEVICE_OUT_PROXY))
            handle = mCompreRxHandle;
        if (handle) {
            mA2dpUseCase = mParent->useCaseStringToEnum(handle->useCase);
            ALOGD("doRouting-startA2dpPlayback_l-usecase %x", mA2dpUseCase);
            status = mParent->startA2dpPlayback_l(mA2dpUseCase);
            if(status) {
                ALOGW("startA2dpPlayback for broadcast returned = %d", status);
                status_t err = mParent->stopA2dpPlayback_l(mA2dpUseCase);
                if(err) {
                    ALOGW("stop A2dp playback for hardware output failed = %d", err);
                }
                mRouteAudioToA2dp = false;
            }
        }
        if (handle == NULL || status != NO_ERROR) {
            mDevices = mCurDevice;
            if(mPcmRxHandle) {
                mALSADevice->switchDeviceUseCase(mPcmRxHandle, devices, mParent->mode());
            }
            if(mCompreRxHandle) {
                mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices, mParent->mode());
            }
        }
    }
    if(status)
        mCurDevice = mDevices;
    ALOGD("doRouting status = %d ",status);
    return status;
}

status_t AudioBroadcastStreamALSA::pause_l()
{
    ALOGE("pause");
    if(mPcmRxHandle) {
        if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
            ALOGE("PAUSE failed for use case %s", mPcmRxHandle->useCase);
        }
    }
    if(mCompreRxHandle) {
        if (ioctl(mCompreRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
            ALOGE("PAUSE failed on use case %s", mCompreRxHandle->useCase);
        }
        isSessionPaused = true;
    }
    if (mRouteAudioToA2dp) {
        ALOGD("Pause - suspendA2dpPlayback_l - usecase %x", mA2dpUseCase);
        status_t status = mParent->suspendA2dpPlayback_l(mA2dpUseCase);
        if(status != NO_ERROR) {
            ALOGE("Suspend Proxy from Pause returned error = %d",status);
            return BAD_VALUE;
        }
    }
    return NO_ERROR;
}

status_t AudioBroadcastStreamALSA::resume_l()
{
    if (mRouteAudioToA2dp) {
        ALOGD("startA2dpPlayback - resume - usecase %x", mA2dpUseCase);
        status_t status = mParent->startA2dpPlayback_l(mA2dpUseCase);
        if(status != NO_ERROR) {
            ALOGE("startA2dpPlayback for broadcast returned = %d", status);
            return BAD_VALUE;
        }
    }
    if(mPcmRxHandle) {
        if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,0) < 0) {
            ALOGE("RESUME failed for use case %s", mPcmRxHandle->useCase);
        }
    }
    if(mCompreRxHandle) {
        if(isSessionPaused == true) {
            if (ioctl(mCompreRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,0) < 0)
                ALOGE("RESUME failed on use case %s", mCompreRxHandle->useCase);
        }
        isSessionPaused = false;
    }
    return NO_ERROR;
}

uint32_t AudioBroadcastStreamALSA::setDecoderConfig(char *buffer, size_t bytes)
{
    uint32_t bytesConsumed = 0;
    if(mFormat == AUDIO_FORMAT_WMA || mFormat == AUDIO_FORMAT_WMA_PRO) {
        if ((mUseTunnelDecoder) && (mWMAConfigDataSet == false)) {
            ALOGV("Configuring the WMA params");
            status_t err = mALSADevice->setWMAParams(mCompreRxHandle,
                               (int *)buffer, bytes/sizeof(int));
            if (err) {
                ALOGE("WMA param config failed");
                return BAD_VALUE;
            }
            err = openTunnelDevice(mDevices);
            if (err) {
                ALOGE("opening of tunnel device failed");
                return BAD_VALUE;
            }
            mWMAConfigDataSet = true;
            bytesConsumed = bytes;
        }
    } else if(mFormat == AUDIO_FORMAT_AAC ||
           mFormat == AUDIO_FORMAT_HE_AAC_V1 ||
           mFormat == AUDIO_FORMAT_AAC_ADIF ||
           mFormat == AUDIO_FORMAT_HE_AAC_V2) {
        if(mMS11Decoder != NULL) {
            if(mAacConfigDataSet == false) {
                if(mMS11Decoder->setAACConfig((unsigned char *)buffer,
                                     bytes) == true)
                    mAacConfigDataSet = true;
                bytesConsumed = bytes;
            }
        }
    }
    return bytesConsumed;
}

void AudioBroadcastStreamALSA::copyBitstreamInternalBuffer(char *buffer,
                                   size_t bytes)
{
    // copy bitstream to internal buffer
    mBitstreamSM->copyBitstreamToInternalBuffer((char *)buffer, bytes);

    //update input meta data list prior decode
    if(mPcmRxHandle)
        update_input_meta_data_list_pre_decode(PCM_OUT);
    if(mCompreRxHandle)
        update_input_meta_data_list_pre_decode(COMPRESSED_OUT);
}

bool AudioBroadcastStreamALSA::decode(char *buffer, size_t bytes)
{
    ALOGV("decode");
    bool    continueDecode = false;
    char    *bufPtr;
    size_t  bytesConsumedInDecode = 0;
    size_t  copyOutputBytesSize = 0;
    if (mMS11Decoder != NULL) {
        size_t  copyBytesMS11 = 0;
        uint32_t outSampleRate = mSampleRate;
        uint32_t outChannels = mChannels;
        // eos handling
        if(bytes == 0) {
            if(mFormat == AUDIO_FORMAT_AAC_ADIF)
                mBitstreamSM->appendSilenceToBitstreamInternalBuffer(
                                                      mMinBytesReqToDecode,0x0);
            else
                return false;
        }
        //decode
        if(mBitstreamSM->sufficientBitstreamToDecode(mMinBytesReqToDecode)
                             == true) {
            bufPtr = mBitstreamSM->getInputBufferPtr();
            copyBytesMS11 = mBitstreamSM->bitStreamBufSize();

            mMS11Decoder->copyBitstreamToMS11InpBuf(bufPtr,copyBytesMS11);
            bytesConsumedInDecode = mMS11Decoder->streamDecode(
                                        &outSampleRate, &outChannels);
            mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
        }
        // update metadata list after each decode
        if(mPcmRxHandle && mDDFirstFrameBuffered)
            update_input_meta_data_list_post_decode(PCM_OUT,
                                                    bytesConsumedInDecode);
        if(mCompreRxHandle && mDDFirstFrameBuffered)
            update_input_meta_data_list_post_decode(COMPRESSED_OUT,
                                                    bytesConsumedInDecode);
        if(!mDDFirstFrameBuffered)
            mDDFirstFrameBuffered = true;

        //handle change in sample rate
        if((mSampleRate != outSampleRate) || (mChannels != outChannels)) {
            mSampleRate = outSampleRate;
            mChannels = outChannels;
            if(mPcmRxHandle && mRoutePcmAudio) {
                status_t status = closeDevice(mPcmRxHandle);
                if(status != NO_ERROR)
                    ALOGE("change in sample rate - close pcm device fail");
                status = openPcmDevice(mDevices);
                if(status != NO_ERROR)
                    ALOGE("change in sample rate - open pcm device fail");
            }
            mChannelStatusSet = false;
        }
        // copy the output of decoder to HAL internal buffers
        if(mRoutePcmAudio) {
            if(mRoutePCMMChToDSP) {
                bufPtr=mBitstreamSM->getOutputBufferWritePtr(PCM_MCH_OUT);
                copyOutputBytesSize = mMS11Decoder->copyOutputFromMS11Buf(PCM_MCH_OUT,
                                                        bufPtr);
                mBitstreamSM->setOutputBufferWritePtr(PCM_MCH_OUT,copyOutputBytesSize);
            }
            else if(mRoutePCMStereoToDSP) {
                bufPtr=mBitstreamSM->getOutputBufferWritePtr(PCM_2CH_OUT);
                copyOutputBytesSize = mMS11Decoder->copyOutputFromMS11Buf(PCM_2CH_OUT,
                                                        bufPtr);
                mBitstreamSM->setOutputBufferWritePtr(PCM_2CH_OUT,copyOutputBytesSize);
            }
        }
        if((mSpdifFormat == COMPRESSED_FORMAT) ||
           (mHdmiFormat == COMPRESSED_FORMAT)) {
            bufPtr=mBitstreamSM->getOutputBufferWritePtr(SPDIF_OUT);
            copyOutputBytesSize = mMS11Decoder->copyOutputFromMS11Buf(SPDIF_OUT,
                                                    bufPtr);
            mBitstreamSM->setOutputBufferWritePtr(SPDIF_OUT,copyOutputBytesSize);
        }
        if(copyOutputBytesSize &&
           mBitstreamSM->sufficientBitstreamToDecode(mMinBytesReqToDecode) == true)
            continueDecode = true;

        // set channel status
        if(mChannelStatusSet == false) {
            if(mSpdifFormat == PCM_FORMAT) {
                if(mALSADevice->get_linearpcm_channel_status(mSampleRate,
                                    mChannelStatus)) {
                    ALOGE("channel status set error ");
                    return BAD_VALUE;
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            } else if(mSpdifFormat == COMPRESSED_FORMAT) {
                if(mALSADevice->get_compressed_channel_status(
                               mBitstreamSM->getOutputBufferPtr(SPDIF_OUT),
                               copyBytesMS11,
                               mChannelStatus,AUDIO_PARSER_CODEC_AC3)) {
                    ALOGE("channel status set error ");
                    return BAD_VALUE;
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            }
            mChannelStatusSet = true;
        }
    } else if (mUseTunnelDecoder && mCompreRxHandle) {
        // Set the channel status after first frame decode/transcode
        if(bytes == 0)
            mBitstreamSM->appendSilenceToBitstreamInternalBuffer(
                                                    mMinBytesReqToDecode,0xff);
        // decode
        {
            bytesConsumedInDecode = mBitstreamSM->getInputBufferWritePtr() -
                                      mBitstreamSM->getInputBufferPtr();
        }
        // update metadata list after each decode
        update_input_meta_data_list_post_decode(COMPRESSED_OUT,
                                                    bytesConsumedInDecode);
        // handle change in sample rate
        {
        }
        // copy the output of decoder to HAL internal buffers
        {
            bufPtr = mBitstreamSM->getOutputBufferWritePtr(COMPRESSED_OUT);
            copyOutputBytesSize = bytesConsumedInDecode;
            memcpy(bufPtr, mBitstreamSM->getInputBufferPtr(), copyOutputBytesSize);
            mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
            mBitstreamSM->setOutputBufferWritePtr(COMPRESSED_OUT,
                                                  copyOutputBytesSize);
        }

        continueDecode = false;
        // set channel status
        if(mChannelStatusSet == false) {
            if(mSpdifFormat == PCM_FORMAT) {
                if (mALSADevice->get_linearpcm_channel_status(mSampleRate,
                                      mChannelStatus)) {
                    ALOGE("channel status set error ");
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            } else if(mSpdifFormat == COMPRESSED_FORMAT) {
                if (mALSADevice->get_compressed_channel_status(
                                     buffer, bytes, mChannelStatus,
                                     AUDIO_PARSER_CODEC_DTS)) {
                    ALOGE("channel status set error ");
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            }
            mChannelStatusSet = true;
        }
    } else if(mRoutePcmAudio && mPcmRxHandle && mRoutePCMStereoToDSP) {
        // Set the channel status after first frame decode/transcode
        if(bytes == 0)
            mBitstreamSM->appendSilenceToBitstreamInternalBuffer(
                                                    mMinBytesReqToDecode,0x0);
        // decode
        {
            bytesConsumedInDecode = mBitstreamSM->getInputBufferWritePtr() -
                                      mBitstreamSM->getInputBufferPtr();
        }
        // update metadata list after each decode
        update_input_meta_data_list_post_decode(PCM_OUT, bytesConsumedInDecode);
        // handle change in sample rate
        {
        }
        // copy the output of decoder to HAL internal buffers
        {
            bufPtr = mBitstreamSM->getOutputBufferWritePtr(PCM_2CH_OUT);
            copyOutputBytesSize = bytesConsumedInDecode;
            memcpy(bufPtr, mBitstreamSM->getInputBufferPtr(), copyOutputBytesSize);
            mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
            mBitstreamSM->setOutputBufferWritePtr(PCM_2CH_OUT,
                                                  copyOutputBytesSize);
        }

        continueDecode = false;
        // set channel status
        if(mChannelStatusSet == false) {
            if(mSpdifFormat == PCM_FORMAT) {
                if (mALSADevice->get_linearpcm_channel_status(mSampleRate,
                                      mChannelStatus)) {
                    ALOGE("channel status set error ");
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            }
            mChannelStatusSet = true;
        }
    } else if(mRoutePcmAudio && mPcmRxHandle && mRoutePCMMChToDSP) {
        // Set the channel status after first frame decode/transcode
        if(bytes == 0)
            mBitstreamSM->appendSilenceToBitstreamInternalBuffer(
                                                    mMinBytesReqToDecode,0x0);
        // decode
        {
            bytesConsumedInDecode = mBitstreamSM->getInputBufferWritePtr() -
                                      mBitstreamSM->getInputBufferPtr();
        }
        // update metadata list after each decode
        update_input_meta_data_list_post_decode(PCM_OUT, bytesConsumedInDecode);
        // handle change in sample rate
        {
        }
        // copy the output of decoder to HAL internal buffers
        {
            bufPtr = mBitstreamSM->getOutputBufferWritePtr(PCM_MCH_OUT);
            copyOutputBytesSize = bytesConsumedInDecode;
            memcpy(bufPtr, mBitstreamSM->getInputBufferPtr(), copyOutputBytesSize);
            mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
            mBitstreamSM->setOutputBufferWritePtr(PCM_MCH_OUT,
                                                  copyOutputBytesSize);
        }

        continueDecode = false;
        // set channel status
        if(mChannelStatusSet == false) {
            if(mSpdifFormat == PCM_FORMAT) {
                if (mALSADevice->get_linearpcm_channel_status(mSampleRate,
                                      mChannelStatus)) {
                    ALOGE("channel status set error ");
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            }
            mChannelStatusSet = true;
        }
    } else {
        continueDecode = false;
    }

    return continueDecode;
}

uint32_t AudioBroadcastStreamALSA::render(bool continueDecode)
{
    ALOGV("render");
    snd_pcm_sframes_t n;
    uint32_t renderedPcmBytes = 0;
    int      period_size;
    uint32_t requiredSize;

    Mutex::Autolock autoLock(mLock);
    if(mPcmRxHandle && mRoutePcmAudio && mRoutePCMStereoToDSP) {
        period_size = mPcmRxHandle->periodSize;
        requiredSize = period_size - mOutputMetadataLength;
        while(mBitstreamSM->sufficientSamplesToRender(PCM_2CH_OUT,
                                 requiredSize) == true) {
            if(mTimeStampModeSet) {
                update_time_stamp_pre_write_to_driver(PCM_OUT);
                ALOGV("ts- %lld", mOutputMetadataPcm.timestamp);
                memcpy(mPcmWriteTempBuffer, &mOutputMetadataPcm,
                           mOutputMetadataLength);
            }
            memcpy(mPcmWriteTempBuffer+mOutputMetadataLength,
                       mBitstreamSM->getOutputBufferPtr(PCM_2CH_OUT),
                       requiredSize);
            n = pcm_write(mPcmRxHandle->handle, mPcmWriteTempBuffer,
                          period_size);
            ALOGE("pcm_write returned with %d", n);
            if(n < 0) {
                // Recovery is part of pcm_write. TODO split is later.
                ALOGE("pcm_write returned n < 0");
                return static_cast<ssize_t>(n);
            } else {
                mFrameCount++;
                renderedPcmBytes += static_cast<ssize_t>((period_size));
                mBitstreamSM->copyResidueOutputToStart(PCM_2CH_OUT,
                                  requiredSize);
                if(mTimeStampModeSet)
                    update_time_stamp_post_write_to_driver(PCM_OUT,
                        (mBitstreamSM->getOutputBufferWritePtr(PCM_2CH_OUT) -
                             mBitstreamSM->getOutputBufferPtr(PCM_2CH_OUT)),
                         requiredSize);
            }
        }
    }
    if(mPcmRxHandle && mRoutePcmAudio && mRoutePCMMChToDSP) {
        period_size = mPcmRxHandle->periodSize;
        requiredSize = period_size - mOutputMetadataLength;
        while(mBitstreamSM->sufficientSamplesToRender(PCM_MCH_OUT,
                                 requiredSize) == true) {
            if(mTimeStampModeSet) {
                update_time_stamp_pre_write_to_driver(PCM_OUT);
                ALOGV("ts- %lld", mOutputMetadataPcm.timestamp);
                memcpy(mPcmWriteTempBuffer, &mOutputMetadataPcm,
                           mOutputMetadataLength);
            }
            memcpy(mPcmWriteTempBuffer+mOutputMetadataLength,
                       mBitstreamSM->getOutputBufferPtr(PCM_MCH_OUT),
                       requiredSize);
            n = pcm_write(mPcmRxHandle->handle, mPcmWriteTempBuffer,
                          period_size);
            ALOGE("pcm_write returned with %d", n);
            if(n < 0) {
                // Recovery is part of pcm_write. TODO split is later.
                ALOGE("pcm_write returned n < 0");
                return static_cast<ssize_t>(n);
            } else {
                mFrameCount++;
                renderedPcmBytes += static_cast<ssize_t>((period_size));
                mBitstreamSM->copyResidueOutputToStart(PCM_MCH_OUT,
                                  requiredSize);
                if(mTimeStampModeSet)
                    update_time_stamp_post_write_to_driver(PCM_OUT,
                        (mBitstreamSM->getOutputBufferWritePtr(PCM_MCH_OUT) -
                             mBitstreamSM->getOutputBufferPtr(PCM_MCH_OUT)),
                         requiredSize);
            }
        }
    }
    if((mCompreRxHandle) &&
       ((mSpdifFormat == COMPRESSED_FORMAT) ||
        (mHdmiFormat == COMPRESSED_FORMAT) ||
        (mUseTunnelDecoder))) {
        period_size = mCompreRxHandle->periodSize;
        requiredSize = mBitstreamSM->getOutputBufferWritePtr(SPDIF_OUT) -
                          mBitstreamSM->getOutputBufferPtr(SPDIF_OUT);

        ALOGV("requiredSize- %d", requiredSize);
        while(mBitstreamSM->sufficientSamplesToRender(SPDIF_OUT,
                                1) == true) {
            if(mTimeStampModeSet) {
                update_time_stamp_pre_write_to_driver(COMPRESSED_OUT);
                ALOGV("ts- %lld", mOutputMetadataCompre.timestamp);
                memcpy(mCompreWriteTempBuffer, &mOutputMetadataCompre,
                           mOutputMetadataLength);
                memcpy(mCompreWriteTempBuffer+mOutputMetadataLength,
                           mBitstreamSM->getOutputBufferPtr(COMPRESSED_OUT),
                           requiredSize);
            }
            else {
                mOutputMetadataCompre.metadataLength = sizeof(mOutputMetadataCompre);
                mOutputMetadataCompre.bufferLength = requiredSize;
                mOutputMetadataCompre.timestamp = 0;
                memcpy(mCompreWriteTempBuffer, &mOutputMetadataCompre,
                           mOutputMetadataCompre.metadataLength);
                memcpy(mCompreWriteTempBuffer+mOutputMetadataCompre.metadataLength,
                           mBitstreamSM->getOutputBufferPtr(COMPRESSED_OUT),
                           requiredSize);
            }
            n = writeToCompressedDriver(mCompreWriteTempBuffer, period_size);
            ALOGD("pcm_write returned with %d", n);
            if (n < 0) {
                // Recovery is part of pcm_write. TODO split is later.
                ALOGE("pcm_write returned n < 0");
                return static_cast<ssize_t>(n);
            } else {
                mBitstreamSM->copyResidueOutputToStart(SPDIF_OUT,
                                  requiredSize);
                if(mTimeStampModeSet)
                    update_time_stamp_post_write_to_driver(COMPRESSED_OUT,
                        (mBitstreamSM->getOutputBufferWritePtr(SPDIF_OUT) -
                             mBitstreamSM->getOutputBufferPtr(SPDIF_OUT)),
                         requiredSize);
            }
        }
    }

    if(!continueDecode) {
        if(mPcmRxHandle)
            update_input_meta_data_list_post_write(PCM_OUT);
        if(mCompreRxHandle)
            update_input_meta_data_list_post_write(COMPRESSED_OUT);
    }

    return renderedPcmBytes;
}

void AudioBroadcastStreamALSA::update_input_meta_data_list_pre_decode(
                                   uint32_t type)
{
    ALOGV("update_input_meta_data_list_pre_decode");
    if(!mTimeStampModeSet)
        return;

    input_metadata_handle_t input_metadata_handle;
    input_metadata_handle.bufferLength   = mReadMetaData.frameSize;
    input_metadata_handle.consumedLength = 0;
    input_metadata_handle.timestamp      =
                              (uint64_t)(mReadMetaData.timestampMsw <<32) |
                              (uint64_t)(mReadMetaData.timestampLsw);
    if(type == PCM_OUT) {
        mInputMetadataListPcm.push_back(input_metadata_handle);
        ALOGV("pushing meta data to pcm list");
    } else {
        mInputMetadataListCompre.push_back(input_metadata_handle);
        ALOGV("pushing meta data to compressed list");
    }

    return;
}

void AudioBroadcastStreamALSA::update_input_meta_data_list_post_decode(
                                   uint32_t type,
                                   uint32_t bytesConsumedInDecode)
{
    ALOGV("update_input_meta_data_list_post_decode");
    if(!mTimeStampModeSet)
        return;

    if(type == PCM_OUT) {
        for(inputMetadataList::iterator it = mInputMetadataListPcm.begin();
                it != mInputMetadataListPcm.end(); ++it) {
            if((it->bufferLength - it->consumedLength) >=
                   bytesConsumedInDecode) {
                it->consumedLength += bytesConsumedInDecode;
                ALOGV("pcm: it->consumedLength - %d", it->consumedLength);
                break;
            } else {
                bytesConsumedInDecode -= (it->bufferLength - it->consumedLength);
                it->consumedLength = it->bufferLength;
                ALOGV("compr: it->consumedLength - %d", it->consumedLength);
                continue;
            }
        }
    } else {
        for(inputMetadataList::iterator it = mInputMetadataListCompre.begin();
                it != mInputMetadataListCompre.end(); ++it) {
            if((it->bufferLength - it->consumedLength) >=
                   bytesConsumedInDecode) {
                it->consumedLength += bytesConsumedInDecode;
                ALOGV("pcm: it->consumedLength - %d", it->consumedLength);
                break;
            } else {
                bytesConsumedInDecode -= (it->bufferLength - it->consumedLength);
                it->consumedLength = it->bufferLength;
                ALOGV("compr: it->consumedLength - %d", it->consumedLength);
                continue;
            }
        }
    }
    return;
}

void AudioBroadcastStreamALSA::update_time_stamp_pre_write_to_driver(
                                   uint32_t type)
{
    ALOGV("update_time_stamp_pre_write_to_driver");
    if(!mTimeStampModeSet)
        return;

    if(type == PCM_OUT) {
        if(!mInputMetadataListPcm.empty()) {
            inputMetadataList::iterator it = mInputMetadataListPcm.begin();
            mOutputMetadataPcm.metadataLength = sizeof(mOutputMetadataPcm);
            mOutputMetadataPcm.timestamp = it->timestamp +
                                               mCompleteBufferTimePcm;
            mOutputMetadataPcm.bufferLength = mPcmRxHandle->periodSize -
                                                  mOutputMetadataLength;
            if(mPartialBufferTimePcm != 0) {
                mInputMetadataListPcm.erase(it);
                ALOGV("erasing from pcm List");
            }
        }
    } else {
        if(!mInputMetadataListCompre.empty()) {
            inputMetadataList::iterator it = mInputMetadataListCompre.begin();
            mOutputMetadataCompre.metadataLength = sizeof(mOutputMetadataCompre);
            mOutputMetadataCompre.timestamp = it->timestamp +
                                                  mCompleteBufferTimeCompre;
            mOutputMetadataCompre.bufferLength =
                          mBitstreamSM->getOutputBufferWritePtr(SPDIF_OUT) -
                          mBitstreamSM->getOutputBufferPtr(SPDIF_OUT);
            if(mPartialBufferTimeCompre != 0) {
                mInputMetadataListCompre.erase(it);
                ALOGV("erasing from compressed List");
            }
        }
    }
    return;
}

void AudioBroadcastStreamALSA::update_time_stamp_post_write_to_driver(
                                   uint32_t type,
                                   uint32_t remainingSamples,
                                   uint32_t requiredBufferSize)
{
    ALOGV("update_time_stamp_post_write_to_driver");
    if(!mTimeStampModeSet)
        return;

    float timeInUsecPerByte = (1000000/(mSampleRate*mChannels*2));
                                // 2 bytes per each sample

    if(type == PCM_OUT) {
        if(remainingSamples != 0) {
            if(remainingSamples < requiredBufferSize) {
                mPartialBufferTimePcm = (uint32_t ) ((requiredBufferSize -
                                                      remainingSamples) *
                                                      timeInUsecPerByte);
                mCompleteBufferTimePcm += (uint32_t) ((requiredBufferSize) *
                                                       timeInUsecPerByte);
            } else {
                if(mPartialBufferTimePcm == 0) {
                    mCompleteBufferTimePcm += (uint32_t) ((requiredBufferSize) *
                                                           timeInUsecPerByte);
                } else {
                    mCompleteBufferTimePcm = (uint32_t) ((requiredBufferSize) *
                                                          timeInUsecPerByte) -
                                              mPartialBufferTimePcm;
                }
            }
        } else {
            mPartialBufferTimePcm = 0;
            mCompleteBufferTimePcm = 0;
        }
    } else {
        if(remainingSamples != 0) {
            if(remainingSamples < requiredBufferSize) {
//NOTE: calculate time approprately for compre ?
                mPartialBufferTimeCompre = (uint32_t) ((requiredBufferSize -
                                                        remainingSamples) *
                                                       timeInUsecPerByte);
                mCompleteBufferTimeCompre += (uint32_t) ((requiredBufferSize) *
                                                      timeInUsecPerByte);
            } else {
                if(mPartialBufferTimeCompre == 0) {
                    mCompleteBufferTimeCompre += (uint32_t) ((requiredBufferSize) *
                                                              timeInUsecPerByte);
                } else {
                    mCompleteBufferTimeCompre = (uint32_t) ((requiredBufferSize) *
                                                             timeInUsecPerByte) -
                                                mPartialBufferTimeCompre;
                }
            }
        } else {
            mPartialBufferTimeCompre = 0;
            mCompleteBufferTimeCompre = 0;
        }
    }
    return;
}

void AudioBroadcastStreamALSA::update_input_meta_data_list_post_write(
                                   uint32_t type)
{
    ALOGV("update_input_meta_data_list_post_write");
    if(!mTimeStampModeSet)
        return;

    if(type == PCM_OUT) {
        if(mPartialBufferTimePcm == 0) {
            for(inputMetadataList::iterator it = mInputMetadataListPcm.begin();
                    it != mInputMetadataListPcm.end(); ++it) {
                ALOGV("pcm: it->consumedLength - %d", it->consumedLength);
                ALOGV("pcm: it->timestamp - %lld", it->timestamp);
                if(it->bufferLength == it->consumedLength) {
                    mInputMetadataListPcm.erase(it);
                    ALOGV("erasing from pcm List");
                } else {
                    break;
                }
            }
        }
    } else {
        if(mPartialBufferTimeCompre == 0) {
            for(inputMetadataList::iterator it = mInputMetadataListCompre.begin();
                    it != mInputMetadataListCompre.end(); ++it) {
                ALOGV("compr: it->consumedLength - %d", it->consumedLength);
                ALOGV("compr: it->timestamp - %lld", it->timestamp);
                if(it->bufferLength == it->consumedLength) {
                    mInputMetadataListCompre.erase(it);
                    ALOGV("erasing from compressed List");
                } else {
                    break;
                }
            }
        }
    }
    return;
}

}       // namespace android_audio_legacy
