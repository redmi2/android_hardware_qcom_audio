/* AudioSessionOutALSA.cpp
 **
 ** Copyright 2008-2009 Wind River Systems
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
 
#include <errno.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>

#define LOG_TAG "AudioSessionOutALSA"
#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0
//#define VERY_VERBOSE_LOGGING
#ifdef VERY_VERBOSE_LOGGING
#define ALOGVV ALOGV
#else
#define ALOGVV(a...) do { } while(0)
#endif
#include <utils/Log.h>
#include <utils/String8.h>

#include <cutils/properties.h>
#include <media/AudioRecord.h>
#include <hardware_legacy/power.h>

#include <linux/ioctl.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sys/poll.h>
#include <sys/eventfd.h>
#include <linux/unistd.h>

#include "AudioHardwareALSA.h"

namespace android_audio_legacy
{
/*******************************************************************************
Description: Constructor
*******************************************************************************/
AudioSessionOutALSA::AudioSessionOutALSA(AudioHardwareALSA *parent,
                                         uint32_t   devices,
                                         int        format,
                                         uint32_t   channels,
                                         uint32_t   samplingRate,
                                         int        sessionId,
                                         status_t   *status)
{
    ALOGV("AudioSessionOutALSA");
    Mutex::Autolock autoLock(mLock);

    ALOGD("devices: %d, format: %d, channels: %d, samplingRate: %d",
        devices, format, channelMapToChannels(channels), samplingRate);
    /* ------------------------------------------------------------------------
    Error Handling
    -------------------------------------------------------------------------*/
    *status = BAD_VALUE;
    if(parent == NULL) {
        ALOGE("invalid parent");
        return;
    }
    if(!(devices & AudioSystem::DEVICE_OUT_ALL) || // check for valid device
       (samplingRate == 0) || // check for valid sample rate
       ((channelMapToChannels(channels) < 1) &&
        (channelMapToChannels(channels) > 8)) || // check for valid channels
       !(isSupportedFormat(format))) { // check for valid format
        ALOGE("invalid config");
        return;
    }
    if(isMS11SupportedFormats(format)) {
        if(canAc3PassThroughWithoutMS11(format)) {
            ALOGD("AC3 can be played as pass through without MS11");
        } else {
            if(!(dlopen ("libms11.so", RTLD_NOW))) {
                ALOGE("MS11 Decoder not available");
                return;
            }
        }
    }
    *status = NO_ERROR;
    /* ------------------------------------------------------------------------
    Physical presence of SPDIF is not known to Audio Policy Manager.
    Hence Enable SPDIF as the default device to the current active device
    NOTE: Having SPDIF OR'ed in the current active device in APM, the below can
          be avoided
    -------------------------------------------------------------------------*/
    devices         |= AudioSystem::DEVICE_OUT_SPDIF;
    /* ------------------------------------------------------------------------
    Set config parameters from input arguments
    -------------------------------------------------------------------------*/
    mParent         = parent;
    mDevices        = devices;
    mFormat         = format;
    mChannels       = channelMapToChannels(channels);
    mSampleRate     = samplingRate;
    mSessionId      = sessionId;
    /* ------------------------------------------------------------------------
    Initialize
    -------------------------------------------------------------------------*/
    initialize();
    /* ------------------------------------------------------------------------
    fix up devices for supporting A2DP playback.
    remove the A2DP and other devices and add PROXY device
    -------------------------------------------------------------------------*/
    fixUpDevicesForA2DPPlayback();
    /* ------------------------------------------------------------------------
    Adjust sample rate for MS11 supported formats - AAC, AC3 and EAC3
    cas
    -------------------------------------------------------------------------*/
    fixupSampleRateChannelModeMS11Formats();
    /* ------------------------------------------------------------------------
    Update output formats chosen for playback
    required formats from user might not be supported. So, look into the table
    based on the device and update the format states that are supported locally
    -------------------------------------------------------------------------*/
    updateDeviceSupportedFormats();
    /* ------------------------------------------------------------------------
    Update use decoder type and routing flags and corresponding states
    decoderType will cache the decode types such as decode/passthrough/transcode
    and in s/w or dsp. Besides, the states to open decode/passthrough/transcode
    handles with the corresponding devices and device formats are updated
    -------------------------------------------------------------------------*/
    updateDecodeTypeAndRoutingStates();
    /* ------------------------------------------------------------------------
    Update rxHandle states
    Based on the states, we open the driver and store the handle at appropriate
    index
    -------------------------------------------------------------------------*/
    updateRxHandleStates();
    /* ------------------------------------------------------------------------
    setup routing
    -------------------------------------------------------------------------*/
    *status = routingSetup();
    if(*status != NO_ERROR)
        ALOGE("unable to setup the routing path successfully");
}

/*******************************************************************************
Description: Destructor
*******************************************************************************/
AudioSessionOutALSA::~AudioSessionOutALSA()
{
    ALOGV("~AudioSessionOutALSA");

    reset();
    if (mRouteAudioToA2dp)
        a2dpRenderingControl(A2DP_RENDER_STOP);
}

/*******************************************************************************
Description: latency
*******************************************************************************/
uint32_t AudioSessionOutALSA::latency() const
{
    ALOGVV("latency");
    /*
    The timestamp is received from DSP for both PCM and COMPRESSED.
    Hence, the latency is approximately the start threshold.
    NOTE: latency has to be updated with the delay till the samples are send to
    DSP. eg. start threshold delay and apr delays if possible
    */
    // The value has to be set based on experiments and based on format and the
    // corresponding channel mode, sample rate and bit rate if possible
    return 0;
}

/*******************************************************************************
Description: write
*******************************************************************************/
ssize_t AudioSessionOutALSA::write(const void *buffer, size_t bytes)
{
    ALOGD("write: bytes: %d", bytes);
    Mutex::Autolock autoLock(mLock);

    bool continueDecode;
    ssize_t sent = 0;

    if (mRouteAudioToA2dp &&
        mA2dpUseCase == AudioHardwareALSA::USECASE_NONE) {
        a2dpRenderingControl(A2DP_RENDER_SETUP);
    }
    if(!mDecoderConfigSet && isDecoderConfigRequired() &&
       setDecodeConfig((char *)buffer, bytes)) {
        ALOGD("decoder configuration set");
        return bytes;
    }
    copyBitstreamInternalBuffer((char *)buffer, bytes);

    do {
        continueDecode = decode((char *)buffer, bytes);

        render(continueDecode);

    } while(continueDecode == true);

    if(bytes == 0) {
        mLock.unlock();
        eosHandling();
        mLock.lock();
    }
    return bytes;
}

/*******************************************************************************
Description: start
*******************************************************************************/
status_t AudioSessionOutALSA::start()
{
    ALOGD("start");
    Mutex::Autolock autoLock(mLock);

    status_t status = NO_ERROR;
    if(mPaused) {
        if (mRouteAudioToA2dp)
            status = a2dpRenderingControl(A2DP_RENDER_START);
        for(int i=0; i<mNumRxHandlesActive; i++) {
            if(mRxHandle[i] &&
               mRxHandle[i]->handle &&
               mRxHandleRouteFormat[i] != ROUTE_DSP_TRANSCODED_COMPRESSED) {
                if (ioctl(mRxHandle[i]->handle->fd, SNDRV_PCM_IOCTL_PAUSE,0) < 0) {
                    ALOGE("PAUSE failed for use case %s", mRxHandle[i]->useCase);
                }
            }
        }
    }
    mPaused = false;
#ifdef DEBUG
    updateDumpWithPlaybackControls(RESUME);
#endif
    return status;
}

/*******************************************************************************
Description: pause
*******************************************************************************/
status_t AudioSessionOutALSA::pause()
{
    ALOGD("pause");
    Mutex::Autolock autoLock(mLock);

    status_t status = NO_ERROR;
    mPaused = true;
    for(int i=0; i<mNumRxHandlesActive; i++) {
        if(mRxHandle[i] &&
           mRxHandle[i]->handle &&
           mRxHandleRouteFormat[i] != ROUTE_DSP_TRANSCODED_COMPRESSED) {
            if (ioctl(mRxHandle[i]->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
                ALOGE("PAUSE failed for use case %s", mRxHandle[i]->useCase);
            }
        }
    }
    if (mRouteAudioToA2dp)
        status = a2dpRenderingControl(A2DP_RENDER_SUSPEND);
#ifdef DEBUG
    updateDumpWithPlaybackControls(PAUSE);
#endif
    return status;
}

/*******************************************************************************
Description: flush
*******************************************************************************/
status_t AudioSessionOutALSA::flush()
{
    ALOGD("Flush");
    status_t status = NO_ERROR;
    Mutex::Autolock autoLock(mLock);
    return flush_l();
}

/*******************************************************************************
Description: flush_l
*******************************************************************************/
status_t AudioSessionOutALSA::flush_l()
{
    ALOGD("flush_l");
    status_t status = NO_ERROR;
    for(int i=0; i<mNumRxHandlesActive; i++) {
        if(mRxHandle[i] &&
           mRxHandle[i]->handle &&
           mRxHandleRouteFormat[i] != ROUTE_DSP_TRANSCODED_COMPRESSED) {
            if(!mPaused) {
                if (ioctl(mRxHandle[i]->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
                    ALOGE("PAUSE failed for use case %s", mRxHandle[i]->useCase);
                }
            }
            pcm_prepare(mRxHandle[i]->handle);
            ALOGV("flush(): prepare completed");
        }
    }
    mFrameCountMutex.lock();
    mFrameCount = 0;
    mFrameCountMutex.unlock();
    if(mBitstreamSM)
        mBitstreamSM->resetBitstreamPtr();
    if(mUseMS11Decoder && mMS11Decoder) {
        delete mMS11Decoder;
        status = openMS11Instance();
        if(status) {
            ALOGE("open MS11 Instance failed");
            return BAD_VALUE;
        }
    }
#ifdef DEBUG
    updateDumpWithPlaybackControls(SEEK);
#endif
    ALOGD("flush X");
    return NO_ERROR;
}

/*******************************************************************************
Description: stop
*******************************************************************************/
status_t AudioSessionOutALSA::stop()
{
    ALOGD("stop");
    Mutex::Autolock autoLock(mLock);

    status_t status = NO_ERROR;
    reset();
    if (mRouteAudioToA2dp) {
        status = a2dpRenderingControl(A2DP_RENDER_SUSPEND);
        mA2dpUseCase = AudioHardwareALSA::USECASE_NONE;
    }
    return status;
}

/*******************************************************************************
Description: dump
*******************************************************************************/
status_t AudioSessionOutALSA::dump(int fd, const Vector<String16>& args)
{
    ALOGVV("dump");
    return NO_ERROR;
}

/*******************************************************************************
Description: setVolume
*******************************************************************************/
status_t AudioSessionOutALSA::setVolume(float left, float right)
{
    ALOGD("setVolume");
    Mutex::Autolock autoLock(mLock);

    status_t status = NO_ERROR;
    float volume;
    volume = (left + right) / 2;
    if (volume < 0.0) {
        ALOGW("AudioSessionOutALSA::setVolume(%f) under 0.0, assuming 0.0\n", volume);
        volume = 0.0;
    } else if(volume > 1.0) {
        ALOGW("AudioSessionOutALSA::setVolume(%f) over 1.0, assuming 1.0\n", volume);
        volume = 1.0;
    }
    mStreamVol = lrint((volume * 0x2000)+0.5);
    for(int i=0; i<mNumRxHandlesActive; i++) {
        if(mRxHandle[i] &&
           mRxHandle[i]->handle &&
           mRxHandleRouteFormat[i] == ROUTE_UNCOMPRESSED) {
            status = mALSADevice->setPlaybackVolume(mRxHandle[i], mStreamVol);
        }
    }
    return status;
}

/*******************************************************************************
Description: standby
*******************************************************************************/
status_t AudioSessionOutALSA::standby()
{
    ALOGD("standby");
    Mutex::Autolock autoLock(mLock);

    status_t status = NO_ERROR;
    reset();
    if (mRouteAudioToA2dp)
        status = a2dpRenderingControl(A2DP_RENDER_STOP);
    if (mPowerLock) {
        release_wake_lock ("AudioSessionOutLock");
        mPowerLock = false;
    }
    return status;
}

/*******************************************************************************
Description: setParameters
*******************************************************************************/
status_t AudioSessionOutALSA::setParameters(const String8& keyValuePairs)
{
    ALOGV("setParameters");
    Mutex::Autolock autoLock(mControlLock);
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key = String8(AudioParameter::keyRouting);
    int device;
    if (param.getInt(key, device) == NO_ERROR) {
        // Ignore routing if device is 0.
        if(device) {
            device |= AudioSystem::DEVICE_OUT_SPDIF;
            ALOGD("setParameters(): keyRouting with device %d", device);
            doRouting(device);
        }
        param.remove(key);
    } else {
        mControlLock.unlock();
        mParent->setParameters(keyValuePairs);
    }
    return NO_ERROR;
}

/*******************************************************************************
Description: getParameters
*******************************************************************************/
String8 AudioSessionOutALSA::getParameters(const String8& keys)
{
    ALOGV("getParameters");
    Mutex::Autolock autoLock(mControlLock);
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);
    int devices = mDevices;
    ALOGV("getParameters mDevices %d mRouteAudioToA2dp %d", mDevices, mRouteAudioToA2dp);
    if (param.get(key, value) == NO_ERROR) {
        if((mDevices & AudioSystem::DEVICE_OUT_PROXY) && mRouteAudioToA2dp) {
            devices |= AudioSystem::DEVICE_OUT_BLUETOOTH_A2DP;
            devices &= ~AudioSystem::DEVICE_OUT_PROXY;
        }
        param.addInt(key, (int)devices);
    }

    ALOGV("getParameters() %s", param.toString().string());
    return param.toString();
}

/*******************************************************************************
Description: getRenderPosition
*******************************************************************************/
status_t AudioSessionOutALSA::getRenderPosition(uint32_t *dspFrames)
{
    ALOGV("getRenderPosition");
    Mutex::Autolock autoLock(mLock);
    *dspFrames = mFrameCount;
    return NO_ERROR;
}

/*******************************************************************************
Description: getNextWriteTimestamp
*******************************************************************************/
status_t AudioSessionOutALSA::getNextWriteTimestamp(int64_t *timeStamp)
{
    ALOGV("getNextWriteTimestamp");

    struct snd_compr_tstamp tstamp;
    if (!timeStamp) {
        ALOGE("Invalid address to send time stamp to Player");
        return BAD_VALUE;
    }
    for(int i=0; i<mNumRxHandlesActive; i++) {
        if(mRxHandle[i] &&
           mRxHandle[i]->handle &&
           mRxHandleRouteFormat[i] != ROUTE_DSP_TRANSCODED_COMPRESSED) {
            if(ioctl(mRxHandle[i]->handle->fd, SNDRV_COMPRESS_TSTAMP, &tstamp)) {
                ALOGE("Failed SNDRV_COMPRESS_TSTAMP\n");
                return BAD_VALUE;
            } else {
                ALOGV("Timestamp returned = %lld\n", tstamp.timestamp);
                *timeStamp = tstamp.timestamp;
                return NO_ERROR;
            }
        }
    }
    ALOGE("No valid driver opened for updating time stamp");
    *timeStamp = 0;
    return BAD_VALUE;
}

/*******************************************************************************
Description: set the observer so that EOS can be posted to the player of reqd
*******************************************************************************/
status_t AudioSessionOutALSA::setObserver(void *observer)
{
    ALOGV("setObserver");
    /*
    We donot need observer here, as the EOS buffer which is zero length buffer
    is acknowledged only when the rendering is completed by DSP. The same can
    refer to EOS from Player. If an observer is set, then we post EOS
    */
    mObserver = reinterpret_cast<AudioEventObserver *>(observer);
    return NO_ERROR;
}

/*******************************************************************************
Description: Initialize Flags and device handles
*******************************************************************************/
uint32_t AudioSessionOutALSA::channelMapToChannels(uint32_t channelMap)
{
    ALOGVV("channelMapToChannels");
    uint32_t channels = 0;
    while(channelMap) {
        channels++;
        channelMap = channelMap & (channelMap - 1);
    }
    return channels;
}

/*******************************************************************************
Description: check if the format is supported in HAL
*******************************************************************************/
bool AudioSessionOutALSA::isSupportedFormat(int format)
{
    ALOGVV("channelMapToChannels");
    for(int i=0; i<NUM_SUPPORTED_CODECS; i++)
        if(format == supportedFormats[i])
            return true;

    return false;
}

/*******************************************************************************
Description: check for MS11 supported formats
*******************************************************************************/
bool AudioSessionOutALSA::isMS11SupportedFormats(int format)
{
    ALOGVV("isMS11SupportedFormats");
    if(((format == AudioSystem::AAC) ||
        (format == AudioSystem::HE_AAC_V1) ||
        (format == AudioSystem::HE_AAC_V2) ||
        (format == AudioSystem::AC3) ||
        (format == AudioSystem::AC3_PLUS) ||
        (format == AudioSystem::EAC3))) {
        return true;
    } else {
        return false;
    }
}

/*******************************************************************************
Description: check if ac3 can played as pass through without MS11 decoder
*******************************************************************************/
bool AudioSessionOutALSA::canAc3PassThroughWithoutMS11(int format)
{
    ALOGVV("canAc3PassThroughWithoutMS11");
    if(format == AudioSystem::AC3) {
        if(((mHdmiFormat == COMPRESSED) ||
            (mHdmiFormat == COMPRESSED_CONVERT_EAC3_AC3) ||
            (mHdmiFormat == COMPRESSED_CONVERT_ANY_AC3)) &&
            ((mSpdifFormat == COMPRESSED) ||
             (mSpdifFormat == COMPRESSED_CONVERT_EAC3_AC3) ||
             (mSpdifFormat == COMPRESSED_CONVERT_ANY_AC3))) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(AC3) >= 0) {
                return true;
            }
        }
    }
    return false;
}

/*******************************************************************************
Description: get the format index
*******************************************************************************/
int AudioSessionOutALSA::getFormatIndex()
{
    ALOGVV("getFormatIndex");
    int idx = 0;
    for(int i=0; i<NUM_SUPPORTED_CODECS; i++) {
        if(mFormat == formatIndex[i][0]) {
            idx = formatIndex[i][1];
            break;
        }
    }
   return idx;
}

/*******************************************************************************
Description: fix up devices for supporting A2DP playback
*******************************************************************************/
void AudioSessionOutALSA::fixUpDevicesForA2DPPlayback()
{
    ALOGVV("fixUpDevicesForA2DPPlayback");
    if(mDevices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        mRouteAudioToA2dp = true;
        mDevices  &= ~AudioSystem::DEVICE_OUT_ALL_A2DP;
        mDevices  &= ~AudioSystem::DEVICE_OUT_SPDIF;
        mDevices |=  AudioSystem::DEVICE_OUT_PROXY;
    }
}

/*******************************************************************************
Description: get index of handle based on device handle format
*******************************************************************************/
int AudioSessionOutALSA::getIndexHandleBasedOnHandleFormat(int handleFormat)
{
    ALOGVV("getIndexHandleBasedOnHandleFormat");
    int index = 0;
    for(index=0; index < mNumRxHandlesActive; index++) {
        if(mRxHandleRouteFormat[index] == handleFormat)
            break;
    }
    return index;
}

/*******************************************************************************
Description: get index of handle based on device handle device
*******************************************************************************/
int AudioSessionOutALSA::getIndexHandleBasedOnHandleDevices(int handleDevices)
{
    ALOGVV("getIndexHandleBasedOnHandleDevices");
    int index = 0;
    for(index=0; index < mNumRxHandlesActive; index++) {
        if(mRxHandleDevices[index] & handleDevices)
            break;
    }
    return index;
}

/*******************************************************************************
Description: Update device supported formats
*******************************************************************************/
void AudioSessionOutALSA::updateDeviceSupportedFormats()
{
    ALOGV("updateDeviceSupportedFormats");
    int index = getFormatIndex();
    int formatIndex;
//NOTE: to be removed with output format control
    char value[128];
    property_get("mpq.audio.spdif.format",value,"0");
    if(!strncmp(value,"ac3",sizeof(value)))
        mParent->mSpdifOutputFormat = COMPRESSED_CONVERT_ANY_AC3;
    else if(!strncmp(value,"dts",sizeof(value)))
        mParent->mSpdifOutputFormat = COMPRESSED_CONVERT_ANY_DTS;
    else
        mParent->mSpdifOutputFormat = UNCOMPRESSED;
    property_get("mpq.audio.hdmi.format",value,"0");
    if(!strncmp(value,"ac3",sizeof(value)))
        mParent->mHdmiOutputFormat = COMPRESSED_CONVERT_ANY_AC3;
    else if(!strncmp(value,"dts",sizeof(value)))
        mParent->mHdmiOutputFormat = COMPRESSED_CONVERT_ANY_DTS;
    else
        mParent->mHdmiOutputFormat = UNCOMPRESSED;
////////
    mSpdifOutputFormat = mParent->mSpdifOutputFormat;
    mHdmiOutputFormat  = mParent->mHdmiOutputFormat;
    ALOGV("mSpdifOutputFormat: %d, mHdmiOutputFormat: %d",
           mSpdifOutputFormat, mHdmiOutputFormat);
    if(!(mDevices & AudioSystem::DEVICE_OUT_SPDIF)) {
        mSpdifFormat = INVALID_FORMAT;
    } else {
        mSpdifFormat = mSpdifOutputFormat;
        /*
        SPDIF doesnot support EAC3 pass through, hence fallback to
        compressed convert to ac3
        */
        if((mFormat == AUDIO_FORMAT_EAC3) &&
           (mSpdifFormat == COMPRESSED)) {
            mSpdifFormat = COMPRESSED_CONVERT_EAC3_AC3;
            ALOGV("Fallback to convert as EAC3 pass through not supported on SPDIF");
        }
        /*
        SPDIF doesnot support 11025, 22050, 44100. So fall back to PCM format
        If MS11 supported formats, sample rate <= 24000 are not transcoded
        */
        if(usecaseDecodeHdmiSpdif[NUM_STATES_FOR_EACH_DEVICE_FMT*index+ROUTE_FORMAT_IDX]
                                 [mSpdifFormat] == FORMAT_COMPR) {
           if(((mSampleRate == 11025) ||
               (mSampleRate == 22050) ||
               (mSampleRate == 44100)) ||
              ((isMS11SupportedFormats(mFormat) && (mSampleRate <= 24000)))) {
               ALOGV("Fallback to uncompressed as EAC3 pass through not supported on SPDIF");
               mSpdifFormat = UNCOMPRESSED;
           }
        }
    }

    if(!(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)) {
        mHdmiFormat = INVALID_FORMAT;
    } else {
        mHdmiFormat = mHdmiOutputFormat;
        /*
        If MS11 supported formats, sample rate <= 24000 are not transcoded
        */
        if(usecaseDecodeHdmiSpdif[NUM_STATES_FOR_EACH_DEVICE_FMT*index+ROUTE_FORMAT_IDX]
                                 [mHdmiFormat] == FORMAT_COMPR) {
            if((isMS11SupportedFormats(mFormat) && (mSampleRate <= 24000))) {
                ALOGD("fallback to uncompressed as sample rate is less than 32kHz");
                mHdmiFormat = UNCOMPRESSED;
            }
        }
        fixUpHdmiFormatBasedOnEDID();
    }
    ALOGV("After fix up mSpdifFormat: %d, mHdmiFormat: %d",
           mSpdifFormat, mHdmiFormat);
    mALSADevice->mSpdifFormat = mSpdifFormat;
    mALSADevice->mHdmiFormat = mHdmiFormat;
}

/*******************************************************************************
Description: fix Up Hdmi Format Based On EDID
*******************************************************************************/
void AudioSessionOutALSA::fixUpHdmiFormatBasedOnEDID()
{
    ALOGV("fixUpHdmiFormatBasedOnEDID");
    switch(mFormat) {
    case AUDIO_FORMAT_EAC3:
        if(mHdmiFormat == COMPRESSED) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(DOLBY_DIGITAL_PLUS_1) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
                return;
            }
        } else if((mHdmiFormat == COMPRESSED_CONVERT_EAC3_AC3) ||
                  (mHdmiFormat == COMPRESSED_CONVERT_ANY_AC3)) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(AC3) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else if(mHdmiFormat == COMPRESSED_CONVERT_ANY_DTS) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(DTS) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else {
            mHdmiFormat = UNCOMPRESSED;
        }
        break;
    case AUDIO_FORMAT_AC3:
        if((mHdmiFormat == COMPRESSED) ||
           (mHdmiFormat == COMPRESSED_CONVERT_EAC3_AC3) ||
           (mHdmiFormat == COMPRESSED_CONVERT_ANY_AC3)) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(AC3) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else if(mHdmiFormat == COMPRESSED_CONVERT_ANY_DTS) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(DTS) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else {
            mHdmiFormat = UNCOMPRESSED;
        }
        break;
    case AUDIO_FORMAT_AAC:
    case AUDIO_FORMAT_HE_AAC_V1:
    case AUDIO_FORMAT_HE_AAC_V2:
    case AUDIO_FORMAT_AAC_ADIF:
        if(mHdmiFormat == COMPRESSED_CONVERT_ANY_AC3) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(AC3) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else if(mHdmiFormat == COMPRESSED_CONVERT_ANY_DTS) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(DTS) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else {
            mHdmiFormat = UNCOMPRESSED;
        }
        break;
    case AUDIO_FORMAT_DTS:
    case AUDIO_FORMAT_DTS_LBR:
        if((mHdmiFormat == COMPRESSED) ||
           (mHdmiFormat == COMPRESSED_CONVERT_ANY_DTS)) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(DTS) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else {
            mHdmiFormat = UNCOMPRESSED;
        }
        break;
    default:
        if(mHdmiFormat == COMPRESSED_CONVERT_ANY_DTS) {
            if(mALSADevice->getFormatHDMIIndexEDIDInfo(DTS) < 0) {
                ALOGD("Falling back to UNCOMPRESSED as PASS THROUGH is not supported");
                mHdmiFormat = UNCOMPRESSED;
            }
        } else {
            mHdmiFormat = UNCOMPRESSED;
        }
        break;
    }
    return;
}

/*******************************************************************************
Description: Initialize Flags and device handles
*******************************************************************************/
void AudioSessionOutALSA::initialize()
{
    ALOGVV("initialize");
    mALSADevice                = mParent->mALSADevice;
    mUcMgr                     = mParent->mUcMgr;

    // routing states
    mA2dpUseCase               = AudioHardwareALSA::USECASE_NONE;
    mRouteAudioToA2dp          = false;
    mObserver                  = NULL;

    reinitialize();
}

/*******************************************************************************
Description: reset
*******************************************************************************/
void AudioSessionOutALSA::reset()
{
    ALOGV("reset");
    for(int index=0; index < mNumRxHandlesActive; index++) {
        if(mRxHandle[index]) {
            closeDevice(mRxHandle[index]);
            mRxHandle[index] = NULL;
        }
    }
    if(mMS11Decoder) {
        delete mMS11Decoder;
        mMS11Decoder = NULL;
    }
    if(mBitstreamSM) {
        delete mBitstreamSM;
        mBitstreamSM = NULL;
    }
    if(mWriteTempBuffer)
        free(mWriteTempBuffer);

    reinitialize();
}

/*******************************************************************************
Description: reinitialize
*******************************************************************************/
void AudioSessionOutALSA::reinitialize()
{
    ALOGVV("reinitialize");
    mBufferSize                = 0;
    // device states
    mSpdifFormat               = UNCOMPRESSED;
    mHdmiFormat                = UNCOMPRESSED;
    // decoder states
    mDecoderType               = 0;
    mDecoderConfigSet          = false;
    mUseMS11Decoder            = false;
    mMS11Decoder               = NULL;
    mBitstreamSM               = NULL;
    mMinBytesReqToDecode       = 0;
    mPaused                    = false;
    mStreamVol                 = 0;
    mIsMS11FilePlaybackMode    = true;
    mDecoderConfigBuffer       = NULL;
    mDecoderConfigBufferLength = 0;
    mFirstBitstreamBuffer      = true;
    // rendering
    mFrameCount                = 0;
    // routing states
    mOpenDecodeRoute           = false;
    mOpenPassthroughRoute      = false;
    mOpenTranscodeRoute        = false;
    mRouteDecodeFormat         = false;
    mRoutePassthroughFormat    = false;
    mRouteTrancodeFormat       = false;
    mChannelStatusSet          = false;
    mWriteTempBuffer           = NULL;
    mNumRxHandlesActive        = 0;
    for(int i=0; i<NUM_DEVICES_SUPPORT_COMPR_DATA; i++) {
        mRxHandle[i]               = NULL;
        mRxHandleRouteFormat[i]    = ROUTE_UNCOMPRESSED;
        mRxHandleDevices[i]        = AUDIO_DEVICE_NONE;
        mRxHandleRouteFormatType[i]= UNCOMPRESSED;
    }
}

/*******************************************************************************
Description: fixup sample rate and channel info based on format
*******************************************************************************/
void AudioSessionOutALSA::fixupSampleRateChannelModeMS11Formats()
{
    ALOGV("fixupSampleRateChannelModeMS11Formats");
/*
NOTE: For AAC, the output of MS11 is 48000 for the sample rates greater than
      24000. The samples rates <= 24000 will be at their native sample rate
      For AC3, the PCM output is at its native sample rate if the decoding is
      single decode usecase for MS11.
*/
    if(mFormat == AUDIO_FORMAT_AAC ||
       mFormat == AUDIO_FORMAT_HE_AAC_V1 ||
       mFormat == AUDIO_FORMAT_HE_AAC_V2 ||
       mFormat == AUDIO_FORMAT_AAC_ADIF) {
        mSampleRate = mSampleRate > 24000 ? 48000 : mSampleRate;
        mChannels       = 6;
    } else if (mFormat == AUDIO_FORMAT_AC3 ||
               mFormat == AUDIO_FORMAT_EAC3) {
        mChannels   = 6;
    }
}

/*******************************************************************************
Description: update use case and routing flags
*******************************************************************************/
void AudioSessionOutALSA::updateDecodeTypeAndRoutingStates()
{
    ALOGV("updateDecodeTypeAndRoutingStates");
    int formatIndex = getFormatIndex();
    int decodeType;

    mOpenDecodeRoute = false;
    mOpenPassthroughRoute = false;
    mOpenTranscodeRoute = false;
    mDecodeFormatDevices = mDevices;
    mPassthroughFormatDevices = AUDIO_DEVICE_NONE;
    mTranscodeFormatDevices = AUDIO_DEVICE_NONE;

    if(isMS11SupportedFormats(mFormat))
        mUseMS11Decoder = true;

    ALOGV("formatIndex: %d", formatIndex);
    if(mDevices & AudioSystem::DEVICE_OUT_SPDIF) {
        decodeType = usecaseDecodeHdmiSpdif[NUM_STATES_FOR_EACH_DEVICE_FMT*formatIndex]
                                           [mSpdifFormat];
        ALOGVV("decoderType: %d", decodeType);
        mDecoderType = decodeType;
        for(int idx=0; idx<NUM_DECODE_PATH; idx++) {
            if(routeToDriver[idx][DECODER_TYPE_IDX] & decodeType) {
                switch(routeToDriver[idx][ROUTE_FORMAT_IDX]) {
                case ROUTE_UNCOMPRESSED:
                    ALOGVV("ROUTE_UNCOMPRESSED");
                    mOpenDecodeRoute = true;
                    mRouteDecodeFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    break;
                case ROUTE_COMPRESSED:
                    ALOGVV("ROUTE_COMPRESSED");
                    mOpenPassthroughRoute = true;
                    mRoutePassthroughFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    mDecodeFormatDevices &= ~AudioSystem::DEVICE_OUT_SPDIF;
                    mPassthroughFormatDevices = AudioSystem::DEVICE_OUT_SPDIF;
                    break;
                case ROUTE_DSP_TRANSCODED_COMPRESSED:
                    ALOGVV("ROUTE_DSP_TRANSCODED_COMPRESSED");
                    mOpenTranscodeRoute = true;
                    mRouteTrancodeFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    mTranscodeFormatDevices = AudioSystem::DEVICE_OUT_SPDIF;
                    break;
                case ROUTE_SW_TRANSCODED_COMPRESSED:
                    ALOGVV("ROUTE_SW_TRANSCODED_COMPRESSED");
                    mOpenTranscodeRoute = true;
                    mRouteTrancodeFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    mDecodeFormatDevices &= ~AudioSystem::DEVICE_OUT_SPDIF;
                    mTranscodeFormatDevices = AudioSystem::DEVICE_OUT_SPDIF;
                    break;
                default:
                    break;
                }
            }
        }
    }
    if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
        decodeType = usecaseDecodeHdmiSpdif[NUM_STATES_FOR_EACH_DEVICE_FMT*formatIndex]
                                           [mHdmiFormat];
        ALOGVV("decoderType: %d", decodeType);
        mDecoderType |= decodeType;
        for(int idx=0; idx<NUM_DECODE_PATH; idx++) {
            if(routeToDriver[idx][DECODER_TYPE_IDX] & decodeType) {
                switch(routeToDriver[idx][ROUTE_FORMAT_IDX]) {
                case ROUTE_UNCOMPRESSED:
                    ALOGVV("ROUTE_UNCOMPRESSED");
                    mOpenDecodeRoute = true;
                    mRouteDecodeFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    break;
                case ROUTE_COMPRESSED:
                    ALOGVV("ROUTE_COMPRESSED");
                    mOpenPassthroughRoute = true;
                    mRoutePassthroughFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    mDecodeFormatDevices &= ~AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    mPassthroughFormatDevices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    break;
                case ROUTE_DSP_TRANSCODED_COMPRESSED:
                    ALOGVV("ROUTE_DSP_TRANSCODED_COMPRESSED");
                    mOpenTranscodeRoute = true;
                    mRouteTrancodeFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    mTranscodeFormatDevices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    break;
                case ROUTE_SW_TRANSCODED_COMPRESSED:
                    ALOGVV("ROUTE_SW_TRANSCODED_COMPRESSED");
                    mOpenTranscodeRoute = true;
                    mRouteTrancodeFormat = routeToDriver[idx][ROUTE_FORMAT_IDX];
                    mDecodeFormatDevices &= ~AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    mTranscodeFormatDevices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    break;
                default:
                    break;
                }
            }
        }
    }
    if(mDevices & ~(AudioSystem::DEVICE_OUT_AUX_DIGITAL |
                    AudioSystem::DEVICE_OUT_SPDIF)) {
        mDecoderType |= usecaseDecodeFormat[NUM_STATES_FOR_EACH_DEVICE_FMT*formatIndex];
        mOpenDecodeRoute = true;
        mRouteDecodeFormat = ROUTE_UNCOMPRESSED;
    }
}

/*******************************************************************************
Description: update handle states
*******************************************************************************/
void AudioSessionOutALSA::updateRxHandleStates()
{
    ALOGD("updateRxHandleStates");
    int index = 0;
    if(mOpenDecodeRoute) {
        mRxHandleRouteFormat[index] = mRouteDecodeFormat;
        mRxHandleDevices[index] = mDecodeFormatDevices;
        mRxHandleRouteFormatType[index] = getDeviceFormat(mDecodeFormatDevices);
        ALOGD("mOpenDecodeRoute: index: %d routeformat: %d, devices: 0x%x: "
               "routeformatType: %d", index, mRxHandleRouteFormat[index],
               mRxHandleDevices[index], mRxHandleRouteFormatType[index]);
        index++;
    }
    if(mOpenPassthroughRoute) {
        mRxHandleRouteFormat[index] = mRoutePassthroughFormat;
        mRxHandleDevices[index] = mPassthroughFormatDevices;
        mRxHandleRouteFormatType[index] = getDeviceFormat(mPassthroughFormatDevices);
        ALOGD("mOpenPassthroughRoute: index: %d routeformat: %d, devices: 0x%x: "
               "routeformatType: %d", index, mRxHandleRouteFormat[index],
               mRxHandleDevices[index], mRxHandleRouteFormatType[index]);
        index++;
    }
    if(mOpenTranscodeRoute) {
        mRxHandleRouteFormat[index] = mRouteTrancodeFormat;
        mRxHandleDevices[index] = mTranscodeFormatDevices;
        mRxHandleRouteFormatType[index] = getDeviceFormat(mTranscodeFormatDevices);
        ALOGD("mOpenTranscodeRoute: index: %d routeformat: %d, devices: 0x%x: "
               "routeformatType: %d", index, mRxHandleRouteFormat[index],
               mRxHandleDevices[index], mRxHandleRouteFormatType[index]);
        index++;
    }
    mNumRxHandlesActive = index;
}

/*******************************************************************************
Description: Get the device format based on the available devices
*******************************************************************************/
int AudioSessionOutALSA::getDeviceFormat(int devices)
{
    ALOGVV("getDeviceFormat");
    if (devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
        return mHdmiFormat;
    else if (devices & AudioSystem::DEVICE_OUT_SPDIF)
        return mSpdifFormat;
    else
        return UNCOMPRESSED;

}

/*******************************************************************************
Description: validate if the decoder requires configuration to be set as first
             buffer
*******************************************************************************/
bool AudioSessionOutALSA::isDecoderConfigRequired()
{
    ALOGVV("isDecoderConfigRequired");
    if(!mIsMS11FilePlaybackMode)
        return false;
    for(int i=0; i<sizeof(decodersRequireConfig)/sizeof(int); i++)
        if(mFormat == decodersRequireConfig[i])
            return true;

    return false;
}

/*******************************************************************************
Description: query if input buffering mode require
*******************************************************************************/
bool AudioSessionOutALSA::isInputBufferingModeReqd()
{
    ALOGVV("isInputBufferingModeReqd");
    if((mDecoderType == SW_PASSTHROUGH) ||
       (mDecoderType == DSP_PASSTHROUGH))
        return true;
    else
        return false;
}

/*******************************************************************************
Description: get levels of buffering, interms of number of buffers
*******************************************************************************/
int AudioSessionOutALSA::getBufferingFactor()
{
    ALOGVV("getBufferingFactor");
    if((mFormat == AUDIO_FORMAT_PCM_16_BIT) ||
       (mFormat == AUDIO_FORMAT_PCM_24_BIT))
        return 1;
    else
        return NUM_OF_PERIODS_COMPR;
}

/*******************************************************************************
Description: setup input path
*******************************************************************************/
status_t AudioSessionOutALSA::routingSetup()
{
    ALOGV("routingSetup");
    status_t status;
    int bufferCount;
    /*
    setup the bitstream state machine
    */
    mBitstreamSM = new AudioBitstreamSM;
    if(false == mBitstreamSM->initBitstreamPtr(getBufferingFactor())) {
        ALOGE("Unable to allocate Memory for i/p and o/p buffering for MS11");
        delete mBitstreamSM;
        return BAD_VALUE;
    }
    if(isInputBufferingModeReqd())
        mBitstreamSM->startInputBufferingMode();
    /*
    setup the MS11 decoder
    */
    if(mUseMS11Decoder) {
        status = openMS11Instance();
        if(status != NO_ERROR) {
            ALOGE("Unable to open MS11 instance succesfully- exiting");
            mUseMS11Decoder = false;
            delete mBitstreamSM;
            return status;
        }
    }
    /*
    setup the buffering data required for decode to start
    AAC_ADIF would require worst case frame size before decode starts
    other decoder formats handles the partial data, hence threshold is zero.
    */
    if(mFormat == AUDIO_FORMAT_AAC_ADIF)
        mMinBytesReqToDecode = AAC_BLOCK_PER_CHANNEL_MS11*mChannels-1;
    else
        mMinBytesReqToDecode = 0;


    if((mFormat != AUDIO_FORMAT_WMA) &&
       (mFormat != AUDIO_FORMAT_WMA_PRO)) {
        for(int index=0; index < mNumRxHandlesActive; index++) {
            status = openPlaybackDevice(index, mRxHandleDevices[index],
                                        mRxHandleRouteFormat[index]);
            if(status) {
                ALOGE("opening Playback device failed");
            } else {
                if(mRxHandleRouteFormat[index] == ROUTE_UNCOMPRESSED)
                    status = mALSADevice->setPlaybackVolume(mRxHandle[index],
                                                            mStreamVol);
                if(status)
                    ALOGE("setPlaybackVolume for WMA playback failed");
            }
        }
        status = openTempBufForMetadataModeRendering();

        if(!status && mRouteAudioToA2dp)
            status = a2dpRenderingControl(A2DP_RENDER_SETUP);
    }

    mBufferSize = getBufferLength();

    return status;
}

/*******************************************************************************
Description: open MS11 instance
*******************************************************************************/
status_t AudioSessionOutALSA::openMS11Instance()
{
    ALOGV("openMS11Instance");
    int32_t formatMS11;
    /*
    MS11 created
    */
    mMS11Decoder = new SoftMS11;
    if(mMS11Decoder->initializeMS11FunctionPointers() == false) {
        ALOGE("Could not resolve all symbols Required for MS11");
        delete mMS11Decoder;
        return BAD_VALUE;
    }
    /*
    update format
    */
    if((mFormat == AUDIO_FORMAT_AC3) ||
       (mFormat == AUDIO_FORMAT_EAC3))
        formatMS11 = FORMAT_DOLBY_DIGITAL_PLUS_MAIN;
    else
        formatMS11 = FORMAT_DOLBY_PULSE_MAIN;
    /*
    set the use case to the MS11 decoder and open the stream for decoding
    */
    if(mMS11Decoder->setUseCaseAndOpenStream(formatMS11,mChannels,mSampleRate,
                                             mIsMS11FilePlaybackMode)) {
        ALOGE("SetUseCaseAndOpen MS11 failed");
        delete mMS11Decoder;
        return BAD_VALUE;
    }
    if(isDecoderConfigRequired() && mDecoderConfigBuffer && mDecoderConfigBufferLength) {
        if(mMS11Decoder->setAACConfig((unsigned char *)mDecoderConfigBuffer,
                                      mDecoderConfigBufferLength) == true) {
            mDecoderConfigSet = true;
        }
    }
    return NO_ERROR;
}

/*******************************************************************************
Description: Initialize Flags and device handles
*******************************************************************************/
status_t AudioSessionOutALSA::a2dpRenderingControl(int state)
{
    status_t status = NO_ERROR;

    int index = getIndexHandleBasedOnHandleDevices(AudioSystem::DEVICE_OUT_PROXY);
    if (mRouteAudioToA2dp) {
        switch(state) {
        case A2DP_RENDER_SETUP:
            if(mRxHandle[index])
                mA2dpUseCase = mParent->useCaseStringToEnum(mRxHandle[index]->useCase);
            // fall through
        case A2DP_RENDER_START:
            ALOGD("startA2dpPlayback_l - resume - usecase %x", mA2dpUseCase);
            status = mParent->startA2dpPlayback_l(mA2dpUseCase);
            if(status)
                ALOGE("startA2dpPlayback_l from resume return error = %d", status);
            break;
        case A2DP_RENDER_STOP:
            ALOGD("destructor - stopA2dpPlayback_l - usecase %x", mA2dpUseCase);
            status = mParent->stopA2dpPlayback_l(mA2dpUseCase);
            if(status)
                ALOGE("destructor - stopA2dpPlayback_l return err = %d", status);
            mRouteAudioToA2dp = false;
            break;
        case A2DP_RENDER_SUSPEND:
            ALOGD("Pause - suspendA2dpPlayback_l - usecase %x", mA2dpUseCase);
            status = mParent->suspendA2dpPlayback_l(mA2dpUseCase);
            if(status)
                ALOGE("suspend Proxy from Pause returned error = %d",status);
            break;
        default:
            ALOGE("Invalid state");
            status = BAD_VALUE;
            break;
        }
    }
    return status;
}

/*******************************************************************************
Description: open playback devices
*******************************************************************************/
status_t AudioSessionOutALSA::openPlaybackDevice(int index, int devices,
                                                 int deviceFormat)
{
    ALOGV("openPlaybackDevice");
    status_t status = NO_ERROR;
    char *use_case;

    if(deviceFormat == ROUTE_UNCOMPRESSED ||
       deviceFormat == ROUTE_COMPRESSED ||
       deviceFormat == ROUTE_SW_TRANSCODED_COMPRESSED) {
        setPlaybackFormat(devices, deviceFormat);
        if(status != NO_ERROR) {
             ALOGE("setPlaybackFormat Failed");
             return BAD_VALUE;
        }
        snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
        if ((use_case == NULL) ||
            (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
             strlen(SND_USE_CASE_VERB_INACTIVE)))) {
            use_case = mALSADevice->getPlaybackUseCase(COMRPESSED_DRIVER, true/*isVerb*/);
            if(use_case == NULL)
                return NO_INIT;
            status = openDevice(use_case, true/*bIsUseCase*/, devices, deviceFormat);
        } else {
            use_case = mALSADevice->getPlaybackUseCase(COMRPESSED_DRIVER, false/*isVerb*/);
            if(use_case == NULL)
                return NO_INIT;
            status = openDevice(use_case, false/*bIsUseCase*/, devices, deviceFormat);
        }
        if(use_case) {
            free(use_case);
            use_case = NULL;
        }
        if(status != NO_ERROR) {
            return status;
        }
        ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
        mRxHandle[index] = &(*it);
        if(deviceFormat == ROUTE_UNCOMPRESSED) {
           if(mUseMS11Decoder &&
              (mRxHandle[index]->channels > 2)) {
               setMS11ChannelMap(mRxHandle[index]);
           } else if((mFormat == AUDIO_FORMAT_PCM_16_BIT ||
                      mFormat == AUDIO_FORMAT_PCM_24_BIT) &&
                     (mRxHandle[index]->channels > 2)) {
               setPCMChannelMap(mRxHandle[index]);
           }
        }
    } else if (deviceFormat == ROUTE_DSP_TRANSCODED_COMPRESSED) {
        int routeUnCompressedIndex;
        for(routeUnCompressedIndex = 0; routeUnCompressedIndex < mNumRxHandlesActive;
            mNumRxHandlesActive++) {
            if(mRxHandleRouteFormat[routeUnCompressedIndex] == ROUTE_UNCOMPRESSED)
                break;
        }
        if(routeUnCompressedIndex == mNumRxHandlesActive) {
            ALOGE("No Route uncompressed session to transcode");
            return NO_INIT;
        }
        mRxHandle[index] = (alsa_handle_t *) calloc(1, sizeof(alsa_handle_t));
        mRxHandle[index]->devices = devices;
        mRxHandle[index]->activeDevice= devices;
        mRxHandle[index]->mode = mParent->mode();
        mRxHandle[index]->ucMgr = mUcMgr;
        mRxHandle[index]->module = mALSADevice;
        //Passthrough to be configured with 2 channels
        mRxHandle[index]->channels = 2;
        mRxHandle[index]->sampleRate = 48000;
        ALOGV("Transcode devices = %d", devices);
        // NOTE: transcode should always be configured once a pcm/compressed format
        // is opened. Hence the use case can always be referred to previous index
        // Transcode coexist with only ROUTE_UNCOMPRESSED. So, get the use case and
        // append it with pseduo string
        strlcpy(mRxHandle[index]->useCase, mRxHandle[routeUnCompressedIndex-1]->useCase,
                sizeof(mRxHandle[index]->useCase));
        strncat(mRxHandle[index]->useCase, SND_USE_CASE_PSEUDO_TUNNEL_SUFFIX,
                sizeof(mRxHandle[index]->useCase));
        mALSADevice->setUseCase(mRxHandle[index], false);
        mALSADevice->configureTranscode(mRxHandle[index]);
    } else if(deviceFormat == ROUTE_NONE) {
        ALOGD("No Routing format to taken an action");
    }
    return NO_ERROR;
}

/*******************************************************************************
Description: set end device playback format
*******************************************************************************/
status_t AudioSessionOutALSA::setPlaybackFormat(int devices, int deviceFormat)
{
    ALOGV("setPlaybackFormat");
    status_t status = NO_ERROR;
    if(devices & AudioSystem::DEVICE_OUT_SPDIF) {
        status = mALSADevice->setPlaybackFormat(deviceFormat == ROUTE_UNCOMPRESSED ? "LPCM" : "Compr",
                                                AudioSystem::DEVICE_OUT_SPDIF,
                                                false);
                                              // NOTE: some crazy stuff was done for the
                                              // last argument. dont see a need now. confirm??
        if(status) {
            ALOGE("set playback format failed");
            return status;
        }
    }
    if(devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL) {
        status = mALSADevice->setPlaybackFormat(deviceFormat == ROUTE_UNCOMPRESSED ? "LPCM" : "Compr",
                                                AudioSystem::DEVICE_OUT_AUX_DIGITAL,
                                                false);
                                              // NOTE: some crazy stuff was done for the
                                              // last argument. dont see a need now. confirm??
        if(status) {
            ALOGE("set playback format failed");
            return status;
        }
    }
    return NO_ERROR;
}

/*******************************************************************************
Description: open the device for playback
*******************************************************************************/
status_t AudioSessionOutALSA::openDevice(const char *useCase, bool bIsUseCase,
                                         int devices, int deviceFormat)
{
    ALOGD("openDevice: E");
    alsa_handle_t alsa_handle;
    status_t status = NO_ERROR;
    int periodSize, periodCount, format;
    getPeriodSizeCountAndFormat(deviceFormat, &periodSize, &periodCount, &format);
    alsa_handle.module      = mALSADevice;
    alsa_handle.bufferSize  = periodSize * periodCount;
    alsa_handle.periodSize  = periodSize;
    alsa_handle.devices     = devices;
    alsa_handle.activeDevice= devices;
    alsa_handle.handle      = 0;
    alsa_handle.channels    = mChannels;
    alsa_handle.sampleRate  = mSampleRate;
    alsa_handle.mode        = mParent->mode();
    alsa_handle.latency     = PLAYBACK_LATENCY;
    alsa_handle.rxHandle    = 0;
    alsa_handle.ucMgr       = mUcMgr;
    alsa_handle.timeStampMode = SNDRV_PCM_TSTAMP_NONE;
    alsa_handle.metaDataMode= true;
    alsa_handle.format      = format;
    alsa_handle.type        = deviceFormat;

    strlcpy(alsa_handle.useCase, useCase, sizeof(alsa_handle.useCase));
    if (mALSADevice->setUseCase(&alsa_handle, bIsUseCase))
        return NO_INIT;
    status = mALSADevice->openPlayback(&alsa_handle, false /* isMmapMode */);
    if(status != NO_ERROR) {
        ALOGE("Could not open the ALSA device for use case %s", alsa_handle.useCase);
        mALSADevice->close(&alsa_handle);
    } else{
        mParent->mDeviceList.push_back(alsa_handle);
    }
    ALOGD("openDevice: X");

    return NO_ERROR;
}

/*******************************************************************************
Description: get the buffer size based on format and device format type
*******************************************************************************/
void AudioSessionOutALSA::getPeriodSizeCountAndFormat(int routeFormat, int *periodSize,
                                       int *periodCount, int *format)
{
    ALOGV("getPeriodSizeCountAndFormat");
    *format = mFormat;
    *periodSize = PERIOD_SIZE_COMPR;
    *periodCount = NUM_OF_PERIODS_COMPR;
    switch(mFormat) {
    case AUDIO_FORMAT_PCM_16_BIT:
        *periodSize = PERIOD_SIZE_PCM_16BIT*mChannels;
        *periodCount = NUM_OF_PERIODS_PCM;
        break;
    case AUDIO_FORMAT_PCM_24_BIT:
        *periodSize = PERIOD_SIZE_PCM_24BIT*mChannels;
        *periodCount = NUM_OF_PERIODS_PCM;
        break;
    case AUDIO_FORMAT_AAC:
    case AUDIO_FORMAT_HE_AAC_V1:
    case AUDIO_FORMAT_HE_AAC_V2:
    case AUDIO_FORMAT_AAC_ADIF:
    case AUDIO_FORMAT_AC3:
    case AUDIO_FORMAT_EAC3:
        if(routeFormat == ROUTE_UNCOMPRESSED) {
            *periodSize = PERIOD_SIZE_PCM_16BIT*mChannels;
            *periodCount = NUM_OF_PERIODS_PCM;
            *format = AUDIO_FORMAT_PCM_16_BIT;
        } else {
            *periodSize = PERIOD_SIZE_COMPR;
            *periodCount = NUM_OF_PERIODS_COMPR;
        }
        break;
    case AUDIO_FORMAT_DTS:
    case AUDIO_FORMAT_DTS_LBR:
    case AUDIO_FORMAT_MP3:
    case AUDIO_FORMAT_WMA:
    case AUDIO_FORMAT_WMA_PRO:
    case AUDIO_FORMAT_MP2:
        *periodSize = PERIOD_SIZE_COMPR;
        *periodCount = NUM_OF_PERIODS_COMPR;
        break;
    }
    ALOGV("periodSize: %d, periodCnt: %d", *periodSize, *periodCount);
    return;
}

/*******************************************************************************
Description: buffer length updated to player
*******************************************************************************/
int AudioSessionOutALSA::getBufferLength()
{
    ALOGV("getBufferLength");
    int bufferSize = PERIOD_SIZE_COMPR;
    switch(mFormat) {
    case AUDIO_FORMAT_PCM_16_BIT:
        bufferSize = (int)((PCM_16_INPUT_BUFFER_SIZE*mSampleRate*mChannels)/48000);
        break;
    case AUDIO_FORMAT_PCM_24_BIT:
        bufferSize = (int)((PCM_24_INPUT_BUFFER_SIZE*mSampleRate*mChannels)/48000);
        break;
    case AUDIO_FORMAT_AAC:
    case AUDIO_FORMAT_HE_AAC_V1:
    case AUDIO_FORMAT_HE_AAC_V2:
    case AUDIO_FORMAT_AAC_ADIF:
        bufferSize = AAC_BLOCK_PER_CHANNEL_MS11 * mChannels;
        break;
    case AUDIO_FORMAT_AC3:
    case AUDIO_FORMAT_EAC3:
        bufferSize = AC3_BUFFER_SIZE;
        break;
    case AUDIO_FORMAT_DTS:
    case AUDIO_FORMAT_DTS_LBR:
        bufferSize = DTS_INPUT_BUFFER_SIZE;
        break;
    case AUDIO_FORMAT_MP3:
        bufferSize = MP3_INPUT_BUFFER_SIZE;
        break;
    case AUDIO_FORMAT_WMA:
    case AUDIO_FORMAT_WMA_PRO:
        bufferSize = WMA_INPUT_BUFFER_SIZE;
        break;
    case AUDIO_FORMAT_MP2:
        bufferSize = MP2_INPUT_BUFFER_SIZE;
        break;
    }
    return bufferSize;
}

/*******************************************************************************
Description: open temp buffer so that meta data mode can be updated properly
*******************************************************************************/
status_t AudioSessionOutALSA::openTempBufForMetadataModeRendering()
{
    int maxLength = 0;
    for(int index=0; index < mNumRxHandlesActive; index++) {
       maxLength = mRxHandle[index]->periodSize > maxLength ?
                       mRxHandle[index]->periodSize :
                       maxLength;
    }
    if (mWriteTempBuffer == NULL) {
        mWriteTempBuffer = (char *) malloc(maxLength);
        if (mWriteTempBuffer == NULL) {
            ALOGE("Memory allocation of temp buffer to write pcm to driver failed");
            return BAD_VALUE;
        }
    }
    return NO_ERROR;
}

/*******************************************************************************
Description: close the device
*******************************************************************************/
status_t AudioSessionOutALSA::closeDevice(alsa_handle_t *pHandle)
{
    status_t status = NO_ERROR;
    ALOGV("closeDevice");
    if(pHandle) {
        ALOGV("useCase %s", pHandle->useCase);
        status = mALSADevice->close(pHandle);
        for(ALSAHandleList::iterator it = mParent->mDeviceList.begin();
            it != mParent->mDeviceList.end(); ++it) {
            alsa_handle_t *it_dup = &(*it);
            if(!strcmp(it_dup->useCase, pHandle->useCase)) {
                mALSADevice->freePlaybackUseCase(pHandle->useCase);
                mParent->mDeviceList.erase(it);
                break;
            }
        }
        if(status) {
            ALOGE("close device failed");
            return BAD_VALUE;
        }
    }
    return NO_ERROR;
}

/*******************************************************************************
Description: setting ms11 channel map
*******************************************************************************/
void AudioSessionOutALSA::setMS11ChannelMap(alsa_handle_t *handle)
{
    ALOGVV("setMS11ChannelMap");
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


/*******************************************************************************
Description: setting PCM channel map
*******************************************************************************/
void AudioSessionOutALSA::setPCMChannelMap(alsa_handle_t *handle)
{
    ALOGVV("setPCMChannelMap");
    char channelMap[8];
    status_t status = NO_ERROR;

    memset(channelMap, 0, sizeof(channelMap));
    switch (handle->channels) {
    case 3:
        ALOGE("TODO: Investigate and add appropriate channel map appropriately");
        ALOGE("TODO: As of default to standard channel maps for 3.0");
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_FC;
        break;
    case 4:
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_LB;
        channelMap[3] = PCM_CHANNEL_RB;
        break;
    case 5:
        ALOGE("TODO: Investigate and add appropriate channel map appropriately");
        ALOGE("TODO: As of default to standard channel maps for 5.0");
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_FC;
        channelMap[3] = PCM_CHANNEL_LB;
        channelMap[4] = PCM_CHANNEL_RB;
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
        ALOGE("TODO: As of default to standard channel maps for 7.0");
        channelMap[0] = PCM_CHANNEL_FL;
        channelMap[1] = PCM_CHANNEL_FR;
        channelMap[2] = PCM_CHANNEL_FC;
        channelMap[3] = PCM_CHANNEL_LB;
        channelMap[4] = PCM_CHANNEL_RB;
        channelMap[5] = PCM_CHANNEL_FLC;
        channelMap[6] = PCM_CHANNEL_FRC;
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

/*******************************************************************************
Description: set decoder config
*******************************************************************************/
int AudioSessionOutALSA::setDecodeConfig(char *buffer, size_t bytes)
{
    ALOGD("setDecodeConfig");
    uint32_t bytesConsumed = 0;
    if(!mDecoderConfigSet) {
        if(mFormat == AUDIO_FORMAT_WMA || mFormat == AUDIO_FORMAT_WMA_PRO) {
            ALOGV("Configuring the WMA params");
            status_t err = mALSADevice->setWMAParams((int *)buffer, bytes/sizeof(int));
            if (err) {
                ALOGE("WMA param config failed");
                return BAD_VALUE;
            }
            for(int index=0; index < mNumRxHandlesActive; index++) {
                if(openPlaybackDevice(index, mRxHandleDevices[index],
                                      mRxHandleRouteFormat[index])) {
                   ALOGE("opening Playback device failed");
                   return 0;
                } else if(mRxHandleRouteFormat[index] == ROUTE_UNCOMPRESSED) {
                    if(mALSADevice->setPlaybackVolume(mRxHandle[index],
                                                            mStreamVol))
                        ALOGE("setPlaybackVolume for playback failed");
                }
            }
            if (openTempBufForMetadataModeRendering()) {
                ALOGE("temp buffer to update Meta data not valid");
                return 0;
            }
            if(mRouteAudioToA2dp)
                if(a2dpRenderingControl(A2DP_RENDER_SETUP))
                    return 0;
        } else if(mFormat == AUDIO_FORMAT_AAC ||
                  mFormat == AUDIO_FORMAT_HE_AAC_V1 ||
                  mFormat == AUDIO_FORMAT_AAC_ADIF ||
                  mFormat == AUDIO_FORMAT_HE_AAC_V2) {
            if(mMS11Decoder != NULL) {
                if(mMS11Decoder->setAACConfig((unsigned char *)buffer,
                                     bytes) == false) {
                    ALOGE("AAC decoder config fail");
                    return 0;
                }
            }
        }
        mDecoderConfigBufferLength = bytesConsumed = bytes;
        mDecoderConfigBuffer = malloc(mDecoderConfigBufferLength);
        memcpy(mDecoderConfigBuffer, buffer, mDecoderConfigBufferLength);
        mDecoderConfigSet = true;
    }
    mDecoderConfigSet = true;
    return bytesConsumed;
}

/*******************************************************************************
Description: copy input to internal buffer
*******************************************************************************/
void AudioSessionOutALSA::copyBitstreamInternalBuffer(char *buffer, size_t bytes)
{
    ALOGV("copyBitstreamInternalBuffer");
    // copy bitstream to internal buffer
    mBitstreamSM->copyBitstreamToInternalBuffer((char *)buffer, bytes);
#ifdef DEBUG
    dumpInputOutput(INPUT, buffer, bytes);
#endif
}

/*******************************************************************************
Description: decode
*******************************************************************************/
bool AudioSessionOutALSA::decode(char *buffer, size_t bytes)
{
    ALOGV("decode");
    bool    continueDecode = false;

    if (mMS11Decoder != NULL) {
        continueDecode = swDecode(buffer, bytes);

        // set channel status
        // Set the channel status after first frame decode/transcode
        if(mChannelStatusSet == false)
            setSpdifChannelStatus(mBitstreamSM->getOutputBufferPtr(COMPRESSED_OUT),
                                  bytes, AUDIO_PARSER_CODEC_AC3);
    } else {
        continueDecode = dspDecode(buffer, bytes);
        // set channel status
        // Set the channel status after first frame decode/transcode
        if(mChannelStatusSet == false)
            setSpdifChannelStatus(mBitstreamSM->getOutputBufferPtr(COMPRESSED_OUT),
                                  bytes, AUDIO_PARSER_CODEC_DTS);
    }
    return continueDecode;
}

/*******************************************************************************
Description: software decode handling
*******************************************************************************/
bool AudioSessionOutALSA::swDecode(char *buffer, size_t bytes)
{
    bool continueDecode = false;
    char    *bufPtr;
    size_t  copyBytesMS11 = 0;
    size_t  bytesConsumedInDecode = 0;
    size_t  copyOutputBytesSize = 0;
    uint32_t outSampleRate = mSampleRate;
    uint32_t outChannels = mChannels;
    ALOGVV("sw Decode");
    // eos handling
    if(bytes == 0) {
        if(mFormat == AUDIO_FORMAT_AAC_ADIF)
            mBitstreamSM->appendSilenceToBitstreamInternalBuffer(
                                                  mMinBytesReqToDecode,0x0);
        else
            return false;
    }
    /*
    check for sync word, if present then configure MS11 for fileplayback mode
    OFF. This is specifically done to handle Widevine usecase, in which the
    ADTS HEADER is not stripped off by the Widevine parser
    */
    if(mFirstBitstreamBuffer == true) {
        uint16_t uData = (*((char *)buffer) << 8) + *((char *)buffer + 1) ;
        if(ADTS_HEADER_SYNC_RESULT == (uData & ADTS_HEADER_SYNC_MASK)) {
            ALOGD("Sync word found hence configure MS11 in file_playback Mode OFF");
            delete mMS11Decoder;
            mIsMS11FilePlaybackMode = false;
            openMS11Instance();
        }
        mFirstBitstreamBuffer = false;
    }
    //decode
    if(mDecoderType == SW_PASSTHROUGH) {
        bytesConsumedInDecode = mBitstreamSM->getInputBufferWritePtr() -
                                  mBitstreamSM->getInputBufferPtr();
    } else {
        if(mBitstreamSM->sufficientBitstreamToDecode(mMinBytesReqToDecode)
                             == true) {
            bufPtr = mBitstreamSM->getInputBufferPtr();
            copyBytesMS11 = mBitstreamSM->bitStreamBufSize();

            mMS11Decoder->copyBitstreamToMS11InpBuf(bufPtr,copyBytesMS11);
            bytesConsumedInDecode = mMS11Decoder->streamDecode(
                                     &outSampleRate, &outChannels);
        }
    }
    //handle change in sample rate
    if((mSampleRate != outSampleRate) || (mChannels != outChannels)) {
        ALOGD("Change in sample rate. New sample rate: %d", outSampleRate);
        mSampleRate = outSampleRate;
        mChannels = outChannels;
        int index = getIndexHandleBasedOnHandleFormat(ROUTE_UNCOMPRESSED);
        if(mRxHandle[index]) {
            status_t status = closeDevice(mRxHandle[index]);
            if(status != NO_ERROR)
                ALOGE("change in sample rate - close pcm device fail");
            status = openPlaybackDevice(index, mRxHandleDevices[index],
                                       mRxHandleRouteFormat[index]);
            if(status != NO_ERROR) {
                ALOGE("change in sample rate - open pcm device fail");
            } else if(mRxHandleRouteFormat[index] == ROUTE_UNCOMPRESSED) {
                status = mALSADevice->setPlaybackVolume(mRxHandle[index],
                                                        mStreamVol);
                if(status)
                    ALOGE("setPlaybackVolume for playback failed");
            }
        }
        mChannelStatusSet = false;
    }
    // copy the output of decoder to HAL internal buffers
    if(mDecoderType & SW_DECODE) {
        bufPtr=mBitstreamSM->getOutputBufferWritePtr(PCM_MCH_OUT);
        copyOutputBytesSize = mMS11Decoder->copyOutputFromMS11Buf(PCM_MCH_OUT,
                                                                 bufPtr);
        mBitstreamSM->setOutputBufferWritePtr(PCM_MCH_OUT,copyOutputBytesSize);
    }
    if(mDecoderType & SW_PASSTHROUGH) {
        bufPtr=mBitstreamSM->getOutputBufferWritePtr(COMPRESSED_OUT);
        copyOutputBytesSize = bytesConsumedInDecode;
        memcpy(bufPtr, mBitstreamSM->getInputBufferPtr(), copyOutputBytesSize);
        mBitstreamSM->setOutputBufferWritePtr(COMPRESSED_OUT,copyOutputBytesSize);
    }
    if(mDecoderType & SW_TRANSCODE) {
        bufPtr=mBitstreamSM->getOutputBufferWritePtr(TRANSCODE_OUT);
        copyOutputBytesSize = mMS11Decoder->copyOutputFromMS11Buf(COMPRESSED_OUT,
                                                bufPtr);
        mBitstreamSM->setOutputBufferWritePtr(TRANSCODE_OUT,copyOutputBytesSize);
    }
    mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
    if(copyOutputBytesSize &&
       mBitstreamSM->sufficientBitstreamToDecode(mMinBytesReqToDecode) == true)
        continueDecode = true;

    return continueDecode;
}

/*******************************************************************************
Description: dsp decode handling
*******************************************************************************/
bool AudioSessionOutALSA::dspDecode(char *buffer, size_t bytes)
{
    char    *bufPtr;
    size_t  bytesConsumedInDecode = 0;
    size_t  copyOutputBytesSize = 0;
    ALOGVV("dspDecode");
    // decode
    {
        bytesConsumedInDecode = mBitstreamSM->getInputBufferWritePtr() -
                                  mBitstreamSM->getInputBufferPtr();
    }
    // handle change in sample rate
    {
    }
    // copy the output of decoder to HAL internal buffers
    if(mDecoderType & DSP_DECODE) {
        ALOGVV("DSP_DECODE");
        bufPtr = mBitstreamSM->getOutputBufferWritePtr(PCM_MCH_OUT);
        copyOutputBytesSize = bytesConsumedInDecode;
        memcpy(bufPtr, mBitstreamSM->getInputBufferPtr(), copyOutputBytesSize);
        mBitstreamSM->setOutputBufferWritePtr(PCM_MCH_OUT,
                                              copyOutputBytesSize);
    }
    if(mDecoderType & DSP_PASSTHROUGH) {
        ALOGVV("DSP_PASSTHROUGH");
        bufPtr = mBitstreamSM->getOutputBufferWritePtr(COMPRESSED_OUT);
        copyOutputBytesSize = bytesConsumedInDecode;
        memcpy(bufPtr, mBitstreamSM->getInputBufferPtr(), copyOutputBytesSize);
        mBitstreamSM->setOutputBufferWritePtr(COMPRESSED_OUT,
                                              copyOutputBytesSize);
    }
    mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
    return false;
}

/*******************************************************************************
Description: set spdif channel status
*******************************************************************************/
void AudioSessionOutALSA::setSpdifChannelStatus(char *buffer, size_t bytes,
                                                audio_parser_code_type codec_type)
{
    ALOGV("setSpdifChannelStatus");
    if(mSpdifFormat == FORMAT_PCM) {
        if (mALSADevice->get_linearpcm_channel_status(mSampleRate,
                              mChannelStatus)) {
            ALOGE("channel status set error ");
        }
        mALSADevice->setChannelStatus(mChannelStatus);
    } else if(mSpdifFormat == FORMAT_COMPR) {
        if (mALSADevice->get_compressed_channel_status(
                             buffer, bytes, mChannelStatus,
                             codec_type)) {
            ALOGE("channel status set error ");
        }
        mALSADevice->setChannelStatus(mChannelStatus);
    }
    mChannelStatusSet = true;
    return;
}

/*******************************************************************************
Description: render
*******************************************************************************/
uint32_t AudioSessionOutALSA::render(bool continueDecode/* used for time stamp mode*/)
{
    snd_pcm_sframes_t n;
    uint32_t renderedPcmBytes = 0;
    int      periodSize;
    uint32_t availableSize;
    int renderType;
    int metadataLength = sizeof(mOutputMetadata);

    ALOGV("render");
    for(int index=0; index < mNumRxHandlesActive; index++) {
        switch(mRxHandleRouteFormat[index]) {
        case ROUTE_UNCOMPRESSED:
            ALOGVV("ROUTE_UNCOMPRESSED");
            renderType = PCM_MCH_OUT;
            break;
        case ROUTE_COMPRESSED:
            ALOGVV("ROUTE_COMPRESSED");
            renderType = COMPRESSED_OUT;
            break;
        case ROUTE_SW_TRANSCODED_COMPRESSED:
            ALOGVV("ROUTE_SW_TRANSCODED_COMPRESSED");
            renderType = TRANSCODE_OUT;
            break;
        case ROUTE_DSP_TRANSCODED_COMPRESSED:
            ALOGVV("ROUTE_DSP_TRANSCODED_COMPRESSED");
            continue;
        default:
            continue;
        };
        periodSize = mRxHandle[index]->periodSize;
        while(mBitstreamSM->sufficientSamplesToRender(renderType,
                                                      1) == true) {
            availableSize = mBitstreamSM->getOutputBufferWritePtr(renderType) -
                            mBitstreamSM->getOutputBufferPtr(renderType);
            mOutputMetadata.metadataLength = metadataLength;
            mOutputMetadata.bufferLength = (availableSize >=
                                             periodSize - metadataLength) ?
                                           periodSize - metadataLength :
                                           availableSize;
            mOutputMetadata.timestamp = 0;
            memcpy(mWriteTempBuffer, &mOutputMetadata, metadataLength);
            memcpy(mWriteTempBuffer+metadataLength,
                   mBitstreamSM->getOutputBufferPtr(renderType),
                   mOutputMetadata.bufferLength);
            n = mParent->hw_pcm_write(mRxHandle[index]->handle,
                                      mWriteTempBuffer,
                                      periodSize);
            ALOGD("pcm_write returned with %d", n);
            if(n < 0) {
                // Recovery is part of pcm_write. TODO split is later.
                ALOGE("pcm_write returned n < 0");
                return static_cast<ssize_t>(n);
            } else {
                if(renderType == ROUTE_UNCOMPRESSED) {
                    mFrameCount++;
#ifdef DEBUG
                    dumpInputOutput(OUTPUT,
                                    mBitstreamSM->getOutputBufferPtr(PCM_MCH_OUT),
                                    mOutputMetadata.bufferLength);
#endif
                }
                renderedPcmBytes += mOutputMetadata.bufferLength;
                mBitstreamSM->copyResidueOutputToStart(renderType,
                                                       mOutputMetadata.bufferLength);
            }
        }
    }
    return renderedPcmBytes;
}

/*******************************************************************************
Description: EOS Handling
*******************************************************************************/
void AudioSessionOutALSA::eosHandling()
{
    ALOGD("eosHandling");
    for(int index=0; index < mNumRxHandlesActive; index++) {
        switch(mRxHandleRouteFormat[index]) {
            case ROUTE_UNCOMPRESSED:
            case ROUTE_COMPRESSED:
            case ROUTE_SW_TRANSCODED_COMPRESSED:
                if(ioctl(mRxHandle[index]->handle->fd, SNDRV_COMPRESS_DRAIN) < 0)
                    ALOGE("DRAIN Handling failed for index: %d", index);
                break;
            case ROUTE_DSP_TRANSCODED_COMPRESSED:
            default:
                 continue;
        }
    }
    if(mObserver)
        mObserver->postEOS(0);
    ALOGD("eosHandling X");
    return;
}

/*******************************************************************************
Description: device switch
*******************************************************************************/
status_t AudioSessionOutALSA::doRouting(int devices)
{
    status_t status = NO_ERROR;
    char *use_case;
    bool stopA2DP = false;
    int index = 0;
    ALOGD("doRouting: devices 0x%x, mDevices = 0x%x", devices,mDevices);
    Mutex::Autolock autoLock(mParent->mLock);
    Mutex::Autolock autoLock1(mLock);

    if(devices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        ALOGV("doRouting - Capture from Proxy");
        //NOTE: TODO - donot remove SPDIF device for A2DP device switch
        devices &= ~(AudioSystem::DEVICE_OUT_ALL_A2DP | AudioSystem::DEVICE_OUT_SPDIF);
        devices |=  AudioSystem::DEVICE_OUT_PROXY;
        mRouteAudioToA2dp = true;

    } else if(!(devices & AudioSystem::DEVICE_OUT_ALL_A2DP)) {
        if(mRouteAudioToA2dp) {
            mRouteAudioToA2dp = false;
            stopA2DP = true;
        }
        if(devices & AudioSystem::DEVICE_OUT_PROXY)
            devices &= ~AudioSystem::DEVICE_OUT_SPDIF;
    }
    //NOTE: make sure to remove this for non a2dp for handling output format
    //change at run time. Format change can happen with device switch to same
    // device but different format
    if(devices == mDevices) {
        ALOGW("Return same device ");
        if(stopA2DP == true) {
            status = a2dpRenderingControl(A2DP_RENDER_STOP);
        }
        return status;
    }

    if(devices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
        mALSADevice->updateHDMIEDIDInfo();
    updateDeviceSupportedFormats();

    if(isInputBufferingModeReqd())
        mBitstreamSM->startInputBufferingMode();
    else
        mBitstreamSM->stopInputBufferingMode();

    if(mOpenDecodeRoute) {
        handleSwitchAndOpenForDeviceSwitch(mDecodeFormatDevices,
                                        mRouteDecodeFormat);
    } else {
        handleCloseForDeviceSwitch(ROUTE_UNCOMPRESSED);
    }
    if(mOpenPassthroughRoute) {
        handleSwitchAndOpenForDeviceSwitch(mPassthroughFormatDevices,
                                        mRoutePassthroughFormat);
    } else {
        handleCloseForDeviceSwitch(ROUTE_COMPRESSED);
    }
    if(mOpenTranscodeRoute) {
        handleSwitchAndOpenForDeviceSwitch(mTranscodeFormatDevices,
                                        mRouteTrancodeFormat);
    } else {
        handleCloseForDeviceSwitch(ROUTE_SW_TRANSCODED_COMPRESSED|
                                   ROUTE_DSP_TRANSCODED_COMPRESSED);
    }

    if(stopA2DP)
        status = a2dpRenderingControl(A2DP_RENDER_STOP);
    else
        status = a2dpRenderingControl(A2DP_RENDER_SETUP);

    return status;
}

/*******************************************************************************
Description: adjust rx handle states. if lower index handles are free, move non
             NULL index handle and state accordingly
*******************************************************************************/
void AudioSessionOutALSA::adjustRxHandleAndStates()
{
    ALOGV("adjustRxHandleAndStates");
    int index = 0;
    if((index+1 < mNumRxHandlesActive) && mRxHandle[index+1] != NULL) {
        mRxHandle[index] = mRxHandle[index+1];
        mRxHandleRouteFormat[index] = mRxHandleRouteFormat[index+1];
        mRxHandleDevices[index] = mRxHandleDevices[index+1];
        mRxHandleRouteFormatType[index] = mRxHandleRouteFormatType[index+1];
    }
    return;
}

/*******************************************************************************
Description: reset the handle and the corresponding states
*******************************************************************************/
void AudioSessionOutALSA::resetRxHandleState(int index)
{
    ALOGV("resetRxHandleState");
    mRxHandle[index]                = NULL;
    mRxHandleRouteFormat[index]     = ROUTE_UNCOMPRESSED;
    mRxHandleDevices[index]         = AUDIO_DEVICE_NONE;
    mRxHandleRouteFormatType[index] = UNCOMPRESSED;
    return;
}

/*******************************************************************************
Description: Handles device switch - switch in the existing handle and opening
             new handle if required
*******************************************************************************/
void AudioSessionOutALSA::handleSwitchAndOpenForDeviceSwitch(int devices, int format)
{
    ALOGV("handleSwitchAndOpenForDeviceSwitch");
    int index;
    for(index = 0; index < mNumRxHandlesActive; index++) {
        if(mRxHandleRouteFormat[index] == format) {
            if(mRxHandleDevices[index] != devices) {
                mALSADevice->switchDeviceUseCase(mRxHandle[index],
                                                 devices,
                                                 mParent->mode());
                break;
            }
        }
    }
    if(index == mNumRxHandlesActive) {
        mRxHandleDevices[index] = devices;
        mRxHandleRouteFormat[index] = format;
        mRxHandleRouteFormatType[index] = getDeviceFormat(devices);
        if(openPlaybackDevice(index, mRxHandleDevices[index],
                           mRxHandleRouteFormat[index])) {
              ALOGE("openPlaybackDevice failed");
        } else {
            mNumRxHandlesActive++;
            if(mRxHandleRouteFormat[index] == ROUTE_UNCOMPRESSED) {
                if(mALSADevice->setPlaybackVolume(mRxHandle[index],
                                                        mStreamVol))
                    ALOGE("setPlaybackVolume for playback failed");
            }
        }
    }
    return;
}

/*******************************************************************************
Description: Handles device switch - close the device handle if not required
*******************************************************************************/
void AudioSessionOutALSA::handleCloseForDeviceSwitch(int format)
{
    ALOGV("handleCloseForDeviceSwitch");
    for(int index = 0; index < mNumRxHandlesActive; index++) {
        if(mRxHandleRouteFormat[index] == format) {
            if(closeDevice(mRxHandle[index]) != NO_ERROR) {
                ALOGE("Error closing pcm route device in doRouting");
                return;
            }
            resetRxHandleState(index);
            adjustRxHandleAndStates();
            mNumRxHandlesActive -= 1;
            break;
        }
    }
    return;
}

#if DEBUG
/*******************************************************************************
Description: dumping the data based on the state of the playback
*******************************************************************************/
void AudioSessionOutALSA::updateDumpWithPlaybackControls(int controlType)
{
    switch(controlType) {
    case PAUSE:
        char debugString[] = "Playback Paused";
        mFpDumpPCMOutput = fopen("/data/pcm_output.raw","a");
        if(mFpDumpPCMOutput != NULL) {
            fwrite(debugString, 1, sizeof(debugString), mFpDumpPCMOutput);
            fclose(mFpDumpPCMOutput);
        }
        mFpDumpInput = fopen("/data/input.raw","a");
        if(mFpDumpInput != NULL) {
            fwrite(debugString, 1, sizeof(debugString), mFpDumpInput);
            fclose(mFpDumpInput);
        }
        break;
    case RESUME:
        char debugString[] = "Playback Resumd";
        mFpDumpPCMOutput = fopen("/data/pcm_output.raw","a");
        if(mFpDumpPCMOutput != NULL) {
            fwrite(debugString, 1, sizeof(debugString), mFpDumpPCMOutput);
            fclose(mFpDumpPCMOutput);
        }
        mFpDumpInput = fopen("/data/input.raw","a");
        if(mFpDumpInput != NULL) {
            fwrite(debugString, 1, sizeof(debugString), mFpDumpInput);
            fclose(mFpDumpInput);
        }
        break;
    case SEEK:
        char debugString[] = "Playback Flushd";
        mFpDumpPCMOutput = fopen("/data/pcm_output.raw","a");
        if(mFpDumpPCMOutput != NULL) {
            fwrite(debugString, 1, sizeof(debugString), mFpDumpPCMOutput);
            fclose(mFpDumpPCMOutput);
        }
        mFpDumpInput = fopen("/data/input.raw","a");
        if(mFpDumpInput != NULL) {
            fwrite(debugString, 1, sizeof(debugString), mFpDumpInput);
            fclose(mFpDumpInput);
        }
        break;
    case PLAY:
        break;
    }
}

/*****************************************************************************
Description: dump input and output
*****************************************************************************/
void AudioSessionOutALSA::dumpInputOutput(int type, char *buffer, size_t bytes)
{
    if(type == INPUT) {
        mFpDumpInput = fopen("/data/input.raw","a");
        if(mFpDumpInput != NULL) {
            fwrite((char *)buffer, 1, bytes, mFpDumpInput);
            fclose(mFpDumpInput);
        }
    } else if(type == OUTPUT) {
        mFpDumpPCMOutput = fopen("/data/pcm_output.raw","a");
        if(mFpDumpPCMOutput != NULL) {
            fwrite(buffer, 1, bytes, mFpDumpPCMOutput);
            fclose(mFpDumpPCMOutput);
        }
    }
}
#endif
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

}       // namespace android_audio_legacy
