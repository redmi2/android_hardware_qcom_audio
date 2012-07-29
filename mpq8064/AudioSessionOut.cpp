/* AudioSessionOutALSA.cpp
 **
 ** Copyright 2008-2009 Wind River Systems
 ** Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

namespace sys_write {
    ssize_t lib_write(int fd, const void *buf, size_t count) {
        return write(fd, buf, count);
    }
};
namespace android_audio_legacy
{
// ----------------------------------------------------------------------------

AudioSessionOutALSA::AudioSessionOutALSA(AudioHardwareALSA *parent,
                                         uint32_t   devices,
                                         int        format,
                                         uint32_t   channels,
                                         uint32_t   samplingRate,
                                         int        sessionId,
                                         status_t   *status)
{
    devices |= AudioSystem::DEVICE_OUT_SPDIF;
    //hardcoding to hdmi default dev
    //devices |= AudioSystem::DEVICE_OUT_AUX_DIGITAL;
    //devices = devices & ~AudioSystem::DEVICE_OUT_SPEAKER;
    //end
    alsa_handle_t alsa_handle;
    char *use_case;
    bool bIsUseCaseSet = false;
    Mutex::Autolock autoLock(mLock);
    // Default initilization
    mParent         = parent;
    mALSADevice     = mParent->mALSADevice;
    mUcMgr          = mParent->mUcMgr;
    mFrameCount     = 0;
    mFormat         = format;
    mSampleRate     = samplingRate;
    ALOGV("channel map = %d", channels);
    mChannels = channels = channelMapToChannels(channels);
    if(format == AUDIO_FORMAT_AAC || format == AUDIO_FORMAT_HE_AAC_V1 ||
       format == AUDIO_FORMAT_HE_AAC_V2 || format == AUDIO_FORMAT_AAC_ADIF) {
        if(samplingRate > 24000) {
            mSampleRate     = 48000;
            mChannels       = 2;
        }
    } else if (format == AUDIO_FORMAT_AC3 || format == AUDIO_FORMAT_EAC3) {
        if(mChannels > 2)
            mChannels   = 2;
    }
    // NOTE: This has to be changed when Multi channel PCM has to be
    // sent over HDMI
    mDevices        = devices;
    mBufferSize     = 0;
    *status         = BAD_VALUE;
    mSessionId      = sessionId;


    mUseTunnelDecode    = false;
    mCaptureFromProxy   = false;
    mRoutePcmAudio      = false;
    mRoutePcmToSpdif    = false;
    mRouteCompreToSpdif = false;
    mRouteDtsToSpdif    = false;
    mRoutePcmToHdmi     = false;
    mRouteCompreToHdmi  = false;
    mRouteDtsToHdmi     = false;
    mRouteAudioToA2dp   = false;
    mOpenMS11Decoder    = false;
    mChannelStatusSet   = false;
    mTunnelPaused       = false;
    mPaused             = false;
    mTunnelSeeking      = false;
    mReachedExtractorEOS= false;
    mSkipWrite          = false;
    strlcpy(mSpdifOutputFormat,mParent->mSpdifOutputFormat,
                                      sizeof(mSpdifOutputFormat));
    strlcpy(mHdmiOutputFormat,mParent->mHdmiOutputFormat,
                                      sizeof(mHdmiOutputFormat));
    updateOutputFormat();

    mPcmRxHandle        = NULL;
    mPcmTxHandle        = NULL;
    mSpdifRxHandle      = NULL;
    mCompreRxHandle     = NULL;
    mMS11Decoder        = NULL;
    mBitstreamSM        = NULL;

    mInputBufferSize    = DEFAULT_BUFFER_SIZE;
    mInputBufferCount   = TUNNEL_DECODER_BUFFER_COUNT;
    mEfd = -1;

    mWMAConfigDataSet    = false;
    mAacConfigDataSet    = false; // flags if AAC config to be set(which is sent in first buffer)
    mEventThread         = NULL;
    mEventThreadAlive    = false;
    mKillEventThread     = false;
    mMinBytesReqToDecode = 0;
    mObserver            = NULL;
    mCurDevice           = 0;

    if(devices == 0) {
        ALOGE("No output device specified");
        return;
    }
    if((format == AUDIO_FORMAT_PCM_16_BIT) && (channels == 0 || channels > 6)) {
        ALOGE("Invalid number of channels %d", channels);
        return;
    }

    if(mDevices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        ALOGE("Set Capture from proxy true");
        mRouteAudioToA2dp = true;
        devices  &= ~AudioSystem::DEVICE_OUT_ALL_A2DP;
        devices  &= ~AudioSystem::DEVICE_OUT_SPDIF;
        devices |=  AudioSystem::DEVICE_OUT_PROXY;
        mDevices = devices;
    }

    if(format == AUDIO_FORMAT_AAC || format == AUDIO_FORMAT_HE_AAC_V1 ||
       format == AUDIO_FORMAT_HE_AAC_V2 || format == AUDIO_FORMAT_AAC_ADIF ||
       format == AUDIO_FORMAT_AC3 || format == AUDIO_FORMAT_EAC3) {
        // Instantiate MS11 decoder for single decode use case
        int32_t format_ms11;
        mMS11Decoder = new SoftMS11;
        if(mMS11Decoder->initializeMS11FunctionPointers() == false) {
            ALOGE("Could not resolve all symbols Required for MS11");
            delete mMS11Decoder;
            return;
        }
        mBitstreamSM = new AudioBitstreamSM;
        if(false == mBitstreamSM->initBitstreamPtr()) {
            ALOGE("Unable to allocate Memory for i/p and o/p buffering for MS11");
            delete mMS11Decoder;
            delete mBitstreamSM;
            return;
        }
        if(format == AUDIO_FORMAT_AAC || format == AUDIO_FORMAT_HE_AAC_V1 ||
           format == AUDIO_FORMAT_HE_AAC_V2 || format == AUDIO_FORMAT_AAC_ADIF)
        {
            if(format == AUDIO_FORMAT_AAC_ADIF)
                mMinBytesReqToDecode = AAC_BLOCK_PER_CHANNEL_MS11*mChannels-1;
            else
                mMinBytesReqToDecode = 0;
            format_ms11 = FORMAT_DOLBY_PULSE_MAIN;
        } else {
            format_ms11 = FORMAT_DOLBY_DIGITAL_PLUS_MAIN;
            mMinBytesReqToDecode = 0;
        }
        if(mMS11Decoder->setUseCaseAndOpenStream(format_ms11,channels,samplingRate)) {
            ALOGE("SetUseCaseAndOpen MS11 failed");
            delete mMS11Decoder;
            delete mBitstreamSM;
            return;
        }
        mOpenMS11Decoder= true; // indicates if MS11 decoder is instantiated
        mAacConfigDataSet = false; // flags if AAC config to be set(which is sent in first buffer)

        updateRoutingFlags();

    } else if(format == AUDIO_FORMAT_WMA || format == AUDIO_FORMAT_WMA_PRO ||
              format == AUDIO_FORMAT_DTS || format == AUDIO_FORMAT_MP3){
        // In this case, DSP will decode and route the PCM data to output devices
        mUseTunnelDecode = true;
        mWMAConfigDataSet = false;

        updateRoutingFlags();

    } else if(format == AUDIO_FORMAT_PCM_16_BIT) {
        mMinBytesReqToDecode = PCM_BLOCK_PER_CHANNEL_MS11*mChannels-1;
        if(channels > 2 && channels <= 6) {
            // Instantiate MS11 decoder for downmix and re-encode
            mMS11Decoder = new SoftMS11;
            if(mMS11Decoder->initializeMS11FunctionPointers() == false) {
                ALOGE("Could not resolve all symbols Required for MS11");
                delete mMS11Decoder;
                return;
            }
            if(mMS11Decoder->setUseCaseAndOpenStream(FORMAT_EXTERNAL_PCM,channels,samplingRate)) {
                ALOGE("SetUseCaseAndOpen MS11 failed");
                delete mMS11Decoder;
                return;
            }
            mOpenMS11Decoder=true;
        }
        mBitstreamSM = new AudioBitstreamSM;
        if(false == mBitstreamSM->initBitstreamPtr()) {
            ALOGE("Unable to allocate Memory for i/p and o/p buffering for MS11");
            delete mBitstreamSM;
            return;
        }
        updateRoutingFlags();

    } else {
        ALOGE("Unsupported format %d", format);
        return;
    }

    if(mRoutePcmAudio) {
        // If the same audio PCM is to be routed to SPDIF also, do not remove from
        // device list
        if(!mRoutePcmToSpdif) {
            devices = devices & ~AudioSystem::DEVICE_OUT_SPDIF;
        }
        if(!mRoutePcmToHdmi) {
            devices = devices & ~AudioSystem::DEVICE_OUT_AUX_DIGITAL;
        }
        if(mRoutePcmToSpdif) {
            if(!strncmp(mSpdifOutputFormat,"lpcm",sizeof(mSpdifOutputFormat))) {
                *status = mALSADevice->setPlaybackFormat("LPCM",
                                                AudioSystem::DEVICE_OUT_SPDIF);
                if(*status != NO_ERROR)
                    return;
            } else {
                //ToDo: handle DTS;
            }
        }
        if(mRoutePcmToHdmi) {
            if(!strncmp(mHdmiOutputFormat,"lpcm",sizeof(mHdmiOutputFormat))) {
                *status = mALSADevice->setPlaybackFormat("LPCM",
                                          AudioSystem::DEVICE_OUT_AUX_DIGITAL);
                if (*status != NO_ERROR)
                    return;
            } else {
                //ToDo: handle DTS
            }
        }
        snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
        if ((use_case == NULL) || (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                                           strlen(SND_USE_CASE_VERB_INACTIVE)))) {
            *status = openDevice(SND_USE_CASE_VERB_HIFI2, true, devices);
        } else {
            *status = openDevice(SND_USE_CASE_MOD_PLAY_MUSIC2, false, devices);
        }
        if(use_case) {
            free(use_case);
            use_case = NULL;
        }
        if(*status != NO_ERROR) {
            ALOGE("openDevice error return = %d", *status);
            return;
        }
        ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
        mPcmRxHandle = &(*it);
        mBufferSize = mPcmRxHandle->periodSize;
    }
    if (mUseTunnelDecode) {
        createThreadsForTunnelDecode();
        if (format != AUDIO_FORMAT_WMA && format != AUDIO_FORMAT_WMA_PRO)
            *status = openTunnelDevice(mDevices);
        else
            *status = NO_ERROR;
    } else if (mRouteCompreToSpdif && mRouteCompreToHdmi) {
        createThreadsForTunnelDecode();
        devices = AudioSystem::DEVICE_OUT_SPDIF | AudioSystem::DEVICE_OUT_AUX_DIGITAL;
        if(mCaptureFromProxy) {
            devices |= AudioSystem::DEVICE_OUT_PROXY;
        }
        *status = openTunnelDevice(devices);
        if(*status != NO_ERROR)
            return;
        ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
        mCompreRxHandle = &(*it);
    } else if (mRouteCompreToSpdif) {
        createThreadsForTunnelDecode();
        devices = AudioSystem::DEVICE_OUT_SPDIF;
        if(mCaptureFromProxy) {
            devices |= AudioSystem::DEVICE_OUT_PROXY;
        }
        *status = openTunnelDevice(devices);
        if(*status != NO_ERROR)
            return;
        ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
        mCompreRxHandle = &(*it);
    } else if (mRouteCompreToHdmi) {
        createThreadsForTunnelDecode();
        devices = AudioSystem::DEVICE_OUT_AUX_DIGITAL;
        *status = openTunnelDevice(devices);
        if(*status != NO_ERROR)
            return;
    }
    ALOGV("mRouteAudioToA2dp = %d", mRouteAudioToA2dp);
    if (mRouteAudioToA2dp) {
        status_t err = NO_ERROR;
        ALOGD("startA2dpPlayback_l - A2DPDirectOutput");
        err = mParent->startA2dpPlayback_l(AudioHardwareALSA::A2DPDirectOutput);
        *status = err;
    }
    mCurDevice = devices;
}

AudioSessionOutALSA::~AudioSessionOutALSA()
{
    ALOGV("~AudioSessionOutALSA");

    mSkipWrite = true;
    mWriteCv.signal();

    if (mRouteAudioToA2dp) {
        ALOGD("destructor - suspendA2dpPlayback - A2DPDirectOutput");
        status_t err = mParent->suspendA2dpPlayback(AudioHardwareALSA::A2DPDirectOutput);
        if(err) {
            ALOGE("destructor - suspendA2dpPlayback-A2DPDirectOutput return err = %d", err);
        }
        mRouteAudioToA2dp = false;
    }

    //TODO: This might need to be Locked using Parent lock
    reset();
}

status_t AudioSessionOutALSA::setParameters(const String8& keyValuePairs)
{
    Mutex::Autolock autoLock(mLock);
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
    }
    return NO_ERROR;
}

String8 AudioSessionOutALSA::getParameters(const String8& keys)
{
    Mutex::Autolock autoLock(mLock);
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        param.addInt(key, (int)mDevices);
    }

    ALOGV("getParameters() %s", param.toString().string());
    return param.toString();
}

status_t AudioSessionOutALSA::setVolume(float left, float right)
{
    Mutex::Autolock autoLock(mLock);
    float volume;
    status_t status = NO_ERROR;

    volume = (left + right) / 2;
    if (volume < 0.0) {
        ALOGW("AudioSessionOutALSA::setVolume(%f) under 0.0, assuming 0.0\n", volume);
        volume = 0.0;
    } else if (volume > 1.0) {
        ALOGW("AudioSessionOutALSA::setVolume(%f) over 1.0, assuming 1.0\n", volume);
        volume = 1.0;
    }
   mStreamVol = lrint((volume * 0x2000)+0.5);

    ALOGD("Setting stream volume to %d (available range is 0 to 0x2000)\n", mStreamVol);
    ALOGE("ToDo: Implement volume setting for broadcast stream");
    if(mPcmRxHandle) {
        if(!strncmp(mPcmRxHandle->useCase, SND_USE_CASE_VERB_HIFI2,
                                     strlen(SND_USE_CASE_VERB_HIFI2)) ||
                !strncmp(mPcmRxHandle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC2,
                                     strlen(SND_USE_CASE_MOD_PLAY_MUSIC2))) {
            ALOGD("setPCM 1 Volume(%f)\n", volume);
            ALOGD("Setting PCM volume to %d (available range is 0 to 0x2000)\n", mStreamVol);
            status = mPcmRxHandle->module->setPcmVolume(mStreamVol);
        }
        return status;
    }
    else if(mCompreRxHandle) {
        if(!strncmp(mCompreRxHandle->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                                     strlen(SND_USE_CASE_VERB_HIFI_TUNNEL)) ||
                !strncmp(mCompreRxHandle->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL,
                                     strlen(SND_USE_CASE_MOD_PLAY_TUNNEL)) &&
                (mRouteCompreToSpdif == false)) {
            ALOGD("set compressed Volume(%f)\n", volume);
            ALOGD("Setting Compressed volume to %d (available range is 0 to 0x2000)\n", mStreamVol);
            status = mCompreRxHandle->module->setCompressedVolume(mStreamVol);
        }
        return status;
    }
    return INVALID_OPERATION;
}


status_t AudioSessionOutALSA::openTunnelDevice(int devices)
{
    // If audio is to be capture back from proxy device, then route
    // audio to SPDIF and Proxy devices only
    char *use_case;
    status_t status = NO_ERROR;
    if(mCaptureFromProxy) {
        devices = (mDevices & AudioSystem::DEVICE_OUT_SPDIF);
        devices |= AudioSystem::DEVICE_OUT_PROXY;
    }
    mInputBufferSize    = TUNNEL_DECODER_BUFFER_SIZE;
    mInputBufferCount   = TUNNEL_DECODER_BUFFER_COUNT;
    if(mRoutePcmToSpdif) {
        if(!strncmp(mSpdifOutputFormat,"lpcm",sizeof(mSpdifOutputFormat))) {
            status = mALSADevice->setPlaybackFormat("LPCM",
                                              AudioSystem::DEVICE_OUT_SPDIF);
            if(status != NO_ERROR)
                return status;
            if(mALSADevice->get_linearpcm_channel_status(mSampleRate, mChannelStatus)) {
                ALOGE("channel status set error ");
                return -1;
            }
            mALSADevice->setChannelStatus(mChannelStatus);
            mChannelStatusSet = true;
        } else {
            //ToDo: handle DTS;
        }
    } else if(mRouteCompreToSpdif || mRouteDtsToSpdif) {
        status = mALSADevice->setPlaybackFormat("Compr", AudioSystem::DEVICE_OUT_SPDIF);
        if(status != NO_ERROR)
            return status;
    }
    if(mRoutePcmToHdmi) {
        if(!strncmp(mHdmiOutputFormat,"lpcm",sizeof(mHdmiOutputFormat))) {
            status = mALSADevice->setPlaybackFormat("LPCM",
                                                  AudioSystem::DEVICE_OUT_AUX_DIGITAL);
            if (status != NO_ERROR)
               return status;
        }
    } else if (mRouteCompreToHdmi || mRouteDtsToHdmi) {
        status = mALSADevice->setPlaybackFormat("Compr", AudioSystem::DEVICE_OUT_AUX_DIGITAL);
        if (status != NO_ERROR)
           return status;
    }
    snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
    if ((use_case == NULL) || (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                                 strlen(SND_USE_CASE_VERB_INACTIVE)))) {
        status = openDevice(SND_USE_CASE_VERB_HIFI_TUNNEL, true, devices);
    } else {
        status = openDevice(SND_USE_CASE_MOD_PLAY_TUNNEL, false, devices);
    }
    if(use_case) {
        free(use_case);
        use_case = NULL;
    }
    if(status != NO_ERROR) {
        return status;
    }
    ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
    mCompreRxHandle = &(*it);

    //mmap the buffers for playback
    status_t err = mmap_buffer(mCompreRxHandle->handle);
    if(err) {
        ALOGE("MMAP buffer failed - playback err = %d", err);
        return err;
    }
    //prepare the driver for playback
    status = pcm_prepare(mCompreRxHandle->handle);
    if (status) {
        ALOGE("PCM Prepare failed - playback err = %d", err);
        return status;
    }
    bufferAlloc(mCompreRxHandle);
    mBufferSize = mCompreRxHandle->periodSize;
    return status;
}

ssize_t AudioSessionOutALSA::write(const void *buffer, size_t bytes)
{
    int period_size;
    char *use_case;

    Mutex::Autolock autoLock(mLock);
    ALOGV("write:: buffer %p, bytes %d", buffer, bytes);
    if (!mPowerLock) {
        acquire_wake_lock (PARTIAL_WAKE_LOCK, "AudioSessionOutLock");
        mPowerLock = true;
    }
    snd_pcm_sframes_t n;
    size_t            sent = 0;
    status_t          err;
    if (mUseTunnelDecode && mWMAConfigDataSet == false &&
            (mFormat == AUDIO_FORMAT_WMA || mFormat == AUDIO_FORMAT_WMA_PRO)) {
        ALOGV("Configuring the WMA params");
        status_t err = mALSADevice->setWMAParams(mCompreRxHandle,
                                         (int *)buffer, bytes/sizeof(int));
        if (err) {
            ALOGE("WMA param config failed");
            return -1;
        }
        err = openTunnelDevice(mDevices);
        if (err) {
            ALOGE("opening of tunnel device failed");
            if (mObserver)
                mObserver->postEOS(0);
            return -1;
        }
        mWMAConfigDataSet = true;
        return bytes;
    }
    if (mUseTunnelDecode && mCompreRxHandle) {

        writeToCompressedDriver((char *)buffer, bytes);

    } else if(mMS11Decoder != NULL) {
    // 1. Check if MS11 decoder instance is present and if present we need to
    //    preserve the data and supply it to MS 11 decoder.
        // If MS11, the first buffer in AAC format has the AAC config data.

        if(mFormat == AUDIO_FORMAT_AAC || mFormat == AUDIO_FORMAT_HE_AAC_V1 ||
           mFormat == AUDIO_FORMAT_AAC_ADIF || mFormat == AUDIO_FORMAT_HE_AAC_V2) {
            if(mAacConfigDataSet == false) {
                if(mMS11Decoder->setAACConfig((unsigned char *)buffer, bytes) == true)
                    mAacConfigDataSet = true;
                return bytes;
            }
        }
        if(bytes == 0) {
            if(mFormat == AUDIO_FORMAT_AAC_ADIF)
                mBitstreamSM->appendSilenceToBitstreamInternalBuffer(mMinBytesReqToDecode,0);
            else
                return bytes;
        }

        bool    continueDecode=false;
        size_t  bytesConsumedInDecode = 0;
        size_t  copyBytesMS11 = 0;
        char    *bufPtr;
        uint32_t outSampleRate=mSampleRate,outChannels=mChannels;
        mBitstreamSM->copyBitsreamToInternalBuffer((char *)buffer, bytes);

        do
        {
            // flag indicating if the decoding has to be continued so as to
            // get all the output held up with MS11. Examples, AAC can have an
            // output frame of 4096 bytes. While the output of MS11 is 1536, the
            // decoder has to be called more than twice to get the reside samples.
            continueDecode=false;
            if(mBitstreamSM->sufficientBitstreamToDecode(mMinBytesReqToDecode) == true)
            {
                bufPtr = mBitstreamSM->getInputBufferPtr();
                copyBytesMS11 = mBitstreamSM->bitStreamBufSize();

                mMS11Decoder->copyBitstreamToMS11InpBuf(bufPtr,copyBytesMS11);
                bytesConsumedInDecode = mMS11Decoder->streamDecode(&outSampleRate,&outChannels);
                mBitstreamSM->copyResidueBitstreamToStart(bytesConsumedInDecode);
            }
            // Close and open the driver again if the output sample rate change is observed
            // in decode.
            if( (mSampleRate != outSampleRate) || (mChannels != outChannels)) {
                ALOGD("change in sample rate/channel mode - sampleRate:%d,channel mode: %d",
                          outSampleRate, outChannels);
                uint32_t devices = mDevices;
                mSampleRate = outSampleRate;
                mChannels = outChannels;
                if(mPcmRxHandle && mRoutePcmAudio) {
                    status_t status = closeDevice(mPcmRxHandle);
                    if(!mRoutePcmToSpdif) {
                        devices = devices & ~AudioSystem::DEVICE_OUT_SPDIF;
                    }
                    if(!mRoutePcmToHdmi) {
                        devices = devices & ~AudioSystem::DEVICE_OUT_AUX_DIGITAL;
                    }
                    if(mRoutePcmToSpdif) {
                        if(!strncmp(mSpdifOutputFormat, "lpcm",
                               sizeof(mSpdifOutputFormat))) {
                            status = mALSADevice->setPlaybackFormat("LPCM",
                                           AudioSystem::DEVICE_OUT_SPDIF);
                            if(status != NO_ERROR)
                                return status;
                        } else {
                            //ToDo: handle DTS;
                        }
                    }
                    if(mRoutePcmToHdmi) {
                        if(!strncmp(mHdmiOutputFormat, "lpcm",
                                       sizeof(mHdmiOutputFormat))) {
                            status = mALSADevice->setPlaybackFormat("LPCM",
                                          AudioSystem::DEVICE_OUT_AUX_DIGITAL);
                            if (status != NO_ERROR)
                                return status;
                        } else {
                            //ToDo: handle DTS
                        }
                    }
                    snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
                    if ((use_case == NULL) || (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                                                 strlen(SND_USE_CASE_VERB_INACTIVE)))) {
                        status = openDevice(SND_USE_CASE_VERB_HIFI2, true, devices);
                    } else {
                        status = openDevice(SND_USE_CASE_MOD_PLAY_MUSIC2, false, devices);
                    }
                    if(use_case) {
                        free(use_case);
                        use_case = NULL;
                    }
                    if(status != NO_ERROR) {
                        ALOGE("Error opening the driver");
                        break;
                    }
                    ALSAHandleList::iterator it = mParent->mDeviceList.end(); it--;
                    mPcmRxHandle = &(*it);
                    mBufferSize = mPcmRxHandle->periodSize;
                }
                mChannelStatusSet = false;
            }

            // copy the output of MS11 to HAL internal buffers for PCM and SPDIF
            if(mRoutePcmAudio) {
                bufPtr=mBitstreamSM->getOutputBufferWritePtr(PCM_2CH_OUT);
                copyBytesMS11 = mMS11Decoder->copyOutputFromMS11Buf(PCM_2CH_OUT,bufPtr);
                // Note: Set the output Buffer to start for for changein sample rate and channel
                // This has to be done.
                mBitstreamSM->setOutputBufferWritePtr(PCM_2CH_OUT,copyBytesMS11);
                // If output samples size is zero, donot continue and wait for next
                // write for decode
            }
            if(mRouteCompreToSpdif || mRouteCompreToHdmi) {
                bufPtr=mBitstreamSM->getOutputBufferWritePtr(SPDIF_OUT);
                copyBytesMS11 = mMS11Decoder->copyOutputFromMS11Buf(SPDIF_OUT,bufPtr);
                mBitstreamSM->setOutputBufferWritePtr(SPDIF_OUT,copyBytesMS11);
            }
            if(copyBytesMS11)
                continueDecode = true;

            // Set the channel status after first frame decode/transcode and for change
            // in sample rate or channel mode as we close and open the device again
            if(mChannelStatusSet == false) {
                if(mRoutePcmToSpdif) {
                    if(!strncmp(mSpdifOutputFormat,"lpcm",
                                                sizeof(mSpdifOutputFormat))) {
                        if(mALSADevice->get_linearpcm_channel_status(mSampleRate,
                                                        mChannelStatus)) {
                            ALOGE("channel status set error ");
                            return -1;
                        }
                        mALSADevice->setChannelStatus(mChannelStatus);
                    }
                } else if(mRouteCompreToSpdif) {
                    if(mALSADevice->get_compressed_channel_status(
                                   mBitstreamSM->getOutputBufferPtr(SPDIF_OUT),
                                                                 copyBytesMS11,
                                        mChannelStatus,AUDIO_PARSER_CODEC_AC3)) {
                        ALOGE("channel status set error ");
                        return -1;
                    }
                    mALSADevice->setChannelStatus(mChannelStatus);
                }
                mChannelStatusSet = true;
            }

            // Write the output of MS11 to appropriate drivers
            if(mPcmRxHandle && mRoutePcmAudio) {
                period_size = mPcmRxHandle->periodSize;
                while(mBitstreamSM->sufficientSamplesToRender(PCM_2CH_OUT,period_size) == true) {
                    n = pcm_write(mPcmRxHandle->handle,
                             mBitstreamSM->getOutputBufferPtr(PCM_2CH_OUT),
                              period_size);
                    ALOGD("mPcmRxHandle - pcm_write returned with %d", n);
                    if (n == -EBADFD) {
                        // Somehow the stream is in a bad state. The driver probably
                        // has a bug and snd_pcm_recover() doesn't seem to handle this.
                        mPcmRxHandle->module->open(mPcmRxHandle);
                    } else if (n < 0) {
                        // Recovery is part of pcm_write. TODO split is later.
                        ALOGE("mPcmRxHandle - pcm_write returned n < 0");
                        return static_cast<ssize_t>(n);
                    } else {
                        mFrameCount++;
                        sent += static_cast<ssize_t>((period_size));
                        mBitstreamSM->copyResidueOutputToStart(PCM_2CH_OUT,period_size);
                    }
                }
            }
            if(mCompreRxHandle && (mRouteCompreToSpdif||mRouteCompreToHdmi)) {
                period_size = mCompreRxHandle->periodSize;
                while(mBitstreamSM->sufficientSamplesToRender(SPDIF_OUT,period_size) == true) {
                    n = writeToCompressedDriver(
                                   mBitstreamSM->getOutputBufferPtr(SPDIF_OUT),
                                   period_size);
                    ALOGD("mCompreRxHandle - pcm_write returned with %d", n);
                    if (n == -EBADFD) {
                        // Somehow the stream is in a bad state. The driver probably
                        // has a bug and snd_pcm_recover() doesn't seem to handle this.
                        mCompreRxHandle->module->open(mCompreRxHandle);
                    } else if (n < 0) {
                        // Recovery is part of pcm_write. TODO split is later.
                        ALOGE("mCompreRxHandle - pcm_write returned n < 0");
                        return static_cast<ssize_t>(n);
                    } else {
                        mBitstreamSM->copyResidueOutputToStart(SPDIF_OUT,period_size);
                    }
                }
            }
	} while( (continueDecode == true) && (mBitstreamSM->sufficientBitstreamToDecode(mMinBytesReqToDecode) == true));
    } else {
        // 2. Get the output data from Software decoder and write to PCM driver
        if(bytes == 0)
            return bytes;
        if(mPcmRxHandle && mRoutePcmAudio) {
            int write_pending = bytes;
            period_size = mPcmRxHandle->periodSize;

            mBitstreamSM->copyBitsreamToInternalBuffer((char *)buffer, bytes);

            while(mPcmRxHandle->handle &&
                  (mBitstreamSM->sufficientBitstreamToDecode(period_size)
                                     == true)) {
                ALOGV("Calling pcm_write");
                n = pcm_write(mPcmRxHandle->handle,
                         (char *)buffer + sent,
                          period_size);
                ALOGD("pcm_write returned with %d", n);
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
                    mFrameCount++;
                    sent += static_cast<ssize_t>((period_size));
                    mBitstreamSM->copyResidueBitstreamToStart(period_size);
                }
            }
        }
    }
    return sent;
}

int32_t AudioSessionOutALSA::writeToCompressedDriver(char *buffer, int bytes)
{
        int n;
        ALOGV("Signal Event thread\n");
        mEventCv.signal();
        mInputMemRequestMutex.lock();
        ALOGV("write Empty Queue size() = %d,\
        Filled Queue size() = %d ",\
        mInputMemEmptyQueue.size(),\
        mInputMemFilledQueue.size());
        if (mInputMemEmptyQueue.empty()) {
            ALOGV("Write: waiting on mWriteCv");
            mLock.unlock();
            mWriteCv.wait(mInputMemRequestMutex);
            mLock.lock();
            if (mSkipWrite) {
                ALOGV("Write: Flushing the previous write buffer");
                mSkipWrite = false;
                mInputMemRequestMutex.unlock();
                return 0;
            }
            ALOGV("Write: received a signal to wake up");
        }

        List<BuffersAllocated>::iterator it = mInputMemEmptyQueue.begin();
        BuffersAllocated buf = *it;
        mInputMemEmptyQueue.erase(it);
        mInputMemRequestMutex.unlock();
        memcpy(buf.memBuf, buffer, bytes);
        buf.bytesToWrite = bytes;
        mInputMemResponseMutex.lock();
        mInputMemFilledQueue.push_back(buf);
        mInputMemResponseMutex.unlock();

        ALOGV("PCM write start");
        pcm * local_handle = (struct pcm *)mCompreRxHandle->handle;
        // Set the channel status after first frame decode/transcode and for change
        // in sample rate or channel mode as we close and open the device again
        if(mChannelStatusSet == false) {
            if(mRoutePcmToSpdif) {
                if (mALSADevice->get_linearpcm_channel_status(mSampleRate,
                                        mChannelStatus)) {
                    ALOGE("channel status set error ");
                    return -1;
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            } else if(mRouteDtsToSpdif) {
                if (mALSADevice->get_compressed_channel_status(
                        buffer, bytes, mChannelStatus,
                        AUDIO_PARSER_CODEC_DTS)) {
                    ALOGE("channel status set error ");
                    return -1;
                }
                mALSADevice->setChannelStatus(mChannelStatus);
            }
            mChannelStatusSet = true;
        }
        n = pcm_write(local_handle, buf.memBuf, local_handle->period_size);
        if (bytes < local_handle->period_size) {
        ALOGD("Last buffer case");
        uint64_t writeValue = SIGNAL_EVENT_THREAD;
        sys_write::lib_write(mEfd, &writeValue, sizeof(uint64_t));

            //TODO : Is this code reqd - start seems to fail?
            if (ioctl(local_handle->fd, SNDRV_PCM_IOCTL_START) < 0)
                ALOGE("AUDIO Start failed");
            else
                local_handle->start = 1;
        }
        ALOGV("PCM write complete");
        return n;
}

void AudioSessionOutALSA::bufferAlloc(alsa_handle_t *handle) {
    void  *mem_buf = NULL;
    int i = 0;

    struct pcm * local_handle = (struct pcm *)handle->handle;
    int32_t nSize = local_handle->period_size;
    ALOGV("number of input buffers = %d", mInputBufferCount);
    ALOGV("memBufferAlloc calling with required size %d", nSize);
    for (i = 0; i < mInputBufferCount; i++) {
        mem_buf = (int32_t *)local_handle->addr + (nSize * i/sizeof(int));
        BuffersAllocated buf(mem_buf, nSize);
        memset(buf.memBuf, 0x0, nSize);
        mInputMemEmptyQueue.push_back(buf);
        mInputBufPool.push_back(buf);
        ALOGD("The MEM that is allocated - buffer is %x",\
            (unsigned int)mem_buf);
    }
}

void AudioSessionOutALSA::bufferDeAlloc() {
    //Remove all the buffers from request queue
    while (!mInputBufPool.empty()) {
        List<BuffersAllocated>::iterator it = mInputBufPool.begin();
        BuffersAllocated &memBuffer = *it;
        ALOGD("Removing input buffer from Buffer Pool ");
        mInputBufPool.erase(it);
   }
}

void AudioSessionOutALSA::requestAndWaitForEventThreadExit() {

    if (!mEventThreadAlive)
        return;
    mKillEventThread = true;
    if(mEfd != -1) {
        ALOGE("Writing to mEfd %d",mEfd);
        uint64_t writeValue = KILL_EVENT_THREAD;
        sys_write::lib_write(mEfd, &writeValue, sizeof(uint64_t));
    }
    mEventCv.signal();
    pthread_join(mEventThread,NULL);
    ALOGD("event thread killed");
}

void * AudioSessionOutALSA::eventThreadWrapper(void *me) {
    static_cast<AudioSessionOutALSA *>(me)->eventThreadEntry();
    return NULL;
}

void  AudioSessionOutALSA::eventThreadEntry() {

    int rc = 0;
    int err_poll = 0;
    int avail = 0;
    int i = 0;
    struct pollfd pfd[NUM_FDS];
    struct pcm * local_handle = NULL;
    mEventMutex.lock();
    int timeout = -1;
    pid_t tid  = gettid();
    androidSetThreadPriority(tid, ANDROID_PRIORITY_AUDIO);
    prctl(PR_SET_NAME, (unsigned long)"HAL Audio EventThread", 0, 0, 0);

    ALOGV("eventThreadEntry wait for signal \n");
    mEventCv.wait(mEventMutex);
    ALOGV("eventThreadEntry ready to work \n");
    mEventMutex.unlock();

    ALOGV("Allocating poll fd");
    if(!mKillEventThread) {
        ALOGV("Allocating poll fd");
        local_handle = (struct pcm *)mCompreRxHandle->handle;
        pfd[0].fd = local_handle->timer_fd;
        pfd[0].events = (POLLIN | POLLERR | POLLNVAL);
        ALOGV("Allocated poll fd");
        mEfd = eventfd(0,0);
        pfd[1].fd = mEfd;
        pfd[1].events = (POLLIN | POLLERR | POLLNVAL);
    }
    while(!mKillEventThread && ((err_poll = poll(pfd, NUM_FDS, timeout)) >=0)) {
        ALOGV("pfd[0].revents =%d ", pfd[0].revents);
        ALOGV("pfd[1].revents =%d ", pfd[1].revents);
        if (err_poll == EINTR)
            ALOGE("Timer is intrrupted");
        if (pfd[1].revents & POLLIN) {
            uint64_t u;
            read(mEfd, &u, sizeof(uint64_t));
            ALOGV("POLLIN event occured on the event fd, value written to %llu",\
                    (unsigned long long)u);
            pfd[1].revents = 0;
            if (u == SIGNAL_EVENT_THREAD) {
                ALOGV("### Setting mReachedExtractorEOS");
                mReachedExtractorEOS = true;
                continue;
            }
        }
        if ((pfd[1].revents & POLLERR) || (pfd[1].revents & POLLNVAL)) {
            ALOGE("POLLERR or INVALID POLL");
        }
        struct snd_timer_tread rbuf[4];
        read(local_handle->timer_fd, rbuf, sizeof(struct snd_timer_tread) * 4 );
        if((pfd[0].revents & POLLERR) || (pfd[0].revents & POLLNVAL))
            continue;

        if (pfd[0].revents & POLLIN && !mKillEventThread) {
            pfd[0].revents = 0;
            if (mTunnelPaused)
                continue;
            ALOGV("After an event occurs");
            {
                if (mInputMemFilledQueue.empty()) {
                    ALOGV("Filled queue is empty");
                    continue;
                }
                mInputMemResponseMutex.lock();
                BuffersAllocated buf = *(mInputMemFilledQueue.begin());
                mInputMemFilledQueue.erase(mInputMemFilledQueue.begin());
                ALOGV("mInputMemFilledQueue %d", mInputMemFilledQueue.size());
                if (mInputMemFilledQueue.empty() && mReachedExtractorEOS) {
                    ALOGV("Posting the EOS to the MPQ player");
                    //post the EOS To MPQ player
                    if (mObserver)
                        mObserver->postEOS(0);
                }
                mInputMemResponseMutex.unlock();

                mInputMemRequestMutex.lock();

                mInputMemEmptyQueue.push_back(buf);

                mInputMemRequestMutex.unlock();
                mWriteCv.signal();
            }
        }
    }
    mEventThreadAlive = false;
    if (mEfd != -1) {
        close(mEfd);
        mEfd = -1;
    }

    ALOGD("Event Thread is dying.");
    return;

}

void AudioSessionOutALSA::createThreadsForTunnelDecode() {
    mKillEventThread = false;
    mEventThreadAlive = true;
    ALOGD("Creating Event Thread");
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_create(&mEventThread, &attr, eventThreadWrapper, this);
    ALOGD("Event Thread created");
}

status_t AudioSessionOutALSA::start()
{
    Mutex::Autolock autoLock(mLock);
    status_t err = NO_ERROR;
    
    if(mPaused) {
        if (mRouteAudioToA2dp) {
            ALOGD("startA2dpPlayback - resume - A2DPDirectOutput");
            err = mParent->startA2dpPlayback(AudioHardwareALSA::A2DPDirectOutput);
            if(err) {
                ALOGE("startA2dpPlayback from resume return error = %d", err);
                return err;
            }
        }
        err = resume_l();
        return err;
    }
    
    // 1. Set the absolute time stamp
    // ToDo: We need the ioctl from driver to set the time stamp

    // 2. Signal the driver to start rendering data
    if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_START)) {
        ALOGE("start:SNDRV_PCM_IOCTL_START failed\n");
        err = UNKNOWN_ERROR;
    }
    return err;
}
status_t AudioSessionOutALSA::pause()
{
    Mutex::Autolock autoLock(mLock);
    status_t err = NO_ERROR;
    err = pause_l();

    if(err) {
        ALOGE("pause returned error");
        return err;
    }
    if (mRouteAudioToA2dp) {
        ALOGD("Pause - suspendA2dpPlayback - A2DPDirectOutput");
        err = mParent->suspendA2dpPlayback(AudioHardwareALSA::A2DPDirectOutput);
        if(err != NO_ERROR) {
            ALOGE("Suspend Proxy from Pause returned error = %d",err);
            return err;
        }
    }
    return err;
}
status_t AudioSessionOutALSA::pause_l()
{
    ALOGE("AudioSessionOutALSA::pause");
    mPaused = true;
    if(mPcmRxHandle) {
        if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
            ALOGE("PAUSE failed for use case %s", mPcmRxHandle->useCase);
        }
    }
    if(mCompreRxHandle) {
        if (ioctl(mCompreRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
            ALOGE("PAUSE failed on use case %s", mCompreRxHandle->useCase);
        }
        mTunnelPaused = true;
    }
    return NO_ERROR;
}

status_t AudioSessionOutALSA::drainTunnel()
{
    status_t err = OK;
    if (!mCompreRxHandle)
        return -1;
    mCompreRxHandle->handle->start = 0;
    err = pcm_prepare(mCompreRxHandle->handle);
    if(err != OK) {
        ALOGE("pcm_prepare -seek = %d",err);
        //Posting EOS
        if (mObserver)
            mObserver->postEOS(0);
    }
    ALOGV("drainTunnel Empty Queue size() = %d,"
       " Filled Queue size() = %d ",\
        mInputMemEmptyQueue.size(),\
        mInputMemFilledQueue.size());
    ALOGV("Reset, drain and prepare completed");
    mCompreRxHandle->handle->sync_ptr->flags =
        SNDRV_PCM_SYNC_PTR_APPL | SNDRV_PCM_SYNC_PTR_AVAIL_MIN;
    sync_ptr(mCompreRxHandle->handle);
    ALOGV("appl_ptr= %d",\
        (int)mCompreRxHandle->handle->sync_ptr->c.control.appl_ptr);
    return err;
}

status_t AudioSessionOutALSA::flush()
{
    Mutex::Autolock autoLock(mLock);
    ALOGD("AudioSessionOutALSA::flush E");
    int err = 0;
    if (mCompreRxHandle) {
        struct pcm * local_handle = mCompreRxHandle->handle;
        ALOGV("Paused case, %d",mTunnelPaused);

        mInputMemResponseMutex.lock();
        mInputMemRequestMutex.lock();
        mInputMemFilledQueue.clear();
        mInputMemEmptyQueue.clear();
        List<BuffersAllocated>::iterator it = mInputBufPool.begin();
        for (;it!=mInputBufPool.end();++it) {
            memset((*it).memBuf, 0x0, (*it).memBufsize);
            mInputMemEmptyQueue.push_back(*it);
        }

        mInputMemRequestMutex.unlock();
        mInputMemResponseMutex.unlock();
        ALOGV("Transferred all the buffers from response queue to\
            request queue to handle seek");
        if (!mTunnelPaused) {
            if ((err = ioctl(local_handle->fd, SNDRV_PCM_IOCTL_PAUSE,1)) < 0) {
                ALOGE("Audio Pause failed");
                return err;
            }
            mReachedExtractorEOS = false;
            if ((err = drainTunnel()) != OK)
                return err;
        } else {
            mTunnelSeeking = true;
        }
        mSkipWrite = true;
        mWriteCv.signal();
    }
    /*if(mPcmRxHandle) {
        if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,1) < 0) {
            ALOGE("flush(): Audio Pause failed");
        }
        mPcmRxHandle->handle->start = 0;
        pcm_prepare(mPcmRxHandle->handle);
        ALOGV("flush(): Reset, drain and prepare completed");
        mPcmRxHandle->handle->sync_ptr->flags = (SNDRV_PCM_SYNC_PTR_APPL |
                                                 SNDRV_PCM_SYNC_PTR_AVAIL_MIN);
        sync_ptr(mPcmRxHandle->handle);
    }*/
    mFrameCount = 0;
    if(mOpenMS11Decoder == true)
        mBitstreamSM->resetBitstreamPtr();
    ALOGD("AudioSessionOutALSA::flush X");
    return NO_ERROR;
}

status_t AudioSessionOutALSA::resume_l()
{
    if(mPcmRxHandle) {
        if (ioctl(mPcmRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,0) < 0) {
            ALOGE("RESUME failed for use case %s", mPcmRxHandle->useCase);
        }
    }
    if(mCompreRxHandle) {
        if (mTunnelSeeking) {
            drainTunnel();
            mTunnelSeeking = false;
        } else if (ioctl(mCompreRxHandle->handle->fd, SNDRV_PCM_IOCTL_PAUSE,0) < 0) {
            ALOGE("RESUME failed on use case %s", mCompreRxHandle->useCase);
        }
        mTunnelPaused = false;
    }
    mPaused = false;
    return NO_ERROR;
}

status_t AudioSessionOutALSA::stop()
{
    Mutex::Autolock autoLock(mLock);
    ALOGV("AudioSessionOutALSA- stop");
    // close all the existing PCM devices
    // ToDo: How to make sure all the data is rendered before closing
    mSkipWrite = true;
    mWriteCv.signal();

    if (mRouteAudioToA2dp) {
        ALOGD("stop - suspendA2dpPlayback - A2DPDirectOutput");
        status_t err = mParent->suspendA2dpPlayback(AudioHardwareALSA::A2DPDirectOutput);
        if(err) {
            ALOGE("stop-suspendA2dpPlayback-A2DPDirectOutput return err = %d", err);
            return err;
        }
        mRouteAudioToA2dp = false;
    }

    //TODO: This might need to be Locked using Parent lock
    reset();

    return NO_ERROR;
}

status_t AudioSessionOutALSA::dump(int fd, const Vector<String16>& args)
{
    return NO_ERROR;
}

status_t AudioSessionOutALSA::standby()
{
    Mutex::Autolock autoLock(mLock);
    ALOGD("standby");
    if (mRouteAudioToA2dp) {
         ALOGD("standby - suspendA2dpPlayback - A2DPDirectOutput");
         status_t err = mParent->suspendA2dpPlayback(AudioHardwareALSA::A2DPDirectOutput);
         if(err) {
             ALOGE("standby-suspendA2dpPlayback-A2DPDirectOutput return er = %d", err);
         }
    }

    if(mPcmRxHandle) {
        mPcmRxHandle->module->standby(mPcmRxHandle);
    }

    if (mPowerLock) {
        release_wake_lock ("AudioOutLock");
        mPowerLock = false;
    }

    mFrameCount = 0;
    if(mOpenMS11Decoder == true)
        mBitstreamSM->resetBitstreamPtr();
    return NO_ERROR;
}

#define USEC_TO_MSEC(x) ((x + 999) / 1000)

uint32_t AudioSessionOutALSA::latency() const
{
    // Android wants latency in milliseconds.
    return USEC_TO_MSEC (mPcmRxHandle->latency);
}

status_t AudioSessionOutALSA::setObserver(void *observer)
{
    ALOGV("Registering the callback \n");
    mObserver = reinterpret_cast<AudioEventObserver *>(observer);
    return NO_ERROR;
}

status_t AudioSessionOutALSA::getNextWriteTimestamp(int64_t *timeStamp)
{
    ALOGV("getTimeStamp \n");

    struct snd_compr_tstamp tstamp;
    if (!timeStamp)
        return -1;

    *timeStamp = -1;
    Mutex::Autolock autoLock(mLock);
    if (mCompreRxHandle && mUseTunnelDecode) {
        tstamp.timestamp = -1;
        if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_TSTAMP, &tstamp)){
            ALOGE("Failed SNDRV_COMPRESS_TSTAMP\n");
            return -1;
        } else {
            ALOGV("Timestamp returned = %lld\n", tstamp.timestamp);
            *timeStamp = tstamp.timestamp;
            return NO_ERROR;
        }
    } else if(mMS11Decoder) {
        if(mPcmRxHandle) {
            *timeStamp = -(uint64_t)(latency()*1000) + (((uint64_t)(((int64_t)(
                        (mFrameCount * mPcmRxHandle->periodSize)/ (2*mChannels)))
                     * 1000000)) / mSampleRate);
        } else if(mCompreRxHandle){
            if (ioctl(mCompreRxHandle->handle->fd, SNDRV_COMPRESS_TSTAMP,
                      &tstamp)) {
                ALOGE("Failed SNDRV_COMPRESS_TSTAMP\n");
                return -1;
            } else {
                *timeStamp = tstamp.timestamp;
            }
        } else {
            *timeStamp = 0;
        }
        ALOGV("Timestamp returned = %lld\n",*timeStamp);
    } else {
        int bitFormat = audio_bytes_per_sample((audio_format_t)mFormat);
        if(mPcmRxHandle && mSampleRate && mChannels && bitFormat)
            *timeStamp = -(uint64_t)(latency()*1000) + (((uint64_t)(((int64_t)(
                    (mFrameCount * mPcmRxHandle->periodSize)/ (mChannels*(bitFormat))))
                     * 1000000)) / mSampleRate);
        else
            *timeStamp = 0;
        ALOGV("Timestamp returned = %lld\n",*timeStamp);
    }
    return NO_ERROR;
}

// return the number of audio frames written by the audio dsp to DAC since
// the output has exited standby
status_t AudioSessionOutALSA::getRenderPosition(uint32_t *dspFrames)
{
    Mutex::Autolock autoLock(mLock);
    *dspFrames = mFrameCount;
    return NO_ERROR;
}

status_t AudioSessionOutALSA::openDevice(char *useCase, bool bIsUseCase, int devices)
{
    alsa_handle_t alsa_handle;
    status_t status = NO_ERROR;
    ALOGD("openDevice: E");
    alsa_handle.module      = mALSADevice;
    alsa_handle.bufferSize  = mInputBufferSize;
    alsa_handle.devices     = devices;
    alsa_handle.activeDevice= devices;
    alsa_handle.handle      = 0;
    alsa_handle.format      = (mFormat == AUDIO_FORMAT_PCM_16_BIT ? SNDRV_PCM_FORMAT_S16_LE : mFormat);
    alsa_handle.channels    = mChannels;
    alsa_handle.sampleRate  = mSampleRate;
    alsa_handle.mode        = mParent->mode();
    alsa_handle.latency     = PLAYBACK_LATENCY;
    alsa_handle.rxHandle    = 0;
    alsa_handle.ucMgr       = mUcMgr;
    strlcpy(alsa_handle.useCase, useCase, sizeof(alsa_handle.useCase));

    mALSADevice->setUseCase(&alsa_handle, bIsUseCase);
    status = mALSADevice->open(&alsa_handle);
    if(status != NO_ERROR) {
        ALOGE("Could not open the ALSA device for use case %s", alsa_handle.useCase);
        mALSADevice->close(&alsa_handle);
    } else{
        mParent->mDeviceList.push_back(alsa_handle);
    }
    ALOGD("openDevice: X");
    return status;
}

status_t AudioSessionOutALSA::closeDevice(alsa_handle_t *pHandle)
{
    status_t status = NO_ERROR;
    ALOGV("closeDevice: useCase %s", pHandle->useCase);
    if(pHandle) {
        status = mALSADevice->close(pHandle);
    }
    return status;
}

status_t AudioSessionOutALSA::doRouting(int devices)
{
    status_t status = NO_ERROR;
    char *use_case;
    Mutex::Autolock autoLock(mParent->mLock);

    ALOGV("doRouting: devices 0x%x, mDevices = 0x%x", devices,mDevices);
    if(devices & AudioSystem::DEVICE_OUT_ALL_A2DP) {
        ALOGV("doRouting - Capture from Proxy");
        devices &= ~AudioSystem::DEVICE_OUT_ALL_A2DP;
        devices &= ~AudioSystem::DEVICE_OUT_SPDIF;
        devices |=  AudioSystem::DEVICE_OUT_PROXY;
        if(devices != mDevices)
            pause_l();
        mRouteAudioToA2dp = true;

    } else if(!(devices & AudioSystem::DEVICE_OUT_ALL_A2DP) && mRouteAudioToA2dp){
        mRouteAudioToA2dp = false;
        devices &= ~AudioSystem::DEVICE_OUT_PROXY;
        ALOGD("doRouting-stopA2dpPlayback_l-A2DPDirectOutput disable");
        status = mParent->stopA2dpPlayback_l(AudioHardwareALSA::A2DPDirectOutput);
    }
    if(devices == mDevices) {
        ALOGW("Return same device ");
        return status;
    }
    mDevices = devices;

    if(mPcmRxHandle) {
        mALSADevice->switchDeviceUseCase(mPcmRxHandle, devices, mParent->mode());
    }
    if(mUseTunnelDecode &&  mCompreRxHandle) {
        mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices, mParent->mode());

    } else if(mRouteCompreToSpdif && !(devices & AudioSystem::DEVICE_OUT_SPDIF)) {
        mRouteCompreToSpdif = false;
        status = closeDevice(mCompreRxHandle);
    } else if(!mUseTunnelDecode && !mRouteCompreToSpdif &&
                                    (devices & AudioSystem::DEVICE_OUT_SPDIF)) {
        snd_use_case_get(mUcMgr, "_verb", (const char **)&use_case);
        if ((use_case == NULL) || (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                                   strlen(SND_USE_CASE_VERB_INACTIVE)))) {
            status = openDevice(SND_USE_CASE_VERB_HIFI_TUNNEL, true,
                                                   AudioSystem::DEVICE_OUT_SPDIF);
        } else {
            status = openDevice(SND_USE_CASE_MOD_PLAY_TUNNEL, false,
                                                   AudioSystem::DEVICE_OUT_SPDIF);
        }
        if(use_case != NULL) {
           free(use_case);
           use_case = NULL;
        }
        /*if(status == NO_ERROR) {
            mRouteCompreToSpdif = true;
        }*/
    }
    if(mRouteAudioToA2dp ) {
        ALOGD("doRouting-startA2dpPlayback_l-A2DPDirectOutput-enable");
        status = mParent->startA2dpPlayback_l(AudioHardwareALSA::A2DPDirectOutput);
        if(status) {
            ALOGW("startA2dpPlayback_l for direct output failed err = %d", status);
            status_t err = mParent->stopA2dpPlayback_l(AudioHardwareALSA::A2DPDirectOutput);
            if(err) {
                ALOGW("stop A2dp playback for hardware output failed = %d", err);
            }
            mRouteAudioToA2dp = false;
            mDevices = mCurDevice;
            if(mPcmRxHandle) {
                mALSADevice->switchDeviceUseCase(mPcmRxHandle, devices, mParent->mode());
            }
            if(mUseTunnelDecode &&  mCompreRxHandle) {
               mALSADevice->switchDeviceUseCase(mCompreRxHandle, devices, mParent->mode());
            }
        }
        resume_l();
    }
    if(status)
        mCurDevice = mDevices;
    ALOGD("doRouting status = %d ",status);
    return status;
}

void AudioSessionOutALSA::updateOutputFormat()
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

void AudioSessionOutALSA::updateRoutingFlags()
{
    if(mDevices & AudioSystem::DEVICE_OUT_PROXY)
        mCaptureFromProxy = true;

    if(!strncmp(mSpdifOutputFormat,"lpcm",sizeof(mSpdifOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_SPDIF)
            mRoutePcmToSpdif = true;
    }
    if(!strncmp(mHdmiOutputFormat,"lpcm",sizeof(mHdmiOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            mRoutePcmToHdmi = true;
    }
    if(!strncmp(mSpdifOutputFormat,"ac3",sizeof(mSpdifOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_SPDIF)
            if(mSampleRate > 24000 && mOpenMS11Decoder)
                mRouteCompreToSpdif = true;
            else {
                strlcpy(mSpdifOutputFormat,"lpcm",
                    sizeof(mSpdifOutputFormat));
                mRoutePcmToSpdif = true;
            }
    }
    if(!strncmp(mHdmiOutputFormat,"ac3",sizeof(mHdmiOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            if(mSampleRate > 24000 && mOpenMS11Decoder)
                mRouteCompreToHdmi = true;
            else {
                strlcpy(mHdmiOutputFormat,"lpcm",
                        sizeof(mHdmiOutputFormat));
                mRoutePcmToHdmi = true;
            }
    }
    if(!strncmp(mSpdifOutputFormat,"dts",sizeof(mSpdifOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_SPDIF)
            if(mFormat != AUDIO_FORMAT_PCM_16_BIT && mUseTunnelDecode == true)
                mRouteDtsToSpdif = true;
            else {
                strlcpy(mSpdifOutputFormat,"lpcm",
                        sizeof(mSpdifOutputFormat));
                mRoutePcmToSpdif = true;
            }
    }
    if(!strncmp(mHdmiOutputFormat,"dts",sizeof(mHdmiOutputFormat))) {
        if(mDevices & AudioSystem::DEVICE_OUT_AUX_DIGITAL)
            if(mFormat != AUDIO_FORMAT_PCM_16_BIT && mUseTunnelDecode == true)
                mRouteDtsToHdmi = true;
            else {
                strlcpy(mHdmiOutputFormat,"lpcm",
                                      sizeof(mHdmiOutputFormat));
                mRoutePcmToHdmi = true;
            }
    }

    if( mRoutePcmToSpdif || mRoutePcmToHdmi ||
             ((mDevices & ~AudioSystem::DEVICE_OUT_SPDIF) &&
                  (mDevices & ~AudioSystem::DEVICE_OUT_AUX_DIGITAL))) {
        mRoutePcmAudio = true;
    }

    if(mUseTunnelDecode)
        mRoutePcmAudio = false;

}
uint32_t AudioSessionOutALSA::channelMapToChannels(uint32_t channelMap) {
    uint32_t channels = 0;
    while(channelMap) {
        channels++;
        channelMap = channelMap & (channelMap - 1);
    }
    return channels;
}

void AudioSessionOutALSA::reset() {

    if(mPcmRxHandle) {
        closeDevice(mPcmRxHandle);
        mPcmRxHandle = NULL;
    }

    if (mUseTunnelDecode || mRouteCompreToHdmi || mRouteCompreToSpdif)
        requestAndWaitForEventThreadExit();

    if(mCompreRxHandle) {
        closeDevice(mCompreRxHandle);
        mCompreRxHandle = NULL;
    }

    if(mPcmTxHandle) {
        closeDevice(mPcmTxHandle);
        mPcmTxHandle = NULL;
    }

    if(mMS11Decoder) {
        delete mMS11Decoder;
        mMS11Decoder = NULL;
    }
    if(mBitstreamSM) {
        delete mBitstreamSM;
        mBitstreamSM = NULL;
    }

    for(ALSAHandleList::iterator it = mParent->mDeviceList.begin();
            it != mParent->mDeviceList.end(); ++it) {
        if((!strncmp(it->useCase, SND_USE_CASE_VERB_HIFI_TUNNEL,
                            strlen(SND_USE_CASE_VERB_HIFI_TUNNEL))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_TUNNEL,
                            strlen(SND_USE_CASE_MOD_PLAY_TUNNEL))) ||
           (!strncmp(it->useCase, SND_USE_CASE_VERB_HIFI2,
                            strlen(SND_USE_CASE_VERB_HIFI2))) ||
           (!strncmp(it->useCase, SND_USE_CASE_MOD_PLAY_MUSIC2,
                            strlen(SND_USE_CASE_MOD_PLAY_MUSIC2)))) {
            mParent->mDeviceList.erase(it);
        }
    }

    mSessionId = -1;

    mRoutePcmAudio = false;
    mRouteCompreAudio = false;
    mRoutePcmToSpdif = false;
    mRouteCompreToSpdif = false;
    mRoutePcmToHdmi = false;
    mRouteCompreToHdmi = false;
    mUseTunnelDecode = false;
    mCaptureFromProxy = false;
    mOpenMS11Decoder = false;
    mAacConfigDataSet = false;
    mWMAConfigDataSet = false;
    mOpenMS11Decoder = false;

}

}       // namespace android_audio_legacy
