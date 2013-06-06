/* AudioStreamOutALSA.cpp
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

#define LOG_TAG "AudioStreamOutALSA"
//#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0
#include <utils/Log.h>
#include <utils/String8.h>

#include <cutils/properties.h>
#include <media/AudioRecord.h>
#include <hardware_legacy/power.h>

#include "AudioHardwareALSA.h"

namespace android_audio_legacy
{

// ----------------------------------------------------------------------------

static const int DEFAULT_SAMPLE_RATE = ALSA_DEFAULT_SAMPLE_RATE;

// ----------------------------------------------------------------------------

AudioStreamOutALSA::AudioStreamOutALSA(AudioHardwareALSA *parent, alsa_handle_t *handle) :
    ALSAStreamOps(parent, handle),
    mParent(parent),
    mFrameCount(0),
    mA2dpUseCase(AudioHardwareALSA::USECASE_NONE)
{
}

AudioStreamOutALSA::~AudioStreamOutALSA()
{
    if (mParent->mRouteAudioToA2dp) {
         ALOGD("stopA2dpPlayback_l - usecase  %x", mA2dpUseCase);
         status_t err = mParent->stopA2dpPlayback_l(mA2dpUseCase);
         if (err)
             ALOGW("stopA2dpPlayback_l return err  %d", err);
    }
    close();
}

uint32_t AudioStreamOutALSA::channels() const
{
    int c = ALSAStreamOps::channels();
    return c;
}

status_t AudioStreamOutALSA::setVolume(float left, float right)
{
    int lpa_vol;
    float volume;
    status_t status = NO_ERROR;

    if(!strncmp(mHandle->useCase, SND_USE_CASE_VERB_HIFI_LOW_POWER,
                            strlen(SND_USE_CASE_VERB_HIFI_LOW_POWER)) ||
       !strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_LPA,
                            strlen(SND_USE_CASE_MOD_PLAY_LPA))) {
        volume = (left + right) / 2;
        if (volume < 0.0) {
            ALOGW("AudioSessionOutMSM7xxx::setVolume(%f) under 0.0, assuming 0.0\n", volume);
            volume = 0.0;
        } else if (volume > 1.0) {
            ALOGW("AudioSessionOutMSM7xxx::setVolume(%f) over 1.0, assuming 1.0\n", volume);
            volume = 1.0;
        }
        lpa_vol = lrint((volume * 100.0)+0.5);
        ALOGD("setLpaVolume(%f)\n", volume);
        ALOGD("Setting LPA volume to %d (available range is 0 to 100)\n", lpa_vol);
        mHandle->module->setLpaVolume(lpa_vol);

        return status;
    }
    return INVALID_OPERATION;
}

status_t AudioStreamOutALSA::setParameters(const String8& keyValuePairs) {
    ALOGV("AudioStreamOut: setParameters %s", keyValuePairs.string());
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 keyStandby = String8(STANDBY_DEVICES_KEY);
    String8 keyResume = String8(RESUME_DEVICES_KEY);
    int device;
    if(param.getInt(keyStandby, device) == NO_ERROR) {
        if(mHandle->handle == NULL) {
           mHandle->playbackMode = STANDBY;
           mHandle->activeDevice = 0;
        } else if((mHandle->activeDevice & device) == mHandle->activeDevice) {
            mHandle->playbackMode = STANDBY;
            standby();
            mHandle->activeDevice = 0;
        } else if(mHandle->activeDevice & device) {
            mHandle->module->switchDeviceUseCase(mHandle,
                 mHandle->activeDevice & (~device),
                 mHandle->mode);
        }
    } else if (param.getInt(keyResume, device) == NO_ERROR) {
        if(mHandle->devices & device) {
            int devices = mHandle->devices;
            mHandle->module->switchDeviceUseCase(mHandle,
                 mHandle->activeDevice | (mHandle->devices & device),
                 mHandle->mode);
            mHandle->devices = devices;
            mHandle->playbackMode = PLAY;
        }
    } else {
       return ALSAStreamOps::setParameters(keyValuePairs);
    }
    return NO_ERROR;
}

String8 AudioStreamOutALSA::getParameters(const String8& keys) {
    ALOGV("AudioStreamOut: getParameters %s", keys.string());
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 keyComprStandby = String8(COMPR_STANDBY_DEVICES_KEY);
    int devices = mDevices;
    if(param.get(keyComprStandby, value) == NO_ERROR) {
        param.addInt(keyComprStandby, 0);
        ALOGV("getParameters() %s", param.toString().string());
        return param.toString();
    } else {
        return ALSAStreamOps::getParameters(keys);
    }
}

ssize_t AudioStreamOutALSA::write(const void *buffer, size_t bytes)
{
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

    int write_pending = bytes;
    bool bIsUseCaseSet = false;
    int devices;
    /*
    If the handle was made to NULL and put to STOP by other streams,
    fake that the handle is valid and rendering data in real time to AudioFlinger
    so that no underruns happen in framework
    */
    {
        Mutex::Autolock autolock1(mParent->mDeviceStateLock);
        devices = mParent->getUnComprDeviceInCurrDevices(mDevices);
        ALOGV("mDevices: %d, activeDevices: %d unComprDevices %d", mDevices, mHandle->activeDevice, devices);
        if((mHandle->handle == NULL) && (mHandle->playbackMode == STANDBY) &&
           (mHandle->channels)) {
            mParent->mDeviceStateLock.unlock();
            int periodTimeInUS = (int)((bytes*1000)/(96*mHandle->channels));
            ALOGD("Ignoring pcm on device with compressed pass through playback");
            usleep(periodTimeInUS);
            return bytes;
        }
    }
    if((mHandle->handle == NULL) && (mHandle->rxHandle == NULL) &&
         (strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                              strlen(SND_USE_CASE_VERB_IP_VOICECALL))) &&
         (strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                              strlen(SND_USE_CASE_VERB_IP_VOICECALL)))) {
        mParent->mLock.lock();
        ALOGV("mDevices =0x%x", mDevices);
        if(mDevices &  AudioSystem::DEVICE_OUT_PROXY) {
            ALOGV("StreamOut write - mRouteAudioToA2dp = %d ", mParent->mRouteAudioToA2dp);
            mParent->mRouteAudioToA2dp = true;
        }
        snd_use_case_get(mHandle->ucMgr, "_verb", (const char **)&use_case);
        if ((use_case == NULL) || (!strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                                            strlen(SND_USE_CASE_VERB_INACTIVE)))) {
            if(!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                                            strlen(SND_USE_CASE_VERB_IP_VOICECALL))){
                 strlcpy(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,sizeof(mHandle->useCase));
             }
             else {
                 strlcpy(mHandle->useCase, SND_USE_CASE_VERB_HIFI, sizeof(mHandle->useCase));
             }
             bIsUseCaseSet = true;
        } else {
            if(!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                                    strlen(SND_USE_CASE_MOD_PLAY_VOIP))) {
                strlcpy(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,sizeof(mHandle->useCase));
             } else {
                 strlcpy(mHandle->useCase, SND_USE_CASE_MOD_PLAY_MUSIC, sizeof(mHandle->useCase));
             }
        }
        if(use_case) {
            free(use_case);
            use_case = NULL;
        }
        mHandle->devices = mDevices;
        mHandle->activeDevice = devices;
        mHandle->playbackMode = PLAY;
        mHandle->hdmiFormat = mHandle->activeDevice & AudioSystem::DEVICE_OUT_AUX_DIGITAL ?
                              ROUTE_UNCOMPRESSED : ROUTE_NONE;
        mHandle->spdifFormat = mHandle->activeDevice & AudioSystem::DEVICE_OUT_SPDIF ?
                              ROUTE_UNCOMPRESSED : ROUTE_NONE;
        if((!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                                 strlen(SND_USE_CASE_VERB_IP_VOICECALL))) ||
           (!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                                 strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {
            mHandle->mode = AudioSystem::MODE_IN_COMMUNICATION;
        } else {
            mHandle->mode = mParent->mode();
        }
        if (mHandle->module->setUseCase(&(*mHandle), bIsUseCaseSet))
            return NO_INIT;

        if((!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                                 strlen(SND_USE_CASE_VERB_IP_VOICECALL))) ||
          (!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                                strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {
             err = mHandle->module->startVoipCall(mHandle);
        }
        else
             mHandle->module->open(mHandle);
        if(mHandle->handle && mHandle->handle->fd < 0) {
            ALOGE("write:: device open failed");
            mParent->mLock.unlock();
            return 0;
        }
        if (mParent->mRouteAudioToA2dp) {
            status_t err = NO_ERROR;
            mA2dpUseCase = mParent->useCaseStringToEnum(mHandle->useCase);
            if (!(mParent->getA2DPActiveUseCases_l() & mA2dpUseCase)) {
                ALOGD("startA2dpPlayback_l usecase %x", mA2dpUseCase);
                err = mParent->startA2dpPlayback_l(mA2dpUseCase);
                if(err) {
                    ALOGE("startA2dpPlayback_l from write return err = %d", err);
                    mParent->mLock.unlock();
                    return err;
                }
            }
        }
        mParent->mLock.unlock();
    }

    period_size = mHandle->periodSize;
    do {
        if (write_pending < period_size) {
            write_pending = period_size;
        }
        if((mParent->mVoipStreamCount) && (mHandle->rxHandle != 0)) {
            n = mParent->hw_pcm_write(mHandle->rxHandle,
                     (char *)buffer + sent,
                      period_size);
        } else if (mHandle->handle != 0){
            n = mParent->hw_pcm_write(mHandle->handle,
                     (char *)buffer + sent,
                      period_size);
        }
        if (n == -EBADFD) {
            // Somehow the stream is in a bad state. The driver probably
            // has a bug and snd_pcm_recover() doesn't seem to handle this.
            mHandle->module->open(mHandle);
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

    } while ((mHandle->handle||(mHandle->rxHandle && mParent->mVoipStreamCount)) && sent < bytes);

    return sent;
}

status_t AudioStreamOutALSA::dump(int fd, const Vector<String16>& args)
{
    return NO_ERROR;
}

status_t AudioStreamOutALSA::open(int mode)
{
    Mutex::Autolock autoLock(mParent->mLock);

    return ALSAStreamOps::open(mode);
}

status_t AudioStreamOutALSA::close()
{
    Mutex::Autolock autoLock(mParent->mLock);
    ALOGV("close = %d", mParent->mRouteAudioToA2dp);
    if (mParent->mRouteAudioToA2dp) {
         ALOGD("close-stopA2dpPlayback_l- usecase %x", mA2dpUseCase);
         status_t err = mParent->stopA2dpPlayback_l(mA2dpUseCase);
         if(err) {
             ALOGE("stopA2dpPlayback_l from hardware output close return err = %d", err);
             return err;
         }
    }

    if((!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                             strlen(SND_USE_CASE_VERB_IP_VOICECALL))) ||
        (!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                             strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {
         if((mParent->mVoipStreamCount)) {
                return NO_ERROR;
         }
         mParent->mVoipStreamCount = 0;
         mParent->mVoipMicMute = 0;
     }

    ALOGD("close");
    ALSAStreamOps::close();

    if (mPowerLock) {
        release_wake_lock ("AudioOutLock");
        mPowerLock = false;
    }

    return NO_ERROR;
}

status_t AudioStreamOutALSA::standby()
{
     ALOGD("AudioStreamOut: standby()");
     Mutex::Autolock autoLock(mParent->mLock);
     Mutex::Autolock autolock1(mParent->mDeviceStateLock);
     if((!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                              strlen(SND_USE_CASE_VERB_IP_VOICECALL))) ||
       (!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                             strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {

         return NO_ERROR;
     }

    if(mHandle->handle != NULL)
        mHandle->module->standby(mHandle);
    ALOGD("standby = %d", mParent->mRouteAudioToA2dp);

    if (mParent->mRouteAudioToA2dp) {
        ALOGD("standby-stopA2dpPlayback_l- usecase %x", mA2dpUseCase);
        status_t err = mParent->stopA2dpPlayback_l(mA2dpUseCase);
        if(err) {
            ALOGE("stopA2dpPlayback_l from standby return err = %d", err);
            return err;
        }
    }
    if (mPowerLock) {
        release_wake_lock ("AudioOutLock");
        mPowerLock = false;
    }

    mFrameCount = 0;

    return NO_ERROR;
}

#define USEC_TO_MSEC(x) ((x + 999) / 1000)

uint32_t AudioStreamOutALSA::latency() const
{
    // Android wants latency in milliseconds.
    return USEC_TO_MSEC (mHandle->latency);
}

// return the number of audio frames written by the audio dsp to DAC since
// the output has exited standby
status_t AudioStreamOutALSA::getRenderPosition(uint32_t *dspFrames)
{
    *dspFrames = mFrameCount;
    return NO_ERROR;
}

}       // namespace android_audio_legacy
