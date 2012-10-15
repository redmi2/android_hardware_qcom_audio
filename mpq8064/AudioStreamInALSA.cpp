/* AudioStreamInALSA.cpp
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

#define LOG_TAG "AudioStreamInALSA"
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

AudioStreamInALSA::AudioStreamInALSA(AudioHardwareALSA *parent,
        alsa_handle_t *handle,
        AudioSystem::audio_in_acoustics audio_acoustics) :
    ALSAStreamOps(parent, handle),
    mFramesLost(0),
    mParent(parent),
    mAcoustics(audio_acoustics)
{
}

AudioStreamInALSA::~AudioStreamInALSA()
{
    close();
}

status_t AudioStreamInALSA::setGain(float gain)
{
    return 0; //mixer() ? mixer()->setMasterGain(gain) : (status_t)NO_INIT;
}

ssize_t AudioStreamInALSA::read(void *buffer, ssize_t bytes)
{
    int period_size;

    ALOGV("read:: buffer %p, bytes %d", buffer, bytes);

    if (!mPowerLock) {
        acquire_wake_lock (PARTIAL_WAKE_LOCK, "AudioInLock");
        mPowerLock = true;
    }

    int n;
    status_t          err;
    size_t            read = 0;
    char *use_case;
    int newMode = mParent->mode();
    bool bIsUseCaseSet = false;
    if((mHandle->handle == NULL) && (mHandle->rxHandle == NULL) &&
         (strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                              strlen(SND_USE_CASE_VERB_IP_VOICECALL))) &&
         (strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                              strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {
        mParent->mLock.lock();
        snd_use_case_get(mHandle->ucMgr, "_verb", (const char **)&use_case);
        if ((use_case != NULL) && (strncmp(use_case, SND_USE_CASE_VERB_INACTIVE,
                                       strlen(SND_USE_CASE_VERB_INACTIVE)))) {
            if ((mHandle->devices == AudioSystem::DEVICE_IN_VOICE_CALL) &&
                (newMode == AudioSystem::MODE_IN_CALL)) {
                ALOGD("read:: mParent->mIncallMode=%d", mParent->mIncallMode);
                if ((mParent->mIncallMode & AudioSystem::CHANNEL_IN_VOICE_UPLINK) &&
                    (mParent->mIncallMode & AudioSystem::CHANNEL_IN_VOICE_DNLINK)) {
                    strlcpy(mHandle->useCase, SND_USE_CASE_MOD_CAPTURE_VOICE_UL_DL,
                            sizeof(mHandle->useCase));
                } else if (mParent->mIncallMode & AudioSystem::CHANNEL_IN_VOICE_DNLINK) {
                    strlcpy(mHandle->useCase, SND_USE_CASE_MOD_CAPTURE_VOICE_DL,
                            sizeof(mHandle->useCase));
                }
            } else if(mHandle->devices == AudioSystem::DEVICE_IN_FM_RX) {
                strlcpy(mHandle->useCase, SND_USE_CASE_MOD_CAPTURE_FM, sizeof(mHandle->useCase));
            } else if (mHandle->devices == AudioSystem::DEVICE_IN_FM_RX_A2DP) {
                strlcpy(mHandle->useCase, SND_USE_CASE_MOD_CAPTURE_A2DP_FM, sizeof(mHandle->useCase));
            } else if(!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                                        strlen(SND_USE_CASE_MOD_PLAY_VOIP))) {
                strcpy(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP);
            }else {
                strlcpy(mHandle->useCase, SND_USE_CASE_MOD_CAPTURE_MUSIC, sizeof(mHandle->useCase));
            }
        } else {
            if ((mHandle->devices == AudioSystem::DEVICE_IN_VOICE_CALL) &&
                (newMode == AudioSystem::MODE_IN_CALL)) {
                ALOGD("read:: ---- mParent->mIncallMode=%d", mParent->mIncallMode);
                if ((mParent->mIncallMode & AudioSystem::CHANNEL_IN_VOICE_UPLINK) &&
                    (mParent->mIncallMode & AudioSystem::CHANNEL_IN_VOICE_DNLINK)) {
                    strlcpy(mHandle->useCase, SND_USE_CASE_VERB_UL_DL_REC, sizeof(mHandle->useCase));
                } else if (mParent->mIncallMode & AudioSystem::CHANNEL_IN_VOICE_DNLINK) {
                    strlcpy(mHandle->useCase, SND_USE_CASE_VERB_DL_REC, sizeof(mHandle->useCase));
                }
            } else if(mHandle->devices == AudioSystem::DEVICE_IN_FM_RX) {
                strlcpy(mHandle->useCase, SND_USE_CASE_VERB_FM_REC, sizeof(mHandle->useCase));
            } else if (mHandle->devices == AudioSystem::DEVICE_IN_FM_RX_A2DP) {
                strlcpy(mHandle->useCase, SND_USE_CASE_VERB_FM_A2DP_REC, sizeof(mHandle->useCase));
            } else if(!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                                         strlen(SND_USE_CASE_VERB_IP_VOICECALL))){
                    strcpy(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL);
            } else {
                strlcpy(mHandle->useCase, SND_USE_CASE_VERB_HIFI_REC, sizeof(mHandle->useCase));
            }
            bIsUseCaseSet = true;
        }
        if(use_case) {
            free(use_case);
            use_case = NULL;
        }
        if((!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                                 strlen(SND_USE_CASE_VERB_IP_VOICECALL))) ||
            (!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                                  strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {
            mHandle->mode = AudioSystem::MODE_IN_COMMUNICATION;
        } else {
            mHandle->mode = mParent->mode();
        }
        mHandle->activeDevice = mDevices;
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
        if(mHandle->handle->fd < 0) {
            ALOGE("read:: PCM device open failed");
            mParent->mLock.unlock();

            return 0;
        }
        mParent->mLock.unlock();
    }

    period_size = mHandle->periodSize;
    int read_pending = bytes;
    do {
        if (read_pending < period_size) {
            read_pending = period_size;
        }

        n = pcm_read(mHandle->handle, buffer,
            period_size);
        ALOGV("pcm_read() returned n = %d", n);
        if (n && (n == -EIO || n == -EAGAIN || n == -EPIPE || n == -EBADFD)) {
            ALOGW("pcm_read() returned error n %d, Recovering from error\n", n);
            pcm_close(mHandle->handle);
            mHandle->handle = NULL;
            mHandle->module->open(mHandle);
            continue;
        }
        else if (n < 0) {
            ALOGD("pcm_read() returned n < 0");
            return static_cast<ssize_t>(n);
        }
        else {
            read += static_cast<ssize_t>((period_size));
            read_pending -= period_size;
        }

    } while (mHandle->handle && read < bytes);

    return read;
}

status_t AudioStreamInALSA::dump(int fd, const Vector<String16>& args)
{
    return NO_ERROR;
}

status_t AudioStreamInALSA::open(int mode)
{
    Mutex::Autolock autoLock(mParent->mLock);

    status_t status = ALSAStreamOps::open(mode);

    return status;
}

status_t AudioStreamInALSA::close()
{
    Mutex::Autolock autoLock(mParent->mLock);

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
        release_wake_lock ("AudioInLock");
        mPowerLock = false;
    }

    return NO_ERROR;
}

status_t AudioStreamInALSA::standby()
{
    Mutex::Autolock autoLock(mParent->mLock);
    if((!strncmp(mHandle->useCase, SND_USE_CASE_VERB_IP_VOICECALL,
                             strlen(SND_USE_CASE_VERB_IP_VOICECALL))) ||
        (!strncmp(mHandle->useCase, SND_USE_CASE_MOD_PLAY_VOIP,
                             strlen(SND_USE_CASE_MOD_PLAY_VOIP)))) {
         return NO_ERROR;
    }

    ALOGD("standby");

    mHandle->module->standby(mHandle);

    if (mPowerLock) {
        release_wake_lock ("AudioInLock");
        mPowerLock = false;
    }

    return NO_ERROR;
}

void AudioStreamInALSA::resetFramesLost()
{
    Mutex::Autolock autoLock(mParent->mLock);
    mFramesLost = 0;
}

unsigned int AudioStreamInALSA::getInputFramesLost() const
{
    unsigned int count = mFramesLost;
    // Stupid interface wants us to have a side effect of clearing the count
    // but is defined as a const to prevent such a thing.
    ((AudioStreamInALSA *)this)->resetFramesLost();
    return count;
}

status_t AudioStreamInALSA::setAcousticParams(void *params)
{
    Mutex::Autolock autoLock(mParent->mLock);

    return (status_t)NO_ERROR;
}

}       // namespace android_audio_legacy
