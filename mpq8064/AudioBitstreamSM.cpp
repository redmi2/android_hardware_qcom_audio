/*
 * Copyright (C) 2011 The Android Open Source Project
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>

#define LOG_TAG "AudioBitstreamStateMachine"
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


/*
Initialize all input and output pointers
*/
AudioBitstreamSM::AudioBitstreamSM()
{
    mInputBufferCurrPtr = mInputBuffer = mInputBufferWritePtr = NULL;
    mEncOutputBuffer = mEncOutputBufferWritePtr = NULL;
    mPCM2ChOutputBuffer = mPCM2ChOutputBufferWritePtr = NULL;
    mPCMMChOutputBuffer = mPCMMChOutputBufferWritePtr = NULL;
    mPassthroughOutputBuffer = mPassthroughOutputBufferWritePtr = NULL;
    mBufferingFactor = 1;
    mBufferingFactorCnt = 0;
}

/*
Free the allocated memory
*/
AudioBitstreamSM::~AudioBitstreamSM()
{
    if(mInputBuffer != NULL) {
       free(mInputBuffer);
       mInputBuffer = NULL;
    }
    if(mEncOutputBuffer != NULL) {
       free(mEncOutputBuffer);
       mEncOutputBuffer = NULL;
    }
    if(mPCM2ChOutputBuffer != NULL) {
       free(mPCM2ChOutputBuffer);
       mPCM2ChOutputBuffer = NULL;
    }
    if(mPCMMChOutputBuffer != NULL) {
        free(mPCMMChOutputBuffer);
        mPCMMChOutputBuffer = NULL;
    }
    if(mPassthroughOutputBuffer != NULL) {
       free(mPassthroughOutputBuffer);
       mPassthroughOutputBuffer = NULL;
    }
    mBufferingFactor = 1;
    mBufferingFactorCnt = 0;
}

/*
Allocate twice the max buffer size of input and output for sufficient buffering
*/

bool AudioBitstreamSM::initBitstreamPtr()
{
    mInputBuffer=(char *)malloc(SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*(mBufferingFactor+1));
                                // multiplied by 2 to convert to bytes
    if(mInputBuffer != NULL) {
        mInputBufferCurrPtr = mInputBufferWritePtr = mInputBuffer;
    } else {
        ALOGE("MS11 input buffer not allocated");
        return false;
    }

    mEncOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING);
    if(mEncOutputBuffer) {
        mEncOutputBufferWritePtr=mEncOutputBuffer;
    } else {
        ALOGE("MS11 Enc output buffer not allocated");
        return false;
    }
    mPCM2ChOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*STEREO_CHANNELS*FACTOR_FOR_BUFFERING);
    if(mPCM2ChOutputBuffer) {
        mPCM2ChOutputBufferWritePtr=mPCM2ChOutputBuffer;
    } else {
        ALOGE("MS11 PCM2Ch output buffer not allocated");
        return false;
    }
    mPCMMChOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*MAX_OUTPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING);
    if(mPCMMChOutputBuffer) {
        mPCMMChOutputBufferWritePtr=mPCMMChOutputBuffer;
    } else {
        ALOGE("MS11 PCMMCh output buffer not allocated");
        return false;
    }
    mPassthroughOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING);
    if(mPassthroughOutputBuffer) {
        mPassthroughOutputBufferWritePtr=mPassthroughOutputBuffer;
    } else {
        ALOGE("MS11 Enc output buffer not allocated");
        return false;
    }
    return true;
}

/*
bufferingFactor - This mode helps to keep track of the data in the past
so that passthrough to PCM can be supported without much loss of rendering data
*/
bool AudioBitstreamSM::initBitstreamPtr(int inputBufferingFactor)
{
    mBufferingFactor = inputBufferingFactor;
    return (initBitstreamPtr());
}

/*
Reset the buffer pointers to start for. This will be help in flush and close
*/
void AudioBitstreamSM::resetBitstreamPtr()
{
    mInputBufferCurrPtr = mInputBufferWritePtr = mInputBuffer;
    mEncOutputBufferWritePtr = mEncOutputBuffer;
    mPCM2ChOutputBufferWritePtr = mPCM2ChOutputBuffer;
    mPCMMChOutputBufferWritePtr = mPCMMChOutputBuffer;
    mPassthroughOutputBufferWritePtr = mPassthroughOutputBuffer;
    mBufferingFactorCnt = 0;
}

/*
Reset the output buffer pointers to start for port reconfiguration
*/
void AudioBitstreamSM::resetOutputBitstreamPtr()
{
    mEncOutputBufferWritePtr = mEncOutputBuffer;
    mPCM2ChOutputBufferWritePtr = mPCM2ChOutputBuffer;
    mPCMMChOutputBufferWritePtr = mPCMMChOutputBuffer;
    mPassthroughOutputBufferWritePtr = mPassthroughOutputBuffer;
}

/*
Copy the bitstream/pcm from Player to internal buffer.
The incoming bitstream is appended to existing bitstream
*/
void AudioBitstreamSM::copyBitstreamToInternalBuffer(char *bufPtr, size_t bytes)
{
    int32_t bufLen = SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*(mBufferingFactor+1);
    // flush the input buffer if input is not consumed
    if( (mInputBufferWritePtr+bytes) > (mInputBuffer+bufLen) ) {
        ALOGE("Input bitstream is not consumed");
        mInputBufferWritePtr = mInputBuffer;
    }

    memcpy(mInputBufferWritePtr, bufPtr, bytes);
    mInputBufferWritePtr += bytes;
    if(mBufferingFactorCnt < mBufferingFactor)
        mBufferingFactorCnt++;
}

/*
Append zeros to the bitstream, so that the entire bitstream in ADIF is pushed
out for decoding
*/
void AudioBitstreamSM::appendSilenceToBitstreamInternalBuffer(uint32_t bytes, unsigned char value)
{
    int32_t bufLen = SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*(mBufferingFactor+1);
    if( (mInputBufferWritePtr+bytes) > (mInputBuffer+bufLen) ) {
        bytes = bufLen + mInputBuffer - mInputBufferWritePtr;
    }
    for(int i=0; i< bytes; i++)
        *mInputBufferWritePtr++ = value;
    if(mBufferingFactorCnt < mBufferingFactor)
        mBufferingFactorCnt++;
}

/*
Flags if sufficient bitstream is available to proceed to decode based on
the threshold
*/
bool AudioBitstreamSM::sufficientBitstreamToDecode(size_t minThreshBytesToDecode)
{
    bool proceedDecode = false;
    if( (mInputBufferWritePtr-mInputBufferCurrPtr) > minThreshBytesToDecode)
        proceedDecode = true;
    return proceedDecode;
}

/*
Gets the start address of the bitstream buffer. This is used for start of decode
*/
char* AudioBitstreamSM::getInputBufferPtr()
{
    return mInputBufferCurrPtr;
}

/*
Gets the writePtr of the bitstream buffer. This is used for calculating length of
bitstream
*/
char* AudioBitstreamSM::getInputBufferWritePtr()
{
    return mInputBufferWritePtr;
}

/*
Get the output buffer start pointer to start rendering the pcm sampled to driver
*/
char* AudioBitstreamSM::getOutputBufferPtr(int format)
{
    switch(format) {
    case PCM_MCH_OUT:
        return mPCMMChOutputBuffer;
    case PCM_2CH_OUT:
        return mPCM2ChOutputBuffer;
    case COMPRESSED_OUT:
        return mEncOutputBuffer;
    case TRANSCODE_OUT:
        return mPassthroughOutputBuffer;
    default:
        return NULL;
    }
}

/*
Output the pointer from where the next PCM samples can be copied to buffer
*/
char* AudioBitstreamSM::getOutputBufferWritePtr(int format)
{
    switch(format) {
    case PCM_MCH_OUT:
        return mPCMMChOutputBufferWritePtr;
    case PCM_2CH_OUT:
        return mPCM2ChOutputBufferWritePtr;
    case COMPRESSED_OUT:
        return mEncOutputBufferWritePtr;
    case TRANSCODE_OUT:
        return mPassthroughOutputBufferWritePtr;
    default:
        return NULL;
    }
}

/*
Provides the bitstream size available in the internal buffer
*/
size_t AudioBitstreamSM::bitStreamBufSize()
{
    return (mInputBufferWritePtr-mInputBufferCurrPtr);
}

/*
After decode, the residue bitstream in the buffer is moved to start, so as to
avoid circularity constraints
*/
void AudioBitstreamSM::copyResidueBitstreamToStart(size_t bytesConsumedInDecode)
{
    size_t remainingCurrValidBytes = mInputBufferWritePtr -
                              (bytesConsumedInDecode+mInputBufferCurrPtr);
    size_t remainingTotalBytes = mInputBufferWritePtr -
                              (bytesConsumedInDecode+mInputBuffer);
    if(mBufferingFactorCnt == mBufferingFactor) {
        memcpy(mInputBuffer, mInputBuffer+bytesConsumedInDecode, remainingTotalBytes);
        mInputBufferWritePtr = mInputBuffer+remainingTotalBytes;
        mInputBufferCurrPtr = mInputBufferWritePtr-remainingCurrValidBytes;
    } else {
        mInputBufferCurrPtr += bytesConsumedInDecode;
    }
}

/*
Remaing samples less than the one period size required for the pcm driver
is moved to start of the buffer
*/
void AudioBitstreamSM::copyResidueOutputToStart(int format, size_t samplesRendered)
{
    size_t remainingBytes;
    switch(format) {
    case PCM_MCH_OUT:
        remainingBytes = mPCMMChOutputBufferWritePtr-(mPCMMChOutputBuffer+samplesRendered);
        memcpy(mPCMMChOutputBuffer, mPCMMChOutputBuffer+samplesRendered, remainingBytes);
        mPCMMChOutputBufferWritePtr = mPCMMChOutputBuffer + remainingBytes;
        break;
    case PCM_2CH_OUT:
        remainingBytes = mPCM2ChOutputBufferWritePtr-(mPCM2ChOutputBuffer+samplesRendered);
        memcpy(mPCM2ChOutputBuffer, mPCM2ChOutputBuffer+samplesRendered, remainingBytes);
        mPCM2ChOutputBufferWritePtr = mPCM2ChOutputBuffer + remainingBytes;
        break;
    case COMPRESSED_OUT:
        remainingBytes = mEncOutputBufferWritePtr-(mEncOutputBuffer+samplesRendered);
        memcpy(mEncOutputBuffer, mEncOutputBuffer+samplesRendered, remainingBytes);
        mEncOutputBufferWritePtr = mEncOutputBuffer + remainingBytes;
        break;
    case TRANSCODE_OUT:
        remainingBytes = mPassthroughOutputBufferWritePtr-(mPassthroughOutputBuffer+samplesRendered);
        memcpy(mPassthroughOutputBuffer, mPassthroughOutputBuffer+samplesRendered, remainingBytes);
        mPassthroughOutputBufferWritePtr = mPassthroughOutputBuffer + remainingBytes;
        break;
    default:
        break;
    }
}

/*
The write pointer is updated after the incoming PCM samples are copied to the
output buffer
*/
void AudioBitstreamSM::setOutputBufferWritePtr(int format, size_t outputPCMSample)
{
    switch(format) {
    case PCM_MCH_OUT:
        mPCMMChOutputBufferWritePtr += outputPCMSample;
        break;
    case PCM_2CH_OUT:
        mPCM2ChOutputBufferWritePtr += outputPCMSample;
        break;
    case COMPRESSED_OUT:
        mEncOutputBufferWritePtr += outputPCMSample;
        break;
    case TRANSCODE_OUT:
        mPassthroughOutputBufferWritePtr += outputPCMSample;
        break;
    default:
        break;
    }
}

/*
Flags if sufficient samples are available to render to PCM driver
*/
bool AudioBitstreamSM::sufficientSamplesToRender(int format, int minSizeReqdToRender)
{
    bool status = false;
    char *bufPtr = NULL, *bufWritePtr = NULL;
    switch(format) {
    case PCM_MCH_OUT:
        bufPtr = mPCMMChOutputBuffer;
        bufWritePtr = mPCMMChOutputBufferWritePtr;
        break;
    case PCM_2CH_OUT:
        bufPtr = mPCM2ChOutputBuffer;
        bufWritePtr = mPCM2ChOutputBufferWritePtr;
        break;
    case COMPRESSED_OUT:
        bufPtr = mEncOutputBuffer;
        bufWritePtr = mEncOutputBufferWritePtr;
        break;
    case TRANSCODE_OUT:
        bufPtr = mPassthroughOutputBuffer;
        bufWritePtr = mPassthroughOutputBufferWritePtr;
        break;
    default:
        break;
    }
    if( (bufWritePtr-bufPtr) >= minSizeReqdToRender )
        status = true;
    return status;
}

void AudioBitstreamSM::startInputBufferingMode()
{
    mBufferingFactorCnt = 0;
}

void AudioBitstreamSM::stopInputBufferingMode()
{
    size_t remainingCurrValidBytes = mInputBufferWritePtr - mInputBufferCurrPtr;
    mBufferingFactorCnt = mBufferingFactor;
    memcpy(mInputBuffer, mInputBufferCurrPtr, remainingCurrValidBytes);
    mInputBufferCurrPtr = mInputBuffer;
    mInputBufferWritePtr = mInputBuffer+remainingCurrValidBytes;
}

}       // namespace android_audio_legacy
