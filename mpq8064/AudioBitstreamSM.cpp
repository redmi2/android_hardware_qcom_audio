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
    ms11InputBuffer = ms11InputBufferWritePtr = NULL;
    ms11DDEncOutputBuffer = ms11DDEncOutputBufferWritePtr = NULL;
    ms11PCM2ChOutputBuffer = ms11PCM2ChOutputBufferWritePtr = NULL;
    ms11PCMMChOutputBuffer = ms11PCMMChOutputBufferWritePtr = NULL;
}

/*
Free the allocated memory
*/
AudioBitstreamSM::~AudioBitstreamSM()
{
    if(ms11InputBuffer != NULL)
       free(ms11InputBuffer);
    if(ms11DDEncOutputBuffer != NULL)
       free(ms11DDEncOutputBuffer);
    if(ms11PCM2ChOutputBuffer != NULL)
       free(ms11PCM2ChOutputBuffer);
    if(ms11PCMMChOutputBuffer != NULL)
       free(ms11PCMMChOutputBuffer);
}

/*
Allocate twice the max buffer size of input and output for sufficient buffering
*/

bool AudioBitstreamSM::initBitstreamPtr()
{
    ms11InputBuffer=(char *)malloc(SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING);
                                // multiplied by 2 to convert to bytes
    if(ms11InputBuffer != NULL) {
        ms11InputBufferWritePtr = ms11InputBuffer;
    } else {
        ALOGE("MS11 input buffer not allocated");
        return false;
    }

    ms11DDEncOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING);
    if(ms11DDEncOutputBuffer) {
        ms11DDEncOutputBufferWritePtr=ms11DDEncOutputBuffer;
    } else {
        ALOGE("MS11 DDEnc output buffer not allocated");
        return false;
    }
    ms11PCM2ChOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*STEREO_CHANNELS*FACTOR_FOR_BUFFERING);
    if(ms11PCM2ChOutputBuffer) {
        ms11PCM2ChOutputBufferWritePtr=ms11PCM2ChOutputBuffer;
    } else {
        ALOGE("MS11 PCM2Ch output buffer not allocated");
        return false;
    }
    ms11PCMMChOutputBuffer =(char *)malloc(SAMPLES_PER_CHANNEL*MAX_OUTPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING);
    if(ms11PCMMChOutputBuffer) {
        ms11PCMMChOutputBufferWritePtr=ms11PCMMChOutputBuffer;
    } else {
        ALOGE("MS11 PCMMCh output buffer not allocated");
        return false;
    }
    return true;
}

/*
Reset the buffer pointers to start for. This will be help in flush and close
*/
void AudioBitstreamSM::resetBitstreamPtr()
{
    ms11InputBufferWritePtr = ms11InputBuffer;
    ms11DDEncOutputBufferWritePtr = ms11DDEncOutputBuffer;
    ms11PCM2ChOutputBufferWritePtr = ms11PCM2ChOutputBuffer;
    ms11PCMMChOutputBufferWritePtr = ms11PCMMChOutputBuffer;
}

/*
Reset the output buffer pointers to start for port reconfiguration
*/
void AudioBitstreamSM::resetOutputBitstreamPtr()
{
    ms11DDEncOutputBufferWritePtr = ms11DDEncOutputBuffer;
    ms11PCM2ChOutputBufferWritePtr = ms11PCM2ChOutputBuffer;
    ms11PCMMChOutputBufferWritePtr = ms11PCMMChOutputBuffer;
}

/*
Copy the bitstream/pcm from Player to internal buffer.
The incoming bitstream is appended to existing bitstream
*/
void AudioBitstreamSM::copyBitstreamToInternalBuffer(char *bufPtr, size_t bytes)
{
    int32_t bufLen = SAMPLES_PER_CHANNEL*MAX_INPUT_CHANNELS_SUPPORTED*FACTOR_FOR_BUFFERING;
    // flush the input buffer if input is not consumed
    if( (ms11InputBufferWritePtr+bytes) > (ms11InputBuffer+bufLen) ) {
        ALOGE("Input bitstream is not consumed");
        ms11InputBufferWritePtr = ms11InputBuffer;
    }

    memcpy(ms11InputBufferWritePtr, bufPtr, bytes);
    ms11InputBufferWritePtr += bytes;
}

/*
Append zeros to the bitstream, so that the entire bitstream in ADIF is pushed
out for decoding
*/
void AudioBitstreamSM::appendSilenceToBitstreamInternalBuffer(uint32_t bytes, unsigned char value)
{
    for(int i=0; i< bytes; i++)
        *ms11InputBufferWritePtr++ = value;
}

/*
Flags if sufficient bitstream is available to proceed to decode based on
the threshold
*/
bool AudioBitstreamSM::sufficientBitstreamToDecode(size_t minThreshBytesToDecode)
{
    bool proceedDecode = false;
    if( (ms11InputBufferWritePtr-ms11InputBuffer) > minThreshBytesToDecode)
        proceedDecode = true;
    return proceedDecode;
}

/*
Gets the start address of the bitstream buffer. This is used for start of decode
*/
char* AudioBitstreamSM::getInputBufferPtr()
{
    return ms11InputBuffer;
}

/*
Gets the writePtr of the bitstream buffer. This is used for calculating length of
bitstream
*/
char* AudioBitstreamSM::getInputBufferWritePtr()
{
    return ms11InputBufferWritePtr;
}

/*
Get the output buffer start pointer to start rendering the pcm sampled to driver
*/
char* AudioBitstreamSM::getOutputBufferPtr(int format)
{
    if(format == PCM_MCH_OUT)
        return ms11PCMMChOutputBuffer;
    else if(format == PCM_2CH_OUT)
        return ms11PCM2ChOutputBuffer;
    else
        return ms11DDEncOutputBuffer;
}

/*
Output the pointer from where the next PCM samples can be copied to buffer
*/
char* AudioBitstreamSM::getOutputBufferWritePtr(int format)
{
    if(format == PCM_MCH_OUT)
        return ms11PCMMChOutputBufferWritePtr;
    else if(format == PCM_2CH_OUT)
        return ms11PCM2ChOutputBufferWritePtr;
    else
        return ms11DDEncOutputBufferWritePtr;
}

/*
Provides the bitstream size available in the internal buffer
*/
size_t AudioBitstreamSM::bitStreamBufSize()
{
    return (ms11InputBufferWritePtr-ms11InputBuffer);
}

/*
After decode, the residue bitstream in the buffer is moved to start, so as to
avoid circularity constraints
*/
void AudioBitstreamSM::copyResidueBitstreamToStart(size_t bytesConsumedInDecode)
{
    size_t remainingBytes = ms11InputBufferWritePtr -
                                  (bytesConsumedInDecode+ms11InputBuffer);
    memcpy(ms11InputBuffer, ms11InputBuffer+bytesConsumedInDecode, remainingBytes);
    ms11InputBufferWritePtr = ms11InputBuffer+remainingBytes;
}

/*
Remaing samples less than the one period size required for the pcm driver
is moved to start of the buffer
*/
void AudioBitstreamSM::copyResidueOutputToStart(int format, size_t samplesRendered)
{
    size_t remainingBytes;
    if(format == PCM_MCH_OUT)
    {
        remainingBytes = ms11PCMMChOutputBufferWritePtr-(ms11PCMMChOutputBuffer+samplesRendered);
        memcpy(ms11PCMMChOutputBuffer, ms11PCMMChOutputBuffer+samplesRendered, remainingBytes);
        ms11PCMMChOutputBufferWritePtr = ms11PCMMChOutputBuffer + remainingBytes;
    } else if(format == PCM_2CH_OUT) {
        remainingBytes = ms11PCM2ChOutputBufferWritePtr-(ms11PCM2ChOutputBuffer+samplesRendered);
        memcpy(ms11PCM2ChOutputBuffer, ms11PCM2ChOutputBuffer+samplesRendered, remainingBytes);
        ms11PCM2ChOutputBufferWritePtr = ms11PCM2ChOutputBuffer + remainingBytes;
    } else {
        remainingBytes = ms11DDEncOutputBufferWritePtr-(ms11DDEncOutputBuffer+samplesRendered);
        memcpy(ms11DDEncOutputBuffer, ms11DDEncOutputBuffer+samplesRendered, remainingBytes);
        ms11DDEncOutputBufferWritePtr = ms11DDEncOutputBuffer + remainingBytes;
    }
}

/*
The write pointer is updated after the incoming PCM samples are copied to the
output buffer
*/
void AudioBitstreamSM::setOutputBufferWritePtr(int format, size_t outputPCMSample)
{
    if(format == PCM_MCH_OUT)
        ms11PCMMChOutputBufferWritePtr += outputPCMSample;
    else if(format == PCM_2CH_OUT)
        ms11PCM2ChOutputBufferWritePtr += outputPCMSample;
    else
        ms11DDEncOutputBufferWritePtr += outputPCMSample;
}

/*
Flags if sufficient samples are available to render to PCM driver
*/
bool AudioBitstreamSM::sufficientSamplesToRender(int format, int minSizeReqdToRender)
{
    bool status = false;
    char *bufPtr, *bufWritePtr;
    if(format == PCM_MCH_OUT)
    {
        bufPtr = ms11PCMMChOutputBuffer;
        bufWritePtr = ms11PCMMChOutputBufferWritePtr;
    } else if (format == PCM_2CH_OUT) {
        bufPtr = ms11PCM2ChOutputBuffer;
        bufWritePtr = ms11PCM2ChOutputBufferWritePtr;
    } else {
        bufPtr = ms11DDEncOutputBuffer;
        bufWritePtr = ms11DDEncOutputBufferWritePtr;
    }
    if( (bufWritePtr-bufPtr) >= minSizeReqdToRender )
        status = true;
    return status;
}

}       // namespace android_audio_legacy
