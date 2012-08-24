/* AudioHardwareALSA.h
 **
 ** Copyright 2008-2010, Wind River Systems
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
class AudioHardwareALSA;
class SoftMS11;
class AudioBitstreamSM;
/**
 * The id of ALSA module
 */
#define ALSA_HARDWARE_MODULE_ID "alsa"
#define ALSA_HARDWARE_NAME      "alsa"

#define DEFAULT_SAMPLING_RATE 48000
#define DEFAULT_CHANNEL_MODE  2
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
#define DEFAULT_IN_BUFFER_SIZE_BROADCAST_PCM_STEREO   2048
#define DEFAULT_IN_BUFFER_SIZE_BROADCAST_PCM_MCH   5376
#define DEFAULT_IN_BUFFER_SIZE_BROADCAST_COMPRESSED   6208

#define VOIP_SAMPLING_RATE_8K 8000
#define VOIP_SAMPLING_RATE_16K 16000
#define VOIP_DEFAULT_CHANNEL_MODE  1
#define VOIP_BUFFER_SIZE_8K    320
#define VOIP_BUFFER_SIZE_16K   640
#define VOIP_BUFFER_MAX_SIZE   VOIP_BUFFER_SIZE_16K
#define VOIP_PLAYBACK_LATENCY      6400
#define VOIP_RECORD_LATENCY        6400

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

#define ANC_FLAG        0x00000001
#define DMIC_FLAG       0x00000002
#define QMIC_FLAG       0x00000004
#define TTY_OFF         0x00000010
#define TTY_FULL        0x00000020
#define TTY_VCO         0x00000040
#define TTY_HCO         0x00000080
#define TTY_CLEAR       0xFFFFFF0F

#define SAMPLES_PER_CHANNEL             1024*2
#define MAX_INPUT_CHANNELS_SUPPORTED    8
#define FACTOR_FOR_BUFFERING            2
#define STEREO_CHANNELS                 2
#define MAX_OUTPUT_CHANNELS_SUPPORTED   6
#define PCM_BLOCK_PER_CHANNEL_MS11      1536*2
#define AAC_BLOCK_PER_CHANNEL_MS11      768
#define NUMBER_BITS_IN_A_BYTE           8

#define PCM_2CH_OUT                 0
#define PCM_MCH_OUT                 1
#define SPDIF_OUT                   2

#ifndef ALSA_DEFAULT_SAMPLE_RATE
#define ALSA_DEFAULT_SAMPLE_RATE 44100 // in Hz
#endif

#define NORMAL_PLAYBACK_SESSION_ID 0
#define LPA_SESSION_ID 1
#define TUNNEL_SESSION_ID 2
#define MPQ_SESSION_ID 3

//Required for Tunnel
#define TUNNEL_DECODER_BUFFER_SIZE 4800
#define TUNNEL_DECODER_BUFFER_COUNT 256
#define SIGNAL_EVENT_THREAD 2
#define SIGNAL_PLAYBACK_THREAD 2
//Values to exit poll via eventfd
#define KILL_EVENT_THREAD 1
#define KILL_PLAYBACK_THREAD 1
#define KILL_CAPTURE_THREAD 1
#define NUM_FDS 3
#define AFE_PROXY_SAMPLE_RATE 48000
#define AFE_PROXY_CHANNEL_COUNT 2

static uint32_t FLUENCE_MODE_ENDFIRE   = 0;
static uint32_t FLUENCE_MODE_BROADSIDE = 1;
class ALSADevice;
class ALSAControl;

enum {
    INVALID_FORMAT               = -1,
    PCM_FORMAT                   = 0,
    COMPRESSED_FORMAT            = 1,
    COMPRESSED_FORCED_PCM_FORMAT = 2
};

struct alsa_handle_t {
    ALSADevice*         module;
    uint32_t            devices;
    char                useCase[MAX_STR_LEN];
    struct pcm *        handle;
    snd_pcm_format_t    format;
    uint32_t            channels;
    uint32_t            sampleRate;
    int                 mode;            // Phone state i.e. incall/normal/incommunication
    unsigned int        latency;         // Delay in usec
    unsigned int        bufferSize;      // Size of sample buffer
    unsigned int        periodSize;
    struct pcm *        rxHandle;
    uint32_t            activeDevice;
    snd_use_case_mgr_t  *ucMgr;
};

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

typedef List<alsa_handle_t> ALSAHandleList;

struct use_case_t {
    char                useCase[MAX_STR_LEN];
};
typedef List<use_case_t> ALSAUseCaseList;

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

    status_t    setPlaybackVolume(int, char *);
    status_t    setPlaybackFormat(const char *value, int device);
    status_t    setCaptureFormat(const char *value);
    status_t    setWMAParams(alsa_handle_t* , int[], int);
    int         getALSABufferSize(alsa_handle_t *handle);
    status_t    setHDMIChannelCount(int channels);
    void        switchDeviceUseCase(alsa_handle_t *handle, uint32_t devices,
                                            uint32_t mode);
    void        setDeviceList(ALSAHandleList *mDeviceList);
    void        setUseCase(alsa_handle_t *handle, bool bIsUseCaseSet);
    void        setUseCase(alsa_handle_t *handle, bool bIsUseCaseSet, char *device);
    status_t    openCapture(alsa_handle_t *handle, bool isMmapMode,
                            bool isCompressed);
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

private:
    int         deviceName(alsa_handle_t *handle, unsigned flags, char **value);
    status_t    setHardwareParams(alsa_handle_t *handle);
    status_t    setSoftwareParams(alsa_handle_t *handle);
    void        switchDevice(uint32_t devices, uint32_t mode);
    status_t    getMixerControl(const char *name, unsigned int &value, int index);
    status_t    setMixerControl(const char *name, unsigned int value, int index);
    status_t    setMixerControl(const char *name, const char *value);
    void        getDevices(uint32_t devices, uint32_t mode,
                                    char **rxDevice, char **txDevice);
    status_t    setCaptureHardwareParams(alsa_handle_t *handle, bool isCompressed);
    status_t    setCaptureSoftwareParams(alsa_handle_t *handle, bool isCompressed);

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
        struct snd_xferi mX;
        unsigned mAvail;
        struct pollfd mPfdProxy[NUM_FDS];
        long mFrames;
    };
    struct proxy_params mProxyParams;
};

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

    virtual status_t    setParameters(const String8& keyValuePairs) {
        return ALSAStreamOps::setParameters(keyValuePairs);
    }

    virtual String8     getParameters(const String8& keys) {
        return ALSAStreamOps::getParameters(keys);
    }

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
    Mutex               mLock;
    uint32_t            mFrameCount;
    uint32_t            mSampleRate;
    uint32_t            mChannels;
    size_t              mBufferSize;
    int                 mFormat;
    int                 mDevices;
    int                 mSecDevices;
    uint32_t            mStreamVol;
    bool                mPowerLock;
    bool                mRoutePcmAudio;
    bool                mRouteAudioToA2dp;
    bool                mUseTunnelDecoder;
    bool                mUseDualTunnel;
    bool                mCaptureFromProxy;
    bool                mUseMS11Decoder;
    uint32_t            mSessionId;
    size_t              mMinBytesReqToDecode;
    bool                mAacConfigDataSet;
    bool                mWMAConfigDataSet;
    unsigned char       mChannelStatus[24];
    bool                mChannelStatusSet;
    bool                mTunnelPaused;
    bool                mPaused;
    bool                mTunnelSeeking;
    bool                mReachedExtractorEOS;
    bool                mSkipWrite;
    char                mSpdifOutputFormat[128];
    char                mHdmiOutputFormat[128];
    uint32_t            mCurDevice;
    int32_t             mSpdifFormat;
    int32_t             mHdmiFormat;
    uint64_t            hw_ptr[2];

    AudioHardwareALSA  *mParent;
    alsa_handle_t *     mPcmRxHandle;
    alsa_handle_t *     mCompreRxHandle;
    alsa_handle_t *     mSecCompreRxHandle;
    ALSADevice *        mALSADevice;
    snd_use_case_mgr_t *mUcMgr;
    SoftMS11           *mMS11Decoder;
    AudioBitstreamSM   *mBitstreamSM;
    AudioEventObserver *mObserver;

    status_t            openPcmDevice(int devices);
    status_t            openDevice(char *pUseCase, bool bIsUseCase, int devices);
    status_t            closeDevice(alsa_handle_t *pDevice);
    status_t            doRouting(int devices);
    void                createThreadsForTunnelDecode();
    void                bufferAlloc(alsa_handle_t *handle);
    void                bufferDeAlloc();
    bool                isReadyToPostEOS(int errPoll, void *fd);
    status_t            drainTunnel();
    status_t            openTunnelDevice(int devices);
    // make sure the event thread also exited
    void                requestAndWaitForEventThreadExit();
    int32_t             writeToCompressedDriver(char *buffer, int bytes);
    static void *       eventThreadWrapper(void *me);
    void                eventThreadEntry();
    void                updateOutputFormat();
    void                updateRoutingFlags();
    void                setSpdifHdmiRoutingFlags(int devices);
    status_t            setPlaybackFormat();
    status_t            pause_l();
    status_t            resume_l();
    void                reset();
    uint32_t            channelMapToChannels(uint32_t channelMap);
    //Structure to hold mem buffer information
    class BuffersAllocated {
    public:
        BuffersAllocated(void *buf1, int32_t nSize) :
        memBuf(buf1), memBufsize(nSize), bytesToWrite(0)
        {}
        void* memBuf;
        int32_t memBufsize;
        uint32_t bytesToWrite;
    };
    List<BuffersAllocated> mInputMemEmptyQueue[2];
    List<BuffersAllocated> mInputMemFilledQueue[2];
    List<BuffersAllocated> mInputBufPool[2];
    int mBuffPoolInitDone;

    //Declare all the threads
    pthread_t mEventThread;

    //Declare the condition Variables and Mutex
    Mutex mInputMemRequestMutex;
    Mutex mInputMemResponseMutex;
    Mutex mEventMutex;

    Condition mWriteCv;
    Condition mEventCv;
    bool mKillEventThread;
    bool mEventThreadAlive;
    int mInputBufferSize;
    int mInputBufferCount;

    //event fd to signal the EOS and Kill from the userspace
    int mEfd;

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
    // Capture and Routing Flags
    bool                mCapturePCMFromDSP;
    bool                mCaptureCompressedFromDSP;
    bool                mRoutePCMStereoToDSP;
    bool                mUseMS11Decoder;
    bool                mUseTunnelDecoder;
    bool                mRoutePcmAudio;
    bool                mRoutingSetupDone;
    bool                mSignalToSetupRoutingPath;
    int                 mInputBufferSize;
    int                 mInputBufferCount;
    int32_t             mMinBytesReqToDecode;
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
    bool                mPlaybackReachedEOS;
    bool                isSessionPaused;

    bool                mRouteAudioToA2dp;
    bool                mCaptureFromProxy;

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
    // Open the MS11 decoder for AAC, AC3 and EC3
    SoftMS11           *mMS11Decoder;
    // Bitstream state machine to handle the buffering on input
    // and output
    AudioBitstreamSM   *mBitstreamSM;

    ALSADevice *        mALSADevice;
    snd_use_case_mgr_t *mUcMgr;
    AudioEventObserver *mObserver;

    Mutex               mRoutingSetupMutex;
    Condition           mRoutingSetupCv;

    //Declare all the threads
    //Capture
    pthread_t           mCaptureThread;
    Mutex               mCaptureMutex;
    Condition           mCaptureCv;
    struct              pollfd mCapturePfd[NUM_FDS];
    struct              snd_xferi mX;
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
    Mutex               mInputMemRequestMutex;
    Mutex               mInputMemResponseMutex;
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
    ssize_t             readFromCapturePath(char *buffer);
    void                resetCapturePathVariables();
    void                exitFromCaptureThread();
    uint32_t            read4BytesFromBuffer(char *buf);
    void                updateCaptureMetaData(char *buf);
    // Playback
    void                setRoutingFlagsBasedOnConfig();
    void                setSpdifHdmiRoutingFlags(int devices);
    status_t            setPlaybackFormat();
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
    int32_t             writeToCompressedDriver(char *buffer, int bytes);
    void                resetPlaybackPathVariables();
    void                exitFromPlaybackThread();

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
            audio_output_flags_t flags,
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


    uint32_t            mCurDevice;
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
    char                mSpdifOutputFormat[128];
    char                mHdmiOutputFormat[128];

    //A2DP variables
    audio_stream_out   *mA2dpStream;
    audio_hw_device_t  *mA2dpDevice;

    bool                mKillA2DPThread;
    bool                mA2dpThreadAlive;
    pthread_t           mA2dpThread;
    Mutex               mA2dpMutex;
    Condition           mA2dpCv;
    volatile bool       mIsA2DPEnabled;

    enum {
      A2DPNone = 0x0,
      A2DPHardwareOutput = 0x1,
      A2DPDirectOutput = 0x2,
      A2DPBroadcast = 0x4,
      A2DPVoip = 0x8,
      A2DPFm = 0x16,
    };
    uint32_t mA2DPActiveUseCases;

public:
    bool mRouteAudioToA2dp;
};

class AudioBitstreamSM
{
public:
    AudioBitstreamSM();
    ~AudioBitstreamSM();
    bool    initBitstreamPtr();
    void    resetBitstreamPtr();
    void    copyBitsreamToInternalBuffer(char *bufPtr, size_t bytes);
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
private:
    // Buffer pointers for input and output to MS11
    char               *ms11InputBuffer;
    char               *ms11InputBufferWritePtr;

    char               *ms11DDEncOutputBuffer;
    char               *ms11DDEncOutputBufferWritePtr;

    char               *ms11PCM2ChOutputBuffer;
    char               *ms11PCM2ChOutputBufferWritePtr;

    char               *ms11PCM6ChOutputBuffer;
    char               *ms11PCM6ChOutputBufferWritePtr;
};

// ----------------------------------------------------------------------------

};        // namespace android_audio_legacy
#endif    // ANDROID_AUDIO_HARDWARE_ALSA_H
