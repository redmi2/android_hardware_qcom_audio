ifeq ($(strip $(BOARD_USES_ALSA_AUDIO)),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm
LOCAL_CFLAGS := -D_POSIX_SOURCE
LOCAL_CFLAGS += -DQCOM_MPQ_BROADCAST

LOCAL_SRC_FILES := \
  AudioHardwareALSA.cpp 	\
  AudioBitstreamSM.cpp          \
  AudioStreamOutALSA.cpp 	\
  AudioBroadcastStream.cpp 	\
  AudioSessionOut.cpp 	\
  AudioStreamInALSA.cpp 	\
  ALSAStreamOps.cpp		\
  ALSADevice.cpp		\
  audio_hw_hal.cpp		\
  AudioUtil.cpp

LOCAL_STATIC_LIBRARIES := \
    libmedia_helper \
    libaudiohw_legacy

LOCAL_SHARED_LIBRARIES := \
    libacdbloader\
    libcutils \
    libutils \
    libhardware \
    libpower    \
    libalsa-intf \
    lib_dlb_msd \
    libaudioparsers

LOCAL_C_INCLUDES := $(TARGET_OUT_HEADERS)/mm-audio/audio-alsa
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audcal
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-acdb-util
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-codecs
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-parsers
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/libalsa-intf
LOCAL_C_INCLUDES += vendor/qcom/proprietary/mm-audio/audio-parsers/inc
LOCAL_C_INCLUDES += hardware/libhardware/include
LOCAL_C_INCLUDES += hardware/libhardware_legacy/include
LOCAL_C_INCLUDES += system/core/include
LOCAL_C_INCLUDES += vendor/qcom/proprietary/mm-audio/audio-codecs/ms11
LOCAL_C_INCLUDES += vendor/qcom/proprietary/mm-audio/audio-codecs/ms11/exec_layer

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_LDFLAGS := -ldl

LOCAL_MODULE := audio.primary.mpq8064
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)

LOCAL_CFLAGS := -D_POSIX_SOURCE

ifeq ($(BOARD_HAVE_BLUETOOTH),true)
  LOCAL_CFLAGS += -DWITH_A2DP
endif
ifeq (1,0)
LOCAL_SRC_FILES := \
    AudioPolicyManagerALSA.cpp	\
    audio_policy_hal.cpp

LOCAL_MODULE := audio_policy.mpq8064
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE_TAGS := optional

LOCAL_STATIC_LIBRARIES := \
    libmedia_helper \
    libaudiopolicy_legacy

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libmedia \
    libaudioparameter

LOCAL_C_INCLUDES := hardware/libhardware_legacy/audio

include $(BUILD_SHARED_LIBRARY)
endif

include $(call all-makefiles-under,$(LOCAL_PATH))
endif
