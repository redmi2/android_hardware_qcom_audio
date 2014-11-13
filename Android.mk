ifneq ($(filter msm8960 msm8226 msm8x26 msm8974 msm8x74,$(TARGET_BOARD_PLATFORM)),)

MY_LOCAL_PATH := $(call my-dir)

include $(MY_LOCAL_PATH)/hal/Android.mk
include $(MY_LOCAL_PATH)/policy_hal/Android.mk
endif
