AUDIO_HW_ROOT := $(call my-dir)

ifeq ($(strip $(BOARD_USES_ALSA_AUDIO)),true)
    include $(AUDIO_HW_ROOT)/alsa_sound/Android.mk
endif

ifeq ($(strip $(BOARD_USES_ALSA_AUDIO)),true)
    include $(AUDIO_HW_ROOT)/libalsa-intf/Android.mk
endif
ifeq ($(call is-board-platform,msm7630_surf),true)
    include $(AUDIO_HW_ROOT)/msm7630/Android.mk
endif
