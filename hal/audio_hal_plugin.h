/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef AUDIO_HAL_PLUGIN_H
#define AUDIO_HAL_PLUGIN_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <audio_hw.h>

#define AUDIO_HAL_PLUGIN_EOK (0)
#define AUDIO_HAL_PLUGIN_EFAIL (-1) /**< Undefined error */
#define AUDIO_HAL_PLUGIN_ENOMEM (-2) /**< Out of memory */
#define AUDIO_HAL_PLUGIN_EINVAL (-3) /**< Invalid argument */
#define AUDIO_HAL_PLUGIN_EBUSY (-4) /**< Plugin driver is busy */
#define AUDIO_HAL_PLUGIN_ENODEV (-5) /**< No device */
#define AUDIO_HAL_PLUGIN_EALREADY (-6) /**< Already done */

#define AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MSG_TYPE  "audio_hal_plugin_msg_type"
#define AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_UC        "audio_hal_plugin_usecase"
#define AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK     "audio_hal_plugin_channel_mask"
#define AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_GAIN      "audio_hal_plugin_gain"
/**
 * Type of audio hal plug-in messages
 */
typedef enum
{
    AUDIO_HAL_PLUGIN_MSG_INVALID = 0,
    AUDIO_HAL_PLUGIN_MSG_CODEC_ENABLE, /**< setup codec routing path */
    AUDIO_HAL_PLUGIN_MSG_CODEC_DISABLE, /**< tear down routing path */
    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_VOLUME, /**< set volume */
    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_MUTE, /**< mute/unmute control */
    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_FADE, /**< fade out control */
    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BALANCE, /**< left/right balance control */
    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BMT, /**< base/mid/treble control */
    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_EQ, /**< EQ control */
    AUDIO_HAL_PLUGIN_MSG_CODEC_TUNNEL_CMD, /**< pass through cmds */
    AUDIO_HAL_PLUGIN_MSG_MAX
} audio_hal_plugin_msg_type_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_ENABLE message
 */
typedef struct audio_hal_plugin_codec_enable
{
    snd_device_t snd_dev;  /**< Requested endpoint device to be enabled. @enum: SND_DEVICE_XXX */
    audio_usecase_t usecase;  /**< Requested audio use case. @enum: USECASE_AUDIO_XXX */
    uint32_t sample_rate;  /**< Requested sample rate for the endpoint device */
    uint32_t bit_width;  /**< Requested bit width per sample for the endpoint device */
    uint32_t num_chs;  /**< Requested number of channels for the endpoint device */
}audio_hal_plugin_codec_enable_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_DISABLE message
 */
typedef struct audio_hal_plugin_codec_disable
{
    snd_device_t snd_dev; /**< Requested the endpoint device to be disabled */
    audio_usecase_t usecase; /**< Requested audio use case */
}audio_hal_plugin_codec_disable_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_VOLUME message
 */
typedef struct audio_hal_plugin_codec_set_pp_vol
{
    snd_device_t snd_dev; /**< The requested the endpoint device */
    audio_usecase_t usecase; /**< Requested audio use case */
    audio_channel_mask_t ch_mask; /**< Requested audio channel mask */
    uint32_t gain; /**< The requested volume setting. Scale from 0 to 100 */
}audio_hal_plugin_codec_set_pp_vol_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_MUTE message
 */
typedef struct audio_hal_plugin_codec_set_pp_mute
{
    snd_device_t snd_dev; /**< The requested endpoint device */
    audio_usecase_t usecase; /**< Requested audio use case */
    audio_channel_mask_t ch_mask; /**< Requested audio channel mask */
    bool flag; /**< Enable/Disable mute flag. 1: mute, 0: unmute */
}audio_hal_plugin_codec_set_pp_mute_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_FADE message
 */
typedef struct audio_hal_plugin_codec_set_pp_fade
{
    snd_device_t snd_dev; /**< The requested endpoint device */
    audio_usecase_t usecase; /**< Requested audio use case */
    uint32_t fade; /**< The requested fade configuration. Scale range is 0 to 100
                                 0 - Refers to maximum at the rear and minimum at the front
                                 100 - Refers to minimum at the rear and maximum at the front */
}audio_hal_plugin_codec_set_pp_fade_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BALANCE message
 */
typedef struct audio_hal_plugin_codec_set_pp_balance
{
    snd_device_t snd_dev; /**< The requested endpoint device */
    audio_usecase_t usecase; /**< Requested audio use case */
    uint32_t balance; /**< The requested balance configuration. Scale range is 0 to 100
                                      0 - Refers to maximum at the left side and minimum at the right side
                                     100 - Refers to minimum at the left side and maximum at the right side */
}audio_hal_plugin_codec_set_pp_balance_t;

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BMT message -TBD
 */

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_EQ message -TBD
 */

/**
 * Payload of AUDIO_HAL_PLUGIN_MSG_CODEC_TUNNEL_CMD message -TBD
 */

/**
 * Initialize the audio hal plug-in module and underlying hw driver
 * One time call at audio hal boot up time
 */
int32_t audio_hal_plugin_init (void);

/**
 * De-Initialize the audio hal plug-in module and underlying hw driver
 * One time call when audio hal get unloaded from system
 */
int32_t audio_hal_plugin_deinit (void);

/**
 * Function to invoke the underlying HW driver realizing the functionality for a given use case.
 */
int32_t audio_hal_plugin_send_msg (
             audio_hal_plugin_msg_type_t msg,
             void * payload, uint32_t payload_size);

#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif
