/* extn_adsp_plugin.c
Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#define LOG_TAG "audio_ext_hw_plugin"
#define LOG_NDEBUG 0

#include <errno.h>
#include <pthread.h>
#include <dlfcn.h>
#include <cutils/log.h>
#include <audio_hw.h>
#include "audio_extn.h"
#include "platform_api.h"
#include "audio_hal_plugin.h"

#ifdef EXT_HW_PLUGIN_ENABLED

typedef int32_t (*audio_hal_plugin_init_t)(void);
typedef int32_t (*audio_hal_plugin_deinit_t)(void);
typedef int32_t (*audio_hal_plugin_send_msg_t)(audio_hal_plugin_msg_type_t,
                                           void *, uint32_t);

struct ext_hw_plugin_data {
    struct audio_device           *adev;
    void                          *plugin_handle;
    audio_hal_plugin_init_t        audio_hal_plugin_init;
    audio_hal_plugin_deinit_t      audio_hal_plugin_deinit;
    audio_hal_plugin_send_msg_t    audio_hal_plugin_send_msg;
};

/* This can be defined in platform specific file or use compile flag */
#define LIB_PLUGIN_DRIVER "libaudiohalplugin.so"

void* audio_extn_ext_hw_plugin_init(struct audio_device *adev)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = NULL;

    my_plugin = calloc(1, sizeof(struct ext_hw_plugin_data));

    if (my_plugin == NULL)
        return NULL;

    my_plugin->adev = adev;

    my_plugin->plugin_handle = dlopen(LIB_PLUGIN_DRIVER, RTLD_NOW);
    if (my_plugin->plugin_handle == NULL) {
        ALOGE("%s: DLOPEN failed for %s", __func__, LIB_PLUGIN_DRIVER);
        ret = -EINVAL;
    } else {
        ALOGV("%s: DLOPEN successful for %s", __func__, LIB_PLUGIN_DRIVER);
        my_plugin->audio_hal_plugin_init = (audio_hal_plugin_init_t)dlsym(
            my_plugin->plugin_handle, "audio_hal_plugin_init");
        if (!my_plugin->audio_hal_plugin_init) {
            ALOGE("%s: Could not find the symbol audio_hal_plugin_init from %s",
                  __func__, LIB_PLUGIN_DRIVER);
            goto plugin_init_fail;
        }

        my_plugin->audio_hal_plugin_deinit = (audio_hal_plugin_deinit_t)dlsym(
           my_plugin->plugin_handle, "audio_hal_plugin_deinit");
        if (!my_plugin->audio_hal_plugin_deinit) {
            ALOGE("%s: Could not find the symbol audio_hal_plugin_deinit from %s",
                  __func__, LIB_PLUGIN_DRIVER);
            goto plugin_init_fail;
        }

        my_plugin->audio_hal_plugin_send_msg = (audio_hal_plugin_send_msg_t)
            dlsym(my_plugin->plugin_handle, "audio_hal_plugin_send_msg");
        if (!my_plugin->audio_hal_plugin_send_msg) {
            ALOGE("%s: Could not find the symbol audio_hal_plugin_send_msg from %s",
                  __func__, LIB_PLUGIN_DRIVER);
            goto plugin_init_fail;
        }

        ret = my_plugin->audio_hal_plugin_init();
        if (ret) {
            ALOGE("%s: audio_hal_plugin_init failed with ret = %d",
               __func__, ret);
            goto plugin_init_fail;
        }
    }
    return my_plugin;

plugin_init_fail:
    dlclose(my_plugin->plugin_handle);
    free(my_plugin);
    return NULL;
}

int audio_extn_ext_hw_plugin_deinit(void *plugin)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if (my_plugin == NULL) {
        return -EINVAL;
    }

    if (my_plugin->audio_hal_plugin_deinit) {
        int ret = my_plugin->audio_hal_plugin_deinit();
        if (ret) {
            ALOGE("%s: audio_hal_plugin_deinit failed with ret = %d",
                  __func__, ret);
        }

        dlclose(my_plugin->plugin_handle);
    }
    free(my_plugin);
    return ret;
}

int audio_extn_ext_hw_plugin_usecase_start(void *plugin, struct audio_usecase *usecase)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if ((my_plugin == NULL) || (usecase == NULL)) {
        return -EINVAL;
    }

    if (my_plugin->audio_hal_plugin_send_msg) {
        audio_hal_plugin_msg_type_t msg = AUDIO_HAL_PLUGIN_MSG_CODEC_ENABLE;
        audio_hal_plugin_codec_enable_t codec_enable;

        codec_enable.usecase = usecase->id;
        if ((usecase->type == PCM_PLAYBACK) || (usecase->type == VOICE_CALL) ||
            (usecase->type == VOIP_CALL) || (usecase->type == PCM_HFP_CALL)) {
            codec_enable.snd_dev = usecase->out_snd_device;
            /* TODO - below should be related with out_snd_dev */
            codec_enable.sample_rate = 48000;
            codec_enable.bit_width = 16;
            codec_enable.num_chs = 2;

            ALOGD("%s: enable audio hal plugin output, %d, %d, %d, %d, %d",
                __func__, (int)codec_enable.usecase,
                (int)codec_enable.snd_dev,
                (int)codec_enable.sample_rate,
                (int)codec_enable.bit_width,
                (int)codec_enable.num_chs);

            ret = my_plugin->audio_hal_plugin_send_msg(msg,
                (void*)&codec_enable, sizeof(codec_enable));
            if (ret) {
                ALOGE("%s: enable audio hal plugin output failed ret = %d",
                    __func__, ret);
                return ret;
            }
        }
        if ((usecase->type == PCM_CAPTURE) || (usecase->type == VOICE_CALL) ||
            (usecase->type == VOIP_CALL) || (usecase->type == PCM_HFP_CALL)) {
            codec_enable.snd_dev = usecase->in_snd_device;
            /* TODO - below should be related with in_snd_dev */
            codec_enable.sample_rate = 48000;
            codec_enable.bit_width = 16;
            codec_enable.num_chs = 2;

            ALOGD("%s: enable audio hal plugin input, %d, %d, %d, %d, %d",
                __func__, (int)codec_enable.usecase,
                (int)codec_enable.snd_dev,
                (int)codec_enable.sample_rate,
                (int)codec_enable.bit_width,
                (int)codec_enable.num_chs);

            ret = my_plugin->audio_hal_plugin_send_msg(msg,
                (void*)&codec_enable, sizeof(codec_enable));
            if (ret) {
                ALOGE("%s: enable audio hal plugin input failed ret = %d",
                    __func__, ret);
                return ret;
            }
        }
    }

    ALOGD("%s: finished ext_hw_plugin usecase start", __func__);

    return ret;
}

int audio_extn_ext_hw_plugin_usecase_stop(void *plugin, struct audio_usecase *usecase)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if ((my_plugin == NULL) || (usecase == NULL)) {
        return -EINVAL;
    }

    if (my_plugin->audio_hal_plugin_send_msg) {
        audio_hal_plugin_msg_type_t msg = AUDIO_HAL_PLUGIN_MSG_CODEC_DISABLE;
        audio_hal_plugin_codec_disable_t codec_disable;

        codec_disable.usecase = usecase->id;
        if ((usecase->type == PCM_PLAYBACK) || (usecase->type == VOICE_CALL) ||
            (usecase->type == VOIP_CALL) || (usecase->type == PCM_HFP_CALL)) {
            codec_disable.snd_dev = usecase->out_snd_device;

            ALOGD("%s: disable audio hal plugin output, %d, %d",
                __func__, (int)codec_disable.usecase,
                (int)codec_disable.snd_dev);

            ret= my_plugin->audio_hal_plugin_send_msg(msg,
                (void*)&codec_disable, sizeof(codec_disable));
            if (ret) {
                ALOGE("%s: disable audio hal plugin output failed ret = %d",
                    __func__, ret);
            }
        }
        if ((usecase->type == PCM_CAPTURE) || (usecase->type == VOICE_CALL) ||
        (usecase->type == VOIP_CALL) || (usecase->type == PCM_HFP_CALL)) {
            codec_disable.snd_dev = usecase->in_snd_device;

            ALOGD("%s: disable audio hal plugin input, %d, %d",
                __func__, (int)codec_disable.usecase,
                (int)codec_disable.snd_dev);

            ret= my_plugin->audio_hal_plugin_send_msg(msg,
                (void*)&codec_disable, sizeof(codec_disable));
            if (ret) {
                ALOGE("%s: disable audio hal plugin input failed ret = %d",
                    __func__, ret);
            }
        }
    }

    ALOGD("%s: finished ext_hw_plugin usecase stop", __func__);

    return ret;
}

static int ext_hw_plugin_string_to_dword(char *string_value, void **dword_ptr,
        uint32_t dword_len)
{
    int ret = 0;
    uint32_t i,tmp;
    uint8_t *dptr = NULL;
    uint8_t *tmpptr = NULL;
    int32_t dlen;
    uint32_t *outptr = NULL;

    dlen = strlen(string_value);
    if (dlen <= 0) {
        ALOGE("%s: NULL data received", __func__);
        return -EINVAL;
    }
    dptr = (uint8_t*) calloc(dlen, sizeof(uint8_t));
    if (dptr == NULL) {
        ALOGE("%s: memory allocation failed", __func__);
        return -ENOMEM;
    }
    dlen = b64decode(string_value, strlen(string_value), dptr);
    if ((dlen <= 0) || ((uint32_t)dlen != 4*dword_len)){
        ALOGE("%s: data decoding failed", __func__);
        ret = -EINVAL;
        goto done_string_to_dword;
    }

    outptr = calloc(dword_len, sizeof(uint32_t));
    if (outptr == NULL) {
        ALOGE("%s: memory allocation failed", __func__);
        ret = -ENOMEM;
        goto done_string_to_dword;
    }

    for(i=0; i<dword_len; i++) {
        tmpptr = dptr+4*i;
        tmp = (uint32_t) *(tmpptr);
        tmp |= ((uint32_t) *(tmpptr+1))<<8;
        tmp |= ((uint32_t) *(tmpptr+2))<<16;
        tmp |= ((uint32_t) *(tmpptr+3))<<24;
        *(outptr + i) = tmp;
    }
    *dword_ptr = (void*)outptr;

done_string_to_dword:
    if (dptr != NULL)
        free(dptr);

    return ret;
}

int audio_extn_ext_hw_plugin_set_parameters(void *plugin, struct str_parms *parms)
{
    char *value=NULL;
    int val, len;
    int ret = 0, err;
    char *kv_pairs = str_parms_to_str(parms);
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if (my_plugin == NULL) {
        ret = -EINVAL;
        goto done;
    }

    if (!my_plugin->audio_hal_plugin_send_msg) {
        ALOGE("%s: NULL audio_hal_plugin_send_msg func ptr", __func__);
        ret = -EINVAL;
        goto done;
    }

    err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MSG_TYPE, &val);
    if (err < 0) {
        ALOGE("%s: Invalid or missing TYPE param for plugin msg", __func__);
        ret = -EINVAL;
        goto done;
    }
    ALOGD("%s: received plugin msg type (%d)", __func__, val);
    str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MSG_TYPE);

    len = strlen(kv_pairs);
    value = (char*)calloc(len, sizeof(char));
    if (value == NULL) {
        ret = -ENOMEM;
        ALOGE("[%s] failed to allocate memory",__func__);
        goto done;
    }

    if (val == AUDIO_HAL_PLUGIN_MSG_CODEC_TUNNEL_CMD) {
        uint32_t i, plsize;
        int32_t *plptr = NULL;

        ALOGE("%s: enter TUNNEL command handling", __func__);

        err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_TUNNEL_SIZE,
                (int*)&plsize);
        if ((err < 0) || (!plsize)) {
            ALOGE("%s: Invalid or missing size param for TUNNEL command", __func__);
            ret = -EINVAL;
            goto done;
        }
        str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_TUNNEL_SIZE);

        err = str_parms_get_str(parms,
                AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_TUNNEL_DATA, value, len);
        if (err < 0) {
            ALOGE("%s: Invalid or missing band_data for TUNNEL command", __func__);
            ret = -EINVAL;
            goto done;
        }
        str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_TUNNEL_DATA);

        ret = ext_hw_plugin_string_to_dword(value, (void**)&plptr, plsize);
        if (ret) {
            ALOGE("%s: Failed to parse payload for TUNNEL command", __func__);
            ret = -EINVAL;
            goto done_tunnel;
        }

        audio_hal_plugin_msg_type_t msg =
                AUDIO_HAL_PLUGIN_MSG_CODEC_TUNNEL_CMD;

        ALOGD("%s: sending codec tunnel cmd msg to HAL plugin driver,size = %d",
                __func__, (int)plsize);

        ret = my_plugin->audio_hal_plugin_send_msg(msg, (void*)plptr, plsize);
        if (ret) {
            ALOGE("%s: Failed to send plugin tunnel cmd err: %d", __func__, ret);
        }

done_tunnel:
        if (plptr!= NULL)
            free(plptr);
    } else {
        audio_usecase_t use_case;
        snd_device_t snd_dev;

        err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_UC,
                &use_case);
        if ((err < 0) || (use_case < 0) || (use_case >= AUDIO_USECASE_MAX)) {
            ALOGE("%s: Invalid or missing usecase param for plugin msg", __func__);
            ret = -EINVAL;
            /* TODO: do we need to support no use case in kvpair? */
            goto done;
        }
        str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_UC);

        struct audio_usecase *uc_info;
        uc_info = get_usecase_from_list(my_plugin->adev, use_case);
        if (uc_info == NULL) {
            ALOGE("%s: Could not find the usecase (%d) in the list",
                    __func__, use_case);
            /* TODO: do we need to cache the setting when usecase is not active? */
            ret = -EINVAL;
            goto done;
        }
        /* TODO: confirm this handles all usecase */
        if (uc_info->out_snd_device) {
            snd_dev = uc_info->out_snd_device;
        } else if (uc_info->in_snd_device) {
            snd_dev = uc_info->in_snd_device;
        } else {
            ALOGE("%s: No valid snd_device found for the usecase (%d)",
                    __func__, use_case);
            ret = -EINVAL;
            goto done;
        }

        switch(val) {
        case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_VOLUME:
        {
            audio_hal_plugin_codec_set_pp_vol_t pp_vol;
            memset(&pp_vol,0,sizeof(pp_vol));
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK,
                    (int*)&pp_vol.ch_mask);
            if ((err < 0)) {
                /* TODO: properly handle no cmask param from client case */
                ALOGE("%s: Invalid or missing CMASK param for SET_PP_VOLUME", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK);
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_GAIN,
                    (int*)&pp_vol.gain);
            if ((err < 0)) {
                /* TODO: properly handle no gain param from client case */
                ALOGE("%s: Invalid or missing GAIN param for SET_PP_VOLUME", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_GAIN);

            audio_hal_plugin_msg_type_t msg =
                    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_VOLUME;
            pp_vol.usecase = use_case;
            pp_vol.snd_dev= snd_dev;

            ALOGD("%s: sending codec pp vol msg to HAL plugin driver, %d, %d, %d, %d",
                    __func__, (int)pp_vol.usecase, (int)pp_vol.snd_dev,
                    (int)pp_vol.ch_mask, (int)pp_vol.gain);

            ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_vol, sizeof(pp_vol));
            if (ret) {
                ALOGE("%s: Failed to set plugin pp vol err: %d", __func__, ret);
            }
            break;
        }
        case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_MUTE:
        {
            int32_t flag;
            audio_hal_plugin_codec_set_pp_mute_t pp_mute;
            memset(&pp_mute,0,sizeof(pp_mute));
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK,
                    (int*)&pp_mute.ch_mask);
            if ((err < 0)) {
                /* TODO: properly handle no cmask param from client case */
                ALOGE("%s: Invalid or missing CMASK param for SET_PP_MUTE", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK);
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MUTE_FLAG,
                    (int*)&flag);
            if ((err < 0)) {
                ALOGE("%s: Invalid or missing FLAG param for SET_PP_MUTE", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MUTE_FLAG);
            pp_mute.flag = (bool)flag;

            audio_hal_plugin_msg_type_t msg =
                    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_MUTE;
            pp_mute.usecase = use_case;
            pp_mute.snd_dev= snd_dev;

            ALOGD("%s: sending codec pp mute msg to HAL plugin driver, %d, %d, %d, %d",
                    __func__, (int)pp_mute.usecase, (int)pp_mute.snd_dev,
                    (int)pp_mute.ch_mask, (int)pp_mute.flag);

            ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_mute,
                    sizeof(pp_mute));
            if (ret) {
                ALOGE("%s: Failed to set plugin pp vol err: %d", __func__, ret);
            }
            break;
        }
        case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_FADE:
        {
            audio_hal_plugin_codec_set_pp_fade_t pp_fade;
            memset(&pp_fade,0,sizeof(pp_fade));
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_FADE,
                    (int*)&pp_fade.fade);
            if ((err < 0)) {
                ALOGE("%s: Invalid or missing FADE param for SET_PP_FADE", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_FADE);

            audio_hal_plugin_msg_type_t msg =
                        AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_FADE;
            pp_fade.usecase = use_case;
            pp_fade.snd_dev= snd_dev;

            ALOGD("%s: sending codec pp fade msg to HAL plugin driver, %d, %d, %d",
                    __func__, (int)pp_fade.usecase, (int)pp_fade.snd_dev,
                    (int)pp_fade.fade);

            ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_fade,
                    sizeof(pp_fade));
            if (ret) {
                ALOGE("%s: Failed to set plugin pp fade err: %d", __func__, ret);
            }
            break;
        }
        case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BALANCE:
        {
            audio_hal_plugin_codec_set_pp_balance_t pp_balance;
            memset(&pp_balance,0,sizeof(pp_balance));
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BALANCE,
                    (int*)&pp_balance.balance);
            if ((err < 0)) {
                ALOGE("%s: Invalid or missing balance param for SET_PP_BALANCE", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BALANCE);

            audio_hal_plugin_msg_type_t msg =
                    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BALANCE;
            pp_balance.usecase = use_case;
            pp_balance.snd_dev= snd_dev;

            ALOGD("%s: sending codec pp balance msg to HAL plugin driver, %d, %d, %d",
                    __func__, (int)pp_balance.usecase, (int)pp_balance.snd_dev,
                    (int)pp_balance.balance);

            ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_balance,
                    sizeof(pp_balance));
            if (ret) {
                ALOGE("%s: Failed to set plugin pp balance err: %d", __func__, ret);
            }
            break;
        }
        case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BMT:
        {
            int32_t filter_type, enable_flag;
            audio_hal_plugin_codec_set_pp_bmt_t pp_bmt;
            memset(&pp_bmt,0,sizeof(pp_bmt));
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BMT_FTYPE,
                    (int*)&filter_type);
            if ((err < 0)) {
                ALOGE("%s: Invalid or missing filter type param for SET_PP_BMT", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BMT_FTYPE);
            if ((filter_type <= AUDIO_HAL_PLUGIN_CODEC_PP_FILTER_TYPE_INVALID) &&
                    (filter_type >= AUDIO_HAL_PLUGIN_CODEC_PP_FILTER_TYPE_MAX)) {
                ALOGE("%s: Invalid filter type value for SET_PP_BMT", __func__);
                ret = -EINVAL;
                goto done;
            }
            pp_bmt.filter_type = filter_type;
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BMT_FLAG,
                    (int*)&enable_flag);
            if ((err < 0)) {
                ALOGE("%s: Invalid or missing enable flag param for SET_PP_BMT", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BMT_FLAG);
            pp_bmt.enable_flag = (bool)enable_flag;
            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BMT_VAL,
                    (int*)&pp_bmt.value);
            if ((err < 0)) {
                ALOGE("%s: Invalid or missing value param for SET_PP_BMT", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_BMT_VAL);

            audio_hal_plugin_msg_type_t msg =
                    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BMT;
            pp_bmt.usecase = use_case;
            pp_bmt.snd_dev= snd_dev;

            ALOGD("%s: sending codec pp bmt msg to HAL plugin driver,%d,%d,%d,%d,%d",
                    __func__, (int)pp_bmt.usecase, (int)pp_bmt.snd_dev,
                    (int)pp_bmt.filter_type, (int)pp_bmt.enable_flag,
                    (int)pp_bmt.value);

            ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_bmt,
                    sizeof(pp_bmt));
            if (ret) {
                ALOGE("%s: Failed to set plugin pp bmt err: %d", __func__, ret);
            }
            break;
        }
        case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_EQ:
        {
            int32_t enable_flag, dlen;
            audio_hal_plugin_codec_set_pp_eq_t pp_eq;
            memset(&pp_eq,0,sizeof(pp_eq));

            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_FLAG,
                    (int*)&enable_flag);
            if (err < 0) {
                ALOGE("%s: Invalid or missing enable flag param for SET_PP_EQ", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_FLAG);
            pp_eq.enable_flag = (bool)enable_flag;

            err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_ID,
                    (int*)&pp_eq.preset_id);
            if ((err < 0) || (pp_eq.preset_id < -1)) {
                ALOGE("%s: Invalid or missing preset_id param for SET_PP_EQ", __func__);
                ret = -EINVAL;
                goto done;
            }
            str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_ID);

            if (pp_eq.preset_id == -1) {
                err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_NUM_BANDS,
                        (int*)&pp_eq.num_bands);
                if ((err < 0) || (!pp_eq.num_bands)) {
                    ALOGE("%s: Invalid or missing num_bands param for SET_PP_EQ", __func__);
                    ret = -EINVAL;
                    goto done;
                }
                str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_NUM_BANDS);

                err = str_parms_get_str(parms,
                        AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_BAND_DATA, value, len);
                if (err < 0) {
                    ALOGE("%s: Invalid or missing band_data for SET_PP_EQ", __func__);
                    ret = -EINVAL;
                    goto done;
                }
                str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_EQ_BAND_DATA);

                ret = ext_hw_plugin_string_to_dword(value, (void**)&pp_eq.bands,
                        3*pp_eq.num_bands);
                if (ret) {
                    ALOGE("%s: Failed to parse band info for SET_PP_EQ", __func__);
                    ret = -EINVAL;
                    goto done_eq;
                }
            }

            audio_hal_plugin_msg_type_t msg =
                    AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_EQ;
            pp_eq.usecase = use_case;
            pp_eq.snd_dev= snd_dev;

            ALOGD("%s: sending codec pp eq msg to HAL plugin driver,%d,%d,%d,%d,%d",
                    __func__, (int)pp_eq.usecase, (int)pp_eq.snd_dev,
                    (int)pp_eq.enable_flag, (int)pp_eq.preset_id,
                    (int)pp_eq.num_bands);

            ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_eq, sizeof(pp_eq));
            if (ret) {
                ALOGE("%s: Failed to set plugin pp eq err: %d", __func__, ret);
            }

done_eq:
            if (pp_eq.bands != NULL)
                free(pp_eq.bands);
            break;
        }
        default:
            ALOGE("%s: Invalid plugin message type: %d", __func__, val);
            ret = -EINVAL;
        }
    }

done:
    ALOGV("%s: exit with code(%d)", __func__, ret);
    free(kv_pairs);
    if(value != NULL)
        free(value);
    return ret;
}
#endif /* EXT_HW_PLUGIN_ENABLED */
