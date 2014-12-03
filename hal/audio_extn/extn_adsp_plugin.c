/* extn_adsp_plugin.c
Copyright (c) 2014, The Linux Foundation. All rights reserved.

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

#define LOG_TAG "audio_hw_dsp_plugin"
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
#define LIB_PLUGIN_DRIVER "/system/lib/libaudiohalplugin.so"

void* audio_extn_ext_hw_plugin_init(struct audio_device *adev)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = NULL;

    my_plugin = calloc(1, sizeof(struct ext_hw_plugin_data));

    if(my_plugin == NULL)
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
        if(ret) {
            ALOGE("%s: audio_hal_plugin_init failed with ret = %d",
               __func__, ret);
	    goto plugin_init_fail;
        }
    }
    return my_plugin;

plugin_init_fail:
    dlclose(my_plugin->plugin_handle);
    return NULL;
}

int audio_extn_ext_hw_plugin_deinit(void *plugin)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if (my_plugin == NULL) {
        return -EINVAL;
    }

    if(my_plugin->audio_hal_plugin_deinit) {
        int ret = my_plugin->audio_hal_plugin_deinit();
        if(ret) {
            ALOGE("%s: audio_hal_plugin_deinit failed with ret = %d",
                  __func__, ret);
        }

        dlclose(my_plugin->plugin_handle);
    }
    free(my_plugin);
    return ret;
}

int audio_extn_ext_hw_plugin_enable(void *plugin, struct stream_out *out, bool enable)
{
    int ret = 0;
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if (my_plugin == NULL) {
        return -EINVAL;
    }

    if(my_plugin->audio_hal_plugin_send_msg) {
        if(enable == true) {
            audio_hal_plugin_msg_type_t msg = AUDIO_HAL_PLUGIN_MSG_CODEC_ENABLE;
            audio_hal_plugin_codec_enable_t codec_enable;

            /* TODO: populate the codec enable structure when CDC driver is ready */
            ALOGD("%s: sending codec enable msg to HAL plugin driver", __func__);

            ret = my_plugin->audio_hal_plugin_send_msg(msg,
                (void*)&codec_enable, sizeof(codec_enable));
            if(ret) {
                ALOGE("%s: AUDIO_HAL_PLUGIN_MSG_CODEC_ENABLE failed with ret = %d",
                     __func__, ret);
            }
        } else {
            audio_hal_plugin_msg_type_t msg = AUDIO_HAL_PLUGIN_MSG_CODEC_DISABLE;
            audio_hal_plugin_codec_disable_t codec_disable;

            /* TODO: populate the codec enable structure when CDC driver is ready */
            ALOGD("%s: sending codec disable msg to HAL plugin driver", __func__);
            ret= my_plugin->audio_hal_plugin_send_msg(msg,
                (void*)&codec_disable, sizeof(codec_disable));
            if(ret) {
                ALOGE("%s: AUDIO_HAL_PLUGIN_MSG_CODEC_DISABLE failed with ret = %d",
                     __func__, ret);
            }
        }
    }

    ALOGD("%s: finished sending msg to audio HAL plugin driver", __func__);

    return ret;
}


int audio_extn_ext_hw_plugin_set_parameters(void *plugin, struct str_parms *parms)
{
    int val;
    int ret = 0, err;
    char *kv_pairs = str_parms_to_str(parms);
    struct ext_hw_plugin_data *my_plugin = (struct ext_hw_plugin_data *)plugin;

    if (my_plugin == NULL) {
        return -EINVAL;
    }

    err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MSG_TYPE, &val);
    if(err >= 0) {
        ALOGD("%s: received plugin msg type (%d)", __func__, val);
        str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_MSG_TYPE);
        if(my_plugin->audio_hal_plugin_send_msg) {
            switch(val) {
            case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_VOLUME:
            {
                audio_hal_plugin_codec_set_pp_vol_t pp_vol;
                err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_UC,
                    &pp_vol.usecase);
                if(err >= 0) {
                    str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_UC);
                } else {
                    /* TODO: properly handle no uc param from client case */
                    return -EINVAL;
                }
                err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK,
                   (int*)&pp_vol.ch_mask);
                if((err >= 0)) {
                    str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_CMASK);
                } else {
                    /* TODO: properly handle no cmask param from client case */
                    return -EINVAL;
                }
                err = str_parms_get_int(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_GAIN,
                    (int*)&pp_vol.gain);
                if((err >= 0)) {
                    str_parms_del(parms, AUDIO_PARAMETER_KEY_EXT_HW_PLUGIN_GAIN);
                } else {
                    /* TODO: properly handle no gain param from client case */
                    return -EINVAL;
                }
                if((pp_vol.usecase>= 0)&&(pp_vol.usecase < AUDIO_USECASE_MAX)
                    &&(pp_vol.ch_mask)) {
                    struct audio_usecase *uc_info;
                    audio_hal_plugin_msg_type_t msg =
                        AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_VOLUME;
                    uc_info = get_usecase_from_list(my_plugin->adev, pp_vol.usecase);
                    if (uc_info == NULL) {
                        ALOGE("%s: Could not find the usecase (%d) in the list",
                            __func__, pp_vol.usecase);
                        /* TODO: cache the setting and apply when the usecase is active */
                        return -EINVAL;
                    }
                    /* TODO: confirm this handles all usecase */
                    if(uc_info->out_snd_device) {
                        pp_vol.snd_dev = uc_info->out_snd_device;
                    } else if(uc_info->in_snd_device) {
                        pp_vol.snd_dev = uc_info->in_snd_device;
                    } else {
                        ALOGE("%s: No valid snd_device found for the usecase (%d)",
                            __func__, pp_vol.usecase);
                        return -EINVAL;
                    }

                    ALOGD("%s: sending codec pp vol msg to HAL plugin driver, %d, %d, %d, %d",
                        __func__, (int)pp_vol.usecase, (int)pp_vol.snd_dev,
                        (int)pp_vol.ch_mask, (int)pp_vol.gain);

                    ret = my_plugin->audio_hal_plugin_send_msg(msg, &pp_vol,
                        sizeof(pp_vol));
                    if (ret) {
                        ALOGE("%s: Failed to set plugin pp vol err: %d", __func__, ret);
                    }
                }
                break;
            }
            case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_MUTE:
                /* TODO: */
                break;
            case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_FADE:
                /* TODO: */
                break;
            case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BALANCE:
                /* TODO:  */
                break;
            case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_BMT:
                /* TODO:  */
                break;
            case AUDIO_HAL_PLUGIN_MSG_CODEC_SET_PP_EQ:
                /* TODO:  */
                break;
            case AUDIO_HAL_PLUGIN_MSG_CODEC_TUNNEL_CMD:
                /* TODO:  */
                break;
            default:
                ALOGE("%s: Invalid plugin message type: %d", __func__, val);
                return -EINVAL;
            }
        }
    }
    return ret;
}
#endif /* EXT_HW_PLUGIN_ENABLED */
