# hardware/libaudio-alsa/Android.mk
#
# Copyright 2008 Wind River Systems
#
# This file was modified by Dolby Laboratories, Inc. The portions of the
# code that are surrounded by "DOLBY..." are copyrighted and
# licensed separately, as follows:
#
#  (C) 2012 Dolby Laboratories, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

ifeq ($(strip $(BOARD_USES_ALSA_AUDIO)),true)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm
LOCAL_CFLAGS := -D_POSIX_SOURCE
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_ACDB)),true)
    LOCAL_CFLAGS += -DQCOM_ACDB_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_ANC_HEADSET)),true)
    LOCAL_CFLAGS += -DQCOM_ANC_HEADSET_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_AUDIO_FORMAT)),true)
    LOCAL_CFLAGS += -DQCOM_AUDIO_FORMAT_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_CSD_CLIENT)),true)
    LOCAL_CFLAGS += -DQCOM_CSDCLIENT_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_FM)),true)
    LOCAL_CFLAGS += -DQCOM_FM_ENABLED
    ifeq ($(PLATFORM_SDK_VERSION),18)
        LOCAL_CFLAGS += -DQCOM_FM_V2_ENABLED
    endif
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_PROXY_DEVICE)),true)
    LOCAL_CFLAGS += -DQCOM_PROXY_DEVICE_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_OUTPUT_FLAGS)),true)
    LOCAL_CFLAGS += -DQCOM_OUTPUT_FLAGS_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_SSR)),true)
    LOCAL_CFLAGS += -DQCOM_SSR_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_USBAUDIO)),true)
    LOCAL_CFLAGS += -DQCOM_USBAUDIO_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_WFD)),true)
    LOCAL_CFLAGS += -DQCOM_WFD_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_ADSP_SSR)),true)
    LOCAL_CFLAGS += -DQCOM_ADSP_SSR_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_FLUENCE)),true)
    LOCAL_CFLAGS += -DQCOM_FLUENCE_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_TUNNEL_LPA)),true)
    LOCAL_CFLAGS += -DQCOM_TUNNEL_LPA_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_MULTICHANNELS)),true)
    LOCAL_CFLAGS += -DQCOM_AUDIO_MULTICHANNELS_ENABLED
endif

ifeq ($(call is-board-platform-in-list,msm8974 msm8226 msm8610),true)
  LOCAL_CFLAGS += -DTARGET_B_FAMILY
  ifdef DOLBY_DAP
    LOCAL_CFLAGS += -DQCOM_DS1_DOLBY_DAP
    LOCAL_CFLAGS += -DQCOM_DS1_DOLBY_DDP
  endif
  ifeq ($(QCOM_LISTEN_FEATURE),true)
    LOCAL_CFLAGS += -DQCOM_LISTEN_FEATURE_ENABLE
  endif
endif

ifneq ($(ALSA_DEFAULT_SAMPLE_RATE),)
    LOCAL_CFLAGS += -DALSA_DEFAULT_SAMPLE_RATE=$(ALSA_DEFAULT_SAMPLE_RATE)
endif

#Do not use Dual MIC scenario in call feature
#Dual MIC solution(Fluence) feature in Built-in MIC used scenarioes.
# 1. Handset
# 2. 3-Pole Headphones
#ifeq ($(strip $(BOARD_USES_FLUENCE_INCALL)),true)
#LOCAL_CFLAGS += -DUSES_FLUENCE_INCALL
#endif

#Do not use separate audio Input path feature
#Separate audio input path can be set using input source of audio parameter
# 1. Voice Recognition
# 2. Camcording
# 3. etc.
#ifeq ($(strip $(BOARD_USES_SEPERATED_AUDIO_INPUT)),true)
#LOCAL_CFLAGS += -DSEPERATED_AUDIO_INPUT
#endif

LOCAL_SRC_FILES := \
  AudioHardwareALSA.cpp         \
  AudioStreamOutALSA.cpp        \
  AudioStreamInALSA.cpp         \
  ALSAStreamOps.cpp             \
  audio_hw_hal.cpp              \
  AudioUsbALSA.cpp              \
  AudioUtil.cpp                 \
  ALSADevice.cpp                \
  AudioSpeakerProtection.cpp

ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_TUNNEL_LPA)),true)
    LOCAL_SRC_FILES +=AudioSessionOut.cpp
endif

LOCAL_STATIC_LIBRARIES := \
    libmedia_helper \
    libaudiohw_legacy \
    libaudiopolicy_legacy \

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \
    libmedia \
    libhardware \
    libc        \
    libpower    \
    libalsa-intf \
    libsurround_proc\
    libaudioutils

ifeq ($(TARGET_SIMULATOR),true)
 LOCAL_LDLIBS += -ldl
else
 LOCAL_SHARED_LIBRARIES += libdl
endif

ifeq ($(QCOM_LISTEN_FEATURE),true)
  LOCAL_SHARED_LIBRARIES += liblistenhardware
endif

LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-alsa
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/libalsa-intf
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/surround_sound/
LOCAL_C_INCLUDES += hardware/libhardware/include
LOCAL_C_INCLUDES += hardware/libhardware_legacy/include
LOCAL_C_INCLUDES += frameworks/base/include
LOCAL_C_INCLUDES += system/core/include
LOCAL_C_INCLUDES += system/media/audio_utils/include
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-acdb-util

ifeq ($(QCOM_LISTEN_FEATURE),true)
  LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/mm-audio/audio-listen
endif


LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

ifeq ($(call is-board-platform,msm8974),true)
  LOCAL_MODULE := audio.primary.msm8974
endif

ifeq ($(call is-board-platform,msm8226),true)
  LOCAL_MODULE := audio.primary.msm8226
endif

ifeq ($(call is-board-platform,msm8960),true)
  LOCAL_MODULE := audio.primary.msm8960
endif

ifeq ($(call is-board-platform,msm8610),true)
  LOCAL_MODULE := audio.primary.msm8610
endif

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

#Conditional compilation of QCOM's audio policy manager
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_APMALSA)),true)

include $(CLEAR_VARS)

LOCAL_CFLAGS := -D_POSIX_SOURCE
ifneq ($(strip $(-QCOM_AUDIO_FEATURE_DISABLED_ACDB)),true)
  LOCAL_CFLAGS += -DQCOM_ACDB_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_ANC_HEADSET)),true)
  LOCAL_CFLAGS += -DQCOM_ANC_HEADSET_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_AUDIO_FORMAT)),true)
  LOCAL_CFLAGS += -DQCOM_AUDIO_FORMAT_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_CSD_CLIENT)),true)
  LOCAL_CFLAGS += -DQCOM_CSDCLIENT_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_FM)),true)
  LOCAL_CFLAGS += -DQCOM_FM_ENABLED
  ifeq ($(PLATFORM_SDK_VERSION),18)
    LOCAL_CFLAGS += -DQCOM_FM_V2_ENABLED
  endif
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_PROXY_DEVICE)),true)
  LOCAL_CFLAGS += -DQCOM_PROXY_DEVICE_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_SSR)),true)
  LOCAL_CFLAGS += -DQCOM_SSR_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_USBAUDIO)),true)
  LOCAL_CFLAGS += -DQCOM_USBAUDIO_ENABLED
endif
ifneq ($(strip $(QCOM_AUDIO_FEATURE_DISABLED_WFD)),true)
  LOCAL_CFLAGS += -DQCOM_WFD_ENABLED
endif

ifeq ($(PLATFORM_SDK_VERSION),18)
  LOCAL_CFLAGS += -DQCOM_APM_VERSION_JBMR2
endif

LOCAL_SRC_FILES := \
    audio_policy_hal.cpp \
    AudioPolicyManagerALSA.cpp

ifdef DOLBY_UDC_MULTICHANNEL
  LOCAL_CFLAGS += -DDOLBY_UDC_MULTICHANNEL
endif #DOLBY_UDC_MULTICHANNEL

ifeq ($(call is-board-platform,msm8974),true)
  LOCAL_MODULE := audio_policy.msm8974
endif

ifeq ($(call is-board-platform,msm8226),true)
  LOCAL_MODULE := audio_policy.msm8226
endif

ifeq ($(call is-board-platform,msm8960),true)
  LOCAL_MODULE := audio_policy.msm8960
endif

ifeq ($(call is-board-platform,msm8610),true)
  LOCAL_MODULE := audio_policy.msm8610
endif

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE_TAGS := optional

LOCAL_STATIC_LIBRARIES := \
    libmedia_helper \
    libaudiopolicy_legacy

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \

LOCAL_C_INCLUDES += hardware/libhardware_legacy/audio

include $(BUILD_SHARED_LIBRARY)

endif # endif compile QCOM's audio policy manager
endif
