/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <errno.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <dlfcn.h>
#include <math.h>

#define LOG_TAG "AudioSpeakerProtection"
#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0
#include <utils/Log.h>
#include <utils/String8.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <cutils/properties.h>
#include <media/AudioRecord.h>
#include <hardware_legacy/power.h>
#include <pthread.h>

#include "AudioHardwareALSA.h"
#ifdef QCOM_USBAUDIO_ENABLED
#include "AudioUsbALSA.h"
#endif
#include <linux/msm_audio_acdb.h>
#include "AudioUtil.h"
#include "acdb-id-mapper.h"

extern "C"
{
#ifdef QCOM_ACDB_ENABLED
    static void (*acdb_send_audio_cal)(int acdb_id, int capability);
#endif
    static int (*client_register_callback)
    (char *client_name, int (*callback)(int, void *, void *), void *data);
    static void (*thermal_client_unregister_callback)(int handle);
    static int (*thermal_client_request)(char *client_name, int req_data);
}         // extern "C"

static int thermal_client_callback(int temp, void *user_data, void *reserved)
{
    android_audio_legacy::AudioSpeakerProtection *handle;
    if (!user_data) {
        ALOGE("spkr_prot invalid value from thermal client");
        return -EINVAL;
    } else {
      handle = reinterpret_cast<android_audio_legacy::AudioSpeakerProtection*>
          (user_data);
      ALOGE("spkr_prot temp returned %d", temp);
      handle->updateSpkrT0(temp);
    }
    return 0;
}

namespace android_audio_legacy
{
/*Range of spkr temparatures -30C to 80C*/
#define MIN_SPKR_TEMP_Q6 (-30 * (1 << 6))
#define MAX_SPKR_TEMP_Q6 (80 * (1 << 6))

/*Set safe temp value to 25C*/
#define SAFE_SPKR_TEMP 40
#define SAFE_SPKR_TEMP_Q6 (SAFE_SPKR_TEMP * (1 << 6))

/*Range of resistance values 2ohms to 40 ohms*/
#define MIN_RESISTANCE_SPKR_Q24 (2 * (1 << 24))
#define MAX_RESISTANCE_SPKR_Q24 (40 * (1 << 24))

/*Path where the calibration file will be stored*/
#define CALIB_FILE "/data/misc/audio/audio.cal"

/*Time between retries for calibartion or intial wait time
  after boot up*/
#define WAIT_TIME_SPKR_CALIB (60 * 1000 * 1000)

/*Once calibration is started sleep for 1 sec to allow
  the calibration to kick off*/
#define SLEEP_AFTER_CALIB_START (1000 * 1000)

/*If calibration is in progress wait for 200 msec before querying
  for status again*/
#define WAIT_FOR_GET_CALIB_STATUS (200 * 1000)

/*Speaker states*/
#define SPKR_NOT_CALIBRATED -1
#define SPKR_CALIBRATED 1

/*Speaker processing state*/
#define SPKR_PROCESSING_IN_PROGRESS 1
#define SPKR_PROCESSING_IN_IDLE 0

/*Thermal Daemon Timeout in secs*/
#define SPKR_CALIB_THERMAL_TIMEOUT 60 * 2

/*Modes of Speaker Protection*/
enum speaker_protection_mode {
    SPKR_PROTECTION_DISABLED = -1,
    SPKR_PROTECTION_MODE_PROCESSING = 0,
    SPKR_PROTECTION_MODE_CALIBRATE = 1,
};

void AudioSpeakerProtection::updateSpkrT0(int t0)
{
    Mutex::Autolock autolock1(mSpkrProtThermalSyncMutex);
    ALOGE("spkr_prot set t0 %d and signal", t0);
    if (mSpkrProtMode == SPKR_NOT_CALIBRATED) {
        mSpkrProtT0 = t0;
    }
    mSpkrProtThermalSync.signal();
}

AudioSpeakerProtection::AudioSpeakerProtection():
	mParent(NULL),mALSADevice(NULL),mSpkrProtMode(SPKR_NOT_CALIBRATED),
    mSpkrProcessingState(SPKR_PROCESSING_IN_IDLE),
    mThermalClientHandle(0),mThermalHandle(NULL),
    mAcdbHandle(NULL), mUcMgr(NULL),mSpkrProtT0(-1)
{
}

void AudioSpeakerProtection::initialize(void *handle)
{
    if (!handle) {
        ALOGE("Invalid params for speaker protection");
        return;
    }
    mParent = reinterpret_cast<AudioHardwareALSA*>(handle);
#ifdef QCOM_ACDB_ENABLED
    mAcdbHandle = mParent->mAcdbHandle;
    mALSADevice = mParent->mALSADevice;
    mUcMgr = mParent->mUcMgr;
    acdb_send_audio_cal = (void (*)(int, int)) \
    ::dlsym(mAcdbHandle,"acdb_loader_send_audio_cal");
    if (!acdb_send_audio_cal) {
        ALOGE("dlsym:Error:%s Loading acdb_loader_send_audio_cal",
        dlerror());
        return;
    }
#else
    ALOGE("ACDB not enabled disable spkr processing");
    return;
#endif
    /*Load thermal daemon library*/
    mThermalHandle = ::dlopen("/vendor/lib/libthermalclient.so", RTLD_NOW);
    if (!mThermalHandle) {
        ALOGE("spkr_prot DLOPEN for thermal client failed");
    } else {
        /*Query callback function symbol*/
        client_register_callback =
       (int (*)(char *, int (*)(int, void *, void *),void *))
        ::dlsym(mThermalHandle, "thermal_client_register_callback");
        thermal_client_unregister_callback =
        (void (*)(int) )
        ::dlsym(mThermalHandle, "thermal_client_unregister_callback");
        if (!client_register_callback || !thermal_client_unregister_callback) {
            ALOGE("spkr_prot DLSYM thermal_client_register_callback failed");
        } else {
            /*Register callback function*/
            mThermalClientHandle =
            client_register_callback("spkr", thermal_client_callback, this);
            if (!mThermalClientHandle) {
                ALOGE("spkr_prot client_register_callback failed");
            } else {
                ALOGV("spkr_prot client_register_callback success");
                thermal_client_request = (int (*)(char *, int))
                ::dlsym(mThermalHandle, "thermal_client_request");
            }
        }
    }
    if (thermal_client_request) {
        ALOGE("spkr_prot Create calibration thread");
        (void)pthread_create(&mSpkrCalibrationThread,
        (const pthread_attr_t *) NULL, spkrCalibrationThread, this);
    } else {
        ALOGE("spkr_prot thermal_client_request failed");
        if (mThermalClientHandle)
            thermal_client_unregister_callback(mThermalClientHandle);
        if (mThermalHandle)
            ::dlclose(mThermalHandle);
        mThermalHandle = NULL;
    }
}
AudioSpeakerProtection::~AudioSpeakerProtection()
{
    if (mThermalClientHandle)
        thermal_client_unregister_callback(mThermalClientHandle);
    mThermalClientHandle = 0;
    if (mThermalHandle)
        ::dlclose(mThermalHandle);
    mThermalHandle = NULL;
}


void AudioSpeakerProtection::startSpkrProcessing()
{
    alsa_handle_t alsa_handle_tx;
    Mutex::Autolock autolock1(mMutexSpkrProt);
    unsigned long sec;
    int device = AudioSystem::DEVICE_OUT_SPEAKER;
#ifdef QCOM_ACDB_ENABLED
    if (!acdb_send_audio_cal) {
        ALOGE("spkr_prot_thread acdb_send_audio_cal is NULL");
        return;
    }
#endif
    if (mSpkrProtMode != SPKR_CALIBRATED) {
        ALOGV("Processing is not enabled for start");
        return;
    }
    if (mSpkrProcessingState == SPKR_PROCESSING_IN_PROGRESS) {
        ALOGV("Processing in progress for start");
        return;
    }
    char *name = NULL;
    bool mod = false;
    /*Check if any use case is active which is not using speaker*/
    snd_use_case_get(mUcMgr,"_verb",(const char **)&name);
    if (name) {
        ALOGV("spkr_prot_thread use case present %s",name);
        /*Check if use case is set to Inactive*/
        if (strncmp(name,"Inactive",strlen("Inactive"))) {
            mod = true;
        }
    }
    strlcpy(alsa_handle_tx.useCase,
    ((mod)?SND_USE_CASE_MOD_SPKR_PROT_TX:SND_USE_CASE_VERB_SPKR_PROT_TX),
            sizeof(alsa_handle_tx.useCase));
    alsa_handle_tx.module = mALSADevice;
    alsa_handle_tx.bufferSize =
    MIN_CAPTURE_BUFFER_SIZE_PER_CH * DEFAULT_CHANNEL_MODE;
    alsa_handle_tx.devices = device;
    alsa_handle_tx.handle = 0;
    alsa_handle_tx.format = SNDRV_PCM_FORMAT_S16_LE;
    alsa_handle_tx.channels = DEFAULT_CHANNEL_MODE;
    alsa_handle_tx.sampleRate = DEFAULT_SAMPLING_RATE;
    alsa_handle_tx.latency = 0;
    alsa_handle_tx.rxHandle = 0;
    alsa_handle_tx.ucMgr = mUcMgr;
    mParent->mDeviceList.push_back(alsa_handle_tx);
    ALSAHandleList::iterator it;
    it = mParent->mDeviceList.end();
    it--;
    if (!snd_use_case_set(mUcMgr, ((mod)?"_enamod":"_verb"),
       ((mod)?SND_USE_CASE_MOD_SPKR_PROT_TX:SND_USE_CASE_VERB_SPKR_PROT_TX))) {
#ifdef QCOM_ACDB_ENABLED
        acdb_send_audio_cal(DEVICE_SPEAKER_MONO_TX_PROT_ACDB_ID,
                            MSM_SNDDEV_CAP_TX);
#endif
        if (mALSADevice->startSpkProtRxTx(&(*it), false) == NO_ERROR) {
            mSpkrProcessingState = SPKR_PROCESSING_IN_PROGRESS;
            ALOGV("Spkr Processing started");
        } else
            ALOGE("mSpkrProtMode Failed to start TX processing");
    } else {
        ALOGE("spkr_prot set usecase %s failed",
              SND_USE_CASE_MOD_SPKR_PROT_TX);
    }
}

void AudioSpeakerProtection::stopSpkrProcessing()
{
    Mutex::Autolock autolock1(mMutexSpkrProt);
    if (mSpkrProcessingState == SPKR_PROCESSING_IN_IDLE) {
        ALOGV("Processing is not enabled in stop");
        return;
    }
    unsigned long sec;
    if (mALSADevice->isSpeakerinUse(sec)) {
        ALOGV("spkr_prot_thread in spk use skip stop");
        return;
    }
    for(ALSAHandleList::iterator it = mParent->mDeviceList.begin();
        it != mParent->mDeviceList.end(); ++it) {
        if(!strcmp(it->useCase, SND_USE_CASE_MOD_SPKR_PROT_TX) ||
           !strcmp(it->useCase, SND_USE_CASE_VERB_SPKR_PROT_TX)) {
            mALSADevice->close(&(*it));
            mParent->mDeviceList.erase(it);
            mSpkrProcessingState = SPKR_PROCESSING_IN_IDLE;
            ALOGV("Spkr Processing Idle");
            break;
        }
    }
}

int AudioSpeakerProtection::spkCalibrate(int t0)
{
    int device = AudioSystem::DEVICE_OUT_SPEAKER;
    int newMode = mParent->mode();
    alsa_handle_t alsa_handle;
    int acdb_fd = -1;
    struct msm_spk_prot_cfg protCfg;
    struct msm_spk_prot_status status;
    Mutex::Autolock autolock1(mMutexSpkrProt);
#ifdef QCOM_ACDB_ENABLED
    if (!acdb_send_audio_cal) {
        ALOGE("spkr_prot_thread acdb_send_audio_cal is NULL");
        return -ENODEV;
    }
#endif

    acdb_fd = open("/dev/msm_acdb",O_RDWR | O_NONBLOCK);
    if (acdb_fd < 0) {
        ALOGE("spkr_prot_thread open msm_acdb failed");
        return -ENODEV;
    } else {
        protCfg.mode = SPKR_PROTECTION_MODE_CALIBRATE;
        protCfg.t0 = t0;
        if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg)) {
            ALOGE("spkr_prot_thread set failed AUDIO_SET_SPEAKER_PROT");
            status.status = -ENODEV;
            goto bail_out;
        }
    }
    strlcpy(alsa_handle.useCase, SND_USE_CASE_VERB_SPKR_CALIB_RX,
            sizeof(alsa_handle.useCase));
    alsa_handle.module = mALSADevice;
    alsa_handle.bufferSize =
    MIN_CAPTURE_BUFFER_SIZE_PER_CH * DEFAULT_CHANNEL_MODE;
    alsa_handle.devices = device;
    alsa_handle.handle = 0;
    alsa_handle.format = SNDRV_PCM_FORMAT_S16_LE;
    alsa_handle.channels = DEFAULT_CHANNEL_MODE;
    alsa_handle.sampleRate = DEFAULT_SAMPLING_RATE;
    alsa_handle.latency = 0;
    alsa_handle.rxHandle = 0;
    alsa_handle.ucMgr = mUcMgr;
    mALSADevice->route(&alsa_handle, (uint32_t)device, newMode);
    if (snd_use_case_set(mUcMgr, "_verb", SND_USE_CASE_VERB_SPKR_CALIB_RX)) {
        ALOGE("spkr_prot_thread set usecase failed %s for calibration",
              SND_USE_CASE_VERB_SPKR_CALIB_RX);
        status.status = -ENODEV;
        goto bail_out;
    }
#ifdef QCOM_ACDB_ENABLED
    acdb_send_audio_cal(DEVICE_SPEAKER_MONO_RX_PROT_ACDB_ID,
                        MSM_SNDDEV_CAP_RX);
#endif
    if (mALSADevice->startSpkProtRxTx(&alsa_handle, true) !=  NO_ERROR) {
        ALOGE("spkr_prot_thread set startspk_prot_rx failed");
        status.status = -ENODEV;
        goto bail_out;
    }
    alsa_handle_t alsa_handle_tx;
    strlcpy(alsa_handle_tx.useCase, SND_USE_CASE_MOD_SPKR_PROT_TX,
    sizeof(alsa_handle_tx.useCase));
    alsa_handle_tx.module = mALSADevice;
    alsa_handle_tx.bufferSize =
    MIN_CAPTURE_BUFFER_SIZE_PER_CH * DEFAULT_CHANNEL_MODE;
    alsa_handle_tx.devices = device;
    alsa_handle_tx.handle = 0;
    alsa_handle_tx.format = SNDRV_PCM_FORMAT_S16_LE;
    alsa_handle_tx.channels = DEFAULT_CHANNEL_MODE;
    alsa_handle_tx.sampleRate = DEFAULT_SAMPLING_RATE;
    alsa_handle_tx.latency = 0;
    alsa_handle_tx.rxHandle = 0;
    alsa_handle_tx.ucMgr = mUcMgr;
    if (snd_use_case_set(mUcMgr, "_enamod", SND_USE_CASE_MOD_SPKR_PROT_TX)) {
        ALOGE("spkr_prot_thread set usecase failed %s for calibration",
              SND_USE_CASE_VERB_SPKR_CALIB_RX);
        mALSADevice->close(&alsa_handle);
        status.status = -ENODEV;
        goto bail_out;
    }
#ifdef QCOM_ACDB_ENABLED
    acdb_send_audio_cal(DEVICE_SPEAKER_MONO_TX_PROT_ACDB_ID,
                        MSM_SNDDEV_CAP_TX);
#endif
    if (mALSADevice->startSpkProtRxTx(&alsa_handle_tx, false) != NO_ERROR) {
        ALOGE("spkr_prot_thread set startspk_prot_tx failed");
        mALSADevice->close(&alsa_handle);
        status.status = -ENODEV;
        goto bail_out;
    }
    usleep(SLEEP_AFTER_CALIB_START);
    if (acdb_fd > 0) {
        status.status = -EINVAL;
        while (!ioctl(acdb_fd, AUDIO_GET_SPEAKER_PROT,&status)) {
            /*sleep for 200 ms to check for status check*/
            if (!status.status) {
                ALOGE("spkr_prot_thread calib Success R0 %d",
                 status.r0);
                FILE *fp;
                fp = fopen(CALIB_FILE,"wb");
                if (!fp) {
                    ALOGE("spkr_prot_thread File open failed %s",
                    strerror(errno));
                    status.status = -ENODEV;
                } else {
                    fwrite(&status.r0, sizeof(status.r0),1,fp);
                    fwrite(&protCfg.t0, sizeof(protCfg.t0),1,fp);
                    fclose(fp);
                }
                break;
            } else if (status.status == -EAGAIN) {
                  ALOGE("spkr_prot_thread try again");
                  usleep(WAIT_FOR_GET_CALIB_STATUS);
            } else {
                ALOGE("spkr_prot_thread get failed status %d",
                status.status);
                break;
            }
        }
        mALSADevice->close(&alsa_handle);
        mALSADevice->close(&alsa_handle_tx);
bail_out:
        if (!status.status) {
            protCfg.mode = SPKR_PROTECTION_MODE_PROCESSING;
            protCfg.r0 = status.r0;
            /*TO DO: T0 Needs to be provided to the API*/
            if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg))
                ALOGE("spkr_prot_thread disable calib mode");
            else
                mSpkrProtMode = SPKR_CALIBRATED;
            mALSADevice->spkrCalibStatusUpdate(true);
        } else {
            mALSADevice->spkrCalibStatusUpdate(false);
            protCfg.mode = SPKR_PROTECTION_DISABLED;
            mSpkrProtMode = SPKR_NOT_CALIBRATED;
            /*TO DO: T0 Needs to be provided to the API*/
            if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg))
                ALOGE("spkr_prot_thread disable calib mode failed");
            status.status = -ENODEV;
        }
        close(acdb_fd);
    }
    return status.status;
}

void *AudioSpeakerProtection::spkrCalibrationThread(void *me)
{
    AudioSpeakerProtection *alsad = reinterpret_cast<AudioSpeakerProtection*>(me);
    unsigned long sec = 0;
    int t0;
    bool goAhead = false;
    struct msm_spk_prot_cfg protCfg;
    FILE *fp;
    int acdb_fd;

    alsad->mALSADevice->spkrCalibStatusUpdate(false);
    fp = fopen(CALIB_FILE,"rb");
    if (fp) {
        fread(&protCfg.r0,sizeof(protCfg.r0),1,fp);
        ALOGD("spkr_prot_thread r0 value %d",protCfg.r0);
        fread(&protCfg.t0, sizeof(protCfg.t0), 1, fp);
        ALOGD("spkr_prot_thread t0 value %d",protCfg.t0);
        fclose(fp);
        /*Valid tempature range: -30C to 80C(in q6 format)
          Valid Resistance range: 2 ohms to 40 ohms(in q24 format)*/
        if (protCfg.t0 > MIN_SPKR_TEMP_Q6 &&
            protCfg.t0 < MAX_SPKR_TEMP_Q6 &&
            protCfg.r0 >= MIN_RESISTANCE_SPKR_Q24
            && protCfg.r0 < MAX_RESISTANCE_SPKR_Q24) {
            ALOGD("spkr_prot_thread Spkr calibrated");
            acdb_fd = open("/dev/msm_acdb",O_RDWR | O_NONBLOCK);
            if (acdb_fd > 0) {
                /*Set processing mode with t0/r0*/
                protCfg.mode = SPKR_PROTECTION_MODE_PROCESSING;
                if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg)) {
                    ALOGE("spkr_prot_thread enable prot failed");
                    alsad->mSpkrProtMode = SPKR_NOT_CALIBRATED;
                } else
                    alsad->mSpkrProtMode = SPKR_CALIBRATED;
                close(acdb_fd);
            }
            fclose(fp);
            alsad->mALSADevice->spkrCalibStatusUpdate(true);
            pthread_exit(0);
            return NULL;
        }
    }
    while (1) {
        /*Once device has booted up sleep for X time
          Currently set to 1 min needs to be adjusted*/
        usleep(WAIT_TIME_SPKR_CALIB);
        ALOGV("spkr_prot_thread up to start calibration");
        if (alsad->mALSADevice->isSpeakerinUse(sec)) {
            ALOGD("spkr_prot_thread Speaker in use retry calibration");
            continue;
        } else {
            /*TODO: If speaker is idle > X mins proceed requesting
              equilibrium Temp*/
            ALOGD("spkr_prot_thread speaker idle %d", sec);
        }
        if (!thermal_client_request("spkr",1)) {
            ALOGE("spkr_prot wait for callback from thermal daemon");
            Mutex::Autolock autolock1(alsad->mSpkrProtThermalSyncMutex);
            status_t ret = NO_ERROR;
            ret = alsad->mSpkrProtThermalSync.waitRelative(
               alsad->mSpkrProtThermalSyncMutex,
               seconds(SPKR_CALIB_THERMAL_TIMEOUT));
            /*Convert temp into q6 format*/
            if (!ret) {
                t0 = (alsad->mSpkrProtT0 * (1 << 6));
            } else {
                ALOGE("Thermal Daemon Timeout Assume safe T0");
                alsad->mSpkrProtT0 = SAFE_SPKR_TEMP;
                t0 = SAFE_SPKR_TEMP_Q6;
            }
            alsad->mSpkrProtThermalSyncMutex.unlock();
            ALOGD("spkr_prot Request t0 success value %d",
            alsad->mSpkrProtT0);
        } else {
            ALOGE("spkr_prot Request t0 failed");
            /*Assume safe value for temparature*/
            t0 = SAFE_SPKR_TEMP_Q6;
        }
        goAhead = false;
        if (alsad->mALSADevice->isSpeakerinUse(sec)) {
            ALOGD("spkr_prot_thread Speaker in use retry calibration");
            continue;
        } else {
            ALOGD("spkr_prot_thread speaker idle %d", sec);
            /*TODO: If speaker is idle > X mins proceed for calibration*/
            goAhead = true;
        }
        char *name = NULL;
        /*Check if any use case is active which is not using speaker*/
        snd_use_case_get(alsad->mUcMgr,"_verb",(const char **)&name);
        if (name) {
            ALOGV("spkr_prot_thread use case present %s",name);
            /*Check if use case is set to Inactive*/
            if (strncmp(name,"Inactive",strlen("Inactive"))) {
                goAhead = false;
            }
        }
        if (goAhead) {
            int status;
            status = alsad->spkCalibrate(t0);
            if (status == -EAGAIN) {
                ALOGE("spkr_prot failed to calibrate try again %s",
                      strerror(status));
                continue;
            } else {
                ALOGE("spkr_prot calibrate status %s", strerror(status));
            }
            ALOGE("spkr_prot_thread end calibration");
            break;
        }
    }
    pthread_exit(0);
    return NULL;
}
}
