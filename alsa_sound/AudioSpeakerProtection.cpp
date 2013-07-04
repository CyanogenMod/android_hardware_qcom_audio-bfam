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

#define MIN_SPKR_IDLE_SEC (60 * 30)

/*Once calibration is started sleep for 1 sec to allow
  the calibration to kick off*/
#define SLEEP_AFTER_CALIB_START (3)

/*If calibration is in progress wait for 200 msec before querying
  for status again*/
#define WAIT_FOR_GET_CALIB_STATUS (200 * 1000)

/*Speaker states*/
#define SPKR_NOT_CALIBRATED -1
#define SPKR_CALIBRATED 1

/*Speaker processing state*/
#define SPKR_PROCESSING_IN_PROGRESS 1
#define SPKR_PROCESSING_IN_IDLE 0

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
    if (mSpkrProtMode == MSM_SPKR_PROT_NOT_CALIBRATED) {
        mSpkrProtT0 = t0;
    }
    mSpkrProtThermalSync.signal();
}

AudioSpeakerProtection::AudioSpeakerProtection():
	mParent(NULL),mALSADevice(NULL),mSpkrProtMode(MSM_SPKR_PROT_DISABLED),
    mSpkrProcessingState(SPKR_PROCESSING_IN_IDLE),
    mThermalClientHandle(0),mThermalHandle(NULL),
    mAcdbHandle(NULL), mUcMgr(NULL),mSpkrProtT0(-1),
    mCancelSpkrCalib(0)
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
    if (mSpkrProtMode == MSM_SPKR_PROT_CALIBRATION_IN_PROGRESS ||
        mSpkrProtMode == MSM_SPKR_PROT_DISABLED) {
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
        free(name);
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
    bool cleanup = false;
#ifdef QCOM_ACDB_ENABLED
    if (!acdb_send_audio_cal) {
        ALOGE("spkr_prot_thread acdb_send_audio_cal is NULL");
        return -ENODEV;
    }
#endif
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
    char *name = NULL;
    snd_use_case_get(mUcMgr,"_verb",(const char **)&name);
    if (name && strncmp(name,"Inactive",strlen("Inactive"))) {
        ALOGE("%s usecase active retry", name);
        free(name);
        return -EAGAIN;
    }
    if (name)
       free(name);
    acdb_fd = open("/dev/msm_acdb",O_RDWR | O_NONBLOCK);
    if (acdb_fd < 0) {
        ALOGE("spkr_prot_thread open msm_acdb failed");
        return -ENODEV;
    } else {
        protCfg.mode = MSM_SPKR_PROT_CALIBRATION_IN_PROGRESS;
        protCfg.t0 = t0;
        if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg)) {
            ALOGE("spkr_prot_thread set failed AUDIO_SET_SPEAKER_PROT");
            status.status = -ENODEV;
            goto bail_out;
        }
    }
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
    cleanup = true;
    (void)mSpkrCalibCancel.waitRelative(mMutexSpkrProt,
            seconds(SLEEP_AFTER_CALIB_START));
    mSpkrCalibCancelAckMutex.lock();
    if (mCancelSpkrCalib) {
        status.status = -EAGAIN;
        goto bail_out;
    }
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
            protCfg.mode = MSM_SPKR_PROT_CALIBRATED;
            protCfg.r0 = status.r0;
            /*TO DO: T0 Needs to be provided to the API*/
            if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg))
                ALOGE("spkr_prot_thread disable calib mode");
            else
                mSpkrProtMode = MSM_SPKR_PROT_CALIBRATED;
        } else {
            protCfg.mode = MSM_SPKR_PROT_NOT_CALIBRATED;
            mSpkrProtMode = MSM_SPKR_PROT_NOT_CALIBRATED;
            /*TO DO: T0 Needs to be provided to the API*/
            if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg))
                ALOGE("spkr_prot_thread disable calib mode failed");
        }
        if (acdb_fd > 0)
            close(acdb_fd);
        if (cleanup) {
            if (mCancelSpkrCalib) {
                /*Close RX and TX paths*/
                mALSADevice->close(&alsa_handle);
                mALSADevice->close(&alsa_handle_tx);
                mSpkrCalibCancelAck.signal();
            }
            mCancelSpkrCalib = 0;
            mSpkrCalibCancelAckMutex.unlock();
        }
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
    alsad->mSpeakerProtthreadid = pthread_self();
    acdb_fd = open("/dev/msm_acdb",O_RDWR | O_NONBLOCK);
    if (acdb_fd > 0) {
        /*Set processing mode with t0/r0*/
        protCfg.mode = MSM_SPKR_PROT_NOT_CALIBRATED;
        if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg)) {
            ALOGE("spkr_prot_thread enable prot failed");
            alsad->mSpkrProtMode = MSM_SPKR_PROT_DISABLED;
            close(acdb_fd);
        } else
            alsad->mSpkrProtMode = MSM_SPKR_PROT_NOT_CALIBRATED;
    } else {
        alsad->mSpkrProtMode = MSM_SPKR_PROT_DISABLED;
        ALOGE("Failed to open acdb node");
    }
    if (alsad->mSpkrProtMode == MSM_SPKR_PROT_DISABLED) {
        ALOGE("Speaker protection disabled");
        pthread_exit(0);
        return NULL;
    }
    alsad->mALSADevice->spkrCalibStatusUpdate(true);
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
            protCfg.mode = MSM_SPKR_PROT_CALIBRATED;
            if (ioctl(acdb_fd, AUDIO_SET_SPEAKER_PROT, &protCfg)) {
                ALOGE("spkr_prot_thread enable prot failed");
                alsad->mSpkrProtMode = MSM_SPKR_PROT_DISABLED;
            } else
                alsad->mSpkrProtMode = MSM_SPKR_PROT_CALIBRATED;
            close(acdb_fd);
            pthread_exit(0);
            return NULL;
        }
        close(acdb_fd);
    }
    while (1) {
        ALOGV("spkr_prot_thread up to start calibration");
        if (!thermal_client_request("spkr",1)) {
            ALOGD("spkr_prot wait for callback from thermal daemon");
            Mutex::Autolock autolock1(alsad->mSpkrProtThermalSyncMutex);
            status_t ret = NO_ERROR;
            ret = alsad->mSpkrProtThermalSync.wait(
               alsad->mSpkrProtThermalSyncMutex);
            /*Convert temp into q6 format*/
            t0 = (alsad->mSpkrProtT0 * (1 << 6));
            alsad->mSpkrProtThermalSyncMutex.unlock();
            if (t0 < MIN_SPKR_TEMP_Q6 || t0 > MAX_SPKR_TEMP_Q6) {
                ALOGE("Calibration temparature error %d", alsad->mSpkrProtT0);
                continue;
            }
            ALOGD("spkr_prot Request t0 success value %d",
            alsad->mSpkrProtT0);
        } else {
            ALOGE("spkr_prot Request t0 failed");
            /*Assume safe value for temparature*/
            t0 = SAFE_SPKR_TEMP_Q6;
        }
        goAhead = false;
        {
            Mutex::Autolock autolock1(alsad->mMutexSpkrProt);
            if (alsad->mALSADevice->isSpeakerinUse(sec)) {
                ALOGD("spkr_prot_thread Speaker in use retry calibration");
                continue;
            } else {
                ALOGD("spkr_prot_thread speaker idle %d", sec);
                /*TODO: If speaker is idle > X mins proceed for calibration*/
                if (sec < MIN_SPKR_IDLE_SEC) {
                    ALOGD("spkr_prot_thread speaker idle is less retry");
                    continue;
                }
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
                free(name);
            }
            if (goAhead) {
                int status;
                alsad->mALSADevice->spkrCalibStatusUpdate(false);
                status = alsad->spkCalibrate(t0);
                alsad->mALSADevice->spkrCalibStatusUpdate(true);
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
    }
    pthread_exit(0);
    return NULL;
}
void AudioSpeakerProtection::cancelCalibration()
{
    char *name = NULL;
    pthread_t threadid;
    threadid = pthread_self();
    if (pthread_equal(mSpeakerProtthreadid, threadid))
        return;
    mMutexSpkrProt.lock();
    snd_use_case_get(mUcMgr,"_verb",(const char **)&name);
    if (name && !strncmp(name, SND_USE_CASE_VERB_SPKR_CALIB_RX,
           strlen(SND_USE_CASE_VERB_SPKR_CALIB_RX))) {
            mSpkrCalibCancelAckMutex.lock();
            mCancelSpkrCalib = 1;
            mSpkrCalibCancel.signal();
            mMutexSpkrProt.unlock();
            mSpkrCalibCancelAck.wait(mSpkrCalibCancelAckMutex);
            mSpkrCalibCancelAckMutex.unlock();
    } else
        mMutexSpkrProt.unlock();
    if (name)
       free(name);
}
}
