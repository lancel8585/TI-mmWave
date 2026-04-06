/**
 *   @file  mmw_output.h
 *
 *   @brief
 *      This is the interface/message header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MMW_OUTPUT_H
#define MMW_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/common/sys_common.h>
#include <ti/datapath/dpc/objectdetection/objdetdsp/objectdetection.h>

/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, and also for Mailbox communication
 *  between MSS and DSS on the XWR18xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum MmwDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    /*! @brief   Samples to calculate static azimuth/elevation 
                 heatmap, (all virtual antennas exported) - unused in this demo
     */
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,

    /*! @brief   temperature stats from Radar front end */
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,

    MMWDEMO_OUTPUT_MSG_MAX,

    /*! @brief   Multi-bin vital signs phase waveform (0x0500) */
    MMWDEMO_OUTPUT_MSG_VS_PHASE_WAVEFORM = 0x0500,

    /*! @brief   Vital signs per-bin quality and motion metrics (0x0501) */
    MMWDEMO_OUTPUT_MSG_VS_QUALITY = 0x0501
} MmwDemo_output_message_type;

/*! Number of adjacent range bins for vital signs phase output */
#define VS_NUM_RANGE_BINS   5

/*! Number of phase samples per bin per frame (1 sample per frame, circular buffer depth) */
#define VS_PHASE_BUF_LEN   15

/*!
 * @brief  Multi-bin vital signs phase waveform output
 *
 * @details
 *  Each frame, the MSS extracts the complex zero-Doppler signal from
 *  VS_NUM_RANGE_BINS adjacent range bins, computes atan2 phase, unwraps,
 *  differentiates (frame-to-frame), and stores the latest VS_PHASE_BUF_LEN
 *  samples per bin.  The PC uses adaptive noise cancellation across bins
 *  to separate breathing and cardiac signals.
 */
typedef struct MmwDemo_vsPhaseWaveform_t
{
    /*! @brief  Center range bin index */
    uint16_t    centerRangeBin;

    /*! @brief  Number of range bins output (VS_NUM_RANGE_BINS) */
    uint16_t    numBins;

    /*! @brief  Number of valid samples per bin (up to VS_PHASE_BUF_LEN) */
    uint16_t    numSamples;

    /*! @brief  Reserved for alignment */
    uint16_t    reserved;

    /*! @brief  Phase-difference waveform [bin][sample], radians */
    float       phaseWaveform[VS_NUM_RANGE_BINS][VS_PHASE_BUF_LEN];
} MmwDemo_vsPhaseWaveform;

/*!
 * @brief  Per-frame vital signs quality metrics for the same 5 range bins
 */
typedef struct MmwDemo_vsQuality_t
{
    /*! @brief  Center range bin index */
    uint16_t    centerRangeBin;

    /*! @brief  Number of range bins output (VS_NUM_RANGE_BINS) */
    uint16_t    numBins;

    /*! @brief  Number of valid samples per bin (up to VS_PHASE_BUF_LEN) */
    uint16_t    numSamples;

    /*! @brief  Count of bins that had unwrap jumps in this frame */
    uint16_t    reserved;

    /*! @brief  Per-bin mean magnitude of zero-Doppler sum */
    float       magMean[VS_NUM_RANGE_BINS];

    /*! @brief  Per-bin RMS of phase-difference circular buffer */
    float       phaseDiffRms[VS_NUM_RANGE_BINS];

    /*! @brief  Per-bin short motion / instability score */
    float       motionScore[VS_NUM_RANGE_BINS];

    /*! @brief  Per-bin unwrap jump count accumulated over the circular buffer */
    uint16_t    unwrapJumpCount[VS_NUM_RANGE_BINS];

    /*! @brief  Reserved for alignment / future use */
    uint16_t    reserved2[VS_NUM_RANGE_BINS];
} MmwDemo_vsQuality;

/*!
 * @brief
 *  Message header for reporting detection information from data path.
 *
 * @details
 *  The structure defines the message header.
 */
typedef struct MmwDemo_output_message_header_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
    uint16_t    magicWord[4];

    /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
    uint32_t     version;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;

    /*! @brief   platform type */
    uint32_t    platform;

    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   Time in CPU cycles when the message was created. For XWR16xx/XWR18xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
    uint32_t    timeCpuCycles;

    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObj;

    /*! @brief   Number of TLVs */
    uint32_t    numTLVs;

    /*! @brief   For Advanced Frame config, this is the sub-frame number in the range
     * 0 to (number of subframes - 1). For frame config (not advanced), this is always
     * set to 0. */
    uint32_t    subFrameNumber;
} MmwDemo_output_message_header;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_output_message_stats_t
{
    /*! @brief   Interframe processing time in usec */
    uint32_t     interFrameProcessingTime;

    /*! @brief   Transmission time of output detection information in usec */
    uint32_t     transmitOutputTime;

    /*! @brief   Interframe processing margin in usec */
    uint32_t     interFrameProcessingMargin;

    /*! @brief   Interchirp processing margin in usec */
    uint32_t     interChirpProcessingMargin;

    /*! @brief   CPU Load (%) during active frame duration */
    uint32_t     activeFrameCPULoad;

    /*! @brief   CPU Load (%) during inter frame duration */
    uint32_t     interFrameCPULoad;
} MmwDemo_output_message_stats;

/**
 * @brief
 *  Size of HSRAM Payload data array.
 */
#define MMWDEMO_HSRAM_PAYLOAD_SIZE        (SOC_HSRAM_SIZE - sizeof(DPC_ObjectDetection_ExecuteResult) - \
                                            sizeof(MmwDemo_output_message_stats))

/**
 * @brief
 *  DSS stores demo output and stats in HSRAM.
 */
typedef struct MmwDemo_HSRAM_t
{
    /*! @brief   DPC execution result */
    DPC_ObjectDetection_ExecuteResult result;

    /*! @brief   Output message stats reported by DSS */
    MmwDemo_output_message_stats   outStats;

    /*! @brief   Payload data of result */
    uint8_t                        payload[MMWDEMO_HSRAM_PAYLOAD_SIZE];
} MmwDemo_HSRAM;

/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path.
 */
typedef struct MmwDemo_output_message_tl_t
{
    /*! @brief   TLV type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

} MmwDemo_output_message_tl;

#ifdef __cplusplus
}
#endif

#endif /* MMW_OUTPUT_H */
