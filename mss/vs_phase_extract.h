/**
 *  @file  vs_phase_extract.h
 *
 *  @brief Multi-bin phase extraction for vital signs.
 *
 *  Extracts zero-Doppler phase from adjacent range bins in the radar cube,
 *  performs phase unwrap and frame-to-frame differentiation, maintains a
 *  circular buffer of VS_PHASE_BUF_LEN samples per bin.
 */
#ifndef VS_PHASE_EXTRACT_H
#define VS_PHASE_EXTRACT_H

#include "mmw_output.h"
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/common/sys_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Internal state for phase extraction (persists across frames).
 */
typedef struct VsPhaseState_t
{
    /*! Previous frame's raw atan2 phase per bin (for unwrap) */
    float   prevRawPhase[VS_NUM_RANGE_BINS];

    /*! Previous frame's unwrapped phase per bin (for frame-to-frame diff) */
    float   prevUnwrappedPhase[VS_NUM_RANGE_BINS];

    /*! Cumulative phase-unwrap correction per bin */
    float   diffPhaseCorrCum[VS_NUM_RANGE_BINS];

    /*! Circular buffer: phase-difference waveform per bin */
    float   circBuf[VS_NUM_RANGE_BINS][VS_PHASE_BUF_LEN];

    /*! Write index into circular buffer */
    uint16_t writeIdx;

    /*! Number of frames processed since reset */
    uint16_t frameCount;
} VsPhaseState;

/**
 * @brief  Reset the phase extraction state (call on sensorStart or target change).
 */
void VsPhase_reset(VsPhaseState *state);

/**
 * @brief  Process one frame: extract phase from radar cube, update circular buffer.
 *
 * @param[in,out] state          Persistent state
 * @param[in]     radarCubeData  Pointer to radar cube (FORMAT_1, already address-translated)
 * @param[in]     centerBin      Center range bin index
 * @param[in]     numRangeBins   Total number of range bins in radar cube
 * @param[in]     numDopplerChirps Number of Doppler chirps (slow-time samples)
 * @param[in]     numRX          Number of RX antennas
 * @param[in]     numTX          Number of TX antenna patterns
 * @param[out]    output         Filled output structure for TLV
 */
void VsPhase_processFrame(
    VsPhaseState            *state,
    const cmplx16ImRe_t     *radarCubeData,
    uint16_t                 centerBin,
    uint16_t                 numRangeBins,
    uint16_t                 numDopplerChirps,
    uint8_t                  numRX,
    uint8_t                  numTX,
    MmwDemo_vsPhaseWaveform *output
);

#ifdef __cplusplus
}
#endif

#endif /* VS_PHASE_EXTRACT_H */
