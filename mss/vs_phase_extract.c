/**
 *  @file  vs_phase_extract.c
 *
 *  @brief Multi-bin phase extraction for vital signs on MSS (ARM R4F).
 *
 *  Per frame:
 *    1. For each of VS_NUM_RANGE_BINS adjacent range bins, accumulate the
 *       complex signal across all Doppler chirps (zero-Doppler sum)
 *       using TX0 / RX0.
 *    2. Compute atan2 phase.
 *    3. Phase unwrap (same algorithm as TI vitalsign.c).
 *    4. Frame-to-frame phase difference (proportional to chest displacement).
 *    5. Store in per-bin circular buffer.
 *
 *  Radar cube FORMAT_1:
 *    cmplx16ImRe_t x[numTXPatterns][numDopplerChirps][numRX][numRangeBins]
 *
 *  Phase extraction logic follows TI vitalsign.c lines 250-284:
 *    - phasePrevFrame  = previous raw atan2 (for unwrap function)
 *    - phaseUsedComputationPrev = previous unwrapped phase (for diff)
 *    - phaseUsedComputation = current_unwrapped - prev_unwrapped
 */

#include "vs_phase_extract.h"
#include <math.h>
#include <string.h>

#define PI_F  3.14159265358979323846f

/* ------------------------------------------------------------------ */
/*  Phase unwrap: same logic as TI MmwDemo_computePhaseUnwrap         */
/*  Input:  phase     = current raw atan2 output [-pi, pi]            */
/*          phasePrev = previous raw atan2 output [-pi, pi]           */
/*          diffCorrCum = cumulative 2*pi correction (in/out)         */
/*  Output: unwrapped phase = phase + *diffCorrCum                    */
/* ------------------------------------------------------------------ */
static float phaseUnwrap(float phase, float phasePrev, float *diffCorrCum, uint8_t *jumped)
{
    float diff = phase - phasePrev;

    *jumped = 0U;

    if (diff > PI_F)
    {
        *diffCorrCum -= 2.0f * PI_F;
        *jumped = 1U;
    }
    else if (diff < -PI_F)
    {
        *diffCorrCum += 2.0f * PI_F;
        *jumped = 1U;
    }

    return phase + *diffCorrCum;
}

static float rmsFromBuffer(const float *buf, uint16_t len)
{
    uint16_t i;
    float sum = 0.0f;
    if (len == 0U)
    {
        return 0.0f;
    }
    for (i = 0; i < len; i++)
    {
        sum += buf[i] * buf[i];
    }
    return sqrtf(sum / (float)len);
}

/* ------------------------------------------------------------------ */
void VsPhase_reset(VsPhaseState *state)
{
    memset(state, 0, sizeof(VsPhaseState));
}

/* ------------------------------------------------------------------ */
void VsPhase_processFrame(
    VsPhaseState            *state,
    const cmplx16ImRe_t     *radarCubeData,
    uint16_t                 centerBin,
    uint16_t                 numRangeBins,
    uint16_t                 numDopplerChirps,
    uint8_t                  numRX,
    uint8_t                  numTX,
    MmwDemo_vsPhaseWaveform *output,
    MmwDemo_vsQuality       *quality)
{
    uint16_t bin, chirp, i, src, validSamples;
    int16_t  rangeBinIdx;
    int32_t  sumReal, sumImag;
    float    rawPhase, unwrapped, phaseDiff, magMean;
    uint8_t  jumped;
    (void)numTX;

    /* FORMAT_1: x[numTX][numDopplerChirps][numRX][numRangeBins]
     * For TX0, RX0: index = 0 + chirp * numRX * numRangeBins + 0 + rangeBinIdx
     */
    uint32_t chirpStride = (uint32_t)numRX * (uint32_t)numRangeBins;

    for (bin = 0; bin < VS_NUM_RANGE_BINS; bin++)
    {
        rangeBinIdx = (int16_t)centerBin - (VS_NUM_RANGE_BINS / 2) + (int16_t)bin;

        /* Bounds check */
        if (rangeBinIdx < 0 || rangeBinIdx >= (int16_t)numRangeBins)
        {
            state->circBuf[bin][state->writeIdx] = 0.0f;
            state->jumpBuf[bin][state->writeIdx] = 0U;
            continue;
        }

        /* Accumulate complex signal across all chirps (zero-Doppler sum) */
        sumReal = 0;
        sumImag = 0;
        for (chirp = 0; chirp < numDopplerChirps; chirp++)
        {
            uint32_t idx = chirp * chirpStride + (uint32_t)rangeBinIdx;
            sumReal += (int32_t)radarCubeData[idx].real;
            sumImag += (int32_t)radarCubeData[idx].imag;
        }

        magMean = sqrtf(((float)sumReal * (float)sumReal) +
                        ((float)sumImag * (float)sumImag));
        magMean /= (float)((numDopplerChirps > 0U) ? numDopplerChirps : 1U);

        /* atan2 phase (raw, in [-pi, pi]) */
        rawPhase = atan2f((float)sumImag, (float)sumReal);

        if (state->frameCount == 0)
        {
            /* First frame: initialize state, output zero */
            state->prevRawPhase[bin]       = rawPhase;
            state->prevUnwrappedPhase[bin] = rawPhase;
            state->diffPhaseCorrCum[bin]   = 0.0f;
            state->circBuf[bin][state->writeIdx] = 0.0f;
            state->jumpBuf[bin][state->writeIdx] = 0U;
        }
        else
        {
            /* Phase unwrap: uses raw previous phase for jump detection */
            unwrapped = phaseUnwrap(rawPhase,
                                    state->prevRawPhase[bin],
                                    &state->diffPhaseCorrCum[bin],
                                    &jumped);

            /* Frame-to-frame phase difference = chest displacement signal */
            phaseDiff = unwrapped - state->prevUnwrappedPhase[bin];

            /* Update state for next frame */
            state->prevRawPhase[bin]       = rawPhase;
            state->prevUnwrappedPhase[bin] = unwrapped;

            state->circBuf[bin][state->writeIdx] = phaseDiff;
            state->jumpBuf[bin][state->writeIdx] = jumped;
        }

        quality->magMean[bin] = magMean;
    }

    state->frameCount++;
    state->writeIdx = (state->writeIdx + 1) % VS_PHASE_BUF_LEN;

    /* --- Fill output structure --- */
    output->centerRangeBin = centerBin;
    output->numBins        = VS_NUM_RANGE_BINS;
    output->reserved       = 0;
    quality->centerRangeBin = centerBin;
    quality->numBins        = VS_NUM_RANGE_BINS;
    quality->reserved       = 0;

    validSamples = (state->frameCount < VS_PHASE_BUF_LEN)
                   ? state->frameCount : VS_PHASE_BUF_LEN;
    output->numSamples = validSamples;
    quality->numSamples = validSamples;

    /* Output in chronological order (oldest first) */
    for (bin = 0; bin < VS_NUM_RANGE_BINS; bin++)
    {
        uint16_t jumpCount = 0U;
        float motionRecentBuf[5];
        uint16_t motionCount = 0U;
        for (i = 0; i < VS_PHASE_BUF_LEN; i++)
        {
            if (i < validSamples)
            {
                src = (state->writeIdx + VS_PHASE_BUF_LEN - validSamples + i)
                      % VS_PHASE_BUF_LEN;
                output->phaseWaveform[bin][i] = state->circBuf[bin][src];
                jumpCount += state->jumpBuf[bin][src];
                if (i >= (validSamples > 5U ? (validSamples - 5U) : 0U) && motionCount < 5U)
                {
                    motionRecentBuf[motionCount++] = state->circBuf[bin][src];
                }
            }
            else
            {
                output->phaseWaveform[bin][i] = 0.0f;
            }
        }

        quality->phaseDiffRms[bin] = rmsFromBuffer(output->phaseWaveform[bin], validSamples);
        quality->motionScore[bin]   = rmsFromBuffer(motionRecentBuf, motionCount);
        quality->unwrapJumpCount[bin] = jumpCount;
        quality->reserved2[bin] = 0U;
    }
}
