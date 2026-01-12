/**
 * @file NotchFilter.h
 * @brief IIR Notch Filter for eliminating specific frequencies from control signals
 * 
 * A second-order IIR notch filter designed to eliminate resonance frequencies
 * in the torque control loop. Configurable center frequency and quality factor.
 * 
 * Created: 2024-12-30
 */

#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <cmath>

/**
 * @class NotchFilter
 * @brief Second-order IIR notch (band-stop) filter
 * 
 * Uses the Audio EQ Cookbook biquad notch filter formula.
 * 
 * Parameters:
 *   f₀ = center frequency to eliminate (Hz)
 *   Q  = quality factor (higher = narrower notch)
 *        Q = f₀/BW where BW is the -3dB bandwidth
 *        Typical values: 2-10 for mechanical resonances
 *        Q=2 → wide notch, Q=10 → narrow notch
 * 
 * The filter has unity gain at DC and Nyquist, with a notch at f₀.
 */
class NotchFilter {
private:
    // Filter coefficients
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;       // Denominator coefficients (a0 = 1)
    
    // State variables (previous samples)
    float x1, x2;       // Previous inputs
    float y1, y2;       // Previous outputs
    
    // Configuration
    float center_freq_hz;
    float sample_freq_hz;
    float quality;
    bool enabled;
    bool configured;

public:
    /**
     * @brief Constructor - creates a bypassed filter
     */
    NotchFilter() : 
        b0(1.0f), b1(0.0f), b2(0.0f),
        a1(0.0f), a2(0.0f),
        x1(0.0f), x2(0.0f),
        y1(0.0f), y2(0.0f),
        center_freq_hz(10.0f),
        sample_freq_hz(500.0f),
        quality(5.0f),  // Q factor: 5 = moderate bandwidth
        enabled(false),
        configured(false)
    {}

    /**
     * @brief Configure the notch filter parameters
     * 
     * @param notch_freq_hz Center frequency to eliminate (Hz)
     * @param sample_rate_hz Sample rate of the control loop (Hz)
     * @param q Quality factor (0.8-0.99). Lower = wider notch, higher = narrower
     *          Recommended: 0.85-0.95 for mechanical resonances
     */
    void configure(float notch_freq_hz, float sample_rate_hz, float q = 0.90f) {
        center_freq_hz = notch_freq_hz;
        sample_freq_hz = sample_rate_hz;
        
        // IMPORTANT: For notch filter, Q controls bandwidth
        // Q = f0/BW, so higher Q = narrower notch
        // Typical values: Q=2-10 for mechanical resonances
        // The 'quality' parameter here is interpreted as Q factor (not pole radius)
        // Constrain to reasonable range: Q=1 (very wide) to Q=20 (very narrow)
        float Q = constrain(q, 1.0f, 20.0f);
        quality = Q;
        
        // Normalized angular frequency (0 to π)
        float omega0 = 2.0f * M_PI * notch_freq_hz / sample_rate_hz;
        
        // Sanity check: notch frequency must be less than Nyquist
        if (omega0 >= M_PI || omega0 <= 0.0f) {
            configured = false;
            return;
        }
        
        // Audio EQ Cookbook formula for notch (band-reject) filter
        // Source: Robert Bristow-Johnson's Audio EQ Cookbook
        float cos_omega = cosf(omega0);
        float sin_omega = sinf(omega0);
        float alpha = sin_omega / (2.0f * Q);  // Bandwidth parameter
        
        // Unnormalized coefficients
        float b0_raw = 1.0f;
        float b1_raw = -2.0f * cos_omega;
        float b2_raw = 1.0f;
        float a0_raw = 1.0f + alpha;
        float a1_raw = -2.0f * cos_omega;
        float a2_raw = 1.0f - alpha;
        
        // Normalize by a0 (standard biquad normalization)
        b0 = b0_raw / a0_raw;
        b1 = b1_raw / a0_raw;
        b2 = b2_raw / a0_raw;
        a1 = a1_raw / a0_raw;
        a2 = a2_raw / a0_raw;
        
        // Reset state when reconfiguring to avoid transients
        reset();
        
        configured = true;
    }

    /**
     * @brief Process a single sample through the filter
     * 
     * @param input Input sample
     * @return Filtered output (or input if bypassed)
     */
    float process(float input) {
        // Bypass if not enabled or not configured
        if (!enabled || !configured) {
            return input;
        }
        
        // Direct Form I difference equation:
        // y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
        float output = b0 * input + b1 * x1 + b2 * x2 
                                  - a1 * y1 - a2 * y2;
        
        // Update state (shift register)
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        
        return output;
    }

    /**
     * @brief Reset filter state (clear history)
     * 
     * Call this when starting a new movement or after a discontinuity
     */
    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
    }

    /**
     * @brief Enable or disable the filter
     * 
     * When disabled, process() returns input unchanged (bypass mode)
     * 
     * @param enable true to enable filtering, false to bypass
     */
    void setEnabled(bool enable) {
        if (enable && !enabled) {
            // Enabling: reset state to avoid transients
            reset();
        }
        enabled = enable;
    }

    /**
     * @brief Check if filter is currently enabled
     */
    bool isEnabled() const {
        return enabled;
    }

    /**
     * @brief Check if filter has been configured
     */
    bool isConfigured() const {
        return configured;
    }

    /**
     * @brief Get current center frequency
     */
    float getCenterFrequency() const {
        return center_freq_hz;
    }

    /**
     * @brief Get current quality factor
     */
    float getQuality() const {
        return quality;
    }

    /**
     * @brief Get attenuation at notch frequency (in dB, theoretical)
     * 
     * With ideal coefficients, attenuation approaches infinity.
     * In practice, limited by floating point precision.
     */
    float getNotchDepthDb() const {
        // Theoretical: infinite attenuation at exact notch frequency
        // Practical: depends on Q factor
        // Higher Q = deeper but narrower notch
        return -20.0f * log10f(1.0f - quality);
    }

    /**
     * @brief Get bandwidth of the notch (-3dB points)
     */
    float getBandwidthHz() const {
        // Approximate bandwidth based on Q factor
        // BW ≈ f0 × (1 - r²) / r
        float r = quality;
        return center_freq_hz * (1.0f - r * r) / r;
    }
};

#endif // NOTCH_FILTER_H

