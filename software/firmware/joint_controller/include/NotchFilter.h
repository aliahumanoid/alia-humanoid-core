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
 * Transfer function (z-domain):
 *         1 - 2·cos(ω₀)·z⁻¹ + z⁻²
 * H(z) = ────────────────────────────
 *         1 - 2·r·cos(ω₀)·z⁻¹ + r²·z⁻²
 * 
 * where:
 *   ω₀ = 2π·f₀/fs (normalized center frequency)
 *   r  = quality factor (0.8-0.99, higher = narrower notch)
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
        center_freq_hz(8.0f),
        sample_freq_hz(500.0f),
        quality(0.90f),
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
        quality = constrain(q, 0.5f, 0.99f);
        
        // Normalized angular frequency
        float omega = 2.0f * M_PI * notch_freq_hz / sample_rate_hz;
        float cos_omega = cosf(omega);
        float r = quality;
        
        // Calculate filter coefficients
        // Numerator: 1 - 2·cos(ω)·z⁻¹ + z⁻²
        b0 = 1.0f;
        b1 = -2.0f * cos_omega;
        b2 = 1.0f;
        
        // Denominator: 1 - 2·r·cos(ω)·z⁻¹ + r²·z⁻²
        a1 = -2.0f * r * cos_omega;
        a2 = r * r;
        
        // Normalize for unity gain at DC and high frequencies
        // This ensures the filter only affects the notch frequency
        float gain_dc = (b0 + b1 + b2) / (1.0f + a1 + a2);
        if (fabsf(gain_dc) > 0.001f) {
            b0 /= gain_dc;
            b1 /= gain_dc;
            b2 /= gain_dc;
        }
        
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

