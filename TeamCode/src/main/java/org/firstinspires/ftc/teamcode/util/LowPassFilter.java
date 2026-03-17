package org.firstinspires.ftc.teamcode.util;


/**
 * LowPassFilter - A simple first-order exponential moving average (EMA) filter.
 *
 * Smooths noisy sensor data (e.g., accelerometer readings) by blending the
 * current value with the previous filtered value.
 *
 * Formula: output = alpha * current + (1 - alpha) * previous
 *
 * Alpha controls responsiveness:
 *   alpha = 1.0  -> No filtering (raw input passes through)
 *   alpha = 0.0  -> Maximum filtering (output never changes)
 *   alpha = 0.2  -> Heavy smoothing (used for acceleration data)
 */
public class LowPassFilter {
    boolean firstLoop;
    /** Filter coefficient (0-1) and previous filtered value */
    double alpha, lastValue;

    /**
     * Creates a new LowPassFilter.
     * @param _alpha Smoothing factor between 0 and 1 (lower = smoother, higher = more responsive)
     */
    public LowPassFilter(double _alpha){
        firstLoop = true;
        alpha = _alpha;
    }

    /** Resets the filter state (next call will initialize with the new value) */
    public void reset(){firstLoop = true;}

    /**
     * Processes one sample through the filter.
     * @param value The current raw sensor value
     * @return The filtered (smoothed) value
     */
    public double step(double value){
        if (firstLoop){
            lastValue = value;
            firstLoop = false;
        }

        double _lastValue = lastValue;
        lastValue = value;

        // EMA formula: alpha * current + (1 - alpha) * previous
        return alpha * value + (1-alpha)*_lastValue;
    }
}
