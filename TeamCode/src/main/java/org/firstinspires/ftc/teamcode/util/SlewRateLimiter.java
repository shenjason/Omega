package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

/**
 * SlewRateLimiter - Limits the rate of change of a value to prevent sudden jumps.
 *
 * Used for turret fine-tune angle adjustments to ensure smooth, controlled motion
 * rather than abrupt changes that could cause mechanical stress or overshoot.
 *
 * The maximum change per update cycle is: rate_limit * deltaTime (degrees/sec * sec).
 */
public class SlewRateLimiter {

    boolean firstLoop = true;
    /** Previous timestamp, maximum rate of change (units/sec), and last output value */
    double pT, rate_limit, lastTarget;
    Timer timer;

    /**
     * Creates a new SlewRateLimiter.
     * @param _rate_limit Maximum rate of change in units per second (e.g., 30 deg/sec)
     */
    public SlewRateLimiter(double _rate_limit){
        firstLoop = true;
        rate_limit = _rate_limit;
    }

    /** Resets the rate limiter state */
    public void reset(){firstLoop = true;}

    /**
     * Processes one step of the rate limiter.
     * Clamps the change from the last output to the target value based on the rate limit.
     *
     * @param target Desired target value
     * @return Rate-limited output value
     */
    public double step(double target){
        if (firstLoop){
            pT = timer.getElapsedTime();
            lastTarget = target;
            firstLoop = false;
        }

        // Calculate delta time in seconds
        double dt = (timer.getElapsedTime() - pT)*0.001d;

        double delta = target - lastTarget;

        // Maximum allowed change this cycle
        double max_change = rate_limit * dt;

        // Clamp the change to within the allowed range
        double clamped_change = Math.max(-max_change , Math.min(max_change, delta));

        pT = timer.getElapsedTime();
        lastTarget = target;

        return lastTarget + clamped_change;
    }
}
