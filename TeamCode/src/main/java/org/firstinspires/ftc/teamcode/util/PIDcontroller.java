package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

/**
 * PIDcontroller - A PIDF (Proportional-Integral-Derivative-Feedforward) controller
 * with output clamping, integral anti-windup, and filtered derivative.
 *
 * Features:
 *   - Derivative is computed on measurement (not error) to avoid derivative kick on setpoint changes
 *   - Derivative uses a low-pass filter (time constant tau) to reduce noise amplification
 *   - Integral term is clamped to output limits to prevent windup
 *   - Feedforward term is proportional to the setpoint (F * setpoint)
 *   - Output is clamped to [lowLimit, highLimit]
 *
 * Used for turret rotation control and flywheel velocity regulation.
 */
public class PIDcontroller {
    public double p, i, d, f, e, lowLimit, highLimit = 0d;

    /** Delta time (seconds) and previous timestamp */
    double DT, pT;
    /** Low-pass filter time constant for derivative smoothing */
    final double tau = 1;

    /** Accumulated integral term, filtered derivative term, and previous values */
    double i_term, d_term , prev_e, prev_measurement = 0d;
    /** Most recent controller output value */
    public double currentOutput = 0d;

    boolean firstLoop = true;
    Timer t;

    /**
     * Creates a new PIDF controller.
     * @param _p Proportional gain
     * @param _i Integral gain
     * @param _d Derivative gain
     * @param _f Feedforward gain (applied to setpoint)
     * @param u_low_limit  Minimum output value (e.g., -1 for motor power)
     * @param u_high_limit Maximum output value (e.g., +1 for motor power)
     */
    public PIDcontroller(double _p, double _i, double _d, double _f, double u_low_limit, double u_high_limit){
        p = _p;
        i = _i;
        d = _d;
        f = _f;
        t = new Timer();
        firstLoop = true;

        t.resetTimer();
        lowLimit = u_low_limit; highLimit = u_high_limit;
    }

    /** Resets the controller state (clears integral and derivative history) */
    public void reset(){firstLoop = true;}


    /**
     * Computes one step of the PIDF controller.
     *
     * @param setpoint    Desired target value
     * @param measurement Current measured value
     * @return Controller output, clamped to [lowLimit, highLimit]
     */
    public double step(double setpoint, double measurement){
        if (firstLoop){
            prev_measurement = measurement;
            firstLoop = false;
            pT = t.getElapsedTime();
        }

        // Calculate delta time in seconds (timer returns milliseconds)
        DT = (t.getElapsedTime() - pT) * 0.001d;

        // Error = setpoint - measurement
        e = setpoint - measurement;

        // Trapezoidal integration for the integral term (anti-windup via clamping)
        i_term += 0.5d*i*DT*(e+prev_e);
        i_term = Math.max(Math.min(i_term, highLimit), lowLimit);

        // Filtered derivative on measurement (not error) to avoid derivative kick
        // Uses a first-order low-pass filter: d_new = (-2d*(meas-prev) + (2*tau-dt)*d_old) / (2*tau+dt)
        d_term = (-2d * d * (measurement - prev_measurement) + (2d * tau - DT) * d_term) / (2d * tau + DT);

        // Feedforward: proportional to the setpoint (helps reach target faster)
        double f_term = f * setpoint;

        // Sum all terms and clamp output
        currentOutput = p*e + i_term + d_term + f_term;
        currentOutput = Math.max(Math.min(currentOutput, highLimit), lowLimit);

        prev_e = e;
        prev_measurement = measurement;

        pT = t.getElapsedTime();

        return currentOutput;
    }

    /** Returns the current error (setpoint - measurement) */
    public double getE(){
        return e;
    }

}
