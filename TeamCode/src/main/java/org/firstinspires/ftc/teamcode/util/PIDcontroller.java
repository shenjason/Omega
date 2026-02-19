package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

import org.opencv.core.Mat;

public class PIDcontroller {
    public double p, i, d, f, e, lowLimit, highLimit = 0d;

    final int T = 50;
    final double tau = 1;

    double i_term, d_term , prev_e, prev_measurement = 0d;
    public double currentOutput = 0d;
    Timer t;

    public PIDcontroller(double _p, double _i, double _d, double _f, double u_low_limit, double u_high_limit){
        p = _p;
        i = _i;
        d = _d;
        f = _f;
        t = new Timer();

        t.resetTimer();
        lowLimit = u_low_limit; highLimit = u_high_limit;
    }


    public double step(double setpoint, double measurement){
//        if (t.getElapsedTime() < T) return currentOutput;
        e = setpoint - measurement;

        i_term += 0.5d*i*T*(e+prev_e);
        i_term = Math.max(Math.min(i_term, highLimit), lowLimit);
        d_term = -(2d*d*(measurement-prev_measurement)
                + (2d*tau-T) * d_term)
                / (2d*tau+T);

        double f_term = f * Math.signum(e);

        currentOutput = p*e + i_term + d_term + f_term;


        currentOutput = Math.max(Math.min(currentOutput, highLimit), lowLimit);

        prev_e = e;
        prev_measurement = measurement;

//        t.resetTimer();

        return currentOutput;
    }

    public double getE(){
        return e;
    }


}
