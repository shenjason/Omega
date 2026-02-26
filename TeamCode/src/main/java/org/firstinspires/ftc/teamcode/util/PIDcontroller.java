package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

public class PIDcontroller {
    public double p, i, d, f, e, lowLimit, highLimit = 0d;

    double DT, pT;
    final double tau = 1;

    double i_term, d_term , prev_e, prev_measurement = 0d;
    public double currentOutput = 0d;

    boolean firstLoop = true;
    Timer t;

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

    public void reset(){firstLoop = true;}


    public double step(double setpoint, double measurement){
        if (firstLoop){
            prev_measurement = measurement;
            firstLoop = false;
            pT = t.getElapsedTime();
        }

        DT = (t.getElapsedTime() - pT) * 0.001d;

        e = setpoint - measurement;

        i_term += 0.5d*i*DT*(e+prev_e);
        i_term = Math.max(Math.min(i_term, highLimit), lowLimit);
        d_term = (-2d * d * (measurement - prev_measurement) + (2d * tau - DT) * d_term) / (2d * tau + DT);

        double f_term = f * setpoint;

        currentOutput = p*e + i_term + d_term + f_term;


        currentOutput = Math.max(Math.min(currentOutput, highLimit), lowLimit);

        prev_e = e;
        prev_measurement = measurement;

        pT = t.getElapsedTime();

        return currentOutput;
    }

    public double getE(){
        return e;
    }

}
