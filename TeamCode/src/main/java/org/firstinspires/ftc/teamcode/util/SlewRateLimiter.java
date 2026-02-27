package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.util.Timer;

public class SlewRateLimiter {


    boolean firstLoop = true;
    double pT, rate_limit, lastTarget;
    Timer timer;
    public SlewRateLimiter(double _rate_limit){
        firstLoop = true;
        rate_limit = _rate_limit;
    }

    public void reset(){firstLoop = true;}

    public double step(double target){
        if (firstLoop){
            pT = timer.getElapsedTime();
            lastTarget = target;
            firstLoop = false;
        }

        double dt = (timer.getElapsedTime() - pT)*0.001d;

        double delta = target - lastTarget;

        double max_change = rate_limit * dt;


        double clamped_change = Math.max(-max_change , Math.min(max_change, delta));

        pT = timer.getElapsedTime();
        lastTarget = target;


        return lastTarget + clamped_change;
    }
}
