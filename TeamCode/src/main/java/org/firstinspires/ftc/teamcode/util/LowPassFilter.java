package org.firstinspires.ftc.teamcode.util;


public class LowPassFilter {
    boolean firstLoop;
    double alpha, lastValue;
    public LowPassFilter(double _alpha){
        firstLoop = true;
        alpha = _alpha;
    }

    public void reset(){firstLoop = true;}

    public double step(double value){
        if (firstLoop){
            lastValue = value;
            firstLoop = false;
        }

        double _lastValue = lastValue;
        lastValue = value;

        return alpha * value + (1-alpha)*_lastValue;
    }
}
