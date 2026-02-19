package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Assembly {

    public static final boolean SIDE_RED = false;
    public static final boolean SIDE_BLUE = true;

    protected Telemetry t;
    protected HardwareMap hardwareMap;
    protected boolean debug, side = false;


    public Assembly(HardwareMap _hardwareMap, Telemetry _t, boolean _debug, boolean _side){
        t = _t;
        hardwareMap = _hardwareMap;
        debug = _debug;
        side = _side;

        hardwareInit();
    }

    public abstract void hardwareInit();

    public abstract void update();

    protected void debugAddLine(String line){
        if (!debug) return;
        t.addLine(line);
    }

    protected void debugAddData(String header, Object data){
        if (!debug) return;
        t.addData(header, data);
    }

}
