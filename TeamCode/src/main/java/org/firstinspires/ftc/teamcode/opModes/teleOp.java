package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Assembly;

@TeleOp(name = "TeleOpMain(Red)", group = "TeleOp")
public class teleOp extends teleOpMain{
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_RED;
    }

}
