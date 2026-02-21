package org.firstinspires.ftc.teamcode.opModes;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Assembly;

@TeleOp(name = "TeleOpMain(Red Far Start)", group = "TeleOp")
public class teleOpMainRedFar extends teleOpMainBlue {
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_RED;
        follower.setStartingPose(new Pose((SIDE) ? 36 : 108, 36, (SIDE) ? Math.toRadians(180) : 0));
    }

}
