package org.firstinspires.ftc.teamcode.opModes;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Assembly;

@TeleOp(name = "TeleOpMain(Red)", group = "TeleOp")
public class teleOpMainRed extends teleOpMainBlue {
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_RED;
        follower.setStartingPose(new Pose((SIDE) ? 32 : 112, 72, (SIDE) ? Math.toRadians(180) : 0));
    }

}
