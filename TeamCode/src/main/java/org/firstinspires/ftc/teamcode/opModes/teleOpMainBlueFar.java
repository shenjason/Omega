package org.firstinspires.ftc.teamcode.opModes;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Assembly;

@TeleOp(name = "TeleOpMain(Blue Far Start)", group = "TeleOp")
public class teleOpMainBlueFar extends teleOpMainBlue {
    @Override
    public void setSIDE(){
        SIDE = Assembly.SIDE_BLUE;
        follower.setStartingPose(new Pose((SIDE) ? 36 : 108, 36, (SIDE) ? Math.toRadians(180) : 0));
    }

}
