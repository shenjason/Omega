package org.firstinspires.ftc.teamcode.opModes.tests;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;

@Configurable
@TeleOp(name="TurretPID", group = "Tests")
public class turretPID extends OpMode {
    TelemetryManager telemetryManager;


    public static double P, I, D, F, TARGET_ROTATION;


    public static boolean LOAD = false;

    Turret t;

    Follower follower;

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        t = new Turret(hardwareMap, telemetry, follower, true, Assembly.SIDE_BLUE);

        follower.setStartingPose(new Pose(0,0));

        t.mode = Turret.IDLE_MODE;

        if (LOAD){
            P = t.P;I=t.I; D= t.D; F=t.F;
        }
    }

    @Override
    public void loop() {
        t.P = P;t.I = I; t.D = D; t.F = F;
        t.debugTargetAngle = Math.toRadians(TARGET_ROTATION);

        telemetryManager.addData("Error", t.turretController.getE());
        telemetryManager.addData("Output", t.turretController.currentOutput);

        telemetryManager.update();

        telemetry.update();
        follower.update();
        t.update();
    }
}
