package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Shooter;
import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ActionPress;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name="ShooterPID", group = "Tests")
public class shooterPID extends OpMode {

    TelemetryManager telemetryManager;
    Follower follower;
    public static double P, I, D, F, TARGET_VEL;
    public static boolean LOAD = true;

    Shooter s;



    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        s = new Shooter(hardwareMap, telemetry, follower, true, Assembly.SIDE_BLUE);
        if (LOAD){
            P = s.flywheelP;I=s.flyWheelI; D= s.flyWheelD; F=s.flyWheelF;
        }

        s.turret.mode = Turret.IDLE_MODE;
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        s.flywheelP = P; s.flyWheelD = D; s.flyWheelI = I; s.flyWheelF = F;
        s.setFlywheelVel(TARGET_VEL);

        if (gamepad1.a && !s.shooting){
            s.Shoot();
        }
        if (gamepad1.left_bumper){
            s.setIntakeMotorPower(-1);
        }
        if (gamepad1.right_bumper){
            s.setIntakeMotorPower(0);
        }

        double speed = 0.8;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
                -Math.pow(gamepad1.right_stick_x, 3) * speed * 0.8,
                true // Robot Centric
        );

        if (gamepad1.dpadDownWasPressed()){
            follower.setPose(new Pose(135 , 8.5, Math.toRadians(180)));
        }


        s.update();


        telemetryManager.addData("Error", s.flywheelVelE);
        telemetryManager.addData("TagSize", s.TagSize);
        telemetryManager.addData("CamError", Math.abs(s.turret.Tx));


        telemetryManager.update(telemetry);


        telemetry.update();
        follower.update();
    }
}
