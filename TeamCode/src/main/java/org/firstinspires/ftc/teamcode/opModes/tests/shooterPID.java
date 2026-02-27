package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name="ShooterPID", group = "Tests")
public class shooterPID extends OpMode {

    TelemetryManager telemetryManager;
    Follower follower;
    Robot robot;
    public static double P, I, D, F, TARGET_VEL;
    public static boolean LOAD = true;




    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        robot = new Robot(hardwareMap, telemetry, follower, true, Assembly.SIDE_BLUE);

        follower.startTeleopDrive(true);
        if (LOAD){
            P = robot.shooter.flywheelP;I=robot.shooter.flyWheelI; D=robot.shooter.flyWheelD; F=robot.shooter.flyWheelF;
        }
    }

    @Override
    public void loop() {

        robot.shooter.flywheelP = P; robot.shooter.flyWheelD = D; robot.shooter.flyWheelI = I; robot.shooter.flyWheelF = F;
        robot.shooter.setFlywheelVel(TARGET_VEL);

        double speed = 0.8;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
               gamepad1.right_stick_x * speed * 0.8,
                true // Robot Centric
        );

        if (gamepad1.dpadDownWasPressed()){
            follower.setPose(new Pose(135 , 8.5, Math.toRadians(180)));
        }

        if (gamepad1.right_stick_button){
            robot.shooter.shoot();
        }

        if (gamepad1.dpadRightWasPressed()){
            robot.shooter.turret.offsetAngle += Math.toRadians(5);
        }
        if (gamepad1.dpadLeftWasPressed()){
            robot.shooter.turret.offsetAngle -= Math.toRadians(5);
        }

        robot.intake(gamepad1.left_bumper);

        telemetryManager.addData("Error", robot.shooter.flywheelVelE);
        telemetryManager.addData("TagSize", robot.shooter.TagSize);
        telemetryManager.addData("CamError", Math.abs(robot.shooter.turret.Tx));


        telemetryManager.update(telemetry);


        robot.update();

        telemetry.update();
    }
}
