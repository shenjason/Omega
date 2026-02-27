package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;


@Configurable
@TeleOp(name = "TeleOpMain(Blue)", group = "TeleOp")
public class teleOpMainBlue extends OpMode {

    public static boolean SIDE = Assembly.SIDE_BLUE;
    public static boolean DEBUG = true;

    Follower follower;

    Robot robot;

    public void setSIDE() {
        follower.setStartingPose(new Pose((SIDE) ? 32 : 112, 72, (SIDE) ? Math.toRadians(180) : 0));
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        setSIDE();
        robot = new Robot(hardwareMap, telemetry, follower, DEBUG, SIDE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        robot.update();


        double speed = (gamepad1.right_bumper || gamepad1.right_trigger > 0) ? 1 : 0.5d;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
                -Math.pow(gamepad1.right_stick_x, 3) * speed * 0.8,
                true // Robot Centric
        );



        robot.intake(gamepad1.left_bumper);

        if (gamepad1.aWasPressed()){
            if (robot.shooter.turret.mode == Turret.IDLE_MODE){
                robot.tracking();
            }else{
                robot.idle();
            }
        }

        if (gamepad1.right_stick_button){
            robot.shoot();
        }

        if (gamepad1.dpadDownWasPressed()){
            follower.setPose(new Pose((SIDE) ? 135 : 9 , 8.5, (SIDE) ? Math.toRadians(180) : 0));
        }

        if (gamepad1.dpadRightWasPressed()){
            robot.shooter.turret.offsetAngle += Math.toRadians(5);
        }
        if (gamepad1.dpadLeftWasPressed()){
            robot.shooter.turret.offsetAngle -= Math.toRadians(5);
        }


        telemetry.update();

    }
}
