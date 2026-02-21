package org.firstinspires.ftc.teamcode.opModes;

import android.health.connect.datatypes.units.Power;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;

@Autonomous(name = "Auto Blue (Far)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Blue Far Start)")
@Configurable // Panels
public class autoBlue12F extends OpMode {

    public boolean SIDE = Assembly.SIDE_BLUE;
    public double ROT = Math.toRadians(180);
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    protected AutoPaths paths; // Paths defined in the Paths class

    public Robot robot;
    public static double SPEED = 0.8;
    Timer timer, shooterTimeoutTimer;

    public void setSIDE(){}


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        setSIDE();

        robot = new Robot(hardwareMap, telemetry,follower,false, SIDE);

        follower.setMaxPower(SPEED);
        follower.setMaxPowerScaling(SPEED);

        paths = new AutoPaths(follower, SIDE, ROT); // Build paths

        follower.setStartingPose(paths.x(paths.startPose_far));
        robot.intake(false);


        panelsTelemetry.debug("Status", "Initialized");

        pathState = 0;
        timer = new Timer();
        shooterTimeoutTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        robot.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update();
        telemetry.update();
    }


    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                robot.shooter.setFlywheelVel(1470);
                robot.shooter.turret.debugTargetAngle = Math.toRadians(60) * ((SIDE) ? 1 : -1);
                follower.followPath(paths.start_shoot_far, true);
                shooterTimeoutTimer.resetTimer();
                pathState++;
                break;
            case 2:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready1_far,true);
                    pathState++;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.intake(true);
                    follower.followPath(paths.ready1_load1_far, true);
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.intake(false);
                    follower.followPath(paths.load1_shoot_far, true);
                    shooterTimeoutTimer.resetTimer();
                    pathState++;
                }
                break;
            case 1:
            case 5:
            case 9:
                if (!follower.isBusy() && (robot.shooter.atTargetFlywheelRPM() || shooterTimeoutTimer.getElapsedTimeSeconds() > 3)) {
                    robot.shooter.delayedShootSequence.start();
                    pathState++;
                }
                break;
            case 6:
                if (!robot.shooter.shooting) {
                    follower.followPath(paths.shoot_ready2_far, true);
                    pathState++;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    robot.intake(true);
                    follower.followPath(paths.ready2_load2_far, true);
                    timer.resetTimer();
                    pathState++;
                }
                break;
            case 8 :
                if(!follower.isBusy() && timer.getElapsedTimeSeconds() > 4){
                    robot.intake(false);
                    follower.followPath(paths.load2_shoot_far,true);
                    shooterTimeoutTimer.resetTimer();
                    pathState++;
                }
                break;
            case 10:
                if(!robot.shooter.shooting){
                    robot.shooter.setFlywheelVel(0);
                    robot.shooter.turret.debugTargetAngle = Math.toRadians(0) * ((SIDE) ? 1 : -1);
                    follower.followPath(paths.shoot_end_far,true);
                    pathState=-1;
                }
                break;
        }

        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        return pathState;
    }
}
