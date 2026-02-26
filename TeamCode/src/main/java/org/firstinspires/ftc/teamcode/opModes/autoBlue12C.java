package org.firstinspires.ftc.teamcode.opModes;

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

@Autonomous(name = "Auto Blue (12 artifact)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Blue)")
@Configurable // Panels
public class autoBlue12C extends OpMode {

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

        follower.setStartingPose(SIDE ? (paths.startPose) : new Pose(144-paths.startPose.getX(), paths.startPose.getY(), Math.toRadians(128)));
        robot.intake(false);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pathState = 0;
        timer = new Timer(); shooterTimeoutTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        robot.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        //Log values to Panels and Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        telemetry.update();
    }


    public int autonomousPathUpdate() {
        switch(pathState){
            case 0:
                //set turret angle to 39 degrees
                robot.shooter.setFlywheelVel(1320);
                robot.shooter.turret.debugTargetAngle = Math.toRadians(39) * ((SIDE) ? 1 : -1);
                follower.followPath(paths.start_shoot, true);
                pathState++;
                shooterTimeoutTimer.resetTimer();
                break;
            case 1:
            case 6:
            case 10:
            case 14:
                if (!follower.isBusy() && (robot.shooter.atTargetFlywheelRPM() || shooterTimeoutTimer.getElapsedTimeSeconds() > 2.5)){
                    robot.shoot();
                    pathState++;
                }
                break;
            case 2:
                if (!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready1,true);
                    pathState++;
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    robot.intake(true);
                    follower.followPath(paths.ready1_load1,true);
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    robot.intake(false);
                    follower.followPath(paths.load1_gate,true);
                    timer.resetTimer();
                    pathState++;
                }
                break;
            case 5:{
                if (!follower.isBusy() && timer.getElapsedTimeSeconds() > 1.5){
                    follower.followPath(paths.gate_shoot,true);
                    shooterTimeoutTimer.resetTimer();
                    pathState++;
                }
                break;
            }
            case 7:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready2,true);
                    pathState++;
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    robot.intake(true);
                    follower.followPath(paths.ready2_load2,true);
                    pathState++;
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    robot.intake(false);
                    follower.followPath(paths.load2_shoot,true);
                    shooterTimeoutTimer.resetTimer();
                    pathState++;
                }
                break;
            case 11:
                if(!robot.shooter.shooting){
                    follower.followPath(paths.shoot_ready3,true);
                    pathState++;
                }
                break;
            case 12:

                if(!follower.isBusy()){
                    robot.intake(true);
                    follower.followPath(paths.ready3_load3,true);
                    pathState++;
                }
                break;
            case 13:
                if(!follower.isBusy()){
                    robot.intake(false);
                    follower.followPath(paths.load3_shoot,true);
                    shooterTimeoutTimer.resetTimer();
                    pathState++;
                }
                break;
            case 15:
                if(!robot.shooter.shooting){
                    robot.shooter.offShooter();
                    follower.followPath(paths.shoot_end,true);
                    pathState++;
                    timer.resetTimer();
                }
                break;
            case 16:
                if (timer.getElapsedTimeSeconds() > 0.3){
                    robot.shooter.openGate();
                    robot.shooter.turret.debugTargetAngle = 0;
                    pathState = -1;
                }
                break;
        }
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        return pathState;
    }
}
