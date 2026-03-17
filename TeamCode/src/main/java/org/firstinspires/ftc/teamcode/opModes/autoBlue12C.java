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

/**
 * Autonomous OpMode for Blue alliance, center starting position.
 * Collects and shoots up to 12 artifacts across 3 collection cycles.
 *
 * Strategy:
 *   1. Drive from start to shooting position while spinning up flywheel
 *   2. Shoot pre-loaded artifacts (turret fixed at 39 degrees)
 *   3. Cycle through 3 load stations:
 *      - Drive to ready position -> intake row -> drive to gate/shoot position -> shoot
 *   4. Park in opposing team's parking square
 *
 * In ideal conditions, scores 36+ autonomous points (excluding pattern bonuses).
 *
 * State machine uses pathState variable to track progress through the sequence.
 * Red alliance variant (autoRed12C) extends this and overrides SIDE/ROT.
 */
@Autonomous(name = "Auto Blue (12 artifact)", group = "Autonomous", preselectTeleOp = "TeleOpMain(Blue)")
@Configurable // Panels
public class autoBlue12C extends OpMode {

    public boolean SIDE = Assembly.SIDE_BLUE;
    /** Heading rotation offset — 180 degrees for Blue, 0 for Red (robots face opposite directions) */
    public double ROT = Math.toRadians(180);
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    /** Current state in the autonomous state machine */
    private int pathState;
    /** Pre-built autonomous path segments */
    protected AutoPaths paths;

    public Robot robot;
    /** Maximum drive speed during autonomous (0-1) */
    public static double SPEED = 0.8;
    /** General timer and shooter timeout timer (prevents infinite waiting for flywheel) */
    Timer timer, shooterTimeoutTimer;

    /** Override point for Red alliance subclass to change SIDE and ROT */
    public void setSIDE(){}


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        setSIDE();

        robot = new Robot(hardwareMap, telemetry,follower,false, SIDE);

        follower.setMaxPower(SPEED);
        follower.setMaxPowerScaling(SPEED);

        paths = new AutoPaths(follower, SIDE, ROT);

        // Set starting pose — mirrored for Red alliance
        follower.setStartingPose(SIDE ? (paths.startPose) : new Pose(144-paths.startPose.getX(), paths.startPose.getY(), Math.toRadians(128)));
        robot.intake(false);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pathState = 0;
        timer = new Timer(); shooterTimeoutTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update();
        robot.update();
        pathState = autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());

        telemetry.update();
    }


    /**
     * Autonomous state machine — advances through path segments and actions.
     *
     * State flow:
     *   0:  Start -> Shoot position (spin up flywheel, set turret to 39 deg)
     *   1:  Wait for flywheel ready, then shoot (also states 6, 10, 14)
     *   2:  Wait for shoot complete, drive to ready position 1
     *   3:  Drive to load station 1 with intake on
     *   4:  Drive from load 1 to gate (Bezier curve to avoid obstacles)
     *   5:  Wait at gate, then drive to shoot position
     *   6:  Shoot (cycle 1 complete)
     *   7-10: Cycle 2 (ready2 -> load2 -> shoot)
     *   11-14: Cycle 3 (ready3 -> load3 -> shoot)
     *   15: Drive to end/parking position
     *   16: Open gate, reset turret, complete
     */
    public int autonomousPathUpdate() {
        switch(pathState){
            case 0:
                // Initialize: spin up flywheel and set turret to fixed 39-degree angle
                robot.shooter.setFlywheelVel(1320);
                robot.shooter.turret.debugTargetAngle = Math.toRadians(37) * ((SIDE) ? 1 : -1);
                follower.followPath(paths.start_shoot, true);
                pathState++;
                shooterTimeoutTimer.resetTimer();
                break;

            // --- Shoot states (shared logic for all 4 shoot points) ---
            case 1:  // After initial drive
            case 6:  // After cycle 1 return
            case 10: // After cycle 2 return
            case 14: // After cycle 3 return
                // Wait until path complete AND (flywheel ready OR 2.5s timeout)
                if (!follower.isBusy() && (robot.shooter.atTargetFlywheelRPM() || shooterTimeoutTimer.getElapsedTimeSeconds() > 2.5)){
                    robot.shooter.shoot();
                    pathState++;
                }
                break;

            // --- Cycle 1: Load station 1 (via gate) ---
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
                    follower.followPath(paths.load1_gate,true); // Bezier curve around obstacles
                    timer.resetTimer();
                    pathState++;
                }
                break;
            case 5:{
                // Wait 1.5s at gate for artifacts to settle before returning to shoot
                if (!follower.isBusy() && timer.getElapsedTimeSeconds() > 1.5){
                    follower.followPath(paths.gate_shoot,true);
                    shooterTimeoutTimer.resetTimer();
                    pathState++;
                }
                break;
            }

            // --- Cycle 2: Load station 2 (direct path) ---
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

            // --- Cycle 3: Load station 3 (direct path) ---
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

            // --- End: Park and cleanup ---
            case 15:
                if(!robot.shooter.shooting){
                    robot.shooter.offShooter();
                    follower.followPath(paths.shoot_end,true);
                    pathState++;
                    timer.resetTimer();
                }
                break;
            case 16:
                // Final cleanup: open gate and center turret
                if (timer.getElapsedTimeSeconds() > 0.3){
                    robot.shooter.openGate();
                    robot.shooter.turret.debugTargetAngle = 0;
                    pathState = -1; // Autonomous complete
                }
                break;
        }

        return pathState;
    }
}
