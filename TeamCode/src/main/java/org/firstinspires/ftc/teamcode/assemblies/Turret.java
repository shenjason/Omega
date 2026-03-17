package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.ActionPress;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.opencv.core.Mat;


/**
 * Turret - Controls the rotating turret with Limelight 3A vision-based target tracking.
 *
 * Operates in two modes:
 *   IDLE_MODE (1):     Estimates turret angle using odometry-based robot position and
 *                      known goal coordinates. The turret points toward the goal based
 *                      purely on the robot's pose from Pedro Pathing.
 *
 *   TRACKING_MODE (0): When the Limelight camera detects an AprilTag, fine-tunes the
 *                      turret angle using real-time camera feedback (Tx offset) via a
 *                      secondary PID controller. Falls back to estimation when no target
 *                      is visible.
 *
 * The turret has a +/-65 degree rotation limit and uses a slew rate limiter (30 deg/sec)
 * for smooth fine-tune adjustments.
 */
public class Turret extends Assembly {
    // --- Primary PID (estimation mode) ---
    public double P=2.5,I=0,D=0.1,F=0.2;
    // --- Secondary PID (camera fine-tune mode) ---
    public double _P=1.2,_I=0,_D=0.02, _F = 0.2;
    public PIDcontroller turretController, turretControllerSecondary;

    /** Whether the Limelight camera currently sees a valid target */
    public boolean isInCamera, atLimit;
    /** Ta = AprilTag area (used for RPM calculation), Tx = horizontal pixel offset from center */
    public double Ta, Tx;
    /** Current operating mode: TRACKING_MODE (0) or IDLE_MODE (1) */
    public int mode = IDLE_MODE;
    /** Manual offset angle, debug target angle, prediction fine-tune offset, and target goal X coordinate */
    public double offsetAngle = 0, debugTargetAngle=0, fineTuneOffsetAngle=0, targetPointX;



    private Limelight3A limelight;
    private DcMotor turretMotor;
    private Follower follower;


    private double currentRotation, targetRotation;

    // --- Limelight Pipeline Indices ---
    // GPP/PGP/PPG = color pattern pipelines for DECODE artifact detection
    // BLUE/RED_TARGET_LINE = alliance-specific goal tracking pipelines
    public final static int GPP = 0, PGP = 1, PPG = 2, BLUE_TARGET_LINE = 3, RED_TARGET_LINE = 4, TRACKING_MODE = 0, IDLE_MODE = 1;

    /** Goal coordinates on the field (in inches). Blue goal at x=16, Red goal mirrored. Y = 131 for both. */
    public static final double TARGET_BLUE_X=16, TARGET_Y=131, TARGET_RED_X=144-TARGET_BLUE_X;

    /** Maximum turret rotation in degrees and slew rate limit in deg/sec for fine-tuning */
    public final double angleLimit=65, target_rate_limit=30;

    Timer cameraTTimer;
    /** Rate limiter to smooth fine-tune turret adjustments (prevents jerky motion) */
    SlewRateLimiter turretFineTuneRateLimiter;



    public Turret(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        follower = f;
    }

    @Override
    public void hardwareInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        // Reset encoder to establish zero position at startup
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.setPollRateHz(40); // 40Hz camera polling for responsive tracking

        // Initialize both PID controllers with output clamped to [-1, 1]
        turretController = new PIDcontroller(P, I, D, F, -1, 1);
        turretControllerSecondary = new PIDcontroller(_P, _I, _D, _F, -1, 1);

        // Select the appropriate Limelight pipeline based on alliance side
        limelight.pipelineSwitch((side) ? BLUE_TARGET_LINE : RED_TARGET_LINE);
        targetPointX = (side) ? TARGET_BLUE_X : TARGET_RED_X;
        limelight.start();

        cameraTTimer = new Timer();
        turretFineTuneRateLimiter = new SlewRateLimiter(target_rate_limit);
    }


    /**
     * Main update loop for the turret subsystem.
     * Reads camera data, converts encoder position to rotation angle,
     * and delegates to either fine-tune (camera) or estimation (odometry) control.
     */
    @Override
    public void update() {

        // --- Camera Data Collection ---
        LLResult llResult = limelight.getLatestResult();

        isInCamera = limelightResultVaild(llResult);

        if (isInCamera) {
            Ta = llResult.getTa(); // AprilTag area (larger = closer)
            Tx = llResult.getTx(); // Horizontal offset from camera center (degrees)
        }else {
            Ta = 0; Tx = 0;
        }

        // Convert motor encoder position to turret rotation angle (radians)
        // Formula: (ticks / 384.5 ticks/rev) * (0.16 gear ratio) * 2pi - offset
        currentRotation = (turretMotor.getCurrentPosition()/384.5d*0.16d*2d*Math.PI - offsetAngle);


        debugAddLine("Turret");
        debugAddData("InFrame", isInCamera);
        debugAddData("Mode", mode);
        debugAddData("Tx", Tx);
        debugAddData("Error", Math.toDegrees(turretController.getE()));
        debugAddData("Output", turretController.currentOutput);
        debugAddData("isPointed",atTargetPosition());

        // Use camera fine-tuning when target is visible AND in tracking mode,
        // otherwise fall back to odometry-based estimation
        if (isInCamera && mode==Turret.TRACKING_MODE){fineTuneTurretRotation();}
        else{estimateTurretRotation();}
    }

    /**
     * Camera-assisted fine-tune mode.
     * Uses the Limelight's Tx value (horizontal offset in degrees) to make small
     * corrections to the turret angle via the secondary PID controller.
     * A dead zone of +/-1.5 degrees prevents oscillation when nearly on target.
     * Rate limiting smooths rapid angle changes from the prediction system.
     */
    public void fineTuneTurretRotation(){
        double limitedTx = Math.toRadians(Tx);

        double targetRotation = currentRotation + limitedTx;

        turretControllerSecondary.p = _P; turretControllerSecondary.i = _I; turretControllerSecondary.d = _D; turretControllerSecondary.f = F;

        // Dead zone: don't correct if Tx is within +/-1.5 degrees (prevents jitter)
        double power = (Math.abs(Tx) <= 1.5) ? 0 : turretControllerSecondary.step(turretFineTuneRateLimiter.step(fineTuneOffsetAngle), -limitedTx);
        atLimit = false;

        // If turret would exceed rotation limit, clamp to the limit angle
        if (Math.abs(targetRotation) > Math.toRadians(angleLimit)){
            atLimit = true;
            power = turretController.step(Math.signum(targetRotation) * Math.toRadians(angleLimit), currentRotation);
        }else{ turretController.reset();}


        turretMotor.setPower(power);

        debugAddData("Fine Tune Offset", fineTuneOffsetAngle);
        debugAddData("Fine Tune Error", turretControllerSecondary.getE());
        debugAddData("Fine Tune Power", turretControllerSecondary.currentOutput);
    }


    /**
     * Odometry-based estimation mode.
     * Calculates the angle from the robot's current position to the goal using
     * atan2, then computes the turret rotation needed relative to the robot's heading.
     * Used when the camera doesn't see the target, or in idle mode.
     */
    public void estimateTurretRotation(){
        turretControllerSecondary.reset();

        Pose cp = follower.getPose();

        double X = cp.getX();
        double Y = cp.getY();

        double robotAngle = cp.getHeading();

        // Calculate global angle from robot to goal
        double angle = getAngle(X,Y,targetPointX,TARGET_Y);

        // Convert global angle to turret-relative rotation
        targetRotation = getDiff(angle,robotAngle);

        // Clamp rotation to within the turret's physical limits
        double clamped_target_rot = targetRotation;
        if (Math.abs(targetRotation)>=Math.toRadians(angleLimit)){
            clamped_target_rot = Math.toRadians(angleLimit)*Math.signum(targetRotation);
        }

        turretController.p = P; turretController.i = I; turretController.d = D; turretController.f = F;

        // In tracking mode: aim at the clamped target. In idle mode: use debug angle (for testing).
        turretMotor.setPower(turretController.step((mode == Turret.TRACKING_MODE) ? -clamped_target_rot : debugTargetAngle, currentRotation));

        debugAddData("PoseX", cp.getX());
        debugAddData("PoseY", cp.getY());
        debugAddData("Heading", Math.toDegrees(cp.getHeading()));
        debugAddData("globalTargetRotation", Math.toDegrees(angle));
        debugAddData("targetRotation",  Math.toDegrees(clamped_target_rot));
        debugAddData("currentPos", turretMotor.getCurrentPosition());
        debugAddData("current angle",Math.toDegrees(currentRotation));

    }

    /** Switches Limelight pipeline and waits for a result from the new pipeline */
    public LLResult limelightGetResult(int pipeline_index) {
        limelight.pipelineSwitch(pipeline_index);
        LLResult result = limelight.getLatestResult();
        while (result.getPipelineIndex() != pipeline_index) result = limelight.getLatestResult();
        return result;
    }

    /** Validates a Limelight result (non-null and valid) */
    public boolean limelightResultVaild(LLResult result) {
        return (result != null && result.isValid());
    }

    /**
     * Determines which color pattern (GPP, PGP, or PPG) is visible.
     * Cycles through each pipeline to find a valid detection.
     * @return Pipeline index of the detected pattern, or -1 if none found
     */
    public int determinePattern() {
        if (limelightResultVaild(limelightGetResult(PGP))) {
            return PGP;
        }
        if (limelightResultVaild(limelightGetResult(GPP))) {
            return GPP;
        }
        if (limelightResultVaild(limelightGetResult(PPG))) {
            return PPG;
        }
        return -1;
    }

    /** Returns true if the turret PID error is within +/-2 degrees of target */
    public boolean atTargetPosition(){
        return Math.abs(turretController.getE()) <= Math.toRadians(2);
    }

    /** Calculates the angle (radians) from point (x1,y1) to point (x2,y2) using atan2 */
    public static double getAngle(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1,x2 - x1);
    }

    /** Calculates the shortest angular difference between two angles (handles wraparound) */
    public static double getDiff(double angle1, double angle2){
        return ((angle1-angle2+Math.toRadians(180))%Math.toRadians(360)-Math.toRadians(180));
    }

    /** Returns true if turret is aimed at the goal (idle mode always returns true, tracking requires Tx < 8 deg) */
    public boolean isPointed(){
        return mode == IDLE_MODE || (Math.abs(Tx) <= 8 && isInCamera);
    }

    /** Finds the closest point on a line (y = slope*x + intercept) to a given point (x0, y0) */
    public Pose closestPointOnLine(double slope, double intercept, double x0, double y0, double heading){
        double x = (x0 + slope * (y0 - intercept)) / (1 + slope * slope);
        double y = slope * x + intercept;

        return new Pose(x, y, heading);
    }

    /** Resets the turret encoder position to zero (re-establishes the center reference point) */
    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        offsetAngle = 0;
    }
}
