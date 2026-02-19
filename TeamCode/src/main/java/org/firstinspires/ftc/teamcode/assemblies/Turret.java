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
import org.opencv.core.Mat;


public class Turret extends Assembly {
    public double P=2.1,I=0.0,D=0.45,F=0.04;
    public double _P=1.8,_I=0.0,_D=0.6;
    public PIDcontroller turretController, turretControllerSecondary;
    public boolean isInCamera, atLimit;
    private double targetRotation;
    public double Ta;
    public double Tx;
    private Limelight3A limelight;

    private DcMotor turretMotor;
    private Follower follower;

    private double targetPointX, currentRotation;

    private final double TARGETBLUEX=16, TARGETY=131, TARGETREDX=144-TARGETBLUEX;

    public final static int GPP = 0, PGP = 1, PPG = 2, BLUE_TARGET_LINE = 3, RED_TARGET_LINE = 4, TRACKING_MODE = 0, IDLE_MODE = 1;
    public int mode = IDLE_MODE;
    public double offsetAngle = 0;

    public double debugTargetAngle = 0;

    public TouchSensor magneticSensor;

    Timer cameraTTimer;

    ActionPress magneticSwitchReset;


    public Turret(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        follower = f;
    }

    @Override
    public void hardwareInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
//        magneticSensor = hardwareMap.get(TouchSensor.class, "magneticSensor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.setPollRateHz(40);

        turretController = new PIDcontroller(P, I, D, F, -1, 1);
        turretControllerSecondary = new PIDcontroller(_P, _I, _D, F, -1, 1);

        limelight.pipelineSwitch((side) ? BLUE_TARGET_LINE : RED_TARGET_LINE);
        targetPointX = (side) ? TARGETBLUEX : TARGETREDX;
        limelight.start();

        cameraTTimer = new Timer();

//        magneticSwitchReset = new ActionPress(this::resetEncoder);
    }


    @Override
    public void update() {
//        magneticSwitchReset.update(magneticSensor.isPressed(), (mode==Turret.IDLE_MODE));


        //Camera Data collection
        LLResult llResult = limelight.getLatestResult();

        isInCamera = limelightResultVaild(llResult);

        if (isInCamera) {
            Ta = llResult.getTa(); Tx = llResult.getTx();
        }else {
            Ta = 0; Tx = 0;
        }

        currentRotation = (turretMotor.getCurrentPosition()/384.5d*0.16d*2d*Math.PI - offsetAngle);


        debugAddLine("Turret");
        debugAddData("InFrame", isInCamera);
        debugAddData("Mode", mode);
        debugAddData("Tx", Tx);
        debugAddData("Error", Math.toDegrees(turretController.getE()));
        debugAddData("Output", turretController.currentOutput);
        debugAddData("isPointed",atTargetPosition());

        if (isInCamera && mode==Turret.TRACKING_MODE){
            fineTuneTurretRotation();
        }else{
            estimateTurretRotation();
        }
    }

    public void fineTuneTurretRotation(){
        double limitedTx = Math.toRadians(Tx);

        double targetRotation = currentRotation + limitedTx;
 
        turretControllerSecondary.p = _P; turretControllerSecondary.i = _I; turretControllerSecondary.d = _D; turretControllerSecondary.f = F;

        double power = (Math.abs(Tx) <= 1.5) ? 0 : turretControllerSecondary.step(0, -limitedTx);
        atLimit = false;
        if (Math.abs(targetRotation) > Math.toRadians(70)){
            atLimit = true;
            power = turretController.step(Math.signum(targetRotation) * Math.toRadians(70), currentRotation);
        }


        turretMotor.setPower(power);


        debugAddData("Fineerror", turretControllerSecondary.getE());
        debugAddData("FinePower", turretControllerSecondary.currentOutput);
    }


    public void estimateTurretRotation(){

        Pose cp = follower.getPose();

        double X = cp.getX();
        double Y = cp.getY();

        double robotAngle = cp.getHeading();


        double angle = getAngle(X,Y,targetPointX,TARGETY);


        targetRotation = getDiff(angle,robotAngle);
        double clamped_target_rot = targetRotation;
        if (Math.abs(targetRotation)>=Math.toRadians(70)){
            clamped_target_rot = Math.toRadians(70)*Math.signum(targetRotation);
        }

        turretController.p = P; turretController.i = I; turretController.d = D; turretController.f = F;


        turretMotor.setPower(turretController.step((mode == Turret.TRACKING_MODE) ? -clamped_target_rot : debugTargetAngle, currentRotation));

        debugAddData("PoseX", cp.getX());
        debugAddData("PoseY", cp.getY());
        debugAddData("Heading", Math.toDegrees(cp.getHeading()));
        debugAddData("globalTargetRotation", Math.toDegrees(angle));
        debugAddData("targetRotation",  Math.toDegrees(clamped_target_rot));
        debugAddData("currentPos", turretMotor.getCurrentPosition());
        debugAddData("current angle",Math.toDegrees(currentRotation));

    }

    public LLResult limelightGetResult(int pipeline_index) {
        limelight.pipelineSwitch(pipeline_index);
        LLResult result = limelight.getLatestResult();
        while (result.getPipelineIndex() != pipeline_index) result = limelight.getLatestResult();
        return result;
    }

    public boolean limelightResultVaild(LLResult result) {
        return (result != null && result.isValid());
    }

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

    public boolean atTargetPosition(){
        return Math.abs(turretController.getE()) <= Math.toRadians(2);
    }

    public static double getAngle(double x1, double y1, double x2, double y2) {
        return Math.atan2(y2 - y1,x2 - x1);
    }
    public static double getDiff(double angle1, double angle2){
        return ((angle1-angle2+Math.toRadians(180))%Math.toRadians(360)-Math.toRadians(180));
    }
    public boolean isPointed(){
        return mode == IDLE_MODE || (Math.abs(Tx) <= 8 && isInCamera);
    }

    public Pose closestPointOnLine(double slope, double intercept, double x0, double y0, double heading){
        double x = (x0 + slope * (y0 - intercept)) / (1 + slope * slope);
        double y = slope * x + intercept;

        return new Pose(x, y, heading);
    }


    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        offsetAngle = 0;
    }
}