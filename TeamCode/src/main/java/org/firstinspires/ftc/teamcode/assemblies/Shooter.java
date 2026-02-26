package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

public class Shooter extends Assembly {

    //PID
    public double NOMINAL_VOLTAGE = 12.0d;
    public double flywheelP = 2560, flyWheelI = 0, flyWheelD = 0, flyWheelF = 13.8d;
    //Servo pos
    final double OPEN_GATE_POS = 0.5, CLOSE_GATE_POS = 0.4;
    //Prediction Vars
    final double flyWheelDiameter = 0.072;
    final double flyWheelAngle = 40;
    final double artifactMass = 0.0748;
    final double dragCoff = .0037d;

    final double flyWheelEfficiencyCoff = 1.09d;

    //Prediction FlyWheel Vel adj
    final double a_k=0, b_k=0, c_k=0;
    final double cam_a = 202.1567, cam_b = 1166.6721;



    //Ref
    DcMotorEx flywheelMotor;
    DcMotor intakeMotor1, intakeMotor2;
    Servo gateServo;

    //Misc
    public double flywheelVelE = 0, targetFlyWheelVel = 0;

    public double TagSize, Vk;

    public Turret turret;
    Follower follower;
    public boolean turret_active = true, shooting = false;

    PIDFCoefficients pidfCoefficients;


    public Sequencer shootSequence = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-0.8),
            () -> setIntakeMotorPower(0.8),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 0.7d, 0.1d, 0d, 0d
    ));

    public Sequencer shootSequenceLong = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-0.5),
            () -> setIntakeMotorPower(0.8),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 1.2d, 0.1d, 0d, 0d
    ));


    public void autoAdjustShooterParameters(){
        double vel = calcShooterVel(follower.getPose());
        double vel2;
        if (turret.isInCamera) {
            vel2 = Math.round((cam_a / Math.sqrt(TagSize) + cam_b) * 0.1) * 10;
            debugAddData("velDiff", vel2 - vel);
            if (Math.abs(vel2 - vel) > 60){
                vel = vel2;
                debugAddData("velocityOff", vel2-vel);
            }
        }

        setFlywheelVel(vel);
    }


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, Follower _follower, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        follower = _follower;
        turret = new Turret(hardwareMap, t, _follower, false, side);
    }

    @Override
    public void hardwareInit() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");

        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        gateServo = hardwareMap.get(Servo.class, "gate");


        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidfCoefficients = new PIDFCoefficients(flywheelP, flyWheelI, flyWheelD, flyWheelF);

        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        closeGate();
    }


    public void setFlywheelVel(double vel){
        flywheelMotor.setZeroPowerBehavior((vel==0) ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        targetFlyWheelVel = vel;
    }

    public boolean atTargetFlywheelRPM(){
        return Math.abs(flywheelVelE) <= 20;
    }

    public boolean atTargetFlywheelRPMBroad(){
        return Math.abs(flywheelVelE) <= 40;
    }
    public boolean canShoot(){
        return atTargetFlywheelRPM()&& turret.isPointed();
    }

    public void Shoot(){
        if (!canShoot()) return;
        shootSequence.start();
    }

    public void openGate(){ gateServo.setPosition(OPEN_GATE_POS);}
    public void closeGate(){ gateServo.setPosition(CLOSE_GATE_POS);}

    public void offShooter() {
        setFlywheelVel(0);
    }

    public void setIntakeMotorPower(double power){
        intakeMotor2.setPower(power * Vk);
        intakeMotor1.setPower(power * Vk);
    }


    public double timeOfFlightPrediction(Pose p, double flyWheelVel){
        if (flyWheelVel <= 0) return 10;
        double maxTimeOfFlight = 2;
        double x = 0;
        double exitVel = flyWheelEfficiencyCoff * (flyWheelVel/28) * flyWheelDiameter * Math.PI * 0.5d;
//        debugAddData("ExitVel", exitVel * 39.37);
        double dt = 0.001;
        double currentVelX = exitVel * Math.cos(Math.toRadians(flyWheelAngle));
//        debugAddData("ExitVelX", currentVelX * 39.37);
        double currentVelY = exitVel * Math.sin(Math.toRadians(flyWheelAngle));
        int count = 0;

        double t = 0;
        double distance = calcDistanceFromDepot(p.getX(), p.getY());

        while (x < (distance / 39.37) && count < (maxTimeOfFlight / dt)){
            double currentVel = Math.sqrt(currentVelX * currentVelX + currentVelY * currentVelY);
            double accelX = -(dragCoff/artifactMass) * currentVel * currentVelX;
            double accelY = -9.81 -(dragCoff/artifactMass) * currentVel * currentVelY;

            currentVelX += accelX * dt;
            currentVelY += accelY * dt;

            x += currentVelX * dt;

            count++;
            t+=dt;
        }

        return t;
    }

    public double calcShooterVel(Pose p){
        double d = robotDistanceTo(p);
        return Math.round((a_k*d*d + b_k*d + c_k)*0.1d)*10d;
    }

    public double calcDistanceFromDepot(double X, double Y){
        return Math.sqrt(Math.pow((X - ((side) ? 0 : 144)), 2) + Math.pow(Y-144, 2));
    }

    public double robotDistanceTo(Pose p){
        return Math.sqrt(Math.pow(follower.getPose().getX() - p.getX(), 2) + Math.pow(follower.getPose().getY() - p.getY(), 2));
    }


    public void idleMode(){
        turret.mode = Turret.IDLE_MODE;
    }

    public void trackingMode(){
        turret.mode = Turret.TRACKING_MODE;
    }


    @Override
    public void update() {
        if (turret.mode == Turret.TRACKING_MODE && !shooting) autoAdjustShooterParameters();

        pidfCoefficients = new PIDFCoefficients(flywheelP, flyWheelI, flyWheelD, flyWheelF);


        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor.setVelocity(targetFlyWheelVel);

        flywheelVelE = targetFlyWheelVel - flywheelMotor.getVelocity();

        debugAddLine("Shooter");
        debugAddData("flyWheelVel", flywheelMotor.getVelocity());
        debugAddData("VelError", flywheelVelE);
        debugAddData("flywheelPowerOutput", flywheelMotor.getPower());
        debugAddData("TargetVel", targetFlyWheelVel);
        debugAddData("InFrame", turret.isInCamera);
        debugAddLine("Prediction");
        debugAddData("Flight Of Time: ", timeOfFlightPrediction(follower.getPose(), targetFlyWheelVel));
        debugAddData("Distance from Depot: ", calcDistanceFromDepot(follower.getPose().getX(), follower.getPose().getY()));


        if (turret_active) turret.update();
        TagSize = turret.Ta;

        shootSequence.update();
        shootSequenceLong.update();
    }

}
