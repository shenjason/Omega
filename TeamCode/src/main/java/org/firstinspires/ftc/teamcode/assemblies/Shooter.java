package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
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
    public double flywheelP = 2560, flyWheelI = 0, flyWheelD = 0, flyWheelF = 13.8d;
    //Servo pos
    final double OPEN_GATE_POS = 0.5, CLOSE_GATE_POS = 0.4;
    //Prediction Vars
    final double flyWheelDiameter = 0.072;
    final double flyWheelAngle = 40;
    final double artifactMass = 0.0748;
    final double dragCoff = .0037d;

    final double flyWheelEfficiencyCoff = 1.09;






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
            () -> setIntakeMotorPower(-0.7),
            () -> setIntakeMotorPower(0.9),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 0.7d, 0.1d, 0d, 0d
    ));

    public Sequencer delayedShootSequence = new Sequencer(List.of(
            () -> shooting = true,
            () -> shootSequence.start()
    ), List.of(
            0d,0.8d
    ));


    public void autoAdjustShooterParameters(){
        double vel = 1500;
        if (turret.isInCamera) vel = Math.round((202.1567 / Math.sqrt(TagSize) + 1166.6721) * 0.1) * 10;

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
        intakeMotor2.setPower(-power * Vk);
        intakeMotor1.setPower(power * Vk);
    }


    public double timeOfFlightPrediction(double distance, double flyWheelVel){
        if (flyWheelVel <= 0) return 10;
        double maxTimeOfFlight = 2;
        double x = 0;
        double exitVel = flyWheelEfficiencyCoff * (flyWheelVel/28) * flyWheelDiameter * Math.PI * 0.5d;
        debugAddData("ExitVel", exitVel * 39.37);
        double dt = 0.001;
        double currentVelX = exitVel * Math.cos(Math.toRadians(flyWheelAngle));
        debugAddData("ExitVelX", currentVelX * 39.37);
        double currentVelY = exitVel * Math.sin(Math.toRadians(flyWheelAngle));
        int count = 0;

        double t = 0;

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
        debugAddData("EndVelX", currentVelX * 39.37);
        debugAddData("EndVelY", currentVelY * 39.37);
        debugAddData("EndX", x * 39.37);
        return t;
    }

    public double calcDistanceFromDepot(){
        double X = follower.getPose().getX();
        double Y = follower.getPose().getY();

        return Math.sqrt(Math.pow((X - ((side) ? 0 : 144)), 2) + Math.pow(Y-144, 2));
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
        debugAddData("Flight Of Time: ", timeOfFlightPrediction(calcDistanceFromDepot(), targetFlyWheelVel));
        debugAddData("Distance from Depot: ", calcDistanceFromDepot());



        if (turret_active) turret.update();
        TagSize = turret.Ta;

        shootSequence.update();
        delayedShootSequence.update();
    }

}
