package org.firstinspires.ftc.teamcode.assemblies;

import android.nfc.Tag;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.PIDcontroller;
import org.firstinspires.ftc.teamcode.util.Sequencer;

public class Shooter extends Assembly {
    public double flywheelP = 1280, flyWheelI = 0, flyWheelD = 0, flyWheelF = 13.8d;

    final double OPEN_GATE_POS = 0.5, CLOSE_GATE_POS = 0.4;

    DcMotorEx flywheelMotor;
    DcMotor intakeMotor1, intakeMotor2;
    Servo gateServo;
    public double flywheelVelE = 0, targetFlyWheelVel = 0;

    public double TagSize;



    public Turret turret;

    public boolean turret_active = true, shooting = false;

    PIDFCoefficients pidfCoefficients;


    public Sequencer shootSequence = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-0.9),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 0.7d, 0d, 0.3d, 0d
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


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, Follower follower, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        turret = new Turret(hardwareMap, t, follower, false, side);
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
        intakeMotor2.setPower(-power);
        intakeMotor1.setPower(power);
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

        if (turret_active) turret.update();
        TagSize = turret.Ta;

        shootSequence.update();
        delayedShootSequence.update();
    }

}
