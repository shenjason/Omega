package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Assembly;
import org.firstinspires.ftc.teamcode.util.Sequencer;

import java.util.List;

public class Robot extends Assembly {
    public final double GREEN = 0.45, ORANGE = 0.33, RED = 0.28, BLUE = .611;
    public Shooter shooter;

    public Servo led1, led2;
    public DistanceSensor distanceSensor;

    public Robot(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        shooter = new Shooter(_hardwareMap, _t, f, _debug, _side);

        idle();
        intake(false);
    }

    public void shoot(){
        if (shooter.shooting) return;
        shooter.Shoot();
    }


    public void idle(){
        shooter.idleMode();
        shooter.offShooter();
    }

    public void tracking(){
        shooter.trackingMode();
    }
    public void intake(boolean state){
        if (shooter.shooting) return;
        if (state){
            shooter.closeGate();
            shooter.setIntakeMotorPower(-1);
            return;
        }
        shooter.setIntakeMotorPower(0);
    }



    @Override
    public void hardwareInit() {
        led1 = hardwareMap.get(Servo.class, "light1");
        led2 = hardwareMap.get(Servo.class, "light2");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");


        led1.setPosition(0);
    }


    @Override
    public void update() {

        shooter.update();

        if (shooter.turret.mode == Turret.TRACKING_MODE){
            if (shooter.turret.atLimit){
                led1.setPosition(ORANGE);
                led2.setPosition(ORANGE);
            }else {
                double state = (shooter.canShoot()) ? GREEN : (shooter.atTargetFlywheelRPMBroad()) ? BLUE : RED;
                led1.setPosition(state);
                led2.setPosition(state);
            }
        }else{
            double state = (distanceSensor.getDistance(DistanceUnit.MM) < 95) ? GREEN : RED;
            led1.setPosition(state);
            led2.setPosition(state);
        }
    }
}
