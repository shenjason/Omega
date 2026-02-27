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
    public double flywheelP = 640, flyWheelI = 0, flyWheelD = 0, flyWheelF = 12d;
    //Servo pos
    final double OPEN_GATE_POS = 0.5, CLOSE_GATE_POS = 0.4;
    //Prediction Vars
    final double flyWheelDiameter = 0.072;
    final double flyWheelAngle = 40;
    final double artifactMass = 0.0748;
    final double dragCoff = .0037d;

    final double flyWheelEfficiencyCoff = 1.09d;

    final double shooterLatency = 0.4d;

    //Prediction FlyWheel Vel adj
    final double a_k=0.0132663, b_k=3.10801, c_k=1076.338;
    final double cam_a = 342.85, cam_b = 1133.62;



    //Ref
    DcMotorEx flywheelMotor;
    DcMotor intakeMotor1, intakeMotor2;
    Servo gateServo;

    //Misc
    public double flywheelVelE = 0, targetFlyWheelVel = 0, goalX, goalY;

    public double TagSize, Vk=1;

    public Turret turret;
    Follower follower;
    public boolean turret_active = true, shooting = false;

    PIDFCoefficients pidfCoefficients;


    public Sequencer shootSequence = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-0.7),
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
            () -> setIntakeMotorPower(-0.4),
            () -> setIntakeMotorPower(0.8),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 1.2d, 0.1d, 0d, 0d
    ));


    public void autoAdjustShooterParameters(){
        double vel = 1500;
        if (turret.isInCamera) {
            vel = Math.round((cam_a / Math.sqrt(TagSize) + cam_b) * 0.1) * 10;
        }

        if (vel<1200) vel = 1200;
        turret.fineTuneOffsetAngle = 0;

        setFlywheelVel(vel);
    }


    public Shooter(HardwareMap _hardwareMap, Telemetry _t, Follower _follower, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);
        follower = _follower;
        turret = new Turret(hardwareMap, t, _follower, _debug, side);
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

        goalX = (side) ? 0 : 144;
        goalY = 144;

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


    public double timeOfFlightPrediction(double distance, double flyWheelVel){
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

    public double calcShooterVel(double d){
        return Math.round((a_k*d*d + b_k*d + c_k)*0.1d)*10d;
    }

    public double calcDistanceFromDepot(Pose p){
        double X = p.getX();
        double Y = p.getY();
        return Math.sqrt(Math.pow((X - goalX), 2) + Math.pow(Y-goalY, 2));
    }

    public double distanceBetween(Pose p1, Pose p2){
        return Math.sqrt(Math.pow(p2.getX() - p1.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }


    public void idleMode(){
        turret.mode = Turret.IDLE_MODE;
    }

    public void trackingMode(){
        turret.mode = Turret.TRACKING_MODE;
    }

    public void setShootingParms(){
        double posX_launch = follower.getPose().getX() + (follower.getVelocity().getXComponent() * shooterLatency) + (0.5d*follower.getAcceleration().getXComponent()*Math.pow(shooterLatency,2));
        double posY_launch = follower.getPose().getY() + (follower.getVelocity().getYComponent() * shooterLatency) + (0.5d*follower.getAcceleration().getYComponent()*Math.pow(shooterLatency,2));
        double velX_launch = follower.getVelocity().getXComponent() + (follower.getAcceleration().getXComponent() * shooterLatency);
        double velY_launch = follower.getVelocity().getYComponent() + (follower.getAcceleration().getYComponent() * shooterLatency);

        double distance_to_goal = calcDistanceFromDepot(follower.getPose());
        double target_shooter_vel = calcShooterVel(distance_to_goal);
        double flight_time = timeOfFlightPrediction(distance_to_goal, target_shooter_vel);

        double v_goal_posX = goalX; double v_goal_posY = goalY;

        for (int i=0; i<3; i++){
            v_goal_posX = goalX - (velX_launch * flight_time);
            v_goal_posY = goalY - (velY_launch * flight_time);

            distance_to_goal = distanceBetween(new Pose(posX_launch, posY_launch), new Pose(v_goal_posX, v_goal_posY));

            target_shooter_vel = calcShooterVel(distance_to_goal);
            flight_time = timeOfFlightPrediction(distance_to_goal, target_shooter_vel);
        }

        double target_global_angle = Turret.getAngle(posX_launch, posY_launch, v_goal_posX, v_goal_posY);
        double current_global_angle = Turret.getAngle(follower.getPose().getX(),follower.getPose().getY(),turret.targetPointX,Turret.TARGET_Y);

        turret.fineTuneOffsetAngle = Math.toDegrees(Turret.getDiff(current_global_angle, target_global_angle));
        setFlywheelVel(target_shooter_vel);

        debugAddLine(" ");
        debugAddLine("PredictionOutput");
        debugAddData("Offset", Math.toDegrees(Turret.getDiff(current_global_angle, target_global_angle)));
        debugAddData("ShooterVelocity", target_shooter_vel);

    }


    @Override
    public void update() {
        if (turret.mode == Turret.TRACKING_MODE && !shooting) {
            if (follower.getVelocity().getMagnitude() > 10 && turret.isInCamera){
                setShootingParms();
            }else{
                autoAdjustShooterParameters();
            }
        }


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
        debugAddData("Flight Of Time: ", timeOfFlightPrediction(calcDistanceFromDepot(follower.getPose()), targetFlyWheelVel));
        debugAddData("Distance from Depot: ", calcDistanceFromDepot(follower.getPose()));
        debugAddData("Robot Velocity: ", follower.getVelocity().getMagnitude());
        debugAddData("Robot Acceleration: ", follower.getAcceleration().getMagnitude());


        if (turret_active) turret.update();
        TagSize = turret.Ta;

        shootSequence.update();
        shootSequenceLong.update();
    }

}
