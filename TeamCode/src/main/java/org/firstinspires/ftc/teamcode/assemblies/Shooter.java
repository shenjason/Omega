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
import org.firstinspires.ftc.teamcode.util.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.Sequencer;

/**
 * Shooter - Controls the flywheel launcher, intake motors, gate servo, and turret subsystem.
 *
 * Key responsibilities:
 *   - PIDF velocity control for the flywheel motor
 *   - Ballistic prediction: calculates required flywheel RPM based on distance to goal,
 *     accounting for projectile drag, gravity, and robot motion during flight
 *   - Adaptive RPM via Limelight camera: RPM ~ a / sqrt(TagSize) + b (R² = 0.9791)
 *   - Timed shoot sequences via the Sequencer utility (gate -> intake -> fire -> reset)
 *   - Voltage-compensated intake motor power
 */
public class Shooter extends Assembly {

    // --- Flywheel PIDF Coefficients ---
    public double flywheelP = 640, flyWheelI = 0, flyWheelD = 0, flyWheelF = 12d;

    // --- Gate Servo Positions ---
    final double OPEN_GATE_POS = 0.5, CLOSE_GATE_POS = 0.4;

    // --- Ballistic Prediction Constants ---
    final double flyWheelDiameter = 0.072;   // Flywheel diameter in meters (7.2 cm)
    final double flyWheelAngle = 40;          // Launch angle in degrees
    final double artifactMass = 0.0748;       // Artifact mass in kg (74.8g)
    final double dragCoff = .0037d;           // Aerodynamic drag coefficient

    final double flyWheelEfficiencyCoff = 1.09d; // Efficiency factor for flywheel-to-ball energy transfer

    final double shooterLatency = 0.4d;       // Time delay (seconds) between shoot command and ball exit

    // --- Flywheel Velocity Regression Coefficients ---
    // Quadratic model: vel = a*d^2 + b*d + c (distance-based prediction)
    final double a_k=0.0132663, b_k=3.10801, c_k=1076.338;
    // Camera-based model: vel = cam_a / sqrt(TagSize) + cam_b (AprilTag size-based)
    final double cam_a = 342.85, cam_b = 1133.62;

    /** Low-pass filter smoothing factor for acceleration data */
    final double accel_lowPass_alpha = 0.2;


    // --- Hardware References ---
    DcMotorEx flywheelMotor;
    DcMotor intakeMotor1, intakeMotor2;
    Servo gateServo;

    // --- State Variables ---
    /** Flywheel velocity error (target - actual) and target velocity in ticks/sec */
    public double flywheelVelE = 0, targetFlyWheelVel = 0, goalX, goalY;

    /** AprilTag size from Limelight (used for RPM calculation) and voltage compensation factor */
    public double TagSize, Vk=1;

    public Turret turret;
    Follower follower;
    /** Whether turret tracking is active and whether a shoot sequence is in progress */
    public boolean turret_active = true, shooting = false;

    PIDFCoefficients pidfCoefficients;


    /**
     * Standard shoot sequence: opens gate, reverses intake briefly to push artifact
     * into flywheel, then closes gate. Total duration ~0.8 seconds.
     * Timings (seconds between steps): 0, 0, 0, 0.7, 0.1, 0, 0
     */
    public Sequencer shootSequence = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-1),
            () -> setIntakeMotorPower(0.8),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 1d, 0.1d, 0d, 0d
    ));

    /** Extended shoot sequence with 1.2s forward intake duration (vs 0.7s standard) */
    public Sequencer shootSequenceLong = new Sequencer(List.of(
            () -> shooting = true,
            this::openGate,
            () -> setIntakeMotorPower(-0.75),
            () -> setIntakeMotorPower(0.8),
            this::closeGate,
            () -> setIntakeMotorPower(0),
            () -> shooting = false
    ), List.of(
            0d, 0d, 0d, 1.8d, 0.1d, 0d, 0d
    ));


    /**
     * Adjusts shooter RPM based on AprilTag size detected by Limelight camera.
     * Uses reciprocal square-root regression: RPM ~ cam_a / sqrt(TagSize) + cam_b
     * Falls back to 1500 RPM if camera doesn't have a target, minimum 1200 RPM.
     */
    public void autoAdjustShooterParameters(){
        double vel = calcShooterVel(calcDistanceFromDepot(follower.getPose()));
        if (turret.isInCamera) {
            vel = Math.round((cam_a / Math.sqrt(TagSize) + cam_b) * 0.1) * 10;
        }

        if (vel<1200) vel = 1200;

        turret.fineTuneOffsetAngle = 0;
        if (follower.getPose().getY() <= 32) turret.fineTuneOffsetAngle = (side == SIDE_BLUE) ? Math.toRadians(-1.5) : Math.toRadians(1.5);


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

        // Set goal position based on alliance side (Blue goal at x=0, Red goal at x=144)
        goalX = (side) ? 0 : 144;
        goalY = 144;

        // Initialize low-pass filters for acceleration smoothing
        accel_X_lowPass = new LowPassFilter(accel_lowPass_alpha);
        accel_Y_lowPass = new LowPassFilter(accel_lowPass_alpha);

        closeGate();
    }


    /** Sets flywheel target velocity. Float mode when stopped, brake mode when active. */
    public void setFlywheelVel(double vel){
        flywheelMotor.setZeroPowerBehavior((vel==0) ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
        targetFlyWheelVel = vel;
    }

    /** Returns true if flywheel velocity error is within +/-20 ticks/sec (tight tolerance) */
    public boolean atTargetFlywheelRPM(){
        return Math.abs(flywheelVelE) <= 20;
    }

    /** Returns true if flywheel velocity error is within +/-40 ticks/sec (broad tolerance for LED) */
    public boolean atTargetFlywheelRPMBroad(){
        return Math.abs(flywheelVelE) <= 40;
    }

    /** Returns true if both flywheel is at target RPM AND turret is aimed at goal */
    public boolean canShoot(){
        return (atTargetFlywheelRPM() && turret.isPointed()) || (isInEstimation && atTargetFlywheelRPMBroad());
    }

    /** Triggers the shoot sequence if not already shooting */
    public void shoot(){
        if (shooting) return;
        shootSequence.start();
    }
    public void longShoot(){
        if (shooting) return;
        shootSequenceLong.start();
    }

    public void openGate(){ gateServo.setPosition(OPEN_GATE_POS);}
    public void closeGate(){ gateServo.setPosition(CLOSE_GATE_POS);}

    /** Stops the flywheel motor */
    public void offShooter() {
        setFlywheelVel(0);
    }

    /** Sets intake motor power with voltage compensation (Vk) applied */
    public void setIntakeMotorPower(double power){
        intakeMotor2.setPower(power * Vk);
        intakeMotor1.setPower(power * Vk);
    }


    /**
     * Predicts the time of flight for a projectile using a physics simulation (1ms timestep).
     * Accounts for launch angle, aerodynamic drag, and gravity.
     *
     * @param distance    Distance to goal in inches
     * @param flyWheelVel Flywheel velocity in encoder ticks/sec
     * @return Time of flight in seconds
     */
    public double timeOfFlightPrediction(double distance, double flyWheelVel){
        if (flyWheelVel <= 0) return 10;
        double maxTimeOfFlight = 2;
        double x = 0;
        // Convert flywheel encoder velocity to ball exit velocity (m/s)
        double exitVel = flyWheelEfficiencyCoff * (flyWheelVel/28) * flyWheelDiameter * Math.PI * 0.5d;
        double dt = 0.001;
        double currentVelX = exitVel * Math.cos(Math.toRadians(flyWheelAngle));
        double currentVelY = exitVel * Math.sin(Math.toRadians(flyWheelAngle));
        int count = 0;

        double t = 0;

        // Simulate projectile motion until it reaches the target distance
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

    /** Calculates required flywheel velocity using quadratic regression on distance */
    public double calcShooterVel(double d){
        return Math.round((a_k*d*d + b_k*d + c_k)*0.1d)*10d;
    }

    /** Calculates Euclidean distance from a pose to the goal position (in inches) */
    public double calcDistanceFromDepot(Pose p){
        double X = p.getX();
        double Y = p.getY();
        return Math.sqrt(Math.pow((X - goalX), 2) + Math.pow(Y-goalY, 2));
    }

    /** Calculates Euclidean distance between two poses */
    public double distanceBetween(Pose p1, Pose p2){
        return Math.sqrt(Math.pow(p2.getX() - p1.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }


    /** Sets turret to idle mode (odometry-based estimation only) */
    public void idleMode(){turret.mode = Turret.IDLE_MODE;}

    /** Sets turret to tracking mode (camera-assisted fine-tuning) */
    public void trackingMode(){turret.mode = Turret.TRACKING_MODE;}

    /**
     * Advanced ballistic prediction — calculates optimal shooter parameters by predicting
     * where the robot will be at launch time and where the ball will land, compensating
     * for the robot's velocity and acceleration during flight.
     *
     * Uses 3 iterations of convergence on a "virtual goal" position that accounts for
     * robot movement during the projectile's time of flight.
     */
    public void setShootingParms(){
        // Low-pass filter acceleration to reduce sensor noise
        double filtered_accel_X = accel_X_lowPass.step(follower.getAcceleration().getXComponent());
        double filtered_accel_Y = accel_Y_lowPass.step(follower.getAcceleration().getYComponent());

        // Predict robot position at launch time: pos + vel*t + 0.5*accel*t^2
        double posX_launch = follower.getPose().getX() + (follower.getVelocity().getXComponent() * shooterLatency) + (0.5d*follower.getAcceleration().getXComponent()*Math.pow(shooterLatency,2));
        double posY_launch = follower.getPose().getY() + (follower.getVelocity().getYComponent() * shooterLatency) + (0.5d*follower.getAcceleration().getYComponent()*Math.pow(shooterLatency,2));

        // Predict robot velocity at launch time: vel + accel*t
        double velX_launch = follower.getVelocity().getXComponent() + (filtered_accel_X * shooterLatency);
        double velY_launch = follower.getVelocity().getYComponent() + (filtered_accel_Y * shooterLatency);

        double distance_to_goal = calcDistanceFromDepot(follower.getPose());
        double target_shooter_vel = calcShooterVel(distance_to_goal);
        double flight_time = timeOfFlightPrediction(distance_to_goal, target_shooter_vel);

        double v_goal_posX = goalX; double v_goal_posY = goalY;

        // Iterative convergence: adjust virtual goal for robot movement during flight
        for (int i=0; i<3; i++){
            v_goal_posX = goalX - (velX_launch * flight_time);
            v_goal_posY = goalY - (velY_launch * flight_time);

            distance_to_goal = distanceBetween(new Pose(posX_launch, posY_launch), new Pose(v_goal_posX, v_goal_posY));

            target_shooter_vel = calcShooterVel(distance_to_goal);
            flight_time = timeOfFlightPrediction(distance_to_goal, target_shooter_vel);
        }

        // Calculate turret offset angle to aim at the virtual goal
        double target_global_angle = Turret.getAngle(posX_launch, posY_launch, v_goal_posX, v_goal_posY);

        turret.forcedEstimation = true;
        turret.overrideAngle = target_global_angle;
        setFlywheelVel(target_shooter_vel);

        debugAddLine(" ");
        debugAddLine("PredictionOutput");
        debugAddData("Offset", Math.toDegrees(target_global_angle));
        debugAddData("ShooterVelocity", target_shooter_vel);

    }


    /**
     * Main update loop — adjusts flywheel velocity and turret angle based on
     * camera data (if visible) or prediction model (if not).
     */
    @Override
    public void update() {
        if (turret.mode == Turret.TRACKING_MODE && !shooting) {
            if (turret.isInCamera){
                setShootingParms(); // Full ballistic prediction with motion compensation
            }else{
                autoAdjustShooterParameters(); // Simple RPM estimation
            }
        }

        // Update flywheel PIDF coefficients (allows live tuning)
        pidfCoefficients = new PIDFCoefficients(flywheelP, flyWheelI, flyWheelD, flyWheelF);

        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor.setVelocity(targetFlyWheelVel);

        // Track velocity error for ready-state detection
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
        debugAddData("Robot X", follower.getPose().getX());
        debugAddData("Robot Y", follower.getPose().getY());
        debugAddData("Robot Velocity: ", follower.getVelocity().getMagnitude());
        debugAddData("Robot Acceleration: ", follower.getAcceleration().getMagnitude());

        // Update turret tracking and shoot sequences
        if (turret_active) turret.update();
        TagSize = turret.Ta;

        shootSequence.update();
        shootSequenceLong.update();
    }

}
