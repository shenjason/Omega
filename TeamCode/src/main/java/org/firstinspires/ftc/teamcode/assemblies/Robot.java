package org.firstinspires.ftc.teamcode.assemblies;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Assembly;

/**
 * Robot - Top-level assembly class for Team 23365's Delta robot.
 *
 * Manages the shooter subsystem, LED status indicators, voltage compensation,
 * and the Pedro Pathing follower. Acts as the central coordinator that OpModes
 * interact with to control all robot functions.
 *
 * LED Color Codes (servo position values):
 *   GREEN  (0.45)  - Ready to shoot (flywheel at RPM + turret aimed)
 *   BLUE   (0.611) - Flywheel at RPM but turret still aligning
 *   ORANGE (0.33)  - Turret at rotation limit
 *   RED    (0.28)  - Not ready
 */
public class Robot extends Assembly {
    // LED servo position values mapped to each status color
    public final double GREEN = 0.45, ORANGE = 0.33, RED = 0.28, BLUE = .611;
    public Shooter shooter;

    public Servo led1, led2;
    /** Distance sensor used to detect if an artifact is loaded in the intake */
    public DistanceSensor distanceSensor;

    /** Nominal battery voltage (12V) used as baseline for voltage compensation */
    public final double NOMINAL_VOLTAGE = 12.0d;
    /** Current battery voltage and voltage compensation factor (Vk = nominal / actual) */
    public double VOLTAGE, Vk;

    VoltageSensor vs;
    Follower follower;

    /**
     * Initializes the robot with all subsystems.
     * @param _hardwareMap FTC hardware map for device access
     * @param _t           Telemetry instance for debug output
     * @param f            Pedro Pathing follower for localization and path following
     * @param _debug       Enable/disable debug telemetry output
     * @param _side        Alliance side (SIDE_BLUE or SIDE_RED)
     */
    public Robot(HardwareMap _hardwareMap, Telemetry _t, Follower f, boolean _debug, boolean _side) {
        super(_hardwareMap, _t, _debug, _side);

        shooter = new Shooter(_hardwareMap, _t, f, _debug, _side);

        follower = f;

        // Start in idle mode with intake off
        idle();
        intake(false);
    }

    /** Fires the shooter if the robot is ready (flywheel at target RPM and turret aimed) */
    public void shoot(){
        if (shooter.canShoot()) return;
        shooter.shoot();
    }

    /** Switches to idle mode — stops turret tracking and turns off the flywheel */
    public void idle(){
        shooter.idleMode();
        shooter.offShooter();
    }

    /** Switches to tracking mode — turret begins vision-based target tracking */
    public void tracking(){
        shooter.trackingMode();
    }

    /**
     * Controls the intake system.
     * Prevents intake changes during an active shoot sequence.
     * @param state true = intake on (close gate, run motors in reverse), false = intake off
     */
    public void intake(boolean state){
        if (shooter.shooting) return; // Don't change intake during shoot sequence
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
        vs = hardwareMap.voltageSensor.get("Control Hub");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        led1.setPosition(0);
    }


    /**
     * Main update loop — called every cycle from the OpMode.
     * Handles voltage compensation, subsystem updates, and LED status display.
     */
    @Override
    public void update() {
        // --- Voltage Compensation ---
        // Scales motor power to maintain consistent behavior as battery drains.
        // Vk > 1 when battery is below 12V, boosting motor commands proportionally.
        VOLTAGE = vs.getVoltage();
        Vk = NOMINAL_VOLTAGE / VOLTAGE;

        shooter.Vk = Vk;
        debugAddLine("Robot");
        debugAddData("Nominal Voltage", NOMINAL_VOLTAGE);
        debugAddData("Voltage", VOLTAGE);
        debugAddData("Vk %", Vk * 100);

        // Update Pedro Pathing localization and shooter subsystem
        follower.update();
        shooter.update();

        // --- LED Status Indicator Logic ---
        if (shooter.turret.mode == Turret.TRACKING_MODE){
            if (shooter.turret.atLimit){
                // Orange: turret has hit its ±65° rotation limit
                led1.setPosition(ORANGE);
                led2.setPosition(ORANGE);
            }else {
                // Green = ready to shoot, Blue = flywheel ready but turret aligning, Red = not ready
                double state = (shooter.canShoot()) ? GREEN : (shooter.atTargetFlywheelRPMBroad()) ? BLUE : RED;
                led1.setPosition(state);
                led2.setPosition(state);
            }
        }else{
            // In idle mode: Green if artifact detected in intake (< 95mm), Red otherwise
            double state = (distanceSensor.getDistance(DistanceUnit.MM) < 95) ? GREEN : RED;
            led1.setPosition(state);
            led2.setPosition(state);
        }
    }
}
