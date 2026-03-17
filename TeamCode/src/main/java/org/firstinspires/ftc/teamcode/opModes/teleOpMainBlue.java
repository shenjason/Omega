package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.assemblies.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Assembly;


/**
 * TeleOp OpMode for Blue alliance (standard starting position).
 *
 * Single-driver control scheme:
 *   Left Stick       - Mecanum drive (forward/strafe, robot-centric)
 *   Right Stick X    - Rotation (cubed for fine control)
 *   Hold RB/RT       - Sprint mode (100% speed vs 50% normal)
 *   Hold LB          - Activate intake
 *   A Button         - Toggle between idle and tracking mode
 *   Right Stick Btn  - Fire shooter
 *   D-Pad Left/Right - Adjust turret offset (+/- 5 degrees)
 *   D-Pad Down       - Reset robot pose to depot position
 *
 * Red alliance variants (teleOpMainRed) extend this class and override SIDE.
 */
@Configurable
@TeleOp(name = "TeleOpMain(Blue)", group = "TeleOp")
public class teleOpMainBlue extends OpMode {

    /** Alliance side flag — overridden in Red variants */
    public static boolean SIDE = Assembly.SIDE_BLUE;
    public static boolean DEBUG = true;

    Follower follower;

    Robot robot;

    /** Sets the starting pose based on alliance side (mirrored X position and heading) */
    public void setSIDE() {
        follower.setStartingPose(new Pose((SIDE) ? 32 : 112, 72, (SIDE) ? Math.toRadians(180) : 0));
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        setSIDE();
        robot = new Robot(hardwareMap, telemetry, follower, DEBUG, SIDE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Update all subsystems (voltage compensation, shooter, turret, LEDs)
        robot.update();

        // --- Mecanum Drive ---
        // Normal speed is 50%, sprint (RB or RT) boosts to 100%
        // Right stick X is cubed for finer rotational control at low inputs
        double speed = (gamepad1.right_bumper || gamepad1.right_trigger > 0) ? 1 : 0.5d;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed,
                -Math.pow(gamepad1.right_stick_x, 3) * speed * 0.8,
                true // Robot Centric
        );


        // --- Intake Control ---
        robot.intake(gamepad1.left_bumper);

        // --- Turret Mode Toggle (A button) ---
        // Toggles between idle (odometry estimation) and tracking (camera + flywheel active)
        if (gamepad1.aWasPressed()){
            if (robot.shooter.turret.mode == Turret.IDLE_MODE){
                robot.tracking();
            }else{
                robot.idle();
            }
        }

        // --- Fire Shooter (right stick button) ---
        if (gamepad1.right_stick_button){
            robot.shoot();
        }

        // --- Pose Reset (D-pad Down) ---
        // Resets the robot's position to the depot coordinates (useful after manual repositioning)
        if (gamepad1.dpadDownWasPressed()){
            follower.setPose(new Pose((SIDE) ? 135 : 9 , 8.5, (SIDE) ? Math.toRadians(180) : 0));
        }

        // --- Turret Offset Adjustment (D-pad Left/Right) ---
        // Fine-tunes turret center position by +/- 5 degrees
        if (gamepad1.dpadRightWasPressed()){
            robot.shooter.turret.offsetAngle += Math.toRadians(5);
        }
        if (gamepad1.dpadLeftWasPressed()){
            robot.shooter.turret.offsetAngle -= Math.toRadians(5);
        }


        telemetry.update();

    }
}
