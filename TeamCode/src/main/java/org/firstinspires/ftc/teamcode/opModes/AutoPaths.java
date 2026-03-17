package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Assembly;

/**
 * AutoPaths - Defines all autonomous path segments and waypoint positions.
 *
 * All coordinates are defined for the Blue alliance side. The x() helper method
 * mirrors poses for Red alliance by flipping the X coordinate (144 - x).
 *
 * Field coordinate system: 144" x 144" field, origin at corner.
 *
 * Path naming convention:
 *   start_shoot     = Start position to shooting position
 *   shoot_ready1    = Shooting position to ready position before load station 1
 *   ready1_load1    = Ready position to load station 1
 *   load1_gate      = Load station 1 to gate (uses Bezier curve to avoid obstacles)
 *   gate_shoot      = Gate to shooting position
 *   shoot_end       = Shooting position to final parking position
 *
 * Paths ending in "_far" are for the far starting position variant.
 */
public class AutoPaths {
        /** Robot dimensions used for clearance calculations (inches) */
        public double width =16, height = 17;
        public boolean SIDE = Assembly.SIDE_BLUE;

        // --- Center Start Waypoints (Blue alliance coordinates) ---
        public Pose startPose = (new Pose(26.200, 130.000, Math.toRadians(52)));
        public Pose shoot = (new Pose(56,92, Math.toRadians(180)));          // Shooting position
        public Pose ready1 = (new Pose(48.0,86.0,  Math.toRadians(180)));    // Pre-intake position for row 1
        public Pose load1 = (new Pose(24,86.0, Math.toRadians(180)));        // Load station 1
        public Pose gatePos = (new Pose(20, 72, Math.toRadians(180)));       // Gate position (obstacle avoidance)
        public Pose ready2 = (new Pose(48.0,62.0, Math.toRadians(180)));     // Pre-intake position for row 2
        public Pose load2 = (new Pose(27.0, 62.0,  Math.toRadians(180)));    // Load station 2
        public Pose ready3 = (new Pose(48.0,38.0, Math.toRadians(180)));     // Pre-intake position for row 3
        public Pose load3 = (new Pose(24.0,38.0, Math.toRadians(180)));      // Load station 3
        public Pose end = (new Pose(32.0,72.0, Math.toRadians(180)));        // Final parking position

        // --- Far Start Waypoints ---
        public Pose startPose_far = new Pose(56,8, Math.toRadians(90));
        public Pose shoot_far = new Pose(65,26, Math.toRadians(180));
        public Pose ready1_far = new Pose(48,36,Math.toRadians(180));
        public Pose load1_far = new Pose(18,36,Math.toRadians(180));
        public Pose ready2_far = new Pose(width/2+1,24,Math.toRadians(-90));
        public Pose load2_far = new Pose(width/2+1,height/2+1, Math.toRadians(90));
        public Pose end_far = new Pose(36,36,Math.toRadians(90));

        // --- Center Start PathChains ---
        public PathChain start_shoot, shoot_ready1, ready1_load1, load1_shoot, shoot_ready2,ready2_load2, load2_shoot, shoot_ready3, ready3_load3, load3_shoot, shoot_end, load1_gate, gate_shoot;

        // --- Far Start PathChains ---
        public PathChain start_shoot_far,shoot_ready1_far, ready1_load1_far,load1_shoot_far, shoot_ready2_far, ready2_load2_far,load2_shoot_far,shoot_ready3_far,ready3_load3_far,load3_shoot_far, shoot_end_far;

        /**
         * Builds all path segments using the Pedro Pathing PathBuilder.
         * Paths use BezierLine (straight) or BezierCurve (curved) segments with
         * constant or linear heading interpolation.
         *
         * @param follower Pedro Pathing follower instance (provides the path builder)
         * @param SIDE     Alliance side (determines coordinate mirroring)
         * @param ROT      Heading rotation (180 deg for Blue, 0 for Red)
         */
        public AutoPaths(Follower follower, boolean SIDE, double ROT) {
            this.SIDE = SIDE;

            // --- Center Start Paths ---

            // Start -> Shoot: Linear heading interpolation from start angle to 180 degrees
            start_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(startPose), x(shoot))
                    )
                    .setLinearHeadingInterpolation(SIDE ? startPose.getHeading() : Math.toRadians(128), ROT)
                    .build();

            // Shoot -> Ready 1: Maintain constant heading while repositioning
            shoot_ready1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(ready1))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Ready 1 -> Load 1: Drive into load station
            ready1_load1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready1), x(load1))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Load 1 -> Gate: Bezier curve to navigate around field obstacles
            load1_gate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(x(load1), x(new Pose(32, 72)), x(gatePos))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Gate -> Shoot: Return to shooting position
            gate_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(gatePos), x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Load 1 -> Shoot (direct, alternative to gate route)
            load1_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load1), x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Shoot -> Ready 2
            shoot_ready2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(ready2))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Ready 2 -> Load 2
            ready2_load2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready2),x(load2))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Load 2 -> Shoot
            load2_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load2),x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Shoot -> Ready 3
            shoot_ready3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot),x(ready3))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Ready 3 -> Load 3
            ready3_load3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready3),x(load3))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Load 3 -> Shoot
            load3_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load3),x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // Shoot -> End (parking position)
            shoot_end = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(end))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            // --- Far Start Paths ---

            start_shoot_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(startPose_far), x(shoot_far))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90),ROT)
                    .build();
            shoot_ready1_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot_far), x(ready1_far))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();
            ready1_load1_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready1_far), x(load1_far))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();
            load1_shoot_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load1_far), x(shoot_far))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();
            shoot_ready2_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot_far), x(ready2_far))
                    )
                    .setLinearHeadingInterpolation(ROT,-Math.toRadians(90))
                    .build();
            ready2_load2_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready2_far), x(load2_far))
                    )
                    .setConstantHeadingInterpolation(-Math.toRadians(90))
                    .build();
            load2_shoot_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load2_far), x(shoot_far))
                    )
                    .setLinearHeadingInterpolation(-Math.toRadians(90), ROT)
                    .build();
            shoot_end_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot_far), x(end_far))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();



        }


        /**
         * Mirrors a pose for Red alliance by flipping the X coordinate.
         * Blue coordinates are used as-is; Red coordinates are mirrored: x -> (144 - x).
         * @param p The pose in Blue alliance coordinates
         * @return The pose (unchanged for Blue, mirrored for Red)
         */
        public Pose x(Pose p){
            return (SIDE) ? p : new Pose(144 - p.getX(), p.getY(), p.getHeading());
        }
    }
