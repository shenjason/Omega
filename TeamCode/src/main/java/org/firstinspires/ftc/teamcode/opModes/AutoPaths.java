package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Assembly;

public class AutoPaths {
        public double width =15.5, height = 14.6;
        public boolean SIDE = Assembly.SIDE_BLUE;
        public Pose startPose = (new Pose(26.200, 130.000, Math.toRadians(52)));
        public Pose shoot = (new Pose(52,92, Math.toRadians(180)));
        public Pose ready1 = (new Pose(48.0,86.0,  Math.toRadians(180)));
        public Pose load1 = (new Pose(24,86.0, Math.toRadians(180)));
        public Pose gatePos = (new Pose(20, 72, Math.toRadians(180)));
        public Pose ready2 = (new Pose(48.0,62.0, Math.toRadians(180)));
        public Pose load2 = (new Pose(26.0, 62.0,  Math.toRadians(180)));
        public Pose ready3 = (new Pose(48.0,38.0, Math.toRadians(180)));
        public Pose load3 = (new Pose(24.0,38.0, Math.toRadians(180)));
        public Pose end = (new Pose(32.0,72.0, Math.toRadians(180)));

        public Pose startPose_far = new Pose(48+width/2,height/2, Math.toRadians(90));

        public Pose shoot_far = new Pose(60,24, Math.toRadians(180));

        public Pose ready1_far = new Pose(48,32,Math.toRadians(180));

        public Pose ready2_far = new Pose(48, 56, Math.toRadians(180));

        public Pose load1_far = new Pose(18,32,Math.toRadians(180));

        public Pose load2_far= new Pose(18, 56, Math.toRadians(180));

        public Pose ready3_far = new Pose(width/2+1,24,Math.toRadians(-90));

        public Pose load3_far = new Pose(width/2+1,height/2+1, Math.toRadians(90));
        public Pose end_far = new Pose(36,36,Math.toRadians(90));

        public PathChain start_shoot, shoot_ready1, ready1_load1, load1_shoot, shoot_ready2,ready2_load2, load2_shoot, shoot_ready3, ready3_load3, load3_shoot, shoot_end, load1_gate, gate_shoot;

        public PathChain start_shoot_far,shoot_ready1_far, ready1_load1_far,load1_shoot_far, shoot_ready2_far, ready2_load2_far,load2_shoot_far,shoot_ready3_far,ready3_load3_far,load3_shoot_far, shoot_end_far;
        public AutoPaths(Follower follower, boolean SIDE, double ROT) {
            this.SIDE = SIDE;


            start_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(startPose), x(shoot))
                    )
                    .setLinearHeadingInterpolation(SIDE ? startPose.getHeading() : Math.toRadians(128), ROT)
                    .build();

            shoot_ready1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(ready1))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            ready1_load1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready1), x(load1))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load1_gate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(x(load1), x(new Pose(32, 72)), x(gatePos))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            gate_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(gatePos), x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load1_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load1), x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            shoot_ready2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(ready2))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            ready2_load2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready2),x(load2))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load2_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load2),x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            shoot_ready3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot),x(ready3))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            ready3_load3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready3),x(load3))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            load3_shoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load3),x(shoot))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            shoot_end = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot), x(end))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();

            //the far side
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
                    .setConstantHeadingInterpolation(ROT)
                    .build();
            ready2_load2_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready2_far), x(load2_far))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();
            load2_shoot_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load2_far), x(shoot_far))
                    )
                    .setConstantHeadingInterpolation(ROT)
                    .build();
            shoot_ready3_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(shoot_far), x(ready3_far))
                    )
                    .setLinearHeadingInterpolation(ROT,-Math.toRadians(90))
                    .build();
            ready3_load3_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(ready3_far), x(load3_far))
                    )
                    .setConstantHeadingInterpolation(-Math.toRadians(90))
                    .build();
            load3_shoot_far = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(x(load3_far), x(shoot_far))
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


        public Pose x(Pose p){
            return (SIDE) ? p : new Pose(144 - p.getX(), p.getY(), p.getHeading());
        }
    }