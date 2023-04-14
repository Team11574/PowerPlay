package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Generic.midpoint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static int roundToFactor(double value, int factor) {
        return (int) (factor * Math.round(value / factor));
    }

    public static double invertX(double x) {
        return -x;
    }
    public static double invertHeading(double heading) {
        return (heading + Math.PI) % (2 * Math.PI);
    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        
        double deltaX = 0.5;

        /* ACTUAL GOOD TRAJEECTORY FOR 8 CONE AUTO
        .splineToSplineHeading(
                new Pose2d(33.5, -3.5, Math.toRadians(170.8)),
                directionalHeading
        )
        // Cones 1-6
        .waitSeconds(cycleTime * 6)
        .setTangent(Math.toRadians(270))
        // Center out
        .splineToSplineHeading(
                new Pose2d(24, -12, Math.toRadians(90)),
                Math.toRadians(180)
        )
        .splineToSplineHeading(
                new Pose2d(-24, -12, Math.toRadians(90)),
                Math.toRadians(180)
        )
        .splineToSplineHeading(
                new Pose2d(-33.5, -3.5, Math.toRadians(180-170.8)),
                Math.toRadians(90)
        )
        .waitSeconds(cycleTime * 2)
        .setTangent(Math.toRadians(270))
        .splineToSplineHeading(
                new Pose2d(-12, -12, Math.toRadians(90)),
                Math.toRadians(0)
        )
         */

        Pose2d startPos = new Pose2d(35.5, -63.5, Math.toRadians(90));
        //Pose2d startPos = new Pose2d(invertX(36), -62.5, Math.toRadians(90));
        //Pose2d startPos = new Pose2d(36, -60, Math.toRadians(330));
        //Pose2d startPos = new Pose2d(12, 12, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 3.9, 3.9, 12.75)
                .setDimensions(16.5, 17)
                .followTrajectorySequence(drivetrain -> {
                    drivetrain.setPoseEstimate(startPos);
                    double cycleTime = 2.5; // seconds

                    double directionalHeading = Math.toRadians(90);
                    double x = startPos.getX();
                    double y = startPos.getY();

                    double angle = 35;

                    return drivetrain.trajectorySequenceBuilder(startPos) //r.startPose)
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(
                                    new Vector2d(20, -63.5),
                                    Math.toRadians(180)
                            )
                            .splineToConstantHeading(
                                    new Vector2d(10, -48),
                                    Math.toRadians(90)
                            )
                            .addDisplacementMarker(() -> {
                                //robot.horizontalArm.goToPosition(HorizontalArm.Position.UP);
                            })
                            .splineToSplineHeading(
                                    new Pose2d(10, -12, Math.toRadians(180 + angle)),
                                    Math.toRadians(90)
                            )
                            .splineToConstantHeading(
                                    new Vector2d(11, -10),
                                    Math.toRadians(angle)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(angle))
                            .splineToSplineHeading(
                                    new Pose2d(20, -6, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .splineToConstantHeading(
                                    new Vector2d(30, -6),
                                    Math.toRadians(0)
                            )
                            .addDisplacementMarker(() -> {
                                //robot.horizontalArm.goToPosition(HorizontalArm.Position.OUT);
                            })
                            .splineToConstantHeading(
                                    new Vector2d(40, -6),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(
                                    new Vector2d(20, -6),
                                    Math.toRadians(180)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(11, -10, Math.toRadians(180 + angle)),
                                    Math.toRadians(180 + angle)
                            )
                            /*.splineToConstantHeading(
                                    new Vector2d(20, -16),
                                    Math.toRadians(angle)
                            )
                            .setTangent(Math.toRadians(90))
                            .splineToSplineHeading(
                                    new Pose2d(26, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(40, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(.25)
                            .splineToSplineHeading(
                                    new Pose2d(26, -12, Math.toRadians(180)),
                                    Math.toRadians(180)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(20, -16, Math.toRadians(180+angle)),
                                    Math.toRadians(270)
                            )*/

                            /*.splineToConstantHeading(
                                    new Vector2d(36, -24),
                                    directionalHeading
                            )
                            .splineToSplineHeading(
                                    new Pose2d(35, -12, Math.toRadians(130)),
                                    Math.toRadians(90)
                            )
                            // 1
                            .waitSeconds(.25)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(
                                    new Pose2d(42, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(180))
                            .splineToSplineHeading(
                                    new Pose2d(35, -12, Math.toRadians(130)),
                                    Math.toRadians(180)
                            )
                            // 2
                            .waitSeconds(.25)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(
                                    new Pose2d(42, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(180))
                            .splineToSplineHeading(
                                    new Pose2d(35, -12, Math.toRadians(130)),
                                    Math.toRadians(180)
                            )
                            //3
                            .waitSeconds(.25)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(
                                    new Pose2d(42, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(180))
                            .splineToSplineHeading(
                                    new Pose2d(35, -12, Math.toRadians(130)),
                                    Math.toRadians(180)
                            )
                            // 4
                            .waitSeconds(.25)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(
                                    new Pose2d(42, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(180))
                            .splineToSplineHeading(
                                    new Pose2d(35, -12, Math.toRadians(130)),
                                    Math.toRadians(180)
                            )
                            // 5
                            .waitSeconds(.25)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(
                                    new Pose2d(42, -12, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .waitSeconds(1)
                            .setTangent(Math.toRadians(180))
                            .splineToSplineHeading(
                                    new Pose2d(35, -12, Math.toRadians(130)),
                                    Math.toRadians(180)
                            )
                            .setTangent(Math.toRadians(180))
                            // Center out
                            .splineToSplineHeading(
                                    new Pose2d(24, -11, Math.toRadians(90)),
                                    Math.toRadians(180)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(-24, -12, Math.toRadians(90)),
                                    Math.toRadians(180)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(-33.5, -3.5, Math.toRadians(180-170.8)),
                                    Math.toRadians(90)
                            )
                            .waitSeconds(cycleTime * 2)
                            .setTangent(Math.toRadians(270))
                            .splineToSplineHeading(
                                    new Pose2d(-12, -12, Math.toRadians(90)),
                                    Math.toRadians(0)
                            )*/
                            /* 8 CONE RIGHT SIDE TESTING
                            .splineToSplineHeading(
                                    new Pose2d(33.5, -3.5, Math.toRadians(170.8)),
                                    directionalHeading
                            )
                            // Cones 1-6
                            .waitSeconds(cycleTime * 6)
                            .setTangent(Math.toRadians(270))
                            // Center out
                            .splineToSplineHeading(
                                    new Pose2d(24, -12, Math.toRadians(90)),
                                    Math.toRadians(180)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(-24, -12, Math.toRadians(90)),
                                    Math.toRadians(180)
                            )
                            .splineToSplineHeading(
                                    new Pose2d(-33.5, -3.5, Math.toRadians(180-170.8)),
                                    Math.toRadians(90)
                            )
                            .waitSeconds(cycleTime * 2)
                            .setTangent(Math.toRadians(270))
                            .splineToSplineHeading(
                                    new Pose2d(-12, -12, Math.toRadians(90)),
                                    Math.toRadians(0)
                            )*/
                            /*
                            .forward(58)
                            .back(3)
                            .lineToLinearHeading(new Pose2d(startPos.component1() - 2.5, startPos.component2() + 59, Math.toRadians(170.8)))
                            .waitSeconds(0.5)
                            .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                            .strafeLeft(24)
                            /* NEW TILE MOVEMENT TESTING
                            .setTangent(directionalHeading)
                            .splineToConstantHeading(
                                    new Vector2d(36, -60), directionalHeading
                            )
                            .splineToConstantHeading(new Vector2d(startPos.getX(), startPos.getY()+12), directionalHeading)
                            .splineToConstantHeading(new Vector2d(startPos.getX(), startPos.getY()+ 24), directionalHeading)
                            //.splineToConstantHeading(new Vector2d(startPos.getX()-12, startPos.getY()+48), Math.PI)
                            .splineToSplineHeading(new Pose2d(startPos.getX()-5, startPos.getY()+ 24 - 5, Math.toRadians(225)), Math.toRadians(225))
move                             */

                            /* OLD TILE CALCULATION TESTING
                            .setTangent(r.startHeading)
                            .splineToConstantHeading(
                                    midpoint(t.getVectorByID(r.nextTile), t.getVectorByID(target1)),
                                    r.endHeading)
                            .splineToConstantHeading(
                                    t.getVectorByID(r.nextTile),
                                    r.endHeading)

                            .setTangent(r2.startHeading)
                            .splineToConstantHeading(
                                    midpoint(t.getVectorByID(r2.nextTile), t.getVectorByID(target2)),
                                    r2.endHeading)
                            .splineToConstantHeading(
                                    t.getVectorByID(r2.nextTile),
                                    r2.endHeading)

move                            */
                                    // .setTangent(Math.toRadians(90))
                                    // .splineToConstantHeading(new Vector2d(36, 60), Math.toRadians(90))
                                    // .setTangent(Math.toRadians(180))
                                    //.spl
                                    //.splineToConstantHeading(new Vector2d(36, -36), Math.toRadians(90))
                                    //.splineToConstantHeading(new Vector2d(36, 0), Math.toRadians(90))
                                    //.splineToConstantHeading(new Vector2d(24, 12), Math.toRadians(180))
                                    //.splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(180))
                                    /*
                                    .forward(58)
                                    .back(3)
                                    .lineToLinearHeading(new Pose2d(startPos.component1()-3, startPos.component2() + 57, Math.toRadians(160)))
                                    .addDisplacementMarker(() -> {

                                    })
                                    .waitSeconds(0.5)
                                    .lineToLinearHeading(new Pose2d(36, -13, Math.toRadians(90)))
                                    .strafeLeft(24)
                                    .back(21.5)

                                    .forward(58)
                                    .back(3)
                                    .lineToLinearHeading(new Pose2d(startPos.component1() - 1.5, startPos.component2() + 58.5, Math.toRadians(171)))
                                    .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                                    .strafeRight(21)

                                    .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180)))
                                     */
                        /*
                        .strafeLeft(24)
                                .forward(51.5)
                                .strafeRight(12)
                                .addDisplacementMarker(() -> {
                                    // Raise slides, flip claw, drop cone, etc
                                })
                                .back(2)
                                //.strafeLeft(12)
                                //.strafeRight(12)
                                .strafeRight(33)
                         */
                            .build();
                        }
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}