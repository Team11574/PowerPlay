package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Generic.midpoint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double deltaX = 0.5;

        //Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));
        // Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));
        Pose2d startPos = new Pose2d(12, 12, Math.toRadians(270));


        // Forward 57, left 2, rotate 160 degrees

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.914317478618088, 26.914317478618088, Math.toRadians(114.10780678008499), Math.toRadians(118.62129230769227), 12.75)
                .setDimensions(16.5, 17)
                .followTrajectorySequence(drivetrain -> {
                    drivetrain.setPoseEstimate(startPos);

                    TileCalculation t = new TileCalculation(drivetrain);
                    returnThingy r = t.queueMove(TileCalculation.Move.DOWN);
                    TileCalculation.Tile target1 = t.targetTile;
                    returnThingy r2 = t.queueMove(TileCalculation.Move.RIGHT);
                    TileCalculation.Tile target2 = t.targetTile;
                    System.out.println(t.targetTile);
                    System.out.println(r.nextTile);
                    System.out.println(r.startHeading);
                    System.out.println(r.endHeading);
                    System.out.println(r.startPose);



                    return drivetrain.trajectorySequenceBuilder(startPos) //r.startPose)
                            //.setTangent(Math.toRadians(270))
                            //.splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(270))

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

                            .build();
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
                        }
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}