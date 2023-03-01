package com.example.meepmeeptesting;

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
        Pose2d startPos = new Pose2d(36, -62.5, Math.toRadians(90));

        // Forward 57, left 2, rotate 160 degrees

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.914317478618088, 26.914317478618088, Math.toRadians(114.10780678008499), Math.toRadians(118.62129230769227), 12.75)
                .setDimensions(16.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
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

                                 */
                                .forward(58)
                                .back(3)
                                .lineToLinearHeading(new Pose2d(startPos.component1() - 1.5, startPos.component2() + 58.5, Math.toRadians(171)))
                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                                .strafeRight(21)
                                .build()
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
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}