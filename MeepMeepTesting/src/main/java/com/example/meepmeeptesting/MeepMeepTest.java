package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAng Accel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 17.66)
                //Set dimensions
                .setDimensions(15,15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -64.5, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-31, -40, Math.toRadians(45)), Math.toRadians(45))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-38, -62, Math.toRadians(0)), Math.toRadians(-90))
                        .setReversed(false)
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}