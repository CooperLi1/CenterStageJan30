package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep60Left {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAng Accel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 17.66)
                //Set dimensions
                .setDimensions(15,15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                        //purple
                        .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(4, -36, Math.toRadians(180)), Math.toRadians(180))
                        //.lineToConstantHeading(new Vector2d(12, -36))
                        //yellow
                        .lineToSplineHeading(new Pose2d(51, -30, Math.toRadians(0)))
                        //intake cycle 1
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(24,-12), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(0, -12), Math.toRadians(180))
                        //score cycle 1
                        .setReversed(false)
                        .lineToConstantHeading(new Vector2d(51,-12))
                        .lineToConstantHeading(new Vector2d(51,-42))
                        //park
                        //.setReversed(true)
                        //.splineToConstantHeading(new Vector2d(40, -24), Math.toRadians(90))
                        //.splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
