package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.path.AutoPath;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TestPath extends AutoPath {
    public TestPath(Auto robot) {
        super(robot);

        Pose2d startPose = new Pose2d(12, 64.5, Math.toRadians(-90));

        robot.drivetrain.setPose(startPose);

        TrajectorySequence trajCenter = robot.drivetrain.newTraj()
                .addTemporalMarker(0, () -> {
                    robot.pixels.armGrab();
                })
                .addTemporalMarker(5, () -> {
                    robot.pixels.noseGrab();
                })
                //Purple
                .lineTo(new Vector2d(12, 34))
                //yellow
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, 50), Math.toRadians(90))
                .setReversed(false)
                .turn(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54,39), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48, 15.5), Math.toRadians(90))
                .setReversed(false)
                .lineTo(new Vector2d(60, 15.5))
                .build();
    }

    public void run(Vision.Position position, Park park) {

    }
}
