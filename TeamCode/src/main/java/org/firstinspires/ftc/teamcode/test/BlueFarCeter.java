package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.path.AutoPath;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueFarCeter extends AutoPath {
    private TrajectorySequence trajCenter;
    private TrajectorySequence trajLeft;
    private TrajectorySequence trajRight;

    public BlueFarCeter(Auto robot) {
        super(robot);

        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));

        robot.drivetrain.setPose(startPose);

        trajCenter = robot.drivetrain.newTraj()
                .addTemporalMarker(0, () -> {
                    robot.pixels.armGrab();
                    robot.hang.holdHang();
                    robot.shooter.holdShooter();
                })
                .addTemporalMarker(5, () -> {
                    robot.pixels.noseGrab();
                })
                //Purple
                .lineTo(new Vector2d(-36, 34))
                //cycle 1
                .setReversed(true)
                .splineTo(new Vector2d(-36, 38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-56,52), Math.toRadians(180))
                .lineTo(new Vector2d(-56, 14))
                .turn(Math.toRadians(90))
                .setReversed(false)
                .lineTo(new Vector2d(21, 14))
                .splineToConstantHeading(new Vector2d(54,39), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .back(10) //6
                .splineToConstantHeading(new Vector2d(40, 15), Math.toRadians(-90)) //42, 15
                .setReversed(false)
                .lineTo(new Vector2d(60, 15))
                .build();

        trajRight = robot.drivetrain.newTraj()
                .addTemporalMarker(0, () -> {
                    robot.pixels.armGrab();
                    robot.hang.holdHang();
                    robot.shooter.holdShooter();
                })
                .addTemporalMarker(5, () -> {
                    robot.pixels.noseGrab();
                })
                //Purple
                .splineToLinearHeading(new Pose2d(-43, 38, Math.toRadians(-135)), Math.toRadians(-135))
                //cycle 1
                .setReversed(true)
                .splineTo(new Vector2d(-36,45), Math.toRadians(90))
                .setReversed(false)
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(21, 12))
                .splineToConstantHeading(new Vector2d(54,32), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .back(6) //0
                .splineToConstantHeading(new Vector2d(42,  15), Math.toRadians(-90))
                .setReversed(false)
                .lineTo(new Vector2d(60,  15))
                .build();

        trajLeft = robot.drivetrain.newTraj()
                .addTemporalMarker(0, () -> {
                    robot.pixels.armGrab();
                    robot.hang.holdHang();
                    robot.shooter.holdShooter();
                })
                .addTemporalMarker(5, () -> {
                    robot.pixels.noseGrab();
                })
                //Purple
                .splineTo(new Vector2d(-30, 38), Math.toRadians(-45))
                //cycle 1
                .setReversed(true)
                .splineTo(new Vector2d(-36,45), Math.toRadians(90))
                .setReversed(false)
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(21, 12))
                .splineToConstantHeading(new Vector2d(54,46), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .back(6) //6
                .splineToConstantHeading(new Vector2d(42,  17), Math.toRadians(-90))
                .setReversed(false)
                .lineTo(new Vector2d(60,  17))
                .build();
    }

    public void run(Vision.Position position, Park park) {
        if (!robot.isStopRequested()) {
            switch (position) {
                case LEFT:
                    robot.drivetrain.followTrajAsync(trajLeft);
                    break;
                case CENTER:
                    robot.drivetrain.followTrajAsync(trajCenter);
                    break;
                case RIGHT:
                    robot.drivetrain.followTrajAsync(trajRight);
            }
        }
        while (robot.opModeIsActive()) {
            robot.drivetrain.update();
            Pose2d pose = robot.drivetrain.getPose();
            robot.telemetry.addData("x", pose.getX());
            robot.telemetry.addData("y", pose.getY());
            robot.telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            robot.telemetry.update();
        }
    }
}
