package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.path.AutoPath;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedCloseWall extends AutoPath {
    private TrajectorySequence trajCenter;
    private TrajectorySequence trajLeft;
    private TrajectorySequence trajRight;

    public RedCloseWall(Auto robot) {
        super(robot);

        Pose2d startPose = new Pose2d(12, -64.5, Math.toRadians(90));

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
                .lineTo(new Vector2d(12, -34))
                //yellow
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(-90))
                .setReversed(false)
                .turn(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54.5,-40), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48, -63.5), Math.toRadians(-90))
                .setReversed(false)
                .lineTo(new Vector2d(60, -63.5))
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
                .splineToLinearHeading(new Pose2d(5, -38, Math.toRadians(135)), Math.toRadians(135))
                //yellow
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(-90))
                .setReversed(false)
                .turn(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(54.5,-32), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48, -63.5), Math.toRadians(-90))
                .setReversed(false)
                .lineTo(new Vector2d(60, -63.5))
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
                .splineToLinearHeading(new Pose2d(18, -38, Math.toRadians(45)), Math.toRadians(45))
                //yellow
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(-90))
                .setReversed(false)
                .turn(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(54.5,-46.5), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.deliverFirst(true);
                })
                //park
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48, -63.5), Math.toRadians(-90))
                .setReversed(false)
                .lineTo(new Vector2d(60, -63.5))
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
