package org.firstinspires.ftc.teamcode.auto.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedFarPerimeter extends AutoPath {
    private enum DriveStates {
        PURPLE, YELLOW, PARK;
        static DriveStates currentState;
    }

    private enum DeliveryStates {
        DRIVE, RAISE, RELEASE, RESET;
        static DeliveryStates currentState;
    }

    private final TrajectorySequence centerPurple;
    private final TrajectorySequence leftPurple;
    private final TrajectorySequence rightPurple;

    private final TrajectorySequence centerYellow;
    private final TrajectorySequence leftYellow;
    private final TrajectorySequence rightYellow;

    private final TrajectorySequence parkCenter;
    private final TrajectorySequence parkWall;

    public RedFarPerimeter(Auto robot) {
        super(robot);

        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(90));

        robot.drivetrain.setPose(startPose);

        centerPurple = robot.drivetrain.newTraj(startPose)
                .lineToConstantHeading(new Vector2d(-36, -34))
                .lineToLinearHeading(new Pose2d(-38, -62, 0))
                .build();
        leftPurple = robot.drivetrain.newTraj(startPose)
                .lineToConstantHeading(new Vector2d(-46, -40))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-38, -62, 0), 0)
                .setReversed(false)
                .build();
        rightPurple = robot.drivetrain.newTraj(startPose)
                .splineToLinearHeading(new Pose2d(-31, -40, Math.toRadians(45)), Math.toRadians(45))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-38, -62, Math.toRadians(0)), Math.toRadians(-90))
                .setReversed(false)
                .build();

        centerYellow = robot.drivetrain.newTraj(centerPurple.end())
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(36, -39))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoFar();
                })
                .splineToConstantHeading(new Vector2d(51,-39), Math.toRadians(0), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(15))
                .build();
        leftYellow = robot.drivetrain.newTraj(leftPurple.end())
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoFar();
                })
                .splineToConstantHeading(new Vector2d(51,-33), Math.toRadians(0), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(15))
                .build();
        rightYellow = robot.drivetrain.newTraj(rightPurple.end())
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoFar();
                })
                .splineToConstantHeading(new Vector2d(51,-44), Math.toRadians(0), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        parkCenter = robot.drivetrain.newTraj(leftYellow.end())
                .lineToConstantHeading(new Vector2d(44, -16.5))
                .build();
        parkWall = robot.drivetrain.newTraj(rightYellow.end())
                .lineToConstantHeading(new Vector2d(44, -63.5))
                .build();
    }

    public void run(Vision.Position position, Park park) {
        robot.pixels.armGrab();
        robot.hang.holdHang();
        robot.shooter.holdShooter();
        switch (position) {
            case LEFT:
                robot.drivetrain.followTrajAsync(leftPurple);
                break;
            case CENTER:
                robot.drivetrain.followTrajAsync(centerPurple);
                break;
            case RIGHT:
                robot.drivetrain.followTrajAsync(rightPurple);
        }
        DriveStates.currentState = DriveStates.PURPLE;
        DeliveryStates.currentState = DeliveryStates.DRIVE;
        ElapsedTime timer = new ElapsedTime();
        while (robot.opModeIsActive()) {
            switch (DriveStates.currentState) {
                case PURPLE:
                    if (!robot.drivetrain.isBusy()) {
                        robot.pixels.noseGrab();
                        switch (position) {
                            case LEFT:
                                robot.drivetrain.followTrajAsync(leftYellow);
                                break;
                            case CENTER:
                                robot.drivetrain.followTrajAsync(centerYellow);
                                break;
                            case RIGHT:
                                robot.drivetrain.followTrajAsync(rightYellow);
                        }
                        DriveStates.currentState = DriveStates.YELLOW;
                    }
                    break;
                case YELLOW:
                    switch (DeliveryStates.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.armOuttakeAuto();
                                DeliveryStates.currentState = DeliveryStates.RAISE;
                            }
                            break;
                        case RAISE:
                            if (!robot.pixels.liftBusy()) {
                                robot.pixels.noseClawRelease();
                                timer.reset();
                                DeliveryStates.currentState = DeliveryStates.RELEASE;
                            }
                            break;
                        case RELEASE:
                            if (timer.milliseconds() > 1000) {
                                robot.pixels.armTransport();
                                robot.pixels.liftDown();
                                DeliveryStates.currentState = DeliveryStates.RESET;
                            }
                            break;
                        case RESET:
                            if (!robot.pixels.liftBusy()) {
                                robot.pixels.liftPower(0);
                                robot.pixels.clamClose();
                                switch (park) {
                                    case CENTER:
                                        robot.drivetrain.followTrajAsync(parkCenter);
                                        break;
                                    case WALL:
                                        robot.drivetrain.followTrajAsync(parkWall);
                                }
                                DriveStates.currentState = DriveStates.PARK;
                            }
                    }
                    break;
                case PARK:
                    if (!robot.drivetrain.isBusy()) {
                        robot.stop();
                    }
            }
//            robot.pixels.update();
            robot.drivetrain.update();
            Pose2d pose = robot.drivetrain.getPose();
            robot.telemetry.addData("x", pose.getX());
            robot.telemetry.addData("y", pose.getY());
            robot.telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            robot.telemetry.update();
        }
    }
}
