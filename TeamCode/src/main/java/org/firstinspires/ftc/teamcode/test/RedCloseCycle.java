package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.path.AutoPath;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedCloseCycle extends AutoPath {
    private enum DriveStates {
        PURPLE, YELLOW, RETRIEVAL, DELIVERY, PARK;
        static DriveStates currentState;
    }

    private enum DeliveryStates {
        DRIVE, RAISE, RELEASE, RESET;
        static DeliveryStates currentState;
    }

    private enum RetrievalStates {
        DRIVE, EXTEND, INTAKE, RETRACT;
        static RetrievalStates currentState;
    }

    private final TrajectorySequence centerPurple;
    private final TrajectorySequence leftPurple;
    private final TrajectorySequence rightPurple;

    private final TrajectorySequence centerYellow;
    private  TrajectorySequence leftYellow;
    private TrajectorySequence rightYellow;

    private  TrajectorySequence cycleIntake;

    private  TrajectorySequence cycleDrop;

    private  TrajectorySequence parkCenter;
    private  TrajectorySequence parkWall;

    public RedCloseCycle(Auto robot) {
        super(robot);

        Pose2d startPose = new Pose2d(12, -64.5, Math.toRadians(90));

        robot.drivetrain.setPose(startPose);

        centerPurple = robot.drivetrain.newTraj(startPose)
                .lineTo(new Vector2d(12, -35))
                .build();
        leftPurple = robot.drivetrain.newTraj(startPose)
                .splineToLinearHeading(new Pose2d(5, -38, Math.toRadians(135)), Math.toRadians(135))
                .build();
        rightPurple = robot.drivetrain.newTraj(startPose)
                .splineToLinearHeading(new Pose2d(18, -38.5, Math.toRadians(45)), Math.toRadians(45))
                .build();

        centerYellow = robot.drivetrain.newTraj(centerPurple.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(-90))
                .setReversed(false)
                .turn(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(54.5,-40), Math.toRadians(0))
                .build();
        leftYellow = robot.drivetrain.newTraj(leftPurple.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(-90))
                .setReversed(false)
                .turn(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(54.5,-32), Math.toRadians(0))
                .build();
        rightYellow = robot.drivetrain.newTraj(rightPurple.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(-90))
                .setReversed(false)
                .turn(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(54.5,-46.5), Math.toRadians(0))
                .build();

        cycleIntake = robot.drivetrain.newTraj(centerYellow.end())
                .splineToConstantHeading(new Vector2d(36,-18), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-16,-18), Math.toRadians(0))
                .build();

        cycleDrop = robot.drivetrain.newTraj(cycleIntake.end())
                .splineToConstantHeading(new Vector2d(54.5, -40), Math.toRadians(90))
                .build();

        parkCenter = robot.drivetrain.newTraj(cycleDrop.end())
                .lineTo(new Vector2d(60, -15.5))
                .build();
        parkWall = robot.drivetrain.newTraj(cycleDrop.end())
                .lineTo(new Vector2d(60, -63.5))
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
        RetrievalStates.currentState = RetrievalStates.DRIVE;
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
                                robot.pixels.liftAutoFar();
                                robot.pixels.liftPower(0.5);
                                robot.pixels.armOuttakeAuto();
                                robot.pixels.guideDrop();
                                robot.pixels.pivotDrop();
                                DeliveryStates.currentState = DeliveryStates.RAISE;
                            }
                            break;
                        case RAISE:
                            if (!robot.pixels.liftBusy()) {
                                robot.pixels.noseRelease();
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
                                robot.drivetrain.followTrajAsync(cycleIntake);
                                DeliveryStates.currentState = DeliveryStates.DRIVE;
                                DriveStates.currentState = DriveStates.RETRIEVAL;
                            }
                    }
                    break;
                case RETRIEVAL:
                    switch (RetrievalStates.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.extendoTarget = 1600;
                                RetrievalStates.currentState = RetrievalStates.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (!robot.pixels.extendoBusy()) {
                                robot.pixels.intakeAuto();
                                RetrievalStates.currentState = RetrievalStates.INTAKE;
                            }
                            break;
                        case INTAKE:
//                            if (robot.pixels.bucketPixels() == 2) {
                                robot.pixels.fullIntakeEject();
                                robot.pixels.extendoTarget = 0;
                                RetrievalStates.currentState = RetrievalStates.RETRACT;
//                            }
                            break;
                        case RETRACT:
                            if (!robot.pixels.extendoBusy()) {
                                robot.pixels.armTransfer();
                                robot.pixels.pivotTransfer();
                                robot.pixels.intakeOff();
                                robot.drivetrain.followTrajAsync(cycleDrop);
                                RetrievalStates.currentState = RetrievalStates.DRIVE;
                                DriveStates.currentState = DriveStates.DELIVERY;
                            }
                    }
                    break;
                case DELIVERY:
                    switch (DeliveryStates.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.liftAutoFar();
                                robot.pixels.liftPower(0.5);
                                robot.pixels.armOuttakeAuto();
                                robot.pixels.guideDrop();
                                robot.pixels.pivotDrop();
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
                                switch (park) {
                                    case CENTER:
                                        robot.drivetrain.followTrajAsync(parkCenter);
                                        break;
                                    case WALL:
                                        robot.drivetrain.followTrajAsync(parkWall);
                                }
                                DeliveryStates.currentState = DeliveryStates.DRIVE;
                                DriveStates.currentState = DriveStates.PARK;
                            }
                    }
                    break;
                case PARK:
                    if (!robot.drivetrain.isBusy()) {
                        robot.stop();
                    }
            }
            robot.pixels.update();
            robot.drivetrain.update();
            Pose2d pose = robot.drivetrain.getPose();
            robot.telemetry.addData("x", pose.getX());
            robot.telemetry.addData("y", pose.getY());
            robot.telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            robot.telemetry.update();
        }
    }
}
