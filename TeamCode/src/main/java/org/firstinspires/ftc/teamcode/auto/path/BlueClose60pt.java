package org.firstinspires.ftc.teamcode.auto.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueClose60pt extends AutoPath {

    private enum AutoState {
        PRELOADS, RETRIEVAL, DELIVERY, PARK;
        static AutoState currentState;
    }

    private enum RobotState {
        DRIVE, SCORE, SCORE2, DROPWAIT, INTAKE, RETRACT, RESET, EXTEND;
        static RobotState currentState;
    }

    private final TrajectorySequence centerPreload;
    private final TrajectorySequence leftPreload;
    private final TrajectorySequence rightPreload;

    private final TrajectorySequence centerCycleIntake;
    private final TrajectorySequence leftCycleIntake;
    private final TrajectorySequence rightCycleIntake;

    private final TrajectorySequence cycleReady;

    private final TrajectorySequence cycleScore;
    private final TrajectorySequence parkCenter;
    private final TrajectorySequence parkWall;

    public BlueClose60pt(Auto robot) {
        super(robot);

        Pose2d startPose = new Pose2d(12, 63, Math.toRadians(-90));

        robot.drivetrain.setPose(startPose);
        centerPreload = robot.drivetrain.newTraj(startPose)
                //purple
                .lineToConstantHeading(new Vector2d(12, 33))
                .lineToConstantHeading(new Vector2d(12, 39))
                //yellow
                .lineToLinearHeading(new Pose2d(36, 39, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoNear();
                })
                .lineToConstantHeading(new Vector2d(51,39), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        leftPreload = robot.drivetrain.newTraj(startPose)
                //purple
                .lineToConstantHeading(new Vector2d(22, 40))
                .lineToConstantHeading(new Vector2d(22, 45))
                //yellow
                .lineToLinearHeading(new Pose2d(36, 45, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoNear();
                })
                .lineToConstantHeading(new Vector2d(51,46), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(20))
                .build();
        rightPreload = robot.drivetrain.newTraj(startPose)
                //purple
                .splineToSplineHeading(new Pose2d(12, 48, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(7, 36, Math.toRadians(-135)), Math.toRadians(-135))
                .back(10)
                //yellow
                .lineToLinearHeading(new Pose2d(36, 32, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoNear();
                })
                .lineToConstantHeading(new Vector2d(51,32), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        centerCycleIntake = robot.drivetrain.newTraj(centerPreload.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26,14.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-16, 14.5), Math.toRadians(180))
                .build();
        leftCycleIntake = robot.drivetrain.newTraj(leftPreload.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26,14.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-16, 14.5), Math.toRadians(180))
                .build();
        rightCycleIntake = robot.drivetrain.newTraj(rightPreload.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26,14.5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-16, 14.5), Math.toRadians(180))
                .setReversed(false)
                .build();

        cycleReady = robot.drivetrain.newTraj(centerCycleIntake.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.pixels.armGrab())
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> robot.pixels.noseClawGrab())
                .lineToConstantHeading(new Vector2d(51,12))
                .build();

        cycleScore = robot.drivetrain.newTraj(cycleReady.end())
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> robot.pixels.armOuttakeAuto())
                .lineToConstantHeading(new Vector2d(51,30))
                .build();

        parkCenter = robot.drivetrain.newTraj(cycleScore.end())
                .lineToConstantHeading(new Vector2d(45, 33))
                .lineToConstantHeading(new Vector2d(42, 15.5))
                .lineToConstantHeading(new Vector2d(60, 15.5))
                .build();
        parkWall = robot.drivetrain.newTraj(cycleScore.end())
                .lineToConstantHeading(new Vector2d(45, 45))
                .lineToConstantHeading(new Vector2d(42, 63.5))
                .lineToConstantHeading(new Vector2d(60, 63.5))
                .build();
    }

    public void run(Vision.Position position, Park park) {
        robot.pixels.noseClawGrab();
        robot.pixels.pivotTransfer();
        robot.pixels.clamOpen();
        robot.pixels.armGrab();
        robot.pixels.guideRest();
        robot.hang.holdHang();
        robot.shooter.holdShooter();
        robot.pixels.extendoTarget = -5;

        AutoState.currentState = AutoState.PRELOADS;
        RobotState.currentState = RobotState.DRIVE;

        ElapsedTime timer = new ElapsedTime();
        switch (position) {
            case CENTER:
                robot.drivetrain.followTrajAsync(centerPreload);
                break;
            case LEFT:
                robot.drivetrain.followTrajAsync(leftPreload);
                break;
            case RIGHT:
                robot.drivetrain.followTrajAsync(rightPreload);
        }

        while (robot.opModeIsActive()) {
            switch (AutoState.currentState) {
                case PRELOADS:
                    switch (RobotState.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.armOuttakeAuto();
                                RobotState.currentState = RobotState.SCORE;
                            }
                            break;
                        case SCORE:
                            if (!robot.pixels.liftBusy()) {
                                robot.pixels.noseClawRelease();
                                timer.reset();
                                RobotState.currentState = RobotState.DROPWAIT;
                            }
                            break;
                        case DROPWAIT:
                            if (timer.milliseconds() > 500) {
                                robot.pixels.armTransport();
                                robot.pixels.pivotTransfer();
                                robot.pixels.liftDown();
                                robot.pixels.clamClose();
                                robot.pixels.guideRest();
                                RobotState.currentState = RobotState.RESET;
                            }
                            break;
                        case RESET:
                            if (!robot.pixels.liftBusy()) {
                                robot.pixels.liftPower(0);
                                switch (position) {
                                    case CENTER:
                                        robot.drivetrain.followTrajAsync(centerCycleIntake);
                                        break;
                                    case LEFT:
                                        robot.drivetrain.followTrajAsync(leftCycleIntake);
                                        break;
                                    case RIGHT:
                                        robot.drivetrain.followTrajAsync(rightCycleIntake);
                                }
                                RobotState.currentState = RobotState.DRIVE;
                                AutoState.currentState = AutoState.RETRIEVAL;
                            }
                    }
                    break;
                case RETRIEVAL:
                    switch (RobotState.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.extendoTarget = 1530;
                                robot.pixels.fullIntakeOn();
                                robot.pixels.intakeSetPos(0.605);
                                robot.pixels.clamOverdrive();
                                RobotState.currentState = RobotState.EXTEND;
                            }
                            break;
                        case EXTEND:
                            if (robot.pixels.intakePos() < 0.645 && robot.pixels.extendoPos() > 1480) {
                                robot.pixels.intakeSetPos(robot.pixels.intakePos() + 0.001);
                            }
                            if (!robot.pixels.extendoBusy()) {
                                timer.reset();
                                RobotState.currentState = RobotState.INTAKE;
                            }
                            break;
                        case INTAKE:
                            if (robot.pixels.intakePos() < 0.645) {
                                robot.pixels.intakeSetPos(robot.pixels.intakePos() + 0.001);
                            }
                            if (timer.milliseconds() > 1750) {
                                robot.pixels.armTransfer();
                                robot.pixels.extendoTarget = -5;
                                timer.reset();
                                RobotState.currentState = RobotState.RETRACT;
                            }
                            break;
                        case RETRACT:
                            if (timer.milliseconds() > 650) {
                                robot.pixels.fullIntakeEject();
                            }
                            if (!robot.pixels.extendoBusy()) {
                                robot.pixels.intakeLift();
                                robot.pixels.clamOpen();
                                robot.pixels.fullIntakeOff();
                                robot.drivetrain.followTrajAsync(cycleReady);
                                RobotState.currentState = RobotState.DRIVE;
                                AutoState.currentState = AutoState.DELIVERY;
                            }
                    }
                    break;
                case DELIVERY:
                    switch (RobotState.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.prepareDropAutoWhite();
                                robot.drivetrain.followTrajAsync(cycleScore);
                                RobotState.currentState = RobotState.SCORE;
                            }
                            break;
                        case SCORE:
                            if (!robot.drivetrain.isBusy() && !robot.pixels.liftBusy()) {
                                robot.pixels.noseRelease();
                                timer.reset();
                                RobotState.currentState = RobotState.SCORE2;
                            }
                            break;
                        case SCORE2:
                            if (timer.milliseconds() > 500) {
                                robot.pixels.noseClawRelease();
                                timer.reset();
                                RobotState.currentState = RobotState.DROPWAIT;
                            }
                            break;
                        case DROPWAIT:
                            if (timer.milliseconds() > 400) {
                                robot.pixels.armTransport();
                                robot.pixels.pivotTransfer();
                                robot.pixels.liftDown();
                                robot.pixels.clamClose();
                                robot.pixels.guideRest();
                                RobotState.currentState = RobotState.RESET;
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
                                RobotState.currentState = RobotState.DRIVE;
                                AutoState.currentState = AutoState.PARK;
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
            robot.telemetry.addData("AutoState", AutoState.currentState);
            robot.telemetry.addData("RobotState", RobotState.currentState);
            robot.telemetry.addData("extendo", robot.pixels.extendoPos());
            robot.telemetry.update();
        }
    }
}
