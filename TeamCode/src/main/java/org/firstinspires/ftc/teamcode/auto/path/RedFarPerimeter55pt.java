package org.firstinspires.ftc.teamcode.auto.path;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedFarPerimeter55pt extends AutoPath {
    private enum DriveStates {
        PURPLE, RETRIEVAL, DELIVERY, PARK;
        static DriveStates currentState;
    }

    private enum DeliveryStates {
        DRIVE, EXTEND, INTAKE, RETRACT, RAISE, RELEASE, RELEASE2, RESET;
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

    public RedFarPerimeter55pt(Auto robot) {
        super(robot);
        robot.pixels.setkP(0.016);
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(90));

        robot.drivetrain.setPose(startPose);

        centerPurple = robot.drivetrain.newTraj(startPose)
                .lineToConstantHeading(new Vector2d(-36, -36))
                .lineToConstantHeading(new Vector2d(-38, -62))
                .setTurnConstraint(MAX_ANG_VEL * 0.75, MAX_ANG_ACCEL * 0.75)
                .turn(Math.toRadians(-120))
                .resetTurnConstraint()
                .build();
        leftPurple = robot.drivetrain.newTraj(startPose)
                .lineToConstantHeading(new Vector2d(-46, -40))
                .lineToConstantHeading(new Vector2d(-38, -62))
                .setTurnConstraint(MAX_ANG_VEL * 0.75, MAX_ANG_ACCEL * 0.75)
                .turn(Math.toRadians(-120))
                .resetTurnConstraint()
                .build();
        rightPurple = robot.drivetrain.newTraj(startPose)
                .splineToLinearHeading(new Pose2d(-31, -40, Math.toRadians(45)), Math.toRadians(45))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-38, -62, Math.toRadians(90)), Math.toRadians(-90))
                .setReversed(false)
                .setTurnConstraint(MAX_ANG_VEL * 0.75, MAX_ANG_ACCEL * 0.75)
                .turn(Math.toRadians(-120))
                .resetTurnConstraint()
                .build();

        centerYellow = robot.drivetrain.newTraj(centerPurple.end())
                .turn(Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> robot.pixels.armGrab())
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> robot.pixels.noseClawGrab())
                .lineToConstantHeading(new Vector2d(36, -62))
                .addTemporalMarker(() -> {
                    robot.pixels.fullIntakeOff();
                })
                .lineToConstantHeading(new Vector2d(36, -39))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoFar();
                })
                .lineToConstantHeading(new Vector2d(51,-39), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(20))
                .build();
        leftYellow = robot.drivetrain.newTraj(leftPurple.end())
                .turn(Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> robot.pixels.armGrab())
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> robot.pixels.noseClawGrab())
                .lineToConstantHeading(new Vector2d(36, -62))
                .addTemporalMarker(() -> {
                    robot.pixels.fullIntakeOff();
                })
                .lineToConstantHeading(new Vector2d(36, -33))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoFar();
                })
                .lineToConstantHeading(new Vector2d(51,-33), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(20))
                .build();
        rightYellow = robot.drivetrain.newTraj(rightPurple.end())
                .turn(Math.toRadians(30))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> robot.pixels.armGrab())
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> robot.pixels.noseClawGrab())
                .lineToConstantHeading(new Vector2d(36, -62))
                .addTemporalMarker(() -> {
                    robot.pixels.fullIntakeOff();
                })
                .lineToConstantHeading(new Vector2d(36, -44))
                .addTemporalMarker(() -> {
                    robot.pixels.prepareDropAutoFar();
                })
                .lineToConstantHeading(new Vector2d(51,-44), SampleMecanumDrive.VEL_CONSTRAINT, SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        parkCenter = robot.drivetrain.newTraj(leftYellow.end())
                .lineToConstantHeading(new Vector2d(44, -16.5))
                .build();
        parkWall = robot.drivetrain.newTraj(rightYellow.end())
                .lineToConstantHeading(new Vector2d(44, -63.5))
                .build();
    }

    public void run(Vision.Position position, Park park) {
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
        DeliveryStates.currentState = DeliveryStates.EXTEND;
        ElapsedTime timer = new ElapsedTime();
        while (robot.opModeIsActive()) {
            switch (DriveStates.currentState) {
                case PURPLE:
                    if (!robot.drivetrain.isBusy()) {
                        DriveStates.currentState = DriveStates.RETRIEVAL;
                    }
                    break;
                case RETRIEVAL:
                    switch (DeliveryStates.currentState) {
                        case EXTEND:
                            robot.pixels.extendoTarget = 910;
                            robot.pixels.fullIntakeOn();
                            robot.pixels.intakeSetPos(0.605);
                            robot.pixels.clamOverdrive();
                            if (robot.pixels.intakePos() < 0.64 && robot.pixels.extendoPos() > 850) {
                                robot.pixels.intakeSetPos(robot.pixels.intakePos() + 0.001);
                            }
                            if (!robot.pixels.extendoBusy()) {
                                timer.reset();
                                DeliveryStates.currentState = DeliveryStates.INTAKE;
                            }
                            break;
                        case INTAKE:
                            robot.pixels.clamOverdrive();
                            if (robot.pixels.intakePos() < 0.64) {
                                robot.pixels.intakeSetPos(robot.pixels.intakePos() + 0.001);
                            }
                            if (timer.milliseconds() > 1750) {
                                robot.pixels.armTransfer();
                                robot.pixels.extendoTarget = -5;
                                timer.reset();
                                DeliveryStates.currentState = DeliveryStates.RETRACT;
                            }
                            break;
                        case RETRACT:
                            if (!robot.pixels.extendoBusy()) {
                                robot.pixels.fullIntakeEject();
                                robot.pixels.intakeLift();
                                robot.pixels.clamOpen();
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
                                DriveStates.currentState = DriveStates.DELIVERY;
                                DeliveryStates.currentState = DeliveryStates.DRIVE;
                            }
                    }
                    break;
                case DELIVERY:
                    switch (DeliveryStates.currentState) {
                        case DRIVE:
                            if (!robot.drivetrain.isBusy()) {
                                robot.pixels.armOuttakeAuto();
                                DeliveryStates.currentState = DeliveryStates.RAISE;
                            }
                            break;
                        case RAISE:
                            if (!robot.pixels.liftBusy()) {
                                robot.pixels.noseRelease();
                                if (timer.milliseconds() > 300) {
                                    robot.pixels.armPosition(0.65);
                                    timer.reset();
                                    DeliveryStates.currentState = DeliveryStates.RELEASE;
                                }
                            }
                            break;
                        case RELEASE:
                            if (timer.milliseconds() > 500) {
                                robot.pixels.noseClawRelease();
                                timer.reset();
                                DeliveryStates.currentState = DeliveryStates.RELEASE2;
                            }
                            break;
                        case RELEASE2:
                            if (timer.milliseconds() > 400) {
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
            robot.pixels.update();
            robot.drivetrain.update();
            Pose2d pose = robot.drivetrain.getPose();
            robot.telemetry.addData("x", pose.getX());
            robot.telemetry.addData("y", pose.getY());
            robot.telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            robot.telemetry.addData("DriveState", DriveStates.currentState);
            robot.telemetry.addData("DeliveryState", DeliveryStates.currentState);
            robot.telemetry.addData("extendo", robot.pixels.extendoPos());
            robot.telemetry.update();
        }
    }
}
