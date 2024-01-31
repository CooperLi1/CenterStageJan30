package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class CloseRed50PLeft extends Robot{
    @Override
    public void runOpMode() {
        init(Alliance.RED, true);

        int position = 1;

        Pose2d startPose = new Pose2d(12, -64.5, Math.toRadians(-90));

        drivetrain.setPose(startPose);

        TrajectorySequence trajCenter = drivetrain.newTraj()
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(0.3 * DriveConstants.base, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.175 * DriveConstants.base)
                )
                .setReversed(true)
                //Purple
                .back(26)
                //yellow
                .setReversed(false)
                .forward(2)
                .splineToConstantHeading(new Vector2d(50,-36), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                //park
                .strafeLeft(27)
                .back(12)
                .resetConstraints()
                .build();

        TrajectorySequence trajLeft = drivetrain.newTraj()
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(0.3 * DriveConstants.base, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.175 * DriveConstants.base)
                )
                //pixel pushing
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(10,-36, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                //connection back
                .splineTo(new Vector2d(50,-36), Math.toRadians(0))
                .turn(Math.toRadians(180))
                .waitSeconds(1)
                //park
                .strafeLeft(27)
                .back(12)
                .resetConstraints()
                .build();

        TrajectorySequence trajRight = drivetrain.newTraj()
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(0.3 * DriveConstants.base, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.175 * DriveConstants.base)
                )
                .setReversed(true)
                //Purple
                .splineToConstantHeading(new Vector2d(23,-38), Math.toRadians(90))
                //yellow
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(50,-36), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                //park
                .strafeLeft(27)
                .back(12)
                .resetConstraints()
                .build();

        waitForStart();

        if (!isStopRequested()) {
            if (position == 1){
                drivetrain.followTrajAsync(trajLeft);
            }
            else if (position == 2){
                drivetrain.followTrajAsync(trajCenter);
            }
            else{
                drivetrain.followTrajAsync(trajRight);
            }
        }
        while (opModeIsActive()) {
            drivetrain.update();
            Pose2d pose = drivetrain.getPose();
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
    }
}

