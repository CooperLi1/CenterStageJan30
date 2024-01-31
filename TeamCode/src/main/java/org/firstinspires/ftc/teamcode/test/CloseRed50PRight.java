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
public class CloseRed50PRight extends Robot{
    @Override
    public void runOpMode() {
        init(Alliance.RED, true);

        int pixelpos = 3;

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
            .splineTo(new Vector2d(24,-54), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48,-36, Math.toRadians(180)), Math.toRadians(0))
            .waitSeconds(1)
            //park
            .strafeRight(22)
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
                .splineToLinearHeading(new Pose2d(8,-38.5, Math.toRadians(-30)), Math.toRadians(180))
                .setReversed(false)
                //connection back
                .splineTo(new Vector2d(24,-54), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,-36, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(1)
                //park
                .strafeRight(22)
                .back(12)
                .resetConstraints()
                .build();

        TrajectorySequence trajRight = drivetrain.newTraj()
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(0.3 * DriveConstants.base, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.175 * DriveConstants.base)
                )
                //pixel pushing
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(16,-38.5, Math.toRadians(-150)), Math.toRadians(0))
                .setReversed(false)
                //connection back
                .splineTo(new Vector2d(24,-54), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,-36, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(1)
                //park
                .strafeRight(22)
                .back(12)
                .resetConstraints()
                .build();

        waitForStart();

        if (!isStopRequested()) {
            if (pixelpos == 1){
                drivetrain.followTrajAsync(trajLeft);
            }
            else if (pixelpos == 2){
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

