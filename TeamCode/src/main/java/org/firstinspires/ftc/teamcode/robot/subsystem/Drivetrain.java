package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Drivetrain extends Subsystem {
    private SampleMecanumDrive drive;

    private DcMotorEx fl, fr, bl, br;

    private AnalogInput ul, ur;

    double forwardsPower, strafePower, turnPower, multiplier;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private double loopTime = 0.0;

    public Drivetrain() {
        if (auto) {
            ul = hardwareMap.get(AnalogInput.class, "ul");
            ur = hardwareMap.get(AnalogInput.class, "ur");
            drive = new SampleMecanumDrive(hardwareMap);
        } else {
            fl = hardwareMap.get(DcMotorEx.class, "fl");
            fr = hardwareMap.get(DcMotorEx.class, "fr");
            bl = hardwareMap.get(DcMotorEx.class, "bl");
            br = hardwareMap.get(DcMotorEx.class, "br");
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setDirection(DcMotorSimple.Direction.REVERSE);
            leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));
            rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
            frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "re"));
            rightEncoder.setDirection(Encoder.Direction.REVERSE);
        }
    }

    @Override
    public void onStart() {

    }

    @Override
    public void manualControl() {
        forwardsPower = controls.forwardsPower();
        strafePower = controls.strafePower();
        turnPower = controls.turnPower();
        multiplier = controls.driveMultiplier() / Math.max(Math.abs(forwardsPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
        fl.setPower((forwardsPower + turnPower + strafePower) * multiplier);
        fr.setPower((forwardsPower - turnPower - strafePower) * multiplier);
        bl.setPower((forwardsPower + turnPower - strafePower) * multiplier);
        br.setPower((forwardsPower - turnPower + strafePower) * multiplier);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
//        telemetry.addData("avrg voltage", (fl.getCurrent(CurrentUnit.AMPS) + fr.getCurrent(CurrentUnit.AMPS) + bl.getCurrent(CurrentUnit.AMPS) + br.getCurrent(CurrentUnit.AMPS)) / 4.0);

    }
    public void setPose(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void setPose(Vector2d pose) {
        drive.setPoseEstimate(new Pose2d(pose, getPose().getHeading()));
    }

    public Pose2d getPose() {
        drive.update();
        return drive.getPoseEstimate();
    }

    public TrajectorySequenceBuilder newTraj() {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate());
    }

    public TrajectorySequenceBuilder newTraj(Pose2d start) {
        return drive.trajectorySequenceBuilder(start);
    }

    public TrajectorySequenceBuilder newTraj(Pose2d start, double vel, double accel) {
        return drive.trajectorySequenceBuilder(start, vel, accel);
    }

    public void followTraj(TrajectorySequence traj) {
        drive.followTrajectorySequence(traj);
    }

    public void followTrajAsync(TrajectorySequence traj) { drive.followTrajectorySequenceAsync(traj); }

    public void turn(double angle) {
        drive.turn(angle);
    }

    public double calculateTurnTo(double endHeading) {
        if (endHeading > Math.PI) {
            endHeading -= Math.PI * 2;
        }
        double currentHeading = getPose().getHeading();
        double difOne = (endHeading - currentHeading) % (Math.PI * 2);
        double difTwo = ((endHeading + (Math.PI * 2)) - currentHeading) % (Math.PI * 2);
        if (Math.abs(difOne) < Math.abs(difTwo)) {
            return difOne;
        } else {
            return difTwo;
        }
    }

    public void turnTo(double endHeading) {
        turn(calculateTurnTo(endHeading));
    }

    public void update() {
        drive.update();
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public double ulInch() {
        return (ul.getVoltage() - 0.142) * 12 / 0.138;
    }

    public double urInch() {
        return (ur.getVoltage() - 0.142) * 12 / 0.138;
    }
}
