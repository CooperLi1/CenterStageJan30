/*
This is the class where all your subsystems are stored.
 */

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.subsystem.Hang;

import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Pixels;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;

import java.util.ArrayList;
import java.util.List;

public abstract class Robot extends LinearOpMode {
    // declare subsystem instances
    // protected Example example;
    public Drivetrain drivetrain;
    public Pixels pixels;
    public Hang hang;
    public Shooter shooter;

    private List<Subsystem> subsystems;

    protected void init(Alliance alliance, boolean auto) {
        Subsystem.init(new Controls(gamepad1, gamepad2), alliance, auto, telemetry, hardwareMap, this);
        subsystems = new ArrayList<>();

        // init subsystem instances
        // example = new Example();
        drivetrain = new Drivetrain();
        pixels = new Pixels();
        hang = new Hang();
        shooter = new Shooter();

        // add subsystems to list
        // subsystems.add(example);
        subsystems.add(drivetrain);
        subsystems.add(pixels);
        subsystems.add(hang);
        subsystems.add(shooter);
    }

    protected void teleOp() {
        for (Subsystem subsystem: subsystems) {
            subsystem.onStart();
        }

        while (opModeIsActive()) {
            for (Subsystem subsystem: subsystems) {
                subsystem.manualControl();
            }
            telemetry.update();
        }
    }
}
