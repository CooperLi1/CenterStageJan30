package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Shooter extends Subsystem {
    private final Servo ss, sr;

    private final double ssHold = 0.5;
    private final double ssShoot = 0.16;
    private final double srHold = 0.41;
    private final double srShoot = 0.59;

    private final int wait = 500;

    private ElapsedTime timer = new ElapsedTime();

    public Shooter() {
        ss = hardwareMap.get(Servo.class, "ss");
        sr = hardwareMap.get(Servo.class, "sr");
        holdShooter();
    }

    @Override
    public void onStart() {
        holdShooter();
    }

    @Override
    public void manualControl() {
        if (controls.shoot()) {
            sr.setPosition(srShoot);
        } else {
            timer.reset();
            holdShooter();
        }

        if (timer.milliseconds() > wait) {
            ss.setPosition(ssShoot);
        }
    }

    public void holdShooter() {
        ss.setPosition(ssHold);
        sr.setPosition(srHold);
    }
}
