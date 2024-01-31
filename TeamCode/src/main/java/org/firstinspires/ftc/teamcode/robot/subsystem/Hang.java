package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Hang extends Subsystem {
    private enum State {
        UNRELEASED, RELEASED
    }
    private final Servo lHang, rHang;

    //TODO: tune up pos's
    //released but up
    private final double lUpPos = 0.32;
    private final double rUpPos = 0.75;
    //released but down
    private final double lDownPos = 0.93;
    private final double rDownPos = 0.07;
    //closed and unreleased
    private final double lClosePos = 0.72;
    private final double rClosePos = 0.25;

    private boolean controlPressedLast = false;

    private final int hangTime = 250;
    private final ElapsedTime timer = new ElapsedTime();

    private State state = State.UNRELEASED;

    public Hang() {
        lHang = hardwareMap.get(Servo.class, "hl");
        rHang = hardwareMap.get(Servo.class, "hr");
    }

    @Override
    public void onStart() {
        holdHang();
    }

    @Override
    public void manualControl() {
        switch (state) {
            case UNRELEASED:
                if (controls.hangAction()) {
                    upHang();
                    timer.reset();
                    state = State.RELEASED;
                }
                break;
            case RELEASED:
                if (timer.milliseconds() >= hangTime) {
                    if (controls.hangAction() && !controlPressedLast) {
                            downHang();
                    }
                }
        }
        if (controls.hangAction()) {
            controlPressedLast = true;
        } else {
            controlPressedLast = false;
        }
    }

    public void upHang() {
        lHang.setPosition(lUpPos);
        rHang.setPosition(rUpPos);
    }

    public void downHang() {
        lHang.setPosition(lDownPos);
        rHang.setPosition(rDownPos);
    }

    public void holdHang() {
        lHang.setPosition(lClosePos);
        rHang.setPosition(rClosePos);
    }

}
