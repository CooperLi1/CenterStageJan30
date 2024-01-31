/*
This is where you put all methods regarding TeleOp controls. Gamepads shouldn't be called anywhere else.
 */

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controls {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public Controls (Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    private boolean liftUpPressed = false;
    private boolean liftDownPressed = false;

    public boolean gamepadWait() {return gamepad2.dpad_up;}

    public void gamepad1Rumble() {gamepad1.rumble(500);}

    public void gamepad2Rumble() {gamepad2.rumble(500);}

    public boolean stateReset() {return gamepad2.square;}

    public boolean dropdownSwitch() {return gamepad2.circle && !gamepad2.start;}

    public boolean transfer() {return gamepad2.cross;}

    public boolean lowerIntake() {return gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5;}

    public boolean intakeActivate() {return gamepad2.right_trigger > 0.5 || gamepad2.left_bumper;}

    public boolean ejectActivate() {return gamepad2.right_bumper;}

    public boolean prepareDrop() {
        return gamepad2.cross;
    }

    public boolean dropOne() {
        return gamepad1.left_trigger > 0.5;
    }

    public boolean dropBoth() {
        return gamepad1.left_bumper;
    }

    public double manualLiftControl() {
        return -gamepad2.right_stick_y * 30;
    }

    public boolean manualClam() { return gamepad2.dpad_left; }

    public boolean manualReset() { return gamepad2.dpad_down; }

    public double forwardsPower() {
        return -gamepad1.right_stick_y;
    }

    public double strafePower() {
        return gamepad1.right_stick_x;
    }

    public double turnPower() {
        return -gamepad1.left_stick_x;
    }

    public double driveMultiplier() {
        return 1;
    }

    public boolean resetZaxis() {return gamepad1.share; }

    public boolean hangAction() { return gamepad2.options && gamepad2.share; }

    public boolean extendoIn() { return gamepad1.right_bumper; }
    public double extendo() { return gamepad1.right_trigger; }

    public boolean shoot() {
        return gamepad2.dpad_up;
    }
}