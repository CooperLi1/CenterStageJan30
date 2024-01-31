package org.firstinspires.ftc.teamcode.auto.path;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;

public abstract class AutoPath {
    private boolean testing = true;

    protected Auto robot;
    protected int position;

    protected AutoPath(Auto robot) {
        this.robot = robot;
    }

    public abstract void run(Vision.Position position, Park park);

    protected void gamepadWait() {
        while (testing && robot.opModeIsActive() && !robot.gamepad1.cross && !robot.gamepad2.cross);
    }
}
