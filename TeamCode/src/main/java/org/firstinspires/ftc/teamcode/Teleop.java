package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

@TeleOp
public class Teleop extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        init(Subsystem.alliance, false);

        waitForStart();

        teleOp();
    }
}
