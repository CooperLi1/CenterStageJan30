package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision extends OpenCvPipeline {
    public enum Position {
        LEFT, CENTER, RIGHT
    }

    private int box1X = 1;
    private int box1Y = 140;
    private int box2X = 247;
    private int box2Y = 146;
    private int box3X = 128;
    private int box3Y = 141;
    private int boxWidth = 36;
    private int boxHeight = 40;

    public Position position = Position.CENTER;
    private Alliance alliance;

    public Vision(Alliance alliance) {
        super();
        this.alliance = alliance;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat working = input;
        Imgproc.cvtColor(input, working, Imgproc.COLOR_RGB2YCrCb);
        Mat box1 = working.submat(box1Y, box1Y + boxHeight, box1X, box1X + boxWidth);
        Mat box2 = working.submat(box2Y, box2Y + boxHeight, box2X, box2X + boxWidth);
        Mat box3 = working.submat(box3Y, box3Y + boxHeight, box3X, box3X + boxWidth);

        double highest;
        double val2;
        switch (alliance) {
            case BLUE:
                position = Position.LEFT;
                highest = Core.mean(box1).val[2];
                val2 = Core.mean(box2).val[2];
                if (val2 > highest) {
                    highest = val2;
                    position = Position.RIGHT;
                }
                if (Core.mean(box3).val[2] > highest) {
                    position = Position.CENTER;
                }
                break;
            case RED:
                position = Position.LEFT;
                highest = Core.mean(box1).val[1];
                val2 = Core.mean(box2).val[1];
                if (val2 > highest) {
                    highest = val2;
                    position = Position.RIGHT;
                }
                if (Core.mean(box3).val[1] > highest) {
                    position = Position.CENTER;
                }
        }

        Mat display = working;
        Imgproc.rectangle(
                display,
                new Point(box1X, box1Y),
                new Point(box1X + boxWidth, box1Y + boxHeight),
                new Scalar(255, 255, 255)
        );
        Imgproc.rectangle(
                display,
                new Point(box2X, box2Y),
                new Point(box2X + boxWidth, box2Y + boxHeight),
                new Scalar(0, 255, 0)
        );
        Imgproc.rectangle(
                display,
                new Point(box3X, box3Y),
                new Point(box3X + boxWidth, box3Y + boxHeight),
                new Scalar(255, 0, 255)
        );
        return display;
    }

    public void adjustBox(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad1.left_stick_y < -0.5 || gamepad2.left_stick_y < -0.5) {
            box1Y--;
            while (gamepad1.left_stick_y < -0.5 || gamepad2.left_stick_y < -0.5);
        }
        if (gamepad1.left_stick_y > 0.5 || gamepad2.left_stick_y > 0.5) {
            box1Y++;
            while (gamepad1.left_stick_y > 0.5 || gamepad2.left_stick_y > 0.5);
        }
        if (gamepad1.left_stick_x > 0.5 || gamepad2.left_stick_x > 0.5) {
            box1X++;
            while (gamepad1.left_stick_x > 0.5 || gamepad2.left_stick_x > 0.5);
        }
        if (gamepad1.left_stick_x < -0.5 || gamepad2.left_stick_x < -0.5) {
            box1X--;
            while (gamepad1.left_stick_x < -0.5 || gamepad2.left_stick_x < -0.5);
        }

        if (gamepad1.right_stick_y < -0.5 || gamepad2.right_stick_y < -0.5) {
            box2Y--;
            while (gamepad1.right_stick_y < -0.5 || gamepad2.right_stick_y < -0.5);
        }
        if (gamepad1.right_stick_y > 0.5 || gamepad2.right_stick_y > 0.5) {
            box2Y++;
            while (gamepad1.right_stick_y > 0.5 || gamepad2.right_stick_y > 0.5);
        }
        if (gamepad1.right_stick_x > 0.5 || gamepad2.right_stick_x > 0.5) {
            box2X++;
            while (gamepad1.right_stick_x > 0.5 || gamepad2.right_stick_x > 0.5);
        }
        if (gamepad1.right_stick_x < -0.5 || gamepad2.right_stick_x < -0.5) {
            box2X--;
            while (gamepad1.right_stick_x < -0.5 || gamepad2.right_stick_x < -0.5);
        }

        if (gamepad1.triangle || gamepad2.triangle) {
            box3Y--;
            while (gamepad1.triangle || gamepad2.triangle);
        }
        if (gamepad1.cross || gamepad2.cross) {
            box3Y++;
            while (gamepad1.cross || gamepad2.cross);
        }
        if (gamepad1.circle || gamepad2.circle) {
            box3X++;
            while (gamepad1.circle || gamepad2.circle);
        }
        if (gamepad1.square || gamepad2.square) {
            box3X--;
            while (gamepad1.square || gamepad2.square);
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            boxHeight--;
            while (gamepad1.dpad_up || gamepad2.dpad_up);
        }
        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            boxHeight++;
            while (gamepad1.dpad_down || gamepad2.dpad_down);
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            boxWidth++;
            while (gamepad1.dpad_right || gamepad2.dpad_right);
        }
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            boxWidth--;
            while (gamepad1.dpad_left || gamepad2.dpad_left);
        }
        telemetry.addData("box 1 x", box1X);
        telemetry.addData("box 1 y", box1Y);
        telemetry.addData("box 2 x", box2X);
        telemetry.addData("box 2 y", box2Y);
        telemetry.addData("box 3 x", box3X);
        telemetry.addData("box 3 y", box3Y);
        telemetry.addData("box width", boxWidth);
        telemetry.addData("box height", boxHeight);
    }
}
