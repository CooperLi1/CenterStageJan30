package org.firstinspires.ftc.teamcode.auto.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Parameter {
    private final String name;
    private final String cross;
    private final String circle;
    private final String triangle;
    private final String square;
    private String selection;

    public Parameter(String name, String cross, String circle, String triangle, String square) {
        this.name = name;
        this.cross = cross;
        this.circle = circle;
        this.triangle = triangle;
        this.square = square;
    }

    public Parameter(String name, String cross, String circle, String triangle) {
        this(name, cross, circle, triangle, "");
    }

    public Parameter(String name, String cross, String circle) {
        this(name, cross, circle, "", "");
    }

    public Map<String, String> select(Telemetry telemetry, Gamepad gamepad, LinearOpMode opMode, Map<String, String> output) {
        telemetry.addLine();
        telemetry.addData("Selecting", name);
        telemetry.addData("Cross", cross);
        telemetry.addData("Circle", circle);
        telemetry.addData("Triangle", triangle);
        telemetry.addData("Square", square);
        telemetry.update();
        while (!opMode.isStopRequested()) {
            if (gamepad.cross) {
                selection = cross;
                while (gamepad.cross);
                break;
            } else if (gamepad.circle) {
                selection = circle;
                while (gamepad.circle);
                break;
            } else if (gamepad.triangle) {
                selection = triangle;
                while (gamepad.triangle);
                break;
            } else if (gamepad.square) {
                selection = square;
                while (gamepad.square);
                break;
            }
        }
        telemetry.addData(name, selection).setRetained(true);
        output.put(name, selection);
        return output;
    }
}
