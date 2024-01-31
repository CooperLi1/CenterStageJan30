package org.firstinspires.ftc.teamcode.auto.config;

import android.os.Environment;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@Autonomous
public class PopulateConfig extends LinearOpMode {
    private final String configPathString = "%s/FIRST/AutoConfigs";

    private String[] configNames;
    private int selection = 0;

    private final Parameter[] parameters = {
            new Parameter("alliance", "blue", "red"),
            new Parameter("path", "close", "perimeter", "middle", "cycle"),
            new Parameter("park", "center", "wall"),
            new Parameter("delay", "0", "5", "7", "10")
    };

    @Override
    public void runOpMode() {
        String absolutePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        File configPath = new File(String.format(configPathString, absolutePath));
        configNames = configPath.list();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData("Instructions","\nTriangle to scroll up\nCross to scroll down\nCircle to select\n").setRetained(true);
        printConfigs();
        while (!(isStopRequested() || gamepad1.circle)) {
            if (gamepad1.triangle || gamepad2.triangle) {
                selection = (selection - 1) % configNames.length;
                printConfigs();
                while (gamepad1.triangle || gamepad2.triangle);
            } else if (gamepad1.cross || gamepad2.cross) {
                selection = (selection + 1) % configNames.length;
                printConfigs();
                while (gamepad1.cross || gamepad2.cross);
            }
        }
        while (gamepad1.circle);
        telemetry.clearAll();
        String configName = configNames[selection];
        telemetry.addData("File", configName).setRetained(true);
        Map<String, String> config = new HashMap<>();
        for (Parameter parameter: parameters) {
            parameter.select(telemetry, gamepad1, this, config);
        }
        telemetry.addLine("Press play to confirm selections");
        telemetry.update();
        waitForStart();
        try {
            FileWriter writer = new FileWriter(String.format(configPathString + "/" + configName, absolutePath));
            Gson gson = new Gson();
            gson.toJson(config, writer);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void printConfigs() {
        for (int i = 0; i < configNames.length; i++) {
            String configName = configNames[i];
            if (i == selection) {
                configName = "<font color='green'>" + configName + "</font>";
            }
            telemetry.addLine(configName);
        }
        telemetry.update();
    }
}
