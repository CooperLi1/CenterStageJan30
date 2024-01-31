package org.firstinspires.ftc.teamcode.auto;

import android.os.Environment;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.config.AutoConfig;
import org.firstinspires.ftc.teamcode.auto.config.Park;
import org.firstinspires.ftc.teamcode.auto.config.Path;
import org.firstinspires.ftc.teamcode.auto.path.AutoPath;
import org.firstinspires.ftc.teamcode.auto.path.BlueClose;
import org.firstinspires.ftc.teamcode.auto.path.BlueClose60pt;
import org.firstinspires.ftc.teamcode.auto.path.BlueFarMiddle;
import org.firstinspires.ftc.teamcode.auto.path.BlueFarPerimeter;
import org.firstinspires.ftc.teamcode.auto.path.Default;
import org.firstinspires.ftc.teamcode.auto.path.RedClose;
import org.firstinspires.ftc.teamcode.auto.path.RedClose60pt;
import org.firstinspires.ftc.teamcode.auto.path.RedFarMiddle;
import org.firstinspires.ftc.teamcode.auto.path.RedFarPerimeter;
import org.firstinspires.ftc.teamcode.auto.path.RedFarPerimeter55pt;
import org.firstinspires.ftc.teamcode.auto.vision.Vision;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Locale;

@Autonomous
public class Auto extends Robot {
    private final String configPathString = "%s/FIRST/AutoConfigs";

    private String[] configNames;
    private int selection = 0;

    private AutoConfig config;
    private AutoPath path = new Default(this);

    private AnalogInput us;

    public static boolean startArmIn;

    @Override
    public void runOpMode() {
        String absolutePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        File configPath = new File(String.format(configPathString, absolutePath));
        configNames = configPath.list();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData("Instructions","\nTriangle to scroll up\nCross to scroll down\nCircle to select\n").setRetained(true);
        printConfigs();
        while (!(isStopRequested() || gamepad1.circle || gamepad2.circle)) {
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
        telemetry.clearAll();
        String configName = configNames[selection];
        telemetry.addData("Selected config", configName).setRetained(true);
        Gson gson = new Gson();
        try {
            config = gson.fromJson(new FileReader(String.format(configPathString + "/" + configName, absolutePath)), AutoConfig.class);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        telemetry.addData("alliance", config.alliance).setRetained(true);
        telemetry.addData("path", config.path).setRetained(true);
        telemetry.addData("park", config.park).setRetained(true);
        telemetry.addData("delay", config.delay).setRetained(true);
        Alliance alliance = Alliance.valueOf(config.alliance.toUpperCase(Locale.ROOT));
        Path path = Path.valueOf(config.path.toUpperCase(Locale.ROOT));
        Park park = Park.valueOf(config.park.toUpperCase(Locale.ROOT));
        int delay = Integer.parseInt(config.delay);
        ElapsedTime wait = new ElapsedTime();

        switch (alliance) {
            case BLUE:
                switch (path) {
                    case CYCLE:
                    case CLOSE:
                        us = hardwareMap.get(AnalogInput.class, "ul");
                        break;
                    case MIDDLE:
                    case PERIMETER:
                        us = hardwareMap.get(AnalogInput.class, "ur");
                }
                break;
            case RED:
                switch (path) {
                    case CYCLE:
                    case CLOSE:
                        us = hardwareMap.get(AnalogInput.class, "ur");
                        break;
                    case MIDDLE:
                    case PERIMETER:
                        us = hardwareMap.get(AnalogInput.class, "ul");
                }
        }
        while (opModeInInit() && !gamepad1.cross && !gamepad2.cross) {
            telemetry.addLine("line up the drivetrain, then press cross");
            double dist = usInch();
            telemetry.addData("distance", dist);
            telemetry.addData("voltage", us.getVoltage());
            switch (path) {
                case CYCLE:
                case CLOSE:
                    if (dist < 50) {
                        telemetry.addLine("<font color='red'>move robot away from wall</font>");
                    }
                    if (dist > 52) {
                        telemetry.addLine("<font color='red'>move robot closer to wall</font>");
                    }
                    break;
                case MIDDLE:
                case PERIMETER:
                    if (dist < 27) {
                        telemetry.addLine("<font color='red'>move robot away from wall</font>");
                    }
                    if (dist > 29) {
                        telemetry.addLine("<font color='red'>move robot closer to wall</font>");
                    }
            }
            telemetry.update();
        }
        telemetry.addLine("initializing...");
        telemetry.update();
        if (path == Path.PERIMETER && alliance == Alliance.RED) {
            startArmIn = false;
        } else {
            startArmIn = true;
        }
        init(alliance, true);
        switch (alliance) {
            case BLUE:
                switch (path) {
                    case CLOSE:
                        this.path = new BlueClose(this);
                        break;
                    case PERIMETER:
                        this.path = new BlueFarPerimeter(this);
                        break;
                    case MIDDLE:
                        this.path = new BlueFarMiddle(this);
                        break;
                    case CYCLE:
                        this.path = new BlueClose60pt(this);
                }
                break;
            case RED:
                switch (path) {
                    case CLOSE:
                        this.path = new RedClose(this);
                        break;
                    case PERIMETER:
                        this.path = new RedFarPerimeter55pt(this);
                        break;
                    case MIDDLE:
                        this.path = new RedFarMiddle(this);
                        break;
                    case CYCLE:
                        this.path = new RedClose60pt(this);
                }
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "liftcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Vision pipeline = new Vision(alliance);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        while (opModeInInit()) {
            telemetry.addData("position",  pipeline.position);
            pipeline.adjustBox(gamepad1, gamepad2, telemetry);
            telemetry.update();
        }
        camera.closeCameraDevice();
        wait.reset();
        while (opModeIsActive() && wait.seconds() < delay);
        this.path.run(pipeline.position, park);
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

    public double usInch() {
        return (us.getVoltage() - 0.142) * 12 / 0.138;
    }
}
