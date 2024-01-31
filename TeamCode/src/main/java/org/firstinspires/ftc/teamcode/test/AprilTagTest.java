package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setLensIntrinsics(899.18, 899.18, 320.244, 239.978)

                .build();
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "intakecam"), processor);
        ElapsedTime loopTime = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            loopTime.reset();
            List<AprilTagDetection> currentDetections = processor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.rawPose.x, detection.rawPose.y, detection.rawPose.z));
            }
            telemetry.addData("looptime", 1 / loopTime.seconds());
            telemetry.update();
        }
        portal.close();
    }
}
