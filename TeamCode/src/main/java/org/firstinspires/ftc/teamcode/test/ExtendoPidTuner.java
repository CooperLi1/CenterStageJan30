package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
@Disabled
public class ExtendoPidTuner extends LinearOpMode {
    public static int extendoTarget = 0;
    public static double kP = 0.024, kI = 0, kD = 0.0009, kF = 0.0;
    public double integralSum = 0;
    private double lastError = 0;
    ElapsedTime PIDTimer = new ElapsedTime();
    private DcMotorEx lExtendo;
    private DcMotorEx rExtendo;

    @Override
    public void runOpMode() throws InterruptedException {
        lExtendo = hardwareMap.get(DcMotorEx.class, "le");
        rExtendo = hardwareMap.get(DcMotorEx.class, "re");
        lExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            update();
            telemetry.addData("target", extendoTarget);
            telemetry.addData("actual", lExtendo.getCurrentPosition());
            telemetry.addData("error", lastError);
            telemetry.addData("current", lExtendo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("power", lExtendo.getPower());
            telemetry.update();
        }
    }

    public void update() {
        //MotionState state = profile.get(mpTimer.seconds());

        //double instantTargetPosition = state.getX();

        if (extendoTarget > 1600) {
            extendoTarget = 1600;
        }
        int currentPos = lExtendo.getCurrentPosition();
        extendoPower(PIDController(extendoTarget, currentPos));
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * PIDTimer.seconds();
        double derivative = (error - lastError) / PIDTimer.seconds();
        lastError = error;

        PIDTimer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI) + (Math.signum(error) * kF);
        return output;
    }

    public void extendoPower(double power) {
        lExtendo.setPower(power);
        rExtendo.setPower(power);
    }
}
