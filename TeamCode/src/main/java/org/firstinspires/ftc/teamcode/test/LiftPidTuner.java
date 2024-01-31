package org.firstinspires.ftc.teamcode.test;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
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

import java.util.function.DoubleSupplier;

@TeleOp
@Config
@Disabled
public class LiftPidTuner extends LinearOpMode {
    public static int liftTarget = 0;
    private DcMotorEx lift;
    public static double Kp = 0.02, Ki = 0, Kd = 1, Kg = 0.07;
    DoubleSupplier motorPosition = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return lift.getCurrentPosition();
        }
    };
    RawValue noFilter = new RawValue(motorPosition);
    //feedback controller
    PIDCoefficients liftPIDcoefficients = new PIDCoefficients(Kp,Ki,Kd);
    BasicPID controller = new BasicPID(liftPIDcoefficients);
    //feedforward controller
    FeedforwardCoefficientsEx liftFFcoefficientsEx = new FeedforwardCoefficientsEx(0,0,0,
            Kg,0);
    FeedforwardEx feedforward = new FeedforwardEx(liftFFcoefficientsEx);
    BasicSystem liftPIDF = new BasicSystem(noFilter,controller,feedforward);

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            update();
            telemetry.addData("target", liftTarget);
            telemetry.addData("actual", lift.getCurrentPosition());
            telemetry.addData("current", lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("power", lift.getPower());
            telemetry.update();
        }
    }

    public void update() {
        //MotionState state = profile.get(mpTimer.seconds());

        //double instantTargetPosition = state.getX();

        if (liftTarget == 0 && lift.getCurrentPosition() < 5) {
            lift.setPower(0);
        } else {
            double liftOutput = liftPIDF.update(liftTarget);
            lift.setPower(Math.abs(liftTarget - lift.getCurrentPosition()) < 3 ? Kg : liftOutput);
        }
    }
}
