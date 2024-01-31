package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.teamcode.auto.Auto.startArmIn;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import java.util.function.DoubleSupplier;

@Config
public class Pixels extends Subsystem {
    private enum State {
        INTAKE, TRANSFER, GRAB, PREPARE, ALIGN, NOSE, MOVE, CLAW, BOTH, RESET, MANUALRESET, FINISHED
    }

    private enum TransferState {
        ARM_CLAM_UP, ARM_GRAB
    }

    private enum Dropdown {
        Normal, Stack
    }


    private final DcMotorEx intake;
    private final DcMotorEx lift;
    private final DcMotorEx lExtendo;
    private final DcMotorEx rExtendo;
    private final Servo dropdown, arm, pivot, nose, claw, guide, clam;
    private final CRServo roller;
    //private final DistanceSensor color;
    private final double intakeUp = 0.46;
    private double intakeDown = 0.74;
    private double jamDetectAmp = 6;

    private final double guideDrop = 0.16;
    private final double guideRest = 0.5;
    private final double guideUp = 0.82;

    private final double clamOpen = 0.81;
    private final double clamClose = 0.305;
    private final double clamOverdrive = 0.28;

    private final double pivotDrop = 0.29;
    public final double pivotTransfer = 0.5;

    private final double intakeStack = 0.64;

    private final double intakePower = 1;
    private final double ejectPower = -0.75;


    private final double armGrab = 0.14;
    private final double armTransfer = 0.41;
    private final double armDrop = 0.71;
    private final double armDropAuto = 0.725;
    private final double armTransport = 0.34;

    private final double noseIdle = 0.33;
    private final double nosePixel = 0.45;

    private final double clawGrab = 0.59;
    private final double clawIdle = 0.33;

    private final double rollerIntakePower = 1;
    private final double rollerEjectPower = -1;

    private final double extendoHoldPower = -0.25;
    private final double fullPowerTravel = 500;
    private final int liftMax = 2000;
    private final int liftBackdrop = 630;
    private final int liftAutoNear = 470;
    private final int liftAutoFar = 550;
    private final int liftAutoWhite = 940;

    private final int clamWait = 150;
    private final int armWait = 300;
    private final int grabWait = 400;
    private final int noseWait = 1000;
    private final int clawWait = 1000;
    private final int dropBothWait = 400;

    private final int intakeTime = 3000;
    private final int ejectTime = 2000;

    private boolean clamWasOpen = false;
    private boolean clamChecked = false;
    private boolean armWasTransfer = false;
    private boolean armChecked = false;

    private State state = State.INTAKE;
    private TransferState transferState = TransferState.ARM_CLAM_UP;
    private Dropdown dropdownState = Dropdown.Normal;

    private boolean dropOnePressedLast = false;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime intaketimer = new ElapsedTime();
    private final ElapsedTime dropdowntimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime manualResetTimer = new ElapsedTime();

    private int liftTarget = 0;
    private int liftPosition;

    private int liftIncrement = 200;

    private final double downDrop = 0.74;
    private final double upDrop = 0.645;

    public int extendoTarget = 0;
    private double kP = 0.024, kI = 0, kD = 0.0009, kF = 0.0;;
    public double integralSum = 0;
    private double lastError = 0;
    ElapsedTime PIDTimer = new ElapsedTime();

    //lift PIDF
    public static double Kp = 0.0125, Ki = 0, Kd = 0, Kg = 0.25;
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

    public Pixels() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        dropdown = hardwareMap.get(Servo.class, "dropdown");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lExtendo = hardwareMap.get(DcMotorEx.class, "le");
        rExtendo = hardwareMap.get(DcMotorEx.class, "re");
        arm = hardwareMap.get(Servo.class, "arm");
        pivot = hardwareMap.get(Servo.class, "pivot");
        nose = hardwareMap.get(Servo.class, "nose");
        claw = hardwareMap.get(Servo.class, "claw");
        clam = hardwareMap.get(Servo.class, "bucket");
        guide = hardwareMap.get(Servo.class, "guide");
        roller = hardwareMap.get(CRServo.class, "roller");
        //color = hardwareMap.get(DistanceSensor.class, "color");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        roller.setDirection(DcMotorSimple.Direction.REVERSE);

        lExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (auto) {
            intakeLift();
            guideUp();
            clamOpen();
            if (startArmIn) {
                armGrab();
                noseClawGrab();
            } else {
                armTransfer();
                noseClawRelease();
            }
            pivotTransfer();
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0);
        } else {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void onStart() {
        intakeLift();
        armTransport();
        pivotTransfer();
        guideRest();
        clamClose();
        noseClawRelease();
    }

    @Override
    public void manualControl() {
        liftPosition = lift.getCurrentPosition();
        if (controls.manualReset()) {
            if (state != State.MANUALRESET) {
                if (manualResetTimer.milliseconds() > 1500) {
                    armTransfer();
                    noseClawRelease();
                    guideUp();
                    fullIntakeOff();
                    clamClose();
                    intakeLift();
                    transferState = TransferState.ARM_CLAM_UP;
                    state = State.MANUALRESET;
                    manualResetTimer.reset();
                }
            } else {
                if (manualResetTimer.milliseconds() > 750) {
                    lExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lExtendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    guideRest();
                    transferState = TransferState.ARM_CLAM_UP;
                    state = State.INTAKE;
                    manualResetTimer.reset();
                }
            }
        } else {
            manualResetTimer.reset();
        }
        if (controls.stateReset()) {
            timer.reset();
            switch (state) {
                case INTAKE:
                case TRANSFER:
                case GRAB:
                    transferState = TransferState.ARM_CLAM_UP;
                    state = State.RESET;
                    break;
                case PREPARE:
                case ALIGN:
                case NOSE:
                case MOVE:
                case CLAW:
                case BOTH:
                case FINISHED:
                    if (Math.abs(nose.getPosition() - nosePixel) < 0.05 || Math.abs(claw.getPosition() - clawGrab) < 0.05) {
                        state = State.BOTH;
                    } else if (liftPosition > 325) {
                        transferState = TransferState.ARM_CLAM_UP;
                        state = State.RESET;
                    } else {
                        controls.gamepad2Rumble();
                    }
            }
        }
        switch(dropdownState){
            case Normal:
                intakeDown = downDrop;
                if(controls.dropdownSwitch() && dropdowntimer.milliseconds() > 250){
                    dropdownState = Dropdown.Stack;
                    dropdowntimer.reset();
                }
                break;
            case Stack:
                intakeDown = upDrop;
                if(controls.dropdownSwitch() && dropdowntimer.milliseconds() > 250){
                    dropdownState = Dropdown.Normal;
                    dropdowntimer.reset();
                }
                break;
        }
        switch (state) {
            case INTAKE:
                if (liftPosition < 38) {
                    armTransport();
                }
                if (controls.lowerIntake()) {
                    intakeDrop();
                } else {
                    intakeLift();
                }
                if (controls.intakeActivate()) {
                    fullIntakeOn();
                    intakeAntiJam();
                    if (!controls.manualClam()) {
                        clam.setPosition(clamOverdrive);
                    } else {
                        clamOpen();
                    }
                } else if (controls.ejectActivate()){
                    if (!controls.manualClam()) {
                        clamClose();
                    } else {
                        clamOpen();
                    }
                    fullIntakeEject();
                    intaketimer.reset();
                } else {
                    fullIntakeOff();
                    if (!controls.manualClam()) {
                        clamClose();
                    } else {
                        clamOpen();
                    }
                    intaketimer.reset();
                }
                if (controls.transfer() && Math.abs(lExtendo.getCurrentPosition()) < 40) {
                    state = State.TRANSFER;
                    timer.reset();
                } else {
                    break;
                }
            case TRANSFER:
                //testing boolean forces state machine to wait for gamepadWait
                boolean testing = false;
                //ensure lift is fully down during transfer
                resetLift();
                //Sub state machine handles transfer process
                switch (transferState) {
                    case ARM_CLAM_UP:
                        armTransfer();
                        clamOpen();
                        fullIntakeEject();
                        extendoPower(extendoHoldPower);
                        if (testing) {
                            if (timer.milliseconds() > clamWait && controls.gamepadWait()) {
                                transferState = TransferState.ARM_GRAB;
                                timer.reset();
                            } else {
                                break;
                            }
                        } else {
                            if (timer.milliseconds() > clamWait) {
                                transferState = TransferState.ARM_GRAB;
                                timer.reset();
                            } else {
                                break;
                            }
                        }
                    case ARM_GRAB:
                        fullIntakeOff();
                        pivotTransfer();
                        armGrab();
                        if (timer.milliseconds() > 400 && controls.prepareDrop()) {
                            transferState = TransferState.ARM_CLAM_UP;
                            state = State.GRAB;
                            timer.reset();
                        }
                }
                break;
            case GRAB:
                noseClawGrab();
                if (timer.milliseconds() > grabWait / 2.0) {
                    armOuttake();
                } else if (timer.milliseconds() > grabWait) {
                    state = State.PREPARE;
                    timer.reset();
                } else {
                    break;
                }
            case PREPARE:
                liftTarget = liftBackdrop;
                pivotDrop();
                guideDrop();
                if (liftPosition >= liftBackdrop - 15) {
                    state = State.ALIGN;
                } else {
                    break;
                }
            case ALIGN:
                if (controls.dropOne()) {
                    state = State.NOSE;
                } else if (controls.dropBoth()) {
                    timer.reset();
                    state = State.BOTH;
                    break;
                } else {
                    adjustLift();
                    break;
                }
            case NOSE:
                noseRelease();
                clamClose();
                timer.reset();
                state = State.MOVE;
                break;
            case MOVE:
                if ((controls.dropOne() && !dropOnePressedLast) || controls.dropBoth()) {
                    state = State.CLAW;
                } else {
                    adjustLift();
                    break;
                }
            case CLAW:
                clawRelease();
                clamClose();
                timer.reset();
                state = State.FINISHED;
            case BOTH:
                clamClose();
                noseRelease();
                if (timer.milliseconds() > dropBothWait) {
                    clawRelease();
                    timer.reset();
                    state = State.FINISHED;
                } else {
                    break;
                }
            case FINISHED:
                adjustLift();
                break;
            case MANUALRESET:
                manualResetLift();
                break;
            case RESET:
                //TODO: clean up reset logic
                //check when first switched to reset whether clam is open
                if (!clamChecked) {
                    clamWasOpen = Math.abs(clam.getPosition() - clamOpen) < 0.01;
                    clamChecked = true;
                }
                //check when first switched to reset whether arm is at transfer
                if (!armChecked) {
                    armWasTransfer = Math.abs(arm.getPosition() - armTransfer) < 0.01;
                    armChecked = true;
                }
                noseClawRelease();
                guideRest();
                pivotTransfer();
                resetLift();
                //if clam is open ensure it will not hit arm while closing
                if (clamWasOpen) {
                    //if arm is not at transfer set to transfer
                    if (armWasTransfer) {
                        clamClose();
                        if (timer.milliseconds() > clamWait) {
                            armTransfer();
                            clamChecked = false;
                            armChecked = false;
                            state = State.INTAKE;
                        }
                    } else {
                        armTransfer();
                        if (timer.milliseconds() > armWait) {
                            clamClose();
                            if (timer.milliseconds() > armWait + clamWait) {
                                clamChecked = false;
                                armChecked = false;
                                state = State.INTAKE;
                            }
                        }
                    }

                } else {
                    armTransfer();
                    clamChecked = false;
                    armChecked = false;
                    state = State.INTAKE;
                }
        }
        if (state != State.MANUALRESET) {
            updateLift();
        }
        if (state != State.TRANSFER) {
            if (!controls.extendoIn()) {
                extendoPower(controls.extendo());
            } else if (controls.extendoIn()) {
                if (lExtendo.getCurrentPosition() > 400) {
                    extendoPower(-1);
                } else {
                    extendoPower(-0.8);
                }
            }
        } else {
            extendoPower(extendoHoldPower);
        }

        if (controls.dropOne()) {
            dropOnePressedLast = true;
        } else {
            dropOnePressedLast = false;
        }
        telemetry();
    }

    public void intakeLift() {
        dropdown.setPosition(intakeUp);
    }

    public void intakeDrop() {
        dropdown.setPosition(intakeDown);
    }

    public void stackIntake() { dropdown.setPosition(intakeStack); }

    public double intakePos() { return dropdown.getPosition(); }

    public void intakeSetPos(double pos) { dropdown.setPosition(pos); }

    public void intakeOn() { intake.setPower(intakePower); }

    public void intakeEject() {
        intake.setPower(ejectPower);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void setkP(double val) {kP = val; }

    public void intakeAntiJam() {
        if(intake.getCurrent(CurrentUnit.AMPS) > jamDetectAmp && intaketimer.milliseconds() > 450) {
            intake.setPower(-0.55);
            jamDetectAmp = 2.5;
            if (intaketimer.milliseconds() > 1000) {
                intaketimer.reset();
                jamDetectAmp = 6.5;
            }
        }
    }

    public void rollerOn() { roller.setPower(rollerIntakePower); }

    public void rollerEject() { roller.setPower(rollerEjectPower); }

    public void rollerOff() { roller.setPower(0); }

    public void fullIntakeOn() { intakeOn(); rollerOn();}

    public void fullIntakeEject() { intakeEject(); rollerEject();}

    public void fullIntakeOff() { intakeOff(); rollerOff();}

    public void guideDrop() { guide.setPosition(guideDrop); }

    public void guideRest() { guide.setPosition(guideRest); }

    public void guideUp() { guide.setPosition(guideUp); }

    public void clamClose() { clam.setPosition(clamClose); }

    public void clamOpen() { clam.setPosition(clamOpen); }

    public void pivotTransfer() { pivot.setPosition(pivotTransfer); }

    public void pivotDrop() { pivot.setPosition(pivotDrop); }

    public void armGrab() {
        arm.setPosition(armGrab);
    }

    public void armTransfer() {
        arm.setPosition(armTransfer);
    }

    public void armOuttake() {
        arm.setPosition(armDrop);
    }

    public void armOuttakeAuto() {
        arm.setPosition(armDropAuto);
    }

    public void armTransport() {
        arm.setPosition(armTransport);
    }

    public void noseGrab() {
        nose.setPosition(nosePixel);
    }

    public void clawGrab() {
        claw.setPosition(clawGrab);
    }

    public void noseRelease() {
        nose.setPosition(noseIdle);
    }

    public void clawRelease() {
        claw.setPosition(clawIdle);
    }

    public void noseClawGrab() { noseGrab(); clawGrab();}

    public void noseClawRelease() { noseRelease(); clawRelease();}

    public void liftPower(double power) {
        lift.setPower(power);
    }

    public void extendoPower(double power) {
        lExtendo.setPower(power);
        rExtendo.setPower(power);
    }

    public void adjustLift() {
        liftTarget = (int) (Math.max(Math.min(liftTarget + controls.manualLiftControl(), liftMax), 0));
    }

    public void manualResetLift() {
        liftPower(controls.manualLiftControl() * 0.35);
    }

    public void resetLift() {
        liftTarget = 0;
    }

    public void updateLift() {
        if (liftTarget == 0 && liftPosition < 5) {
            liftPower(0);
        } else {
            double liftOutput = liftPIDF.update(liftTarget);
            liftPower(Math.abs(liftTarget - lift.getCurrentPosition()) < 3 ? Kg : liftOutput);
        }
    }

    public int bucketPixels() {
//        if (color.getDistance(DistanceUnit.CM) < 4.1) {
//            return 2;
//        } else if (color.getDistance(DistanceUnit.CM) < 5.2) {
//            return 1;
//        } else {
//            return 0;
//        }
        return 0;
    }

    public void deliverFirst(boolean reset) {
        lift.setTargetPosition(liftAutoFar);
        lift.setPower(0.5);
        armOuttakeAuto();
        pivotDrop();
        guideDrop();
        autoTimer.reset();
        while (opMode.opModeIsActive() && lift.isBusy() || autoTimer.milliseconds() < armWait) {
        }
        noseRelease();
        clamClose();
        autoTimer.reset();
        while (opMode.opModeIsActive() && autoTimer.milliseconds() < noseWait) {
        }
        if (reset) {
            armTransport();
            lift.setTargetPosition(0);
            while (opMode.opModeIsActive() && lift.isBusy()) {
            }
            lift.setPower(0);
        }
    }

    public void deliverSecond() {
        clawRelease();
        autoTimer.reset();
        while (opMode.opModeIsActive() && autoTimer.milliseconds() < clawWait);
        armTransport();
        lift.setTargetPosition(0);
        while (opMode.opModeIsActive() && lift.isBusy()) {
        }
        lift.setPower(0);
    }

    public void intakeAuto() {
        fullIntakeOn();
        stackIntake();
        autoTimer.reset();
        while (opMode.opModeIsActive() && autoTimer.milliseconds() < intakeTime);
        intakeLift();
        fullIntakeEject();
        autoTimer.reset();
        while (opMode.opModeIsActive() && autoTimer.milliseconds() < ejectTime);
        clamOpen();
    }

    public void intakeExtendo() {
        extendoTarget = 1600;
        fullIntakeOn();
        stackIntake();
        autoTimer.reset();
        while (opMode.opModeIsActive() && autoTimer.milliseconds() < intakeTime);
        intakeLift();
        fullIntakeEject();
        extendoTarget = 0;
        while (lExtendo.getCurrentPosition() > 5);
        clamOpen();
    }

    public void telemetry() {
        telemetry.addData("State: ", state);
        telemetry.addData("TransferState: ", transferState);
        //telemetry.addData("Pixels: ", bucketPixels());
        //telemetry.addData("Dist: ", color.getDistance(DistanceUnit.CM));
        telemetry.addData("LiftPos: ", liftPosition);
        telemetry.addData("LiftTargetPos: ", liftTarget);
        telemetry.addData("Lift Error ", liftTarget-liftPosition);
        telemetry.addData("LiftPower: ", lift.getPower());
        telemetry.addData("extendo pos", lExtendo.getCurrentPosition());
        telemetry.addData("intake current", intake.getCurrent(CurrentUnit.AMPS));
    }

    public void update() {
        //MotionState state = profile.get(mpTimer.seconds());

        //double instantTargetPosition = state.getX();

       if (extendoTarget > 1600) {
            extendoTarget = 1600;
       }
       int currentPos = lExtendo.getCurrentPosition();
       if (currentPos < 100 && extendoTarget < 100) {
           extendoPower(-0.35);
       } else {
           extendoPower(PIDController(extendoTarget, currentPos));
       }
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

    public void liftAutoNear() { lift.setTargetPosition(liftAutoNear); }
    public void liftAutoFar() { lift.setTargetPosition(liftAutoFar); }

    public void liftAutoWhite() { lift.setTargetPosition(liftAutoWhite); }

    public void setLiftTarget(int liftTarget) {
        lift.setTargetPosition(liftTarget);
    }

    public void liftDown() {
        lift.setTargetPosition(0);
    }

    public boolean liftBusy() {
        return lift.isBusy();
    }

    public boolean extendoBusy() {
        return Math.abs(extendoTarget - lExtendo.getCurrentPosition()) > 30;
    }

    public void armPosition(double position) {
        arm.setPosition(position);
    }

    public void prepareDropAutoNear() {
        liftAutoNear();
        liftPower(0.5);
        armPosition(0.65);
        guideDrop();
        pivotDrop();
    }

    public void prepareDropAutoFar() {
        liftAutoFar();
        liftPower(0.5);
        armPosition(0.65);
        guideDrop();
        pivotDrop();
    }

    public void prepareDropAutoWhite() {
        liftAutoWhite();
        liftPower(0.5);
        armPosition(0.65);
        guideDrop();
        pivotDrop();
    }

    public int extendoPos() {
        return lExtendo.getCurrentPosition();
    }

    public void armPos(double pos) {
        arm.setPosition(pos);
    }

    public void clamOverdrive() {
        clam.setPosition(clamOverdrive);
    }
}