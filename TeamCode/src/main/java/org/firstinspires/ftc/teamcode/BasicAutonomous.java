package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Concurrent Autonomous with Sequential Intake", group = "Simple")
public class BasicAutonomous extends LinearOpMode {

    // -------------------- CONSTANTS --------------------
    private static final boolean USE_DEAD_WHEELS = false; // Toggle between encoder-based and time-based movement

    // **General Movement Parameters**
    private static final double WHEEL_DIAMETER_MM = 48.0;
    private static final double TICKS_PER_REV = 1440;
    private static final double DISTANCE_TO_TRAVEL_MM = 1000.0;
    private static final double DRIVE_TIME_SEC = 2.0;

    // **Arm Parameters**
    private static final double ARM_MOVE_DURATION_SEC = 2.0;
    private static final double ARM_POWER = 1;

    // **Arm Swing Parameters (DC Motor)**
    private static final double ARM_SWING_DURATION_SEC = 1.5;
    private static final double ARM_SWING_POWER = 0.5;

    // **Intake Parameters**
    private static final double INTAKE_DURATION_SEC = 2.0;
    private static final double INTAKE_POWER = 1.0;

    // -------------------- HARDWARE DECLARATIONS --------------------
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor armLeft, armRight;
    private DcMotor armSwing;
    private CRServo intakeServo;

    private static final double DISTANCE_PER_TICK = WHEEL_DIAMETER_MM * Math.PI / TICKS_PER_REV;

    @Override
    public void runOpMode() {
        // -------------------- INITIALIZE HARDWARE --------------------
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armSwing = hardwareMap.get(DcMotor.class, "armSwing");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        setMotorBrakeMode(leftFront);
        setMotorBrakeMode(leftBack);
        setMotorBrakeMode(rightFront);
        setMotorBrakeMode(rightBack);
        setMotorBrakeMode(armSwing);

        intakeServo.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // -------------------- START CONCURRENT ACTIONS --------------------

            // Start drive, arm, and arm swing concurrently
            driveForward();
            moveArm();
            moveArmSwing();

            // Ensure all concurrent tasks have finished before intake starts
            sleep((long) (Math.max(DRIVE_TIME_SEC, Math.max(ARM_MOVE_DURATION_SEC, ARM_SWING_DURATION_SEC)) * 1000));

            // -------------------- INTAKE PHASE --------------------
            runIntake();

            // Stop everything after the intake finishes
            stopAll();
        }
    }

    // -------------------- CONCURRENT ACTIONS --------------------

    private void driveForward() {
        if (USE_DEAD_WHEELS) {
            int targetTicks = (int) (DISTANCE_TO_TRAVEL_MM / DISTANCE_PER_TICK);
            resetDriveEncoders();
            setDrivePower(0.5);
        } else {
            setDrivePower(0.5);
            sleep((long) (DRIVE_TIME_SEC * 1000));
            stopDriving();
        }
    }

    private void moveArm() {
        armLeft.setPower(ARM_POWER);
        armRight.setPower(-ARM_POWER);
        sleep((long) (ARM_MOVE_DURATION_SEC * 1000));
        armLeft.setPower(0);
        armRight.setPower(0);
    }

    private void moveArmSwing() {
        armSwing.setPower(-ARM_SWING_POWER);
        sleep((long) (ARM_SWING_DURATION_SEC * 1000));
        armSwing.setPower(0);
    }

    // -------------------- SEQUENTIAL ACTION --------------------

    private void runIntake() {
        intakeServo.setPower(INTAKE_POWER);
        sleep((long) (INTAKE_DURATION_SEC * 1000));
        intakeServo.setPower(0);
    }

    // -------------------- STOP EVERYTHING --------------------

    private void stopAll() {
        stopDriving();
        armLeft.setPower(0);
        armRight.setPower(0);
        armSwing.setPower(0);
        intakeServo.setPower(0);

        telemetry.addData("Status", "All actions completed");
        telemetry.update();
    }

    // -------------------- HELPER METHODS --------------------

    private void setMotorBrakeMode(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setDrivePower(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void stopDriving() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
