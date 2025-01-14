package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Concurrent Autonomous with Sequential Intake", group = "Simple")
public class BasicAutonomous extends LinearOpMode {

    // -------------------- CONSTANTS --------------------
    private static final boolean USE_DEAD_WHEELS = false; // Toggle between encoder-based and time-based movement
    private static final double WHEEL_DIAMETER_MM = 48.0;
    private static final double TICKS_PER_REV = 1440;
    private static final double DISTANCE_TO_TRAVEL_MM = 1000.0;
    private static final double DRIVE_TIME_SEC_1 = 0; // First drive time (low speed)
    private static final double DRIVE_TIME_SEC_2 = 0; // Second drive time (higher speed)

    private static final double DRIVE_SPEED_LOW = 0.2; // Low drive speed
    private static final double DRIVE_SPEED_HIGH = -0.5; // High drive speed

    // Arm parameters (time and encoder positions)
    private static final int ARM_LEFT_POSITION = 2768; // Left arm target position
    private static final int ARM_RIGHT_POSITION = 2493; // Right arm target position
    private static final int ARM_SWING_POSITION = -2212; // Arm swing target position
    private static final double ARM_POWER = 0.1;  // Motor power for arm movements

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

        // Set motors to brake mode when power is zero
        setMotorBrakeMode(leftFront);
        setMotorBrakeMode(leftBack);
        setMotorBrakeMode(rightFront);
        setMotorBrakeMode(rightBack);
        setMotorBrakeMode(armLeft);
        setMotorBrakeMode(armRight);
        setMotorBrakeMode(armSwing);

        armRight.setDirection(DcMotor.Direction.REVERSE);


        intakeServo.setPower(0);  // Initially stop intake

        // Reset arm motor encoders to 0 for a known starting position
        resetArmEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // -------------------- START CONCURRENT ACTIONS --------------------
            // Drive forward at low speed for the first time duration
            driveForward(DRIVE_SPEED_LOW, DRIVE_TIME_SEC_1);

            // Drive forward at high speed for the second time duration
            driveForward(DRIVE_SPEED_HIGH, DRIVE_TIME_SEC_2);

            // Move the arm motors concurrently (armLeft, armRight, armSwing)
            moveArmToPositionConcurrently(ARM_LEFT_POSITION, ARM_RIGHT_POSITION, ARM_SWING_POSITION);

            // Ensure all concurrent tasks have finished before starting the intake
            sleep(3000);  // Adjust to the longest time for movement phases

            // -------------------- INTAKE PHASE --------------------
            runIntake();

            // Stop all motors after the intake finishes
            stopAll();
        }
    }

    // -------------------- CONCURRENT ACTIONS --------------------

    private void driveForward(double speed, double timeSec) {
        setDrivePower(speed);  // Set drive speed
        sleep((long) (timeSec * 1000));  // Sleep for the given time in seconds
        stopDriving();  // Stop after the time has elapsed
    }

    private void moveArmToPositionConcurrently(int armLeftPosition, int armRightPosition, int armSwingPosition) {
        // Set target positions for all motors (armLeft, armRight, armSwing)
        armLeft.setTargetPosition(armLeftPosition);
        armRight.setTargetPosition(armRightPosition);
        armSwing.setTargetPosition(armSwingPosition);

        // Set all motors to RUN_TO_POSITION mode
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power to move arms
        armLeft.setPower(ARM_POWER);
        armRight.setPower(ARM_POWER);
        armSwing.setPower(ARM_POWER);

        // Wait until all motors reach their target position
        while (opModeIsActive() && (armLeft.isBusy() || armRight.isBusy() || armSwing.isBusy())) {
            telemetry.addData("Arm Movement", "Moving concurrently...");
            telemetry.update();
        }

        // Stop all arm motors after reaching their target positions
        armLeft.setPower(0);
        armRight.setPower(0);
        armSwing.setPower(0);
    }

    // -------------------- SEQUENTIAL ACTION --------------------

    private void runIntake() {
        intakeServo.setPower(INTAKE_POWER);  // Start intake
        sleep((long) (INTAKE_DURATION_SEC * 1000));  // Run for the given duration
        intakeServo.setPower(0);  // Stop intake
    }

    // -------------------- STOP EVERYTHING --------------------

    private void stopAll() {
        stopDriving();  // Stop the driving motors
        armLeft.setPower(0);  // Stop arm motors
        armRight.setPower(0);
        armSwing.setPower(0);
        intakeServo.setPower(0);  // Stop intake servo

        telemetry.addData("Status", "All actions completed");
        telemetry.update();
    }

    // -------------------- HELPER METHODS --------------------

    private void setMotorBrakeMode(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Set brake mode when power is zero
    }

    private void resetDriveEncoders() {
        // Reset drive motors encoders for consistent movement
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetArmEncoders() {
        // Reset arm motors encoders for a known starting position
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to RUN_USING_ENCODER mode so that encoders are used during movement
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDrivePower(double power) {
        // Set power to all drive motors
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void stopDriving() {
        // Stop all driving motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
