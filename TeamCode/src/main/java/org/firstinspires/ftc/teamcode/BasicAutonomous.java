package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Concurrent Autonomous with Sequential Intake", group = "Simple")
public class BasicAutonomous extends LinearOpMode {

    // -------------------- CONSTANTS --------------------
    private static final boolean USE_DEAD_WHEELS = true; // Toggle between encoder-based and time-based movement
    private static final double WHEEL_DIAMETER_MM = 48.0;
    private static final double TICKS_PER_REV = 1440;
    private static final double DISTANCE_TO_TRAVEL_MM = 1000.0;
    private static final double DRIVE_TIME_SEC = 2.0;
    
    // Arm parameters (time and encoder positions)
    private static final int ARM_MOVE_POSITION = 500; // Example position (encoder ticks) for arm left and right
    private static final int ARM_SWING_POSITION = 2000; // Example position (encoder ticks) for arm swing
    private static final double ARM_POWER = 0.7;  // Motor power for arm movements

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

        intakeServo.setPower(0);  // Initially stop intake

        // Reset arm motor encoders to 0 for a known starting position
        resetArmEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // -------------------- START CONCURRENT ACTIONS --------------------
            driveForward();
            
            // Move the arm motors concurrently (armLeft, armRight, armSwing)
            moveArmToPositionConcurrently(ARM_MOVE_POSITION, ARM_SWING_POSITION);

            // Ensure all concurrent tasks have finished before intake starts
            sleep(3000);  // Adjust to the longest time for movement phases

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
            // Wait until the robot reaches the target distance (encoder-based movement)
            while (opModeIsActive() && (Math.abs(leftFront.getCurrentPosition()) < targetTicks && Math.abs(rightFront.getCurrentPosition()) < targetTicks)) {
                telemetry.addData("Driving", "Moving forward...");
                telemetry.update();
            }
            stopDriving();
        } else {
            setDrivePower(0.5);
            sleep((long) (DRIVE_TIME_SEC * 1000));
            stopDriving();
        }
    }

    private void moveArmToPositionConcurrently(int armPosition, int swingPosition) {
        // Set target positions for all motors (armLeft, armRight, armSwing)
        armLeft.setTargetPosition(armPosition);
        armRight.setTargetPosition(armPosition);
        armSwing.setTargetPosition(swingPosition);

        // Set all motors to RUN_TO_POSITION mode
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to all motors
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
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetArmEncoders() {
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to RUN_USING_ENCODER so that the encoders are actively used during operations
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
