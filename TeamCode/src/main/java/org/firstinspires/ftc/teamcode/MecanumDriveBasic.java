package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Drive OpMode", group = "TeleOp")
public class MecanumDriveBasic extends OpMode {
    private MecanumDrive mecanumDrive;
    private ElapsedTime runtime = new ElapsedTime();

    private CRServo intakeServo;
    private DcMotor armLeft;
    private DcMotor armRight;
    private DcMotor armSwing;

    private boolean armLeftHasEncoder = false;
    private boolean armRightHasEncoder = false;
    private boolean armSwingHasEncoder = false;

    private static final int SWING_MIN_POSITION = -1000000000;
    private static final int SWING_MAX_POSITION =  1000000000;

    // Target position for the armSwing
    private int armSwingTargetPosition = 0;

    // Arm swing preset positions
    private int ARM_SWING_FLOOR = -3168;
    private int ARM_SWING_CEIL = -2112;
    private int ARM_SWING_STORAGE_POSITION = 0;
    private int ARM_SWING_EXIT_ENTRY_POSITION = -2768;

    // Linear slide presets
    private int ARM_RIGHT_LIN_SLIDE_BUCKET = 2585;
    private int ARM_LEFT_LIN_SLIDE_BUCKET = 2585;

    private boolean LINEAR_SLIDE_TARGET_REACHED = false;

    @Override
    public void init() {
        HardwareMap hardwareMap = this.hardwareMap;
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        armLeft    = hardwareMap.get(DcMotor.class, "armLeft");
        armRight   = hardwareMap.get(DcMotor.class, "armRight");
        armSwing   = hardwareMap.get(DcMotor.class, "armSwing");

        // Initialize arm motors and attempt to check for encoder functionality
        checkEncoderConnection(armLeft, "armLeft");
        checkEncoderConnection(armRight, "armRight");
        checkEncoderConnection(armSwing, "armSwing");

        telemetry.update();
    }

    private void checkEncoderConnection(DcMotor motor, String motorName) {
        try {
            // Reverse the right motor if needed
            armRight.setDirection(DcMotor.Direction.REVERSE);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int initialPosition = motor.getCurrentPosition();

            // Set power briefly to check if encoder changes
            motor.setPower(0.1);
            sleep(200); // Small delay to allow for movement
            motor.setPower(0);

            int newPosition = motor.getCurrentPosition();
            if (initialPosition != newPosition) {
                telemetry.addData(motorName, "Encoder detected");
                switch (motorName) {
                    case "armLeft":
                        armLeftHasEncoder = true;
                        break;
                    case "armRight":
                        armRightHasEncoder = true;
                        break;
                    case "armSwing":
                        armSwingHasEncoder = true;
                        break;
                }
            } else {
                telemetry.addData(motorName, "No encoder detected");
            }
        } catch (Exception e) {
            telemetry.addData(motorName, "Error: Encoder not detected");
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Reset to basic power mode
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // ----------------------
        // Drive (gamepad1)
        // ----------------------
        double drive  =  gamepad1.left_stick_x;    // X-axis
        double strafe = -gamepad1.right_stick_x;   // X-axis (negated)
        double turn   = -gamepad1.left_stick_y;    // Y-axis (negated)

        Vector2d velocity = new Vector2d(strafe, drive);
        PoseVelocity2d powers = new PoseVelocity2d(velocity, turn);
        mecanumDrive.setDrivePowers(powers);

        // ----------------------
        // Intake (gamepad2 triggers)
        // ----------------------
        if (gamepad2.left_trigger > 0.1) {
            intakeServo.setPower(1.0);
        } else if (gamepad2.right_trigger > 0.1) {
            intakeServo.setPower(-1.0);
        } else {
            intakeServo.setPower(0.0);
        }

        // ----------------------
        // Linear slides (armLeft, armRight)
        // Use right joystick (gamepad2.right_stick_y) + right bumper
        // ----------------------
        if (armLeftHasEncoder && armRightHasEncoder) {
            if (gamepad2.right_bumper) {
                // Move to "bucket" position
                armLeft.setTargetPosition(ARM_LEFT_LIN_SLIDE_BUCKET);
                armRight.setTargetPosition(ARM_RIGHT_LIN_SLIDE_BUCKET);

                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armLeft.setPower(0.5);
                armRight.setPower(0.5);
            } else {
                // Manual via joystick
                int deltaPosition = (int) (-gamepad2.right_stick_y * 100);

                armLeft.setTargetPosition(armLeft.getCurrentPosition() + deltaPosition);
                armRight.setTargetPosition(armRight.getCurrentPosition() + deltaPosition);

                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armLeft.setPower(0.5);
                armRight.setPower(0.5);
            }
        } else {
            // If no encoders, just set power directly
            double armPower = gamepad2.right_stick_y;
            armLeft.setPower(armPower);
            armRight.setPower(armPower);
        }

        // ----------------------
        // Arm swing (armSwing) with presets
        // gamepad2.left_stick_y for manual adjustments
        // gamepad2.b / y / x / a for presets
        // ----------------------

        // --- CHANGES START HERE ---
        // 1) Remove the "shortest direction" logic.
        // 2) Set the arm target directly to "armSwingTargetPosition".
        // 3) Use RUN_TO_POSITION with a simple power approach.

        if (armSwingHasEncoder) {
            // Check if user pressed any preset button
            if (gamepad2.b) {
                armSwingTargetPosition = ARM_SWING_FLOOR;           // -3168
            } else if (gamepad2.y) {
                armSwingTargetPosition = ARM_SWING_CEIL;            // -2112
            } else if (gamepad2.x) {
                armSwingTargetPosition = ARM_SWING_STORAGE_POSITION; // 0
            } else if (gamepad2.a) {
                armSwingTargetPosition = ARM_SWING_EXIT_ENTRY_POSITION; // -2768
            }

            // Add manual increments from left stick
            double armSwingInput = -gamepad2.left_stick_y;
            int positionIncrement = (int) (armSwingInput * 10);
            armSwingTargetPosition += positionIncrement;

            // Clip it to extremes (safety)
            armSwingTargetPosition = Math.max(SWING_MIN_POSITION,
                    Math.min(SWING_MAX_POSITION, armSwingTargetPosition));

            // Set the target position directly (no more ±180° logic)
            armSwing.setTargetPosition(armSwingTargetPosition);

            // Pick a power. You can do fancy logic or keep it simple.
            // We'll do a mild approach: if we're more than 1000 ticks away,
            // use full power; else slow a bit to 0.7
            int distanceToTarget = Math.abs(armSwing.getCurrentPosition() - armSwingTargetPosition);
            double motorPower;
            if (distanceToTarget > 1000) {
                motorPower = 1.0;
            } else {
                motorPower = 0.7;
            }

            armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSwing.setPower(motorPower);
        } else {
            // If no encoder, just manual
            double armSwingInput = -gamepad2.left_stick_y;
            armSwing.setPower(armSwingInput);
        }
        // --- CHANGES END HERE ---

        // ----------------------
        // Telemetry
        // ----------------------
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Arm Left Encoder", armLeftHasEncoder);
        telemetry.addData("Arm Right Encoder", armRightHasEncoder);
        telemetry.addData("Arm Swing Encoder", armSwingHasEncoder);

        if (armSwingHasEncoder) {
            telemetry.addData("Arm Swing Target Pos", armSwing.getTargetPosition());
            telemetry.addData("Arm Swing Current Pos", armSwing.getCurrentPosition());
        } else {
            telemetry.addData("Arm Swing Target Pos", "N/A");
            telemetry.addData("Arm Swing Current Pos", "N/A");
        }

        if (armLeftHasEncoder) {
            telemetry.addData("Arm Left Target Pos", armLeft.getTargetPosition());
            telemetry.addData("Arm Left Current Pos", armLeft.getCurrentPosition());
        } else {
            telemetry.addData("Arm Left Target Pos", "N/A");
            telemetry.addData("Arm Left Current Pos", "N/A");
        }

        if (armRightHasEncoder) {
            telemetry.addData("Arm Right Target Pos", armRight.getTargetPosition());
            telemetry.addData("Arm Right Current Pos", armRight.getCurrentPosition());
        } else {
            telemetry.addData("Arm Right Target Pos", "N/A");
            telemetry.addData("Arm Right Current Pos", "N/A");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        intakeServo.setPower(0.0);
        armLeft.setPower(0);
        armRight.setPower(0);
        armSwing.setPower(0);

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}