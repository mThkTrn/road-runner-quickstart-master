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

    private static final int SWING_MIN_POSITION = 0;
    private static final int SWING_MAX_POSITION = 1000;
    private int armSwingTargetPosition = 0;

    @Override
    public void init() {
        HardwareMap hardwareMap = this.hardwareMap;
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armSwing = hardwareMap.get(DcMotor.class, "armSwing");

        // Initialize arm motors and attempt to check for encoder functionality
        checkEncoderConnection(armLeft, "armLeft");
        checkEncoderConnection(armRight, "armRight");
        checkEncoderConnection(armSwing, "armSwing");

        // Reset the encoders to zero at the start
        resetEncoders();

        telemetry.update();
    }

    private void resetEncoders() {
        // Reset encoders for motors that use encoders (armSwing in this case)
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void checkEncoderConnection(DcMotor motor, String motorName) {
        try {
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
                    case "armLeft": armLeftHasEncoder = true; break;
                    case "armRight": armRightHasEncoder = true; break;
                    case "armSwing": armSwingHasEncoder = true; break;
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
        // Drive control using gamepad 1
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        Vector2d velocity = new Vector2d(strafe, drive);
        PoseVelocity2d powers = new PoseVelocity2d(velocity, turn);

        mecanumDrive.setDrivePowers(powers);

        // Intake control using gamepad 2 triggers
        if (gamepad2.left_trigger > 0.1) {
            intakeServo.setPower(1.0);
        } else if (gamepad2.right_trigger > 0.1) {
            intakeServo.setPower(-1.0);
        } else {
            intakeServo.setPower(0.0);
        }

        // Arm control using gamepad 2 right joystick
        double armPower = gamepad2.right_stick_y;
        if (armLeftHasEncoder && armRightHasEncoder) {
            armLeft.setTargetPosition(armLeft.getCurrentPosition() + (int)(armPower * 100));
            armRight.setTargetPosition(armRight.getCurrentPosition() + (int)(armPower * 100));
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLeft.setPower(0.5);
            armRight.setPower(0.5);
        } else {
            armLeft.setPower(armPower);
            armRight.setPower(armPower);
        }

        // Arm swing control with position targeting if encoder detected
        double armSwingInput = -gamepad2.left_stick_y;
        int positionIncrement = (int)(armSwingInput * 10);

        // Clamp the target position to prevent it from going negative
        armSwingTargetPosition += positionIncrement;
        armSwingTargetPosition = Math.max(SWING_MIN_POSITION, Math.min(SWING_MAX_POSITION, armSwingTargetPosition));

        if (armSwingHasEncoder) {
            armSwing.setTargetPosition(armSwingTargetPosition);
            armSwing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSwing.setPower(0.5);
        } else {
            armSwing.setPower(armSwingInput);
        }

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Arm Left Encoder", armLeftHasEncoder);
        telemetry.addData("Arm Right Encoder", armRightHasEncoder);
        telemetry.addData("Arm Swing Encoder", armSwingHasEncoder);
        telemetry.addData("Arm Swing Target Position", armSwingTargetPosition);
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
