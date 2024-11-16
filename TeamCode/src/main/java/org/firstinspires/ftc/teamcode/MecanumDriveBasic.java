//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d; // Import Vector2d
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(name = "Mecanum Drive OpMode", group = "TeleOp")
//public class MecanumDriveBasic extends OpMode {
//    private MecanumDrive mecanumDrive;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    // Declare hardware for intake and arm motors
//    private CRServo intakeServo;
//    private DcMotor armLeft;
//    private DcMotor armRight;
//    private DcMotor armSwing;
//
//    @Override
//    public void init() {
//        HardwareMap hardwareMap = this.hardwareMap;
//        Pose2d initialPose = new Pose2d(0, 0, 0); // Create initial Pose2d
//        mecanumDrive = new MecanumDrive(hardwareMap, initialPose); // Initialize with Pose2d
//
//        // Initialize hardware
//        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
//        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
//        armRight = hardwareMap.get(DcMotor.class, "armRight");
//        armSwing = hardwareMap.get(DcMotor.class, "armSwing");
//
//        // Set arm motors to run using zero power when no input
//        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armSwing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        runtime.reset();
//    }
//
//    @Override
//    public void loop() {
//        // Drive control using gamepad 1
//        double drive = gamepad1.left_stick_y;  // Forward/backward
//        double strafe = gamepad1.right_stick_x;  // Left/right
//        double turn = gamepad1.left_stick_x;   // Rotation
//
//        // Create a Vector2d for the drive and strafe values
//        Vector2d velocity = new Vector2d(strafe, drive); // Note: strafe first, then drive
//        PoseVelocity2d powers = new PoseVelocity2d(velocity, turn); // Use Vector2d for the first argument
//
//        // Set drive powers to the mecanum drive
//        mecanumDrive.setDrivePowers(powers);
//
//        // Intake control using gamepad 2 triggers
//        if (gamepad2.left_trigger > 0.1) {
//            intakeServo.setPower(1.0);  // Intake direction (full speed forward)
//        } else if (gamepad2.right_trigger > 0.1) {
//            intakeServo.setPower(-1.0);  // Outtake direction (full speed backward)
//        } else {
//            intakeServo.setPower(0.0);  // Stop
//        }
//        // Arm control using gamepad 2 right joystick
//        double armPower = gamepad2.right_stick_y;  // Inverted for correct direction (forward is positive)
//        armLeft.setPower(armPower);
//        armRight.setPower(-armPower);
//
//        // Arm swing control (keeping the arm swing stationary when the joystick is neutral)
//        double armSwingPower = gamepad2.left_stick_y;  // Control with left joystick
//        armSwing.setPower(armSwingPower);
//
//        // Telemetry
//        telemetry.addData("Status", "Running");
//        telemetry.addData("Drive Power", drive);
//        telemetry.addData("Strafe Power", strafe);
//        telemetry.addData("Turn Power", turn);
//        telemetry.addData("Arm Left Power", armLeft.getPower());
//        telemetry.addData("Arm Right Power", armRight.getPower());
//        telemetry.addData("Arm Swing Power", armSwing.getPower());
////        telemetry.addData("Intake Servo Position", intakeServo.getPosition());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        // Stop all motors and reset servo
//        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//        intakeServo.setPower(0); // Neutral position for intake servo
//        armLeft.setPower(0);
//        armRight.setPower(0);
//        armSwing.setPower(0);
//        telemetry.addData("Status", "Stopped");
//        telemetry.update();
//    }
//}


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

        telemetry.update();
    }

    private void checkEncoderConnection(DcMotor motor, String motorName) {
        try {
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
