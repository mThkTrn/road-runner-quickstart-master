package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo; // Import CRServo class
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@TeleOp(name = "Mecanum Drive OpMode", group = "TeleOp")
public class MecanumDriveBasic extends OpMode {

    private DcMotor armRight;
    private DcMotor armLeft;
    private CRServo intakeServo; // Declare the continuous rotation servo

    private MecanumDrive mecanumDrive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        HardwareMap hardwareMap = this.hardwareMap;

        // Initialize motors
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");

        // Initialize the continuous rotation servo
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Set motor direction if needed (optional)
        armLeft.setDirection(DcMotor.Direction.REVERSE); // Change as needed

        Pose2d initialPose = new Pose2d(0, 0, 0); // Create initial Pose2d
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose); // Initialize with Pose2d
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Chassis control with gamepad1
        double drive = -gamepad1.left_stick_x;  // Forward/backward
        double strafe = gamepad1.right_stick_x;  // Left/right
        double turn = gamepad1.left_stick_y;   // Rotation

        // Create a Vector2d for the drive and strafe values
        Vector2d velocity = new Vector2d(strafe, drive); // Note: strafe first, then drive
        PoseVelocity2d powers = new PoseVelocity2d(velocity, turn); // Use Vector2d for the first argument

        // Set drive powers to the mecanum drive
        mecanumDrive.setDrivePowers(powers);

        // Arm control with gamepad2
        double armLeftPower = -gamepad2.left_stick_y; // Up/down for armLeft
        double armRightPower = -gamepad2.right_stick_y; // Up/down for armRight

        // Set power for arms
        armLeft.setPower(armLeftPower);
        armRight.setPower(armRightPower);

        // Continuous rotation servo control with gamepad2 trigger
        if (gamepad2.left_trigger > 0.5) { // If left trigger is pressed
            intakeServo.setPower(1.0); // Rotate forward (change as needed)
        } else if (gamepad2.right_trigger > 0.5) { // If right trigger is pressed
            intakeServo.setPower(-1.0); // Rotate backward (change as needed)
        } else {
            intakeServo.setPower(0); // Stop the servo
        }

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
        telemetry.addData("Arm Left Power", armLeftPower);
        telemetry.addData("Arm Right Power", armRightPower);
        telemetry.addData("Intake Servo Power", intakeServo.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        armRight.setPower(0);
        armLeft.setPower(0);
        intakeServo.setPower(0); // Ensure the servo is stopped
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
