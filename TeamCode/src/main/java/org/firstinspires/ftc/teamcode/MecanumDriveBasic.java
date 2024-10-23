package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d; // Import Vector2d

@TeleOp(name = "Mecanum Drive OpMode", group = "TeleOp")
public class MecanumDriveBasic extends OpMode {
    private MecanumDrive mecanumDrive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        HardwareMap hardwareMap = this.hardwareMap;
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
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;  // Left/right
        double turn = gamepad1.right_stick_x;   // Rotation

        // Create a Vector2d for the drive and strafe values
        Vector2d velocity = new Vector2d(strafe, drive); // Note: strafe first, then drive
        PoseVelocity2d powers = new PoseVelocity2d(velocity, turn); // Use Vector2d for the first argument

        // Set drive powers to the mecanum drive
        mecanumDrive.setDrivePowers(powers);

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}