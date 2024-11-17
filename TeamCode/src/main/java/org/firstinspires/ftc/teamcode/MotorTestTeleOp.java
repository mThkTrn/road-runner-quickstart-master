package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Motor Test TeleOp", group = "Test")
public class MotorTestTeleOp extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions if necessary (optional)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);  // If needed
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);  // If needed

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Test the motors sequentially with gamepad buttons

        // If the A button is pressed, run the front left motor
        if (gamepad1.a) {
            frontLeft.setPower(1.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
            telemetry.addData("Motor Test", "Running Front Left");
        } 
        // If the B button is pressed, run the front right motor
        else if (gamepad1.b) {
            frontLeft.setPower(0.0);
            frontRight.setPower(1.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
            telemetry.addData("Motor Test", "Running Front Right");
        } 
        // If the X button is pressed, run the back right motor
        else if (gamepad1.x) {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(1.0);
            telemetry.addData("Motor Test", "Running Back Right");
        } 
        // If the Y button is pressed, run the back left motor
        else if (gamepad1.y) {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(1.0);
            backRight.setPower(0.0);
            telemetry.addData("Motor Test", "Running Back Left");
        } 
        // If no button is pressed, stop all motors
        else {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
            telemetry.addData("Motor Test", "Stopped");
        }

        // Update telemetry to show the status
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors at the end of the teleop period
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}
