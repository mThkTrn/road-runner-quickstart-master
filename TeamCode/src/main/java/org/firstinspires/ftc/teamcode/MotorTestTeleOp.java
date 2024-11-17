package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Mecanum Drive Test", group = "Test")
public class MecanumDriveTest extends OpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Set motor direction if needed (adjust for your robot configuration)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake mode when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Test front left motor
        leftFront.setPower(1.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
        sleep(2000);  // Run front left for 2 seconds

        // Test front right motor
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(1.0);
        rightRear.setPower(0.0);
        sleep(2000);  // Run front right for 2 seconds

        // Test back right motor
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(1.0);
        sleep(2000);  // Run back right for 2 seconds

        // Test back left motor
        leftFront.setPower(0.0);
        leftRear.setPower(1.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
        sleep(2000);  // Run back left for 2 seconds

        // Stop all motors after the test
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Telemetry output
        telemetry.addData("Status", "Test Completed");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    // Sleep helper function
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
