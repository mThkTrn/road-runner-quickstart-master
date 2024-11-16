package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Simple Autonomous", group = "Simple")
public class BasicAutonomous extends OpMode {

    // Motors and servos for the robot
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor linearSlide;
    private Servo armSwing;
    private Servo intake;

    // Parameters for autonomous actions
    private double driveTime = 2.0; // Time to drive forward (in seconds)
    private double slidePosition = 500; // Target position for the linear slide (in encoder ticks)
    private double armSwingTime = 1.5; // Time to move the arm swing (in seconds)
    private double intakeDuration = 2.0; // Time to run the intake (in seconds)

    private double startTime = 0; // Start time for the autonomous actions
    private boolean isDriving = false;
    private boolean isLinearSlideMoving = false;
    private boolean isArmSwingMoving = false;
    private boolean isIntakeRunning = false;

    @Override
    public void init() {
        // Initialize motors and servos
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        armSwing = hardwareMap.get(Servo.class, "armSwing");
        intake = hardwareMap.get(Servo.class, "intake");

        // Set the motors to brake when stopped
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the servo positions
        armSwing.setPosition(0.5);  // Default position for the arm swing (adjust as needed)
        intake.setPosition(0);      // Intake off initially
    }

    @Override
    public void start() {
        startTime = getRuntime();
    }

    @Override
    public void loop() {
        double currentTime = getRuntime() - startTime;

        // Drive forward for the specified time
        if (!isDriving) {
            isDriving = true;
            leftFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(0.5);
        }

        if (currentTime > driveTime && isDriving) {
            // Stop the robot after the set time
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            isDriving = false;
        }

        // Move the linear slide to the specified position
        if (!isLinearSlideMoving && !isDriving) {
            isLinearSlideMoving = true;
            linearSlide.setTargetPosition((int) slidePosition);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.8);  // Set the speed of the linear slide
        }

        if (linearSlide.isBusy() && isLinearSlideMoving) {
            // Wait until the linear slide reaches the target position
            linearSlide.setPower(0.8);
        } else if (!linearSlide.isBusy() && isLinearSlideMoving) {
            // Once the slide reaches its position, stop
            linearSlide.setPower(0);
            isLinearSlideMoving = false;
        }

        // Move the arm swing for a set time
        if (!isArmSwingMoving && !isLinearSlideMoving && !isDriving) {
            isArmSwingMoving = true;
            armSwing.setPosition(1.0);  // Move arm swing to position (adjust as needed)
        }

        if (currentTime > driveTime + armSwingTime && isArmSwingMoving) {
            armSwing.setPosition(0.5); // Return arm swing to original position
            isArmSwingMoving = false;
        }

        // Run the intake for a specific duration
        if (!isIntakeRunning && !isArmSwingMoving) {
            isIntakeRunning = true;
            intake.setPosition(1.0);  // Turn intake on (adjust servo position as needed)
        }

        if (currentTime > driveTime + armSwingTime + intakeDuration && isIntakeRunning) {
            intake.setPosition(0);  // Turn intake off
            isIntakeRunning = false;
        }

        // Provide telemetry
        telemetry.addData("Drive Time", driveTime);
        telemetry.addData("Slide Position", slidePosition);
        telemetry.addData("Arm Swing Time", armSwingTime);
        telemetry.addData("Intake Duration", intakeDuration);
        telemetry.addData("Current Time", currentTime);
        telemetry.addData("Is Driving", isDriving);
        telemetry.addData("Is Linear Slide Moving", isLinearSlideMoving);
        telemetry.addData("Is Arm Swing Moving", isArmSwingMoving);
        telemetry.addData("Is Intake Running", isIntakeRunning);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all actions
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        linearSlide.setPower(0);
        armSwing.setPosition(0.5);  // Set arm swing back to neutral
        intake.setPosition(0);      // Turn intake off
    }
}
