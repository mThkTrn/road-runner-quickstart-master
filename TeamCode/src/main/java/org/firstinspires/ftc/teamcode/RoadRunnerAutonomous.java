//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Trajectory;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "RoadRunner Autonomous (No Trajectory Sequence)", group = "RoadRunner")
//public class RoadRunnerAutonomous extends LinearOpMode {
//
//    // Parameters for distances and angles
//    private static final double FORWARD_DISTANCE = 1.0; // in meters
//    private static final double TURN_ANGLE = Math.toRadians(90); // in radians
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize Road Runner drive
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        // Define the start pose
//        Pose2d startPose = new Pose2d(0, 0, 0);
//        drive.setPoseEstimate(startPose);
//
//        // Create the trajectory to move forward 1 meter
//        Trajectory forwardTrajectory = drive.trajectoryBuilder(startPose)
//                .forward(FORWARD_DISTANCE)
//                .build();
//
//        // Create the trajectory to turn 90 degrees
//        Trajectory turnTrajectory = drive.trajectoryBuilder(forwardTrajectory.end())
//                .turn(TURN_ANGLE)
//                .build();
//
//        // Create the trajectory to move forward again 1 meter
//        Trajectory forwardAgainTrajectory = drive.trajectoryBuilder(turnTrajectory.end())
//                .forward(FORWARD_DISTANCE)
//                .build();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        // Follow the trajectories
//        drive.followTrajectory(forwardTrajectory);
//        drive.followTrajectory(turnTrajectory);
//        drive.followTrajectory(forwardAgainTrajectory);
//
//        // Telemetry to indicate completion
//        telemetry.addData("Status", "Trajectory Complete");
//        telemetry.update();
//    }
//}
