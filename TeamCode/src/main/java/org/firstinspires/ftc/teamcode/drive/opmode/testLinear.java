package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(group = "drive")
public class testLinear extends LinearOpMode {
    public static double shippingHubX = 10;
    public static double shippingHubY = 32;
    public static double shippingHubAngle1 = 45;

    public static double startReturnX = 10;
    public static double startReturnY = 59;
    public static double startReturnAngle = 0;

    public static double warehouseX = 45;
    public static double warehouseY = 59;
    public static double warehouseAngle = 0;


    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        waitForStart();
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(10, 63, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory startTOshippingHub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(shippingHubX,shippingHubY,Math.toRadians(shippingHubAngle1)))
                .build();
        drive.followTrajectory(startTOshippingHub);

        Trajectory shippingHubTOstartReturn = drive.trajectoryBuilder(startTOshippingHub.end())
                .lineToLinearHeading(new Pose2d(startReturnX,startReturnY,Math.toRadians(startReturnAngle)))
                .build();
        drive.followTrajectory(shippingHubTOstartReturn);

        Trajectory startReturnTOwarehouse = drive.trajectoryBuilder(shippingHubTOstartReturn.end())
                .lineToLinearHeading(new Pose2d(warehouseX,warehouseY,Math.toRadians(warehouseAngle)))
                .build();
        drive.followTrajectory(startReturnTOwarehouse);
    }
}