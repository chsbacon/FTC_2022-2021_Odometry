package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(group = "drive")
public class testAuto extends LinearOpMode {

    public static double initialX = 10;
    public static double initalY = 63;
    public static double initialAngle = -90;

    public static double shippingHubX = 10;
    public static double shippingHubY = 33;
    public static double shippingHubAngle = 45;

    public static double startReturnX = 15;
    public static double startReturnY = 59;
    public static double startReturnAngle = 0;

    public static double warehouseX = 48;
    public static double warehouseY = 59;
    public static double warehouseAngle = 0;


    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        waitForStart();
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(initialX, initalY, Math.toRadians(initialAngle));

        drive.setPoseEstimate(startPose);

        Trajectory startTOshippingHub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(shippingHubX,shippingHubY,Math.toRadians(shippingHubAngle)))
                .build();

        Trajectory shippingHubTOstartReturn = drive.trajectoryBuilder(startTOshippingHub.end())
                .lineToLinearHeading(new Pose2d(startReturnX,startReturnY,Math.toRadians(startReturnAngle)))
                .build();

        Trajectory startReturnTOwarehouse = drive.trajectoryBuilder(shippingHubTOstartReturn.end())
                .lineToLinearHeading(new Pose2d(warehouseX,warehouseY,Math.toRadians(warehouseAngle)))
                .build();




        Trajectory warehouseTOstartReturn = drive.trajectoryBuilder(startReturnTOwarehouse.end())
                .lineToLinearHeading(new Pose2d(startReturnX,startReturnY,Math.toRadians(startReturnAngle)))
                .build();

        Trajectory startReturnTOshippingHub = drive.trajectoryBuilder(warehouseTOstartReturn.end())
                .lineToLinearHeading(new Pose2d(shippingHubX,shippingHubY,Math.toRadians(shippingHubAngle)))
                .build();



        drive.followTrajectory(startTOshippingHub);
        drive.followTrajectory(shippingHubTOstartReturn);
        drive.followTrajectory(startReturnTOwarehouse);

        drive.followTrajectory(warehouseTOstartReturn);
        drive.followTrajectory(startReturnTOshippingHub);
        drive.followTrajectory(shippingHubTOstartReturn);
        drive.followTrajectory(startReturnTOwarehouse);






    }
}