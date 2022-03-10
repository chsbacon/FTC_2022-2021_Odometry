package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(group = "drive")
public class testBlueWarehouse extends LinearOpMode {

    public static double initialX = 10;
    public static double initalY = 63;
    public static double initialAngle = 90;

    public static double shippingHubX = -5;
    public static double shippingHubY = 35;
    public static double shippingHubAngle = 60;

    public static double startReturnX = 5;
    public static double startReturnY = 59;
    public static double startReturnAngle = 0;
    public static double startReturnTanget = 45;

    public static double warehouseX = 40;
    public static double warehouseY = 59;
    public static double warehouseAngle = 0;
    public static double warehouseTangent = 0;

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
                .splineToLinearHeading(new Pose2d(startReturnX,startReturnY,Math.toRadians(startReturnAngle)),startReturnTanget)
                .build();

        Trajectory startReturnTOwarehouse = drive.trajectoryBuilder(shippingHubTOstartReturn.end())
                .lineToConstantHeading(new Vector2d(warehouseX,warehouseY))
                .build();

        Trajectory warehouseTOstartReturn = drive.trajectoryBuilder(startReturnTOwarehouse.end())
                .lineToConstantHeading(new Vector2d(startReturnX,startReturnY))
                .build();

        Trajectory startReturnTOshippingHub = drive.trajectoryBuilder(warehouseTOstartReturn.end())
                .lineToLinearHeading(new Pose2d(shippingHubX,shippingHubY,Math.toRadians(shippingHubAngle)))
                .build();



        drive.followTrajectory(startTOshippingHub);
        sleep(500);
        drive.followTrajectory(shippingHubTOstartReturn);
        drive.followTrajectory(startReturnTOwarehouse);
        sleep(500);


        drive.followTrajectory(warehouseTOstartReturn);
        drive.followTrajectory(startReturnTOshippingHub);
        sleep(500);
        drive.followTrajectory(shippingHubTOstartReturn);
        drive.followTrajectory(startReturnTOwarehouse);
        sleep(500);



        drive.followTrajectory(warehouseTOstartReturn);
        drive.followTrajectory(startReturnTOshippingHub);
        sleep(500);
        drive.followTrajectory(shippingHubTOstartReturn);
        drive.followTrajectory(startReturnTOwarehouse);



    }
}