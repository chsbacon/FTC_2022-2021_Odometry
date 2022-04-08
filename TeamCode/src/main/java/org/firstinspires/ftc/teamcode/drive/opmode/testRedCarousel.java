package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ContourPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(group = "drive")
public class testRedCarousel extends LinearOpMode {


    public static double initialX = -36;
    public static double initalY = -63;
    public static double initialAngle = -90;

    public static double shippingHubX = -25;
    public static double shippingHubY = -26;
    public static double shippingHubAngle = -155;

    public static double wallX = -66;
    public static double wallY = -40;
    public static double wallAngle = -90;
    public static double wallTangent = -179;

    public static double carouselX = -66;
    public static double carouselY = -55.5;
    public static double carouselAngle = -90;
    public static double carouselTanget = 0;

    public static double parkX = -66;
    public static double parkY = -36;
    public static double parkAngle = -90;

    public  static int dumpSleep = 500;
    public static double carouselApproachVelMultiplier = .004;



    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 1920; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution
    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    double CrLowerUpdate = 40;
    double CbLowerUpdate = 160;
    double CrUpperUpdate = 255;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;

    double placeHeight = 0;
    double placeHeightAdjust = 0;

    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline());
        // Configuration of Pipeline
        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });


        int TOPHEIGHT = -3200; //4750 for 40
        int MIDHEIGHT = -1850; //2500 for 40
        int LOWHEIGHT = -650;
        int LMtargetPosition = 0;
        while (!isStarted())
        {

            if(myPipeline.error){
                //telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            testing(myPipeline);

            // Watch our YouTube Tutorial for the better explanation

            //telemetry.addData("RectMidpoint: ", myPipeline.getRectMidpointX());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 1300){
                    placeHeight = 3;
                    telemetry.addData("placeHeight: ", placeHeight);
                    LMtargetPosition = TOPHEIGHT;
                    telemetry.update();

                }
                else if(myPipeline.getRectMidpointX() > 600){
                    placeHeight = 2;
                    telemetry.addData("placeHeight: ", placeHeight);
                    telemetry.update();
                    LMtargetPosition = MIDHEIGHT;

                }
                else {
                    placeHeight = 1;
                    telemetry.addData("placeHeight: ", placeHeight);
                    telemetry.update();
                    LMtargetPosition = LOWHEIGHT;

                }
            }
        }

        waitForStart();

        webcam.stopRecordingPipeline();
        webcam.stopStreaming();
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(initialX, initalY, Math.toRadians(initialAngle));
        int LM_Height = LMtargetPosition;
        drive.setPoseEstimate(startPose);


        Trajectory startTOshippingHub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(shippingHubX,shippingHubY,Math.toRadians(shippingHubAngle)))
                .addTemporalMarker(.1, () -> {
                    drive.intakeServo1.setPosition(1); //ground
                    drive.intakeServo2.setPosition(0); //ground
                })
                .addTemporalMarker(.1, () -> {
                    drive.liftMotor.setTargetPosition(LM_Height);
                    drive.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.liftMotor.setPower(.9);
                })
                .addTemporalMarker(3, () -> {
                    drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.liftMotor.setPower(0);
                })
                .build();

        Trajectory shippingHubTOWall = drive.trajectoryBuilder(startTOshippingHub.end())
                .splineToLinearHeading(new Pose2d(wallX,wallY,Math.toRadians(wallAngle)),wallTangent)
                .addTemporalMarker(.1, () -> {
                    drive.intakeServo1.setPosition(.6); //vertical
                    drive.intakeServo2.setPosition(.4); //vertical
                })
                .addTemporalMarker(.1, () -> {
                    drive.liftMotor.setTargetPosition(0);
                    drive.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.liftMotor.setPower(.9);
                })
                .addTemporalMarker(3, () -> {
                    drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.liftMotor.setPower(0);
                })
                .build();


        Trajectory WallTOCarousel = drive.trajectoryBuilder(shippingHubTOWall.end())
                .lineToLinearHeading(new Pose2d(carouselX,carouselY,Math.toRadians(carouselAngle)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * carouselApproachVelMultiplier, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(.1, () -> {
                    drive.intakeServo1.setPosition(.45); //vertical
                    drive.intakeServo2.setPosition(.55); //vertical
                })
                .build();

        Trajectory carouselTOPark = drive.trajectoryBuilder(WallTOCarousel.end())
                .lineToLinearHeading(new Pose2d(parkX,parkY,Math.toRadians(parkAngle)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(.5, () -> {
                    drive.intakeServo1.setPosition(.6); //vertical
                    drive.intakeServo2.setPosition(.4); //vertical
                })
                .build();



        double DS_RecPos = 1;
        double DS_DumpPos = .6;

        drive.followTrajectory(startTOshippingHub);
        drive.dropServo.setPosition(DS_DumpPos);
        sleep(dumpSleep);
        drive.dropServo.setPosition(DS_RecPos);
        drive.followTrajectory(shippingHubTOWall);

        drive.followTrajectory(WallTOCarousel);

        drive.carouselMotor.setPower(.2);
        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.milliseconds() < 4000) && opModeIsActive()){

        }
        drive.carouselMotor.setPower(0);

        drive.followTrajectory(carouselTOPark);






    }


    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.ConfigureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.ConfigureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        //telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        //telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        //telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        //telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }



}


/*
        Trajectory startTOshippingHub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(shippingHubX,shippingHubY,Math.toRadians(shippingHubAngle)))
                .addTemporalMarker(.1, () -> {
                    int MIDHEIGHT = -2500/4;
                    drive.liftMotor.setTargetPosition(MIDHEIGHT);
                    drive.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.liftMotor.setPower(.2);
                })
                .addTemporalMarker(1, () -> {
                    drive.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.liftMotor.setPower(0);
                })
                .build();
  */