package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name="strafeStraight Testing", group="Linear Opmode")
public class strafeStraight extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap2022 robot = new HardwareMap2022();

    public void runOpMode(){

        robot.init(hardwareMap);

        waitForStart();




        while(opModeIsActive()){

            if(gamepad1.a){
                strafeLeft(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 5000);
            }


        }

    }


    void strafeLeft(double pwr, Orientation target, double desiredTime) {
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while((((runtime.milliseconds() < lastTime + desiredTime) && (opModeIsActive())))){

            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
                r = -.07;
            }

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = pwr + r ;
            backLeft = -pwr + r ;
            backRight = -pwr + r ;
            frontRight = pwr + r ;

            //original
            // +    +
            // -    +
            // -    +
            // +    +
            //strafe right
            // -    +
            // +    +
            // +    +
            // -    +

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }

            telemetry.addData("front left", "%.2f", frontLeft);
            telemetry.addData("front right", "%.2f", frontRight);
            telemetry.addData("back left", "%.2f", backLeft);
            telemetry.addData("back right", "%.2f", backRight);
            telemetry.addData("error", error);
            telemetry.addData("currOrient", currOrient);
            telemetry.addData("r",r);
            telemetry.addData("targetOrient", targetOrient);

            telemetry.update();
            //send the power to the motors
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
            robot.frontRightMotor.setPower(frontRight);
        }
        robot.stopDriving();
    }


}
