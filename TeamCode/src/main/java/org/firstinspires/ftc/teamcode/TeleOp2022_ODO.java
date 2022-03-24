/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;


@TeleOp(name="TeleOp 2022", group="Linear Opmode")
//@Disabled
public class TeleOp2022_ODO extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    

    private AndroidSoundPool androidSoundPool;


    public DcMotor  frontLeftMotor   = null;
    public DcMotor  frontRightMotor = null;
    public DcMotor  backLeftMotor = null;
    public DcMotor  backRightMotor = null;

    public DcMotor liftMotor = null;
    public DcMotor spintakeMotor = null;
    public DcMotor carouselMotor = null;



    public Servo dropServo = null;
    public Servo intakeServo1 = null;
    public Servo intakeServo2 = null;










    @Override
    public void runOpMode() {

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "leftFront"); //H1P0
        frontRightMotor  = hardwareMap.get(DcMotor.class, "rightFront"); //H1P1
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear"); //H1P2
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear"); //H1P3

        liftMotor = hardwareMap.get(DcMotor.class,"LM");
        spintakeMotor = hardwareMap.get(DcMotor.class,"SM"); //H2P0
        carouselMotor = hardwareMap.get(DcMotor.class,"CML");



        dropServo = hardwareMap.get(Servo.class,"DS");
        intakeServo1 = hardwareMap.get(Servo.class,"IS1");
        intakeServo2 = hardwareMap.get(Servo.class,"IS2");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spintakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropServo.setPosition(1);
        intakeServo1.setPosition(0);
        intakeServo2.setPosition(0);





        androidSoundPool = new AndroidSoundPool();


        double x;
        double y;
        double r;
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double fastSlow = 1;

        double spintakeMotorState = 0;

        boolean carouselDirection = true;
        double carouselPower = 0;

        boolean liftMotorMovingDown;
        boolean liftMotorMovingUp;

        float intakeServoPos = 0;

        RevBlinkinLedDriver blinkinLedDriver;
        RevBlinkinLedDriver.BlinkinPattern pattern;

        //boolean spintakeMotorState = true;
        int liftMotorTicksTele = 0;
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)


        // Choosing the team color
        telemetry.addData("Gamepad1: Press X for Blue, B for Red", "");
        telemetry.update();


        while (!gamepad1.x && !gamepad1.b) {

        }

        if(gamepad1.x){
            carouselPower = -.25;
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            //blinkinLedDriver.setPattern(pattern);
        }
        if(gamepad1.b){
            carouselPower = .25;
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            //blinkinLedDriver.setPattern(pattern);
        }

        androidSoundPool.initialize(SoundPlayer.getInstance());

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

    
            //GAMEPAD 1 ___________________________________________________________________________
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            r = gamepad1.right_stick_x;
            // do not let rotation dominate movement
            r = r / 2;
            // calculate the power for each wheel
            frontLeft = +y - x + r;
            backLeft = +y + x + r;
            frontRight = -y - x + r;
            backRight = -y + x + r;
            /*
            frontLeftMotor.setPower(frontLeft/fastSlow);
            frontRightMotor.setPower(frontRight/fastSlow);
            backLeftMotor.setPower(backLeft/fastSlow);
            backRightMotor.setPower(backRight/fastSlow);
            */

// glide up and down 3
            if(gamepad1.left_bumper) {
                liftMotor.setPower(1);
                liftMotorMovingDown = false;
                liftMotorMovingUp = true;
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                //blinkinLedDriver.setPattern(pattern);
            }
            else if (gamepad1.right_bumper) {
                liftMotor.setPower(-1);
                liftMotorMovingDown = true;
                liftMotorMovingUp = false;
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                //blinkinLedDriver.setPattern(pattern);
                telemetry.addData("Ticks: ", liftMotor.getCurrentPosition());
                telemetry.update();
            }
            else{
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotorMovingDown = false;
                liftMotorMovingUp = false;
            }

            if((liftMotorMovingDown == true) && liftMotor.getCurrentPosition() < 0){
                liftMotor.setPower(0);
            }
            if((liftMotorMovingUp == true) && Math.abs(liftMotor.getCurrentPosition()) > 4500/2){
                liftMotor.setPower(0);
            }





            if(gamepad1.y){
                dropServo.setPosition(0);
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                //blinkinLedDriver.setPattern(pattern);
            }
            else{
                dropServo.setPosition(1);
            }

/*
            if(gamepad1.b){
                capServo.setPosition(-1);
                telemetry.addData("ServoPos: ", capServo.getPosition());
                telemetry.update();
            }
            else if(gamepad1.a){
                capServo.setPosition(0);
                telemetry.addData("ServoPos: ", capServo.getPosition());
                telemetry.update();
            }
            else if(gamepad1.x){
                capServo.setPosition(1);
                telemetry.addData("ServoPos: ", capServo.getPosition());
                telemetry.update();
            }
            else{
            }
*/



            //GAMEPAD 2 ___________________________________________________________________________
            //intake servo


            if(gamepad2.left_bumper){
                spintakeMotor.setPower(0);
                intakeServo1.setPosition(-.5);
                intakeServo2.setPosition(.5);
            }
            else if (gamepad2.right_bumper){
                intakeServo1.setPosition(1);
                intakeServo2.setPosition(-1);
                spintakeMotor.setPower(0);
            }
            else{
                
            }





            if(gamepad2.y){
                //carouselMotor.setPower(-.25);
                carouselMotor.setPower(carouselPower);

                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE;
                //blinkinLedDriver.setPattern(pattern);



            }
            else{
                carouselMotor.setPower(0);
            }


            //spitake out items
            if(gamepad2.dpad_down){
                spintakeMotor.setPower(.75);
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                //blinkinLedDriver.setPattern(pattern);
                while(gamepad2.dpad_down){

                }
                spintakeMotor.setPower(0);
            }

            //spitake in items
            if(gamepad2.dpad_left){
                spintakeMotor.setPower(-.75);
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                //blinkinLedDriver.setPattern(pattern);
                while(gamepad2.dpad_down){

                }
                spintakeMotor.setPower(0);
            }

            //spintake items in toggle
            if(gamepad2.dpad_up && spintakeMotorState == 0){
                    spintakeMotorState = 1;
                    spintakeMotor.setPower(-.75);
                }
            else if(gamepad2.dpad_up && spintakeMotorState == 1){
                    spintakeMotor.setPower(0);
                    spintakeMotorState = 0;

                }
            else{

            }

            telemetry.addData("Ticks: ", liftMotor.getCurrentPosition());
            telemetry.update();

            }




        }






    //just formatting stuff for the angles -- this was copied and pasted
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    //just formatting stuff for the degrees -- this was copied and pasted
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



}

