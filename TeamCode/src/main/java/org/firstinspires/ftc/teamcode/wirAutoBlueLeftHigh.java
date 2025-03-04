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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="wirAutoBlueLeftHigh", group="Robot")
//@Disabled

public class wirAutoBlueLeftHigh extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime timer = new ElapsedTime();
    public wirHardwareAuto robot = new wirHardwareAuto();
    double high = .55;
    double low = .84;

    @Override
    public void runOpMode() {


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Ready to start", "");
        telemetry.update();
        robot.init(hardwareMap);
        robot.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int tickTolerance =40;
        int sleepTime =500;
        robot.rightFront.setTargetPositionTolerance(tickTolerance);
        robot.leftFront.setTargetPositionTolerance(tickTolerance);
        robot.rightBack.setTargetPositionTolerance(tickTolerance);
        robot.leftBack.setTargetPositionTolerance(tickTolerance);
        // Wait for the game to start (driver presses START)

        waitForStart();
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //3333.6 ticks = 6ft.
        //42 ticks per inch
        encoderDrive(0.5, -12);
        sleep(sleepTime);
        encoderTurn45();
        sleep(sleepTime);
        encoderStraferight(0.5, 12);
        sleep(sleepTime);
        Raiseelevator();
        robot.elbow.setPosition(0.6);
        sleep(4000);
        encoderDrive(0.5, 8);
        sleep(sleepTime);
        robot.elbow.setPosition(high);
        sleep(1000);
        robot.claw2.setPosition(0);
        sleep(1000);
        robot.elbow.setPosition(.61);
        sleep(sleepTime);
        encoderDrive(.5, -3);
        sleep(sleepTime);
        Elevatordown();
        sleep(sleepTime);
        encoderTurn135();
        sleep(sleepTime);
        encoderStraferight(0.5, 11);
        sleep(sleepTime);
        encoderDrive(0.5, 40);
        sleep(sleepTime);
        encoderTurn90();
        sleep(sleepTime);
        encoderDrive(0.5, 14);
        sleep(sleepTime);
        robot.elbow.setPosition(0.55);
        sleep(1500);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void Raiseelevator() {
        int tickPostion = 2900;
        robot.rightArm.setTargetPosition(tickPostion);
        robot.leftArm.setTargetPosition(tickPostion);
        robot.leftArm.setTargetPositionTolerance(10);
        robot.rightArm.setTargetPositionTolerance(10);
        robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftArm.setPower(1);
        robot.rightArm.setPower(1);
    }

    public void Elevatordown() {
        int tickPostion = 0;
        robot.rightArm.setTargetPosition(tickPostion);
        robot.leftArm.setTargetPosition(tickPostion);
        robot.leftArm.setTargetPositionTolerance(10);
        robot.rightArm.setTargetPositionTolerance(10);
        robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftArm.setPower(1);
        robot.rightArm.setPower(1);
    }


    public void encoderDrive(double speed, double inchTarget) {
        inchTarget *= 43;
        robot.rightBack.setTargetPosition((int) -inchTarget);
        robot.leftBack.setTargetPosition((int) -inchTarget);
        robot.rightFront.setTargetPosition((int) -inchTarget);
        robot.leftFront.setTargetPosition((int) -inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftFront.setPower(speed);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.addData("right B: ",robot.rightBack.getCurrentPosition());
            telemetry.addData("left F: ",robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderStrafeleft(double speed, double inchTarget) {
        inchTarget *= 48;
        robot.rightBack.setTargetPosition((int) inchTarget);
        robot.leftBack.setTargetPosition((int) inchTarget);
        robot.rightFront.setTargetPosition((int) -inchTarget);
        robot.leftFront.setTargetPosition((int) -inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftFront.setPower(speed);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderStraferight(double speed, double inchTarget) {
        inchTarget *= 48;
        robot.rightBack.setTargetPosition((int) -inchTarget);
        robot.leftBack.setTargetPosition((int) -inchTarget);
        robot.rightFront.setTargetPosition((int) inchTarget);
        robot.leftFront.setTargetPosition((int) inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftFront.setPower(speed);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderTurn180() {
        int inchTarget = 2200;
        robot.rightBack.setTargetPosition((int) -inchTarget);
        robot.leftBack.setTargetPosition((int) inchTarget);
        robot.rightFront.setTargetPosition((int) -inchTarget);
        robot.leftFront.setTargetPosition((int) inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(0.75);
        robot.leftBack.setPower(0.75);
        robot.rightFront.setPower(0.75);
        robot.leftFront.setPower(0.75);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.addData("right B: ",robot.rightBack.getCurrentPosition());
            telemetry.addData("left F: ",robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderTurn45() {
        int inchTarget = 550;
        robot.rightBack.setTargetPosition((int) inchTarget);
        robot.leftBack.setTargetPosition((int) -inchTarget);
        robot.rightFront.setTargetPosition((int) inchTarget);
        robot.leftFront.setTargetPosition((int) -inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(0.75);
        robot.leftBack.setPower(0.75);
        robot.rightFront.setPower(0.75);
        robot.leftFront.setPower(0.75);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.addData("right B: ",robot.rightBack.getCurrentPosition());
            telemetry.addData("left F: ",robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderTurn90() {
        int inchTarget = 1100;
        robot.rightBack.setTargetPosition((int) inchTarget);
        robot.leftBack.setTargetPosition((int) -inchTarget);
        robot.rightFront.setTargetPosition((int) inchTarget);
        robot.leftFront.setTargetPosition((int) -inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(0.75);
        robot.leftBack.setPower(0.75);
        robot.rightFront.setPower(0.75);
        robot.leftFront.setPower(0.75);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.addData("right B: ",robot.rightBack.getCurrentPosition());
            telemetry.addData("left F: ",robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderTurn135() {
        int inchTarget =1650 ;
        robot.rightBack.setTargetPosition((int) inchTarget);
        robot.leftBack.setTargetPosition((int) -inchTarget);
        robot.rightFront.setTargetPosition((int) inchTarget);
        robot.leftFront.setTargetPosition((int) -inchTarget);

        robot.rightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(0.75);
        robot.leftBack.setPower(0.75);
        robot.rightFront.setPower(0.75);
        robot.leftFront.setPower(0.75);

        while (robot.rightFront.isBusy()|| robot.leftBack.isBusy()){
            telemetry.addData("right F: ",robot.rightFront.getCurrentPosition());
            telemetry.addData("left B: ",robot.leftBack.getCurrentPosition());
            telemetry.addData("right B: ",robot.rightBack.getCurrentPosition());
            telemetry.addData("left F: ",robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
}




