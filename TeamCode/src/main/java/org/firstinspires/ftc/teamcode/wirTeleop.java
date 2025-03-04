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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeLeOp", group="Robot")
//@Disabled
public class wirTeleop extends LinearOpMode {
    wirHardware robot = new wirHardware();
    ElapsedTime timer = new ElapsedTime();
     int tickPostion = 0;
     double elbow = .84;
    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();
        robot.init(hardwareMap);
        //  robot.leftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // robot.rightArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        timer.reset();
        robot.leftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //  timer.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //   telemetry.addData("TickPosition",  tickPostion);
             telemetry.addData("currentRight: ",  robot.rightArm.getCurrentPosition());
            telemetry.addData("currentLeft: ",  robot.leftArm.getCurrentPosition());
            telemetry.addData("tickposition: ", tickPostion);
            telemetry.addData("elbow: ", elbow);
            telemetry.update();
            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = Math.cbrt(gamepad1.left_stick_y);
            turn = Math.cbrt(-gamepad1.right_stick_x);

            // Combine drive and turn for blended motion.1
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }
            if (gamepad1.right_trigger > 0) {
                robot.leftFront.setPower(gamepad1.right_trigger);
                robot.rightFront.setPower(gamepad1.right_trigger);
                robot.leftBack.setPower(-gamepad1.right_trigger);
                robot.rightBack.setPower(-gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                robot.leftFront.setPower(-gamepad1.left_trigger);
                robot.rightFront.setPower(-gamepad1.left_trigger);
                robot.leftBack.setPower(gamepad1.left_trigger);
                robot.rightBack.setPower(gamepad1.left_trigger);
            } else {
                robot.leftFront.setPower(left);
                robot.rightFront.setPower(right);
                robot.leftBack.setPower(left);
                robot.rightBack.setPower(right);
            }
/*
            if (gamepad2.dpad_up) {
                robot.rightArm.setPower(1);
                robot.leftArm.setPower(1);
            } else if (gamepad2.dpad_down) {
                robot.rightArm.setPower(-1);
                robot.leftArm.setPower(-1);
            } else {
                robot.rightArm.setPower(0);
                robot.leftArm.setPower(0);
            }
*/
            robot.rightArm.setTargetPosition(tickPostion);
            robot.leftArm.setTargetPosition(tickPostion);
            robot.leftArm.setTargetPositionTolerance(20);
            robot.rightArm.setTargetPositionTolerance(20);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(1);
            robot.rightArm.setPower(1);

            //2900 high basket
            //1060 high specimen

            if (gamepad2.y) {
                tickPostion = 2900;
            }else if(gamepad2.a){
                tickPostion = 450;
            }else if (gamepad2.x) {
                tickPostion = 1400;
            }else if (gamepad2.left_bumper) {
                if (timer.milliseconds() > 50) {
                    tickPostion -= 50;
                    timer.reset();
                }
            }else if (gamepad2.right_bumper) {
                if (timer.milliseconds() > 50) {
                    tickPostion += 50;
                    timer.reset();
                }
            }
            if (gamepad2.right_trigger > 0) {
                robot.claw2.setPosition(1);
            } else if (gamepad2.left_trigger > 0) {
                robot.claw2.setPosition(0);
            }
            robot.elbow.setPosition(elbow);
            if (gamepad2.dpad_left && elbow<1) {
                if (timer.milliseconds() > 100) {
                    elbow += .01;
                    timer.reset();
                }
            } else if (gamepad2.dpad_right &&elbow >0.22) {
                if (timer.milliseconds() > 100) {
                    elbow -= .01;
                    timer.reset();
                }
            }





        }
    }
 }


//left in 1
//right in 0
