/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeLeOp.
 *
 */

public class wirHardware {


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx leftFront;
     public DcMotorEx leftBack;
    public DcMotorEx rightFront;
     public DcMotorEx rightBack;
     public DcMotorEx leftArm;
     public DcMotorEx rightArm;
     public DcMotorSimple claw1;
     public Servo claw2;
    public DcMotorEx pivot1;
    public DcMotorSimple elbow;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    HardwareMap hwMap           =  null;
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        hwMap =ahwMap;
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront  = hwMap.get(DcMotorEx.class, "leftFront");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        leftArm   = hwMap.get(DcMotorEx.class, "leftArm");
        leftBack  = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        rightArm   = hwMap.get(DcMotorEx.class, "rightArm");
        claw1 = hwMap.get(DcMotorSimple.class, "claw1");
        claw2 = hwMap.get(Servo.class, "claw2");
        pivot1 = hwMap.get(DcMotorEx.class, "pivot1");
        elbow = hwMap.get(DcMotorSimple.class, "elbow");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftArm.setDirection(DcMotorEx.Direction.FORWARD);
        rightArm.setDirection(DcMotorEx.Direction.REVERSE);

        claw1.setDirection(DcMotorSimple.Direction.FORWARD);

        pivot1.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pivot1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftArm.setPower(0);
        rightArm.setPower(0);

        claw1.setPower(0);
        claw2.setPosition(0);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot1.setPower(0);

        pivot1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
/*
        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);
*/
    
    }


}
