package org.firstinspires.ftc.teamcode;

import static com.sun.tools.javac.jvm.ByteCodes.error;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class elevatorPID {

    public static double kA = 0;
    public static double kD = 0;
    public static double kG = 0.2;
    public static double kI = 0.0001;
    public static double kP = 0.006;
    public static double kS = 0.02;
    public static double kV = 0.006;
    public static double targetPosition = 0;


    private final PIDController pid = new PIDController(kP, kI, kD);
    private final ElevatorFeedforward eff = new ElevatorFeedforward(kS, kG, kV, kA);

    private final wirHardware robot;

    public elevatorPID(wirHardware robotInput) {
        robot = robotInput;
        // Reset encoders
        robot.leftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse one motor if needed
        robot.rightArm.setDirection(DcMotorEx.Direction.REVERSE);
        pid.setTolerance(15);
    }

    public void setTargetPosition(double ticks) {
        targetPosition = ticks;

    }

    public void update() {
        pid.setSetPoint(targetPosition);
        double currentPosL = robot.leftArm.getCurrentPosition();
        double currentPosR = robot.rightArm.getCurrentPosition();

        double pidOutL = pid.calculate(currentPosL);
        double pidOutR = pid.calculate(currentPosR);


        double ffOut = eff.calculate(0.0, 0.0);


        double outputL = pidOutL + ffOut;
        double outputR = pidOutR + ffOut;


        // Clamp to motor power range
        outputL = Math.max(-1.0, Math.min(1.0, outputL));
        outputR = Math.max(-1.0, Math.min(1.0, outputR));

        double error = targetPosition - getCurrentPos();

        if (Math.abs(error) <= 15) {
            robot.leftArm.setPower(kG);
            robot.rightArm.setPower(kG);
        } else {
            robot.leftArm.setPower(outputL);
            robot.rightArm.setPower(outputR);        }

    }

    public double getCurrentPos() {
        return (robot.leftArm.getCurrentPosition() + robot.rightArm.getCurrentPosition()) / 2.0;
    }
    public boolean atPosition() {
        return pid.atSetPoint();
    }

}
