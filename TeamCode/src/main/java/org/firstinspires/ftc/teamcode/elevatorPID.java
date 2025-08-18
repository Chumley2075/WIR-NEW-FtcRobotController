package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wirHardware;
@Config

public class elevatorPID {

   private wirHardware robot = new wirHardware();
    private final PIDFController pidf;
    private final ElevatorFeedforward ff;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal, setpoint;
    // example PIDF + feedforward gains
    // --- Dashboard Tunables ---
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0005;
    public static double kF = 0.0;

    public static double kS = 0.0;   // static friction
    public static double kG = 0.2;   // gravity hold
    public static double kV = 0.02;  // velocity
    public static double kA = 0.0;   // acceleration (not really needed)
    private double lastVelocity = 0;  // track previous loop velocity
    private final double dt = 0.02;   // loop time ~20ms
    // Motion constraints (ticks/sec, ticks/sec^2)
    public static double MAX_VELOCITY = 2000;
    public static double MAX_ACCEL = 3000;
    public elevatorPID(HardwareMap hw) {
        robot.init(hw);
        robot.leftArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        robot.leftArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(kP, kI, kD, kF);
        ff = new ElevatorFeedforward(kS, kG, kV, kA);

        // Max velocity (ticks/sec), max accel (ticks/sec^2) — tune to your elevator
        constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);

        goal = new TrapezoidProfile.State(0, 0);
        setpoint = new TrapezoidProfile.State(0, 0);
    }

    public void goToPosition(int ticks) {
        goal = new TrapezoidProfile.State(ticks, 0);  // velocity = 0 at goal
    }
    public void update() {
        pidf.setPIDF(kP, kI, kD,kF);
        // average encoder position
        int currentPos = (robot.leftArm.getCurrentPosition() + robot.rightArm.getCurrentPosition()) / 2;

        // generate new setpoint
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        setpoint = profile.calculate(dt); // update setpoint

        // compute acceleration
        double acceleration = (setpoint.velocity - lastVelocity) / dt;
        lastVelocity = setpoint.velocity;

        // PID on position
        double feedback = pidf.calculate(currentPos, setpoint.position);

        // feedforward based on velocity + acceleration
        double feedforward = ff.calculate(setpoint.velocity, acceleration);

        double power = feedback + feedforward;

        robot.rightArm.setPower(power);
        robot.leftArm.setPower(power);
        telemetry.addData("PID output", feedback);
        telemetry.addData("Feedforward", feedforward);
        telemetry.update();
    }
    public double getPosition(){
        return (robot.rightArm.getCurrentPosition() + robot.leftArm.getCurrentPosition() )/2.0;
    }
}
