package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Disabled
@TeleOp(group = "Custom Tuning")
public class OuttakeMotorPIDTunerWithoutHardware extends OpMode
{
    public static double P = 0.1;
    public static double I = 0;
    public static double D = 0.002;
    public static double targetAngularRate = 0;

    public DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "o");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry.update();
    }

    @Override
    public void loop() {
        motor.setVelocityPIDFCoefficients(P, I, D, 0);
        motor.setVelocity(targetAngularRate);

        telemetry.addData("Outtake Motor Angular Velocity (ticks/sec)", motor.getVelocity());
        telemetry.addData("Outtake Motor Angular Velocity (rev/min)", motor.getVelocity() * 60 * (1 / Outtake.TICKS_PER_REVOLUTION));
        telemetry.update();
    }
}

