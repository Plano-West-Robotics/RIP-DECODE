package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@Disabled
@Config
@TeleOp(group = "Custom Tuning")
public class DashboardOuttakeVelocityPIDFTuner extends BaseTeleOp
{
    public static double F = Outtake.F;
    public static double P = Outtake.P;
    public static double I = Outtake.I;
    public static double D = Outtake.D;
    public static double targetAngularRate = 0;

    public Outtake outtake;

    @Override
    public void setup()
    {
        outtake = new Outtake(hardware);
    }

    @Override
    public void run()
    {
//        ((DcMotorEx) outtake.motor.motor).setVelocityPIDFCoefficients(P, I, D, F);
//        ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);
//
//        telemetry.addData("Outtake Motor Angular Velocity (ticks/sec)", ((DcMotorEx) outtake.motor.motor).getVelocity());
//        telemetry.addData("Outtake Motor Angular Velocity (rev/min)", ((DcMotorEx) outtake.motor.motor).getVelocity() * 60 * (1 / Outtake.TICKS_PER_REVOLUTION));
    }
}
