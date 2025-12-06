package org.firstinspires.ftc.teamcode.teleop.tune;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@Config
@TeleOp(group = "Tune")
public class DashboardOuttakeVelocityPIDFTuner extends BaseTeleOp
{
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
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
        ((DcMotorEx) outtake.motor.motor).setVelocityPIDFCoefficients(P, I, D, F);
        ((DcMotorEx) outtake.motor.motor).setVelocity(targetAngularRate);

        telemetry.addData("Outtake Motor Angular Velocity (ticks/sec)", ((DcMotorEx) outtake.motor.motor).getVelocity());
        telemetry.addData("Outtake Motor Angular Velocity (rev/min)", ((DcMotorEx) outtake.motor.motor).getVelocity() * 60 * (1 / Outtake.TICKS_PER_REVOLUTION));
    }
}
