package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@Config
@TeleOp(group = "Custom Tuning")
public class DashboardHoodedShooterTuner extends BaseTeleOp
{
    public static double P = Outtake.P;
    public static double I = Outtake.I;
    public static double D = Outtake.D;
    public static double F = Outtake.F;
    public static double targetAngularRate = 0;

    public Outtake outtake;
    public Intake intake;

    @Override
    public void setup()
    {
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
    }

    @Override
    public void run()
    {
          outtake.updatePIDFCoefficients(P, I, D, F);
          outtake.setVelocity(targetAngularRate);

          intake.forwardLaunch();

          telemetry.addData("Target Rate", targetAngularRate);
          telemetry.addData("Outtake Motor Avg. Angular Velocity (ticks/sec)", outtake.getAverageVelocity());
    }
}
