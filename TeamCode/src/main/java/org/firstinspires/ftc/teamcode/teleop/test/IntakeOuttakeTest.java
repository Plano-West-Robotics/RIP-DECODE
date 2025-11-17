package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

import java.util.List;

@TeleOp(group = "Test")
public class IntakeOuttakeTest extends BaseTeleOp
{
    public Intake intake;
    public Outtake outtake;

    @Override
    public void setup()
    {
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);

        subsystems.addAll(List.of(intake, outtake));
    }

    @Override
    public void run()
    {
        telemetry.addData("Outtake Control Mode", outtake.getMode());
    }
}
