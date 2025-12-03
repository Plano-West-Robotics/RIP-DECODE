package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@TeleOp(group = "Test")
public class CombinedIntakeOuttakeTest extends BaseTeleOp
{
    public Intake intake;
    public Outtake outtake;

    @Override
    public void setup()
    {
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
    }

    @Override
    public void run()
    {
        intake.update(gamepads);
        outtake.update(gamepads);
        telemetry.addData("Outtake Control Mode", outtake.getMode());
    }
}
