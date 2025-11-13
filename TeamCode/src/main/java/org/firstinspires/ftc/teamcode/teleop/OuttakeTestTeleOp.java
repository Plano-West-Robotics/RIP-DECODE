package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class OuttakeTestTeleOp extends BaseTeleOp
{
    public Outtake outtake;

    @Override
    public void setup()
    {
        outtake = new Outtake(hardware);
    }

    @Override
    public void run()
    {
        double velocity = Outtake.toAngularRate(Outtake.calculateIdealTangentialVelocity(1.524));
    }
}
