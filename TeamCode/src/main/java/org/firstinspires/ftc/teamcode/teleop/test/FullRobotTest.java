package org.firstinspires.ftc.teamcode.teleop.test;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

public class FullRobotTest extends BaseTeleOp
{
    public AbstractDrive drive;
    public Intake intake;
    public Outtake outtake;
    public OuttakeState outtakeState;

    public enum OuttakeState
    {
        INIT,
        ACCELERATING,
        AT_SPEED,
        LAUNCHING
    }

    @Override
    public void setup() {
        outtakeState = OuttakeState.INIT;
        drive = new FieldCentricDrive(hardware);
        intake = new Intake(hardware);
        outtake = new Outtake(hardware);
    }

    @Override
    public void run()
    {
        telemetry.addData("Outtake Mode", outtake.getMode());

        switch (outtakeState)
        {
            case INIT:
                if (gamepads.exceedsThreshold(Analog.GP2_RIGHT_TRIGGER, outtake.TRIGGER_THRESHOLD))
                {
                    outtake.motor.set
                }
        }

    }
}
