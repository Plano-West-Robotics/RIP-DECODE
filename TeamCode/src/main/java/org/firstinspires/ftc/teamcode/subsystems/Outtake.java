package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.control.Analog;
import org.firstinspires.ftc.teamcode.control.Button;
import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class Outtake
{
    public static final double POWER = 0.55;
    public static final double TRIGGER_THRESHOLD = 0.5;

    public MotorWrapper motor;
    public boolean isSpinning;

    public Outtake(Hardware hardware)
    {
        motor = hardware.outtakeMotor;
        isSpinning = true;
    }

    public void update(Gamepads gamepads)
    {
        isSpinning = gamepads.exceedsThreshold(Analog.GP1_RIGHT_TRIGGER, TRIGGER_THRESHOLD);

        motor.setPower(isSpinning ? POWER : 0);
    }
}
