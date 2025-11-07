package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.control.Button;
import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class Intake
{
    public double power = 0.5;

    public MotorWrapper motor;
    public boolean isSpinning;

    public Intake(Hardware hardware)
    {
        motor = hardware.intakeMotor;
        isSpinning = true;
    }

    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP1_A))
        {
            isSpinning = !isSpinning;
        }

        if (gamepads.justPressed(Button.GP1_B))
        {
            power *= -1;
        }

        motor.setPower(isSpinning ? power : 0);
    }
}
