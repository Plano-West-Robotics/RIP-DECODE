package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.control.Button;
import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.base.CRServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class Intake implements Subsystem
{
    public MotorWrapper motor;
    public CRServoWrapper transfer;
    public double motorPower = 1;
    public double servoPower = 1;
    public boolean isSpinning;

    public Intake(Hardware hardware)
    {
        motor = hardware.intakeMotor;
        transfer = hardware.transfer;
        isSpinning = false;
    }

    @Override
    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP1_A)) isSpinning = !isSpinning;
        if (gamepads.justPressed(Button.GP1_B))
        {
            motorPower *= -1;
            servoPower *= -1;
        }
        motor.setPower(isSpinning ? motorPower : 0);
        transfer.setPower(isSpinning ? servoPower : 0);
    }
}
