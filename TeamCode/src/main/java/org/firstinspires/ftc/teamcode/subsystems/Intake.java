package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.core.wrappers.CRServoWrapper;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class Intake implements Subsystem
{
    public static final double LAUNCH_POWER = 1;
    public static final double REGULAR_POWER = 0.5;

    public MotorWrapper motor;
    public double motorPower = LAUNCH_POWER;
    public boolean isSpinning;

    public Intake(Hardware hardware)
    {
        motor = hardware.intakeMotor;
        isSpinning = false;
    }

    @Override
    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP1_A)) isSpinning = !isSpinning;
        if (gamepads.justPressed(Button.GP1_B))
        {
            motorPower *= -1;
        }
        if (gamepads.justPressed(Button.GP1_X))
        {
            if (Math.abs(motorPower) == LAUNCH_POWER)
            {
                motorPower = Math.signum(motorPower) * REGULAR_POWER;
            }
            else if (Math.abs(motorPower) == REGULAR_POWER)
            {
                motorPower = Math.signum(motorPower) * LAUNCH_POWER;
            }
        }

        if (gamepads.justPressed(Button.GP1_DPAD_UP)) motorPower += 0.05;
        if (gamepads.justPressed(Button.GP1_DPAD_DOWN)) motorPower -= 0.05;
        //if (motorPower < 0) motorPower = 0;
        //if (motorPower > 1) motorPower = 1;
        motor.setPower(isSpinning ? motorPower : 0);
    }
}
