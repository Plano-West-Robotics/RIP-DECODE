package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class Intake
{
    public static final double LAUNCH_POWER = 1;
    public static final double REGULAR_POWER = 0.5;

    public MotorWrapper motor;
    public double motorPower = REGULAR_POWER;
    public boolean isSpinning;

    public Intake(Hardware hardware)
    {
        motor = hardware.intakeMotor;
        isSpinning = false;
    }

    public void forwardRegular()
    {
        motorPower = REGULAR_POWER;
        isSpinning = true;
        updateMotor();
    }

    public void forwardLaunch()
    {
        motorPower = LAUNCH_POWER;
        isSpinning = true;
        updateMotor();
    }

    public void reverseRegular()
    {
        motorPower = -REGULAR_POWER;
        isSpinning = true;
        updateMotor();
    }

    public void reverseLaunch()
    {
        motorPower = -LAUNCH_POWER;
        isSpinning = true;
        updateMotor();
    }

    public void start()
    {
        isSpinning = true;
        updateMotor();
    }

    public void stop()
    {
        isSpinning = false;
        updateMotor();
    }

    public void updateMotor()
    {
        motor.setPower(isSpinning ? motorPower : 0);
    }

    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP2_A)) isSpinning = !isSpinning;

        if (gamepads.justPressed(Button.GP2_B))
        {
            motorPower *= -1;
        }

        if (gamepads.justPressed(Button.GP2_X))
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

        updateMotor();
    }
}
