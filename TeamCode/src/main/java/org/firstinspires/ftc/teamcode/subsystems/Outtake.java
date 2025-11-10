package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.control.Analog;
import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class Outtake
{
    public static final double TICKS_PER_REV = 9.3333333333; // not been tested
    public static final double HALF_GRAVITY = 4.9;
    public static final double LAUNCH_ANGLE = Math.PI / 3;
    public static final double DELTA_Y = 0.6345428; // = height of goal - height of outtake

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

    /**
     * @param dx meters
     * @return
     */
    public double calculateIdealVelocity(double dx)
    {
        return Math.sqrt(
            (-HALF_GRAVITY * Math.pow(dx, 2))
            /
            (Math.pow(Math.cos(LAUNCH_ANGLE), 2) * DELTA_Y - Math.sin(LAUNCH_ANGLE) * Math.cos(LAUNCH_ANGLE) * dx)
        );
    }
}
