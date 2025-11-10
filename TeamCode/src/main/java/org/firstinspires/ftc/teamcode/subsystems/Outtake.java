package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.control.Analog;
import org.firstinspires.ftc.teamcode.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.base.MotorWrapper;

public class Outtake
{
    public static final double TICKS_PER_REV = 9.3333333333; // not been tested
    public static final double TICKS_PER_RADIAN = TICKS_PER_REV / (2 * Math.PI);
    public static final double HALF_GRAVITY = 4.9;
    public static final double LAUNCH_ANGLE = Math.PI / 3;
    public static final double Y_DELTA = 0.6345428; // goal height - outtake height (meters)
    public static final double FLYWHEEL_RADIUS = 0.0508; // meters

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
     * @param dx robot's distance from the goal (meters)
     * @return ideal tangential speed of the outtake flywheel (meters per second)
     */
    public static double calculateIdealTangentialVelocity(double dx)
    {
        return Math.sqrt(
            (-HALF_GRAVITY * Math.pow(dx, 2))
            /
            (Math.pow(Math.cos(LAUNCH_ANGLE), 2) * Y_DELTA - Math.sin(LAUNCH_ANGLE) * Math.cos(LAUNCH_ANGLE) * dx)
        );
    }

    /**
     * Calculates the equivalent angular rate given a tangential velocity and the flywheels radius.
     * Return value is meant to be passed to {@code DcMotorEx}'s {@code setVelocity()}
     * @param velocity the tangential velocity (meters per second)
     * @return the angular rate (ticks per second)
     */
    public static double toAngularRate(double velocity)
    {
        double angularVelocity = velocity / FLYWHEEL_RADIUS; // radians per second
        return angularVelocity * TICKS_PER_RADIAN;
    }
}
