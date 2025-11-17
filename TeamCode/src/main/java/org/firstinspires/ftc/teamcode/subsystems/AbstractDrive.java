package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.hardware.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public abstract class AbstractDrive implements Subsystem
{
    public DrivetrainMotors drivetrainMotors;

    public double speed, turnSpeed;
    public boolean slowMode;

    public static class Params
    {
        public static double regularSpeed = 0.85;
        public static double regularTurnSpeed = 0.7;

        public static double slowSpeed = 0.38;
        public static double slowTurnSpeed = 0.55;
    }

    public AbstractDrive(Hardware hardware)
    {
        drivetrainMotors = hardware.drivetrainMotors;
        regularSpeed();
    }

    @Override
    public void update(Gamepads gamepads)
    {
        double drive = gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_Y);
        double strafe = gamepads.getAnalogValue(Analog.GP1_LEFT_STICK_X);
        double turn = gamepads.getAnalogValue(Analog.GP1_RIGHT_STICK_X);
        drive(drive, strafe, turn);
    }

    /**
     * Given the desired {@code drive}, {@code strafe}, and {@code turn}, directs the appropriate
     * amount of power to each drivetrain motor.
     * <p>
     * Should NOT be called in an OpMode's {@code loop()} method! Use {@code update()} instead.
     */
    public abstract void drive(double x, double y, double rx);

    public void toggleSlowMode()
    {
        if (slowMode) regularSpeed();
        else slowSpeed();
    }

    public void regularSpeed()
    {
        speed = Params.regularSpeed;
        turnSpeed = Params.regularTurnSpeed;
        slowMode = false;
    }

    public void slowSpeed()
    {
        speed = Params.slowSpeed;
        turnSpeed = Params.slowTurnSpeed;
        slowMode = true;
    }
}
