package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.PaddlesServoPair;

public class Outtake implements Subsystem
{
    public enum ControlMode
    {
        MANUAL_CONTROL,
        WEBCAM_CONTROL
    }

    public static final double POWER = 0.55;
    public static final double TRIGGER_THRESHOLD = 0.5;

    public static final double TICKS_PER_REVOLUTION = 28;
    public static final double TICKS_PER_RADIAN = TICKS_PER_REVOLUTION / (2 * Math.PI);
    public static final double FLYWHEEL_RADIUS = 0.0508; // meters

    public static final double HALF_GRAVITY = 4.903325; // meters per second squared
    public static final double LAUNCH_ANGLE = Math.PI / 3; // radians
    public static final double DELTA_Y = 0.4345428; // meters; final height - initial height
    public static final double EXTRA_DISTANCE = 0.1905; // the distance from the april tag to the center of the goal from a bird's-eye' view
    /**
     * Scales the theoretically required velocity to account for inefficient energy transfer. This
     * is tested empirically.
     */
    public static final double VELOCITY_MULTIPLIER = 2.7;

    public MotorWrapper motor;
    public PaddlesServoPair paddles;
    public ControlMode mode;

    public Outtake(Hardware hardware)
    {
        motor = hardware.outtakeMotor;
        paddles = hardware.paddles;
        mode = ControlMode.MANUAL_CONTROL;
    }

    @Override
    public void update(Gamepads gamepads)
    {
        boolean triggerActivated = gamepads.exceedsThreshold(Analog.GP2_RIGHT_TRIGGER, TRIGGER_THRESHOLD);

        if (mode == ControlMode.MANUAL_CONTROL)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(triggerActivated ? POWER : 0);
        }
        else
        {
            motor.useEncoder();
        }

        if (gamepads.justPressed(Button.GP2_B))
        {
            mode = mode == ControlMode.MANUAL_CONTROL ? ControlMode.WEBCAM_CONTROL : ControlMode.MANUAL_CONTROL;
        }

        if (gamepads.isPressed(Button.GP2_A))
        {
            paddles.ready();
        }
        else
        {
            paddles.standby();
        }
    }

    public ControlMode getMode()
    {
        return mode;
    }

    /**
     * @param dx robot's distance from the goal (meters)
     * @return ideal tangential speed of the outtake flywheel (meters per second)
     */
    public static double calculateIdealFlywheelTangentialVelocity(double dx)
    {
        dx += EXTRA_DISTANCE;
        double numerator = -HALF_GRAVITY * Math.pow(dx, 2);
        double denominator = Math.pow(Math.cos(LAUNCH_ANGLE), 2) * DELTA_Y - Math.sin(LAUNCH_ANGLE) * Math.cos(LAUNCH_ANGLE) * dx;
        return VELOCITY_MULTIPLIER * Math.sqrt(numerator / denominator);
    }

    /**
     * Converts a given flywheel tangential velocity to an angular rate.
     * @param velocity tangential velocity (meters per second)
     * @return angular rate (ticks per second); should be passed into {@code DcMotorEx}'s {@code setVelocity()}
     */
    public static double toAngularRate(double velocity)
    {
        double angularVelocity = velocity / FLYWHEEL_RADIUS; // radians per second
        return angularVelocity * TICKS_PER_RADIAN;
    }
}
