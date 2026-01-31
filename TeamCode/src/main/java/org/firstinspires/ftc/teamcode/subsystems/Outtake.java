package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.core.wrappers.ServoPairWrapper;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class Outtake
{
    public enum ControlMode
    {
        MANUAL_CONTROL,
        WEBCAM_CONTROL
    }

    public static final double TICKS_PER_REVOLUTION = 28;
    public static final double TICKS_PER_RADIAN = TICKS_PER_REVOLUTION / (2 * Math.PI);
    public static final double MAX_ANGULAR_RATE = 2800; // ticks/sec; This is determined from OuttakeMaxVelocityTest

    public static final double MANUAL_ANGULAR_RATE = 0.375 * MAX_ANGULAR_RATE;
    public static final double TRIGGER_THRESHOLD = 0.5;
    public static final double ANGULAR_RATE_ERROR_TOLERANCE = 120; // ticks/sec

    public static final double FLYWHEEL_RADIUS = 0.0508; // meters
    public static final double HALF_GRAVITY = 4.903325; // meters per second squared
    public static final double LAUNCH_ANGLE = Math.PI / 3; // radians
    public static final double DELTA_Y = 0.8636; // meters; final height - initial height
    public static final double EXTRA_DISTANCE = 0.1905; // the distance from the april tag to the center of the goal from a bird's-eye' view

//    Values from tuning guide from 2019
    public static final double F = 14.5;
    public static final double P = 17;
    public static final double I = 0;
    public static final double D = 2;

    public static final double IDEAL_VOLTAGE = 13.5;

    /**
     * Scales the theoretically required velocity to account for inefficient energy transfer. This
     * is tested empirically.
     */
    public static final double VELOCITY_MULTIPLIER = 2.47;

    //TODO: arbitrary
    public static final double UP = 0;
    public static final double DOWN = 0;

    public MotorWrapper motor;
    public ControlMode mode;
    public VoltageSensor vs;

    public ServoPairWrapper servos;

    public Outtake(Hardware hardware)
    {
        servos = hardware.stoppers;
        motor = hardware.outtakeMotor;
        motor.reverse();
        motor.useEncoder();

        vs = hardware.vs;
        double batteryVoltage = vs.getVoltage();

        ((DcMotorEx) motor.motor).setVelocityPIDFCoefficients(
                P,
                I,
                D,
                F * (IDEAL_VOLTAGE / batteryVoltage));
        manualMode();
    }

    public void stoppersUp() { servos.setPosition(UP); }
    public void stoppersDown() { servos.setPosition(DOWN); }
    public ControlMode getMode()
    {
        return mode;
    }

    public void manualMode()
    {
        mode = ControlMode.MANUAL_CONTROL;
    }

    public void webcamMode()
    {
        mode = ControlMode.WEBCAM_CONTROL;
    }

    public void toggleMode()
    {
        if (mode == ControlMode.MANUAL_CONTROL) webcamMode();
        else manualMode();
    }



    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP1_B)) toggleMode();

        double batteryVoltage = vs.getVoltage();

        ((DcMotorEx) motor.motor).setVelocityPIDFCoefficients(
            P,
            I,
            D,
            F); //* (IDEAL_VOLTAGE / batteryVoltage));


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
