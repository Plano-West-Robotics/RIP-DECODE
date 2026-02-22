package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.core.templates.StageServo;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorPairWrapper;
import org.firstinspires.ftc.teamcode.core.wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.core.wrappers.ServoWrapper;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.RightStopper;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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

    public static final double SLOW_ANGULAR_RATE = 800;
    public static final double FAST_ANGULAR_RATE = 1560;
    public static final double MANUAL_ANGULAR_RATE = 1130;
    public static final double TRIGGER_THRESHOLD = 0.5;
    public static final double NORMAL_ERROR_TOLERANCE = 30; // ticks/sec
    public static final double FAR_ERROR_TOLERANCE = 160; // ticks/sec
    public static final double MANUAL_ANGULAR_RATE_FAR = 1300;

    public static final double FLYWHEEL_RADIUS = 0.0508; // meters
    public static final double HALF_GRAVITY = 4.903325; // meters per second squared
    public static final double LAUNCH_ANGLE = Math.PI / 3; // radians
    public static final double DELTA_Y = 0.8636; // meters; final height - initial height
    public static final double EXTRA_DISTANCE = 0.1905; // the distance from the april tag to the center of the goal from a bird's-eye' view

    public static final double F = 12.5;
    public static final double P = 60;
    public static final double I = 0.1;
    public static final double D = 0;

    public static final double LOWEST_WEBCAM_RANGE = 0.9; // meters
    public static final double MEDIUM_WEBCAM_RANGE = 1.8;
    public static final double HIGH_LOW_WEBCAM_RANGE = 2.35;
    public static final double HIGH_HIGH_WEBCAM_RANGE = 2.7;
    public static final double MAX_WEBCAM_RANGE = 3.2;

    public static final double IDEAL_VOLTAGE = 13.5;

    /**
     * Scales the theoretically required velocity to account for inefficient energy transfer. This
     * is tested empirically.
     */
    public static final double VELOCITY_MULTIPLIER = 2.47;


    public static final double OPEN = 0.28;
    public static final double CLOSED = 0;

    public static final double HOOD_UP = 0;
    public static final double HOOD_DOWN = 1;

    public static final double UP = 0.0;
    public static final double DOWN = 1.0;

    public MotorWrapper top, bot;
    public MotorPairWrapper motors;
    public ControlMode mode;
    public VoltageSensor vs;

    public StageServo<RightStopper.Stage> servo;

    public ServoWrapper hood;

    public Outtake(Hardware hardware)
    {
        servo = hardware.rightStopper;

        vs = hardware.vs;
        double batteryVoltage = vs.getVoltage();

        motors = hardware.outtakeMotors;

        hood = hardware.hood;

        ((DcMotorEx) motors.getLeft().motor).setVelocityPIDFCoefficients(
                P,
                I,
                D,
                F * (IDEAL_VOLTAGE / batteryVoltage)
        );

        ((DcMotorEx) motors.getRight().motor).setVelocityPIDFCoefficients(
            P,
            I,
            D,
            F * (IDEAL_VOLTAGE / batteryVoltage)
        );

        webcamMode();
    }

    public void stopperClose() { servo.setPosition(CLOSED); }
    public void stopperOpen() { servo.setPosition(OPEN); }

    public void hoodUp() { hood.setPosition(HOOD_UP); }
    public void hoodDown() { hood.setPosition(HOOD_DOWN); }


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

    public double getLeftMotorVelocity()
    {
        return ((DcMotorEx) motors.getLeft().motor).getVelocity();
    }

    public double getRightMotorVelocity()
    {
        return ((DcMotorEx) motors.getRight().motor).getVelocity();
    }

    public double getAverageVelocity()
    {
        return (((DcMotorEx) motors.getLeft().motor).getVelocity() + ((DcMotorEx) motors.getRight().motor).getVelocity()) / 2.0;
    }

    public void setVelocity(double angularRate)
    {
        ((DcMotorEx) motors.getLeft().motor).setVelocity(angularRate);
        ((DcMotorEx) motors.getRight().motor).setVelocity(angularRate);
    }

    public void updatePIDFCoefficients(double Kp, double Ki, double Kd, double Kf)
    {
        ((DcMotorEx) motors.getLeft().motor).setVelocityPIDFCoefficients(
                Kp,
                Ki,
                Kd,
                Kf * (IDEAL_VOLTAGE / vs.getVoltage())
        );

        ((DcMotorEx) motors.getRight().motor).setVelocityPIDFCoefficients(
                Kp,
                Ki,
                Kd,
                Kf * (IDEAL_VOLTAGE / vs.getVoltage())
        );
    }

    public void stop()
    {
        setVelocity(0);
    }

    public void update(Gamepads gamepads)
    {
        if (gamepads.justPressed(Button.GP1_B)) toggleMode();

        double batteryVoltage = vs.getVoltage();

        ((DcMotorEx) motors.getLeft().motor).setVelocityPIDFCoefficients(
            P,
            I,
            D,
            F //* (IDEAL_VOLTAGE / batteryVoltage));
        );


        ((DcMotorEx) motors.getRight().motor).setVelocityPIDFCoefficients(
            P,
            I,
            D,
            F //* (IDEAL_VOLTAGE / batteryVoltage));
        );


    }

    /*public void update(Gamepads gamepads, AprilTagWebcam webcam, AprilTagDetection detection)
    {

    }*/

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
     * Regression graph: <a href="https://www.desmos.com/calculator/8vmhai9gcg">...</a>
     * @param dx robot's distance from the AprilTag (meters)
     * @return angular velocity of the outtake flywheel (ticks per second)
     */
    public static double piecewise1CalculateFlywheelTangentialVelocityExperimental(double dx)
    {
        /*
        if (dx >= HIGH_WEBCAM_RANGE)
            return;
        if (dx >= MEDIUM_WEBCAM_RANGE)
            return;
        if (dx >= LOWEST_WEBCAM_RANGE)
            return;
        return 0;
        */

        if (dx < LOWEST_WEBCAM_RANGE)
            return SLOW_ANGULAR_RATE;
        if (dx > MEDIUM_WEBCAM_RANGE)
            return piecewise2CalculateFlywheelTangentialVelocityExperimental(dx);

        dx = dx + 0.1;

        double quadraticTerm = 128.2753 * dx * dx;
        double linearTerm = -163.94281 * dx;
        double yIntercept = 1256.42264;

        return quadraticTerm + linearTerm + yIntercept;
    }

    public static double piecewise2CalculateFlywheelTangentialVelocityExperimental(double dx)
    {
        if (dx <= MEDIUM_WEBCAM_RANGE)
        {
            return piecewise1CalculateFlywheelTangentialVelocityExperimental(dx);
        }
        if (dx > HIGH_LOW_WEBCAM_RANGE)
            return piecewise3CalculateFlywheelTangentialVelocityExperimental(dx);

        double quarticTerm = -13057.3498 * dx * dx * dx * dx;
        double cubicTerm = 107624.992 * dx * dx * dx;
        double quadraticTerm = -331394.085 * dx * dx;
        double linearTerm = 452007.531 * dx;
        double yIntercept = -229134.707;

        return quarticTerm + cubicTerm + quadraticTerm + linearTerm + yIntercept;
    }


    public static double piecewise3CalculateFlywheelTangentialVelocityExperimental(double dx)
    {
        if (dx <= HIGH_LOW_WEBCAM_RANGE)
        {
            return piecewise2CalculateFlywheelTangentialVelocityExperimental(dx);
        }
        if (dx > HIGH_HIGH_WEBCAM_RANGE)
            return piecewise4CalculateFlywheelTangentialVelocityExperimental(dx);

        /*
        double quarticTerm = -13057.3498 * dx * dx * dx * dx;
        double cubicTerm = 107624.992 * dx * dx * dx;
        double quadraticTerm = -331394.085 * dx * dx;
        double linearTerm = 452007.531 * dx;
        double yIntercept = -229134.707;
        */
        return 1500;
    }

    public static double piecewise4CalculateFlywheelTangentialVelocityExperimental(double dx)
    {
        if (dx <= HIGH_HIGH_WEBCAM_RANGE)
        {
            return piecewise3CalculateFlywheelTangentialVelocityExperimental(dx);
        }
        if (dx > MAX_WEBCAM_RANGE)
            return FAST_ANGULAR_RATE;

        double quadraticTerm = -16.38633 * dx * dx;
        double linearTerm = 227.99804 * dx;
        double yIntercept = 1004.53359;

        return quadraticTerm + linearTerm + yIntercept;
    }

    public static int getRange(double dx)
    {
        if (dx >= MAX_WEBCAM_RANGE)
            return 0;
        if (dx >= HIGH_LOW_WEBCAM_RANGE)
            return 3;
        if (dx >= MEDIUM_WEBCAM_RANGE)
            return 2;
        if (dx >= LOWEST_WEBCAM_RANGE)
            return 1;
        return 0;
    }





    public static boolean inHoodRange(double dx)
    {
        return dx >= MEDIUM_WEBCAM_RANGE;
    }
/*
    public static String whichFunction(double dx)
    {
        if (dx < LOWEST_WEBCAM_RANGE)
            return "None";
    }
*/
    public static double calculateCloseRangeLinearFlywheelTangentialVelocity(double dx)
    {
        double slope = 235.93961;
        double yInt = 948.94082;

        return slope * dx + yInt;
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
