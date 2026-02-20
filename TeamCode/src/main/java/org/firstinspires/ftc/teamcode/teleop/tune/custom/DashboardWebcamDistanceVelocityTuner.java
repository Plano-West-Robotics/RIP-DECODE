package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group="Custom Tuning")
public class DashboardWebcamDistanceVelocityTuner extends BaseTeleOp
{
    public static double targetAngularRate;
    public double pastVel;
    public static boolean launch;
    public boolean pastLaunch;
    public static boolean hoodAdjustment;

    public static boolean useCalculated;

    public static double marginOfErrorTPS;
    public static double marginOfErrorExitTPS;
    public static double setpointChangeResetTPS;
    public static double readyTimeMs;
    public double farRangeExitMOE;
    public static boolean farRange; //TODO: at vineet's figure out what far range actually is
    public static boolean requireDetectionToLaunch;

    public boolean withinMOE = false;

    public AprilTagWebcam webcam;
    public Intake intake;
    public Outtake outtake;
    public ElapsedTime t;

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
        t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        marginOfErrorTPS = 50;
        marginOfErrorExitTPS = 75;
        setpointChangeResetTPS = 10;
        readyTimeMs = 750;
        requireDetectionToLaunch = false;
    }

    @Override
    public void run()
    {

        AprilTagDetection detection = webcam.getEitherGoalDetection();

        if (detection == null)
        {
            telemetry.addData("Goal ID Is Detected", false);
            telemetry.addLine();
        }
        else
        {
            telemetry.addData("Goal ID Is Detected", true);
            telemetry.addLine();

            webcam.updateRange(detection.ftcPose.range);
            telemetry.addData("Range: ", webcam.getRange());
            telemetry.addLine();
        }

        double calculatedVel = Outtake.piecewiseCalculateFlywheelTangentialVelocityExperimental(webcam.getRange());
        double setpoint = useCalculated ? calculatedVel : targetAngularRate;
        outtake.setVelocity(setpoint);

        telemetry.addData("Left Motor Velocity", outtake.getLeftMotorVelocity());
        telemetry.addData("Right Motor Velocity", outtake.getRightMotorVelocity());
        telemetry.addData("Average Motor Velocity", outtake.getAverageVelocity());
        telemetry.addData("Setpoint", setpoint);
        telemetry.addData("Worst Case Velocity", Math.min(outtake.getLeftMotorVelocity(), outtake.getRightMotorVelocity()));

        double worstCase = Math.min(outtake.getLeftMotorVelocity(), outtake.getRightMotorVelocity());
        double error = Math.abs(worstCase - setpoint);

        withinMOE = error < marginOfErrorExitTPS;
        /*if (withinMOE)
        {
            withinMOE = error < marginOfErrorExitTPS;
        }
        else
        {
            withinMOE = error < marginOfErrorTPS;
        }*/

        telemetry.addData("Error", error);
        telemetry.addData("Within MOE", withinMOE);
        telemetry.addData("Timer (ms)", t.time());

        if (onEnter(pastLaunch, launch) || Math.abs(setpoint - pastVel) > setpointChangeResetTPS)
        {
            t.reset();
        }

        if (launch)
        {
            if (hoodAdjustment)
            {
                if (farRange)
                {
                    farRangeExitMOE = 0.15 * targetAngularRate;
                    if (error < farRangeExitMOE)
                    {
                        outtake.hoodUp();
                    }
                    else
                    {
                        outtake.hoodDown();
                    }
                }
                else if (error < marginOfErrorTPS)
                    outtake.hoodUp();
                else
                    outtake.hoodDown();
            }
            else
            {
                outtake.hoodUp();
            }

            if (requireDetectionToLaunch && useCalculated && detection == null)
            {
                t.reset();
                intake.stop();
            }
            else if (!withinMOE)
            {
                t.reset();
                intake.stop();
            }

            else
            {
                if (t.time() >= readyTimeMs)
                {
                    intake.forwardLaunch();
                }
                else
                {
                    intake.stop();
                }
            }
        }
        else
        {
            intake.stop();
        }

        pastLaunch = launch;
        pastVel = setpoint;
    }

    public boolean onEnter(boolean pastCondition, boolean presentCondition)
    {
        return !pastCondition && presentCondition;
    }

    public boolean onExit(boolean pastCondition, boolean presentCondition)
    {
        return pastCondition && !presentCondition;
    }
}
