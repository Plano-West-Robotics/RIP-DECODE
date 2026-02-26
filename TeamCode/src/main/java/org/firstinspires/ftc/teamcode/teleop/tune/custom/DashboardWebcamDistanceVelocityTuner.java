package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
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

    public static boolean correctHeading;
    public boolean lastCorrectHeading;

    public static boolean hoodDown;

    public static boolean enableFlywheel;

    public static double marginOfErrorTPS = 70;
    public static double marginOfErrorExitTPS = 120;
    public static double setpointChangeResetTPS = 10;
    public static double readyTimeMs = 750;
    public double farRangeExitMOE;
    public static boolean farRange;

    public boolean withinMOE = false;

    public static boolean correctWebcam;

    public double rx = 0;

    public AprilTagWebcam webcam;
    public Intake intake;
    public Outtake outtake;
    public ElapsedTime t;
    public PIDFController bearingController;
    public FieldCentricDrive drive;

    @Override
    public void setup()
    {
        drive = new FieldCentricDrive(hardware);
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
        t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        bearingController = new PIDFController(
                DashboardWebcamBearingPIDFTuner.P,
                DashboardWebcamBearingPIDFTuner.I,
                DashboardWebcamBearingPIDFTuner.D,
                DashboardWebcamBearingPIDFTuner.F
        );
        outtake.hoodDown();
    }

    @Override
    public void run()
    {
        AprilTagDetection detection = webcam.getEitherGoalDetection();

        if (detection == null)
        {
            telemetry.addData("Goal ID Is Detected", false);
            telemetry.addLine();
            rx = 0;
        }
        else
        {
            telemetry.addData("Goal ID Is Detected", true);
            telemetry.addLine();

            if (correctWebcam)
            {
                webcam.updateRangeWithoutTolerance(detection.ftcPose.range);
            }
            else
            {
                webcam.updateRange(detection.ftcPose.range);
            }
            webcam.updateBearing(detection.ftcPose.bearing);
            rx = bearingController.calculate(webcam.getBearing(), 0);

            telemetry.addData("Range: ", webcam.getRange());
            telemetry.addLine();
        }

        if (correctHeading)
            drive.drive(0, 0, rx);
        else
            drive.drive(0, 0, 0);

        double calculatedVel = Outtake.piecewise1CalculateFlywheelTangentialVelocityExperimental(webcam.getRange());
        double setpoint = useCalculated ? calculatedVel : targetAngularRate;

        if (enableFlywheel)
            outtake.setVelocity(setpoint);
        else
            outtake.setVelocity(0);

        telemetry.addData("Left Motor Velocity", outtake.getLeftMotorVelocity());
        telemetry.addData("Right Motor Velocity", outtake.getRightMotorVelocity());
        telemetry.addData("Average Motor Velocity", outtake.getAverageVelocity());
        telemetry.addData("Setpoint", setpoint);
        telemetry.addData("Worst Case Velocity", Math.min(outtake.getLeftMotorVelocity(), outtake.getRightMotorVelocity()));

        double worstCase = Math.min(outtake.getLeftMotorVelocity(), outtake.getRightMotorVelocity());
        double error = Math.abs(worstCase - setpoint);

        if (farRange)
        {
            marginOfErrorExitTPS = 0.15 * setpoint;
            marginOfErrorTPS = 0.075 * setpoint;
        }

        if (withinMOE)
        {
            withinMOE = error < marginOfErrorExitTPS;
        }
        else
        {
            withinMOE = error < marginOfErrorTPS;
        }

        telemetry.addData("Error", error);
        telemetry.addData("Within MOE", withinMOE);
        telemetry.addData("Timer (ms)", t.time());

        if (onEnter(pastLaunch, launch) || Math.abs(setpoint - pastVel) > setpointChangeResetTPS)
        {
            t.reset();
        }

        if (launch)
        {
            if (hoodDown)
                outtake.hoodDown();
            else
            {
                if (hoodAdjustment)
                {
                    if (error < marginOfErrorTPS)
                        outtake.hoodUp();
                    else
                        outtake.hoodDown();
                }
                else
                    outtake.hoodUp();
            }

            if (!withinMOE)
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
        lastCorrectHeading = correctHeading;
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
