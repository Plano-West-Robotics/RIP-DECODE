package org.firstinspires.ftc.teamcode.teleop.tune.custom;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "Custom Tuning")
public class DashboardWebcamBearingPIDFTuner extends BaseTeleOp
{
    public static double P = 0.05;
    public static double I = 0;
    public static double D = 0.0035;
    public static double F = 0;
    public static boolean useRedGoalId = true;
    public static boolean manualDrive = false;

    public AprilTagWebcam webcam;
    public FieldCentricDrive drive;
    public PIDFController controller;

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        drive = new FieldCentricDrive(hardware);
        controller = new PIDFController(P, I, D, F);
    }

    @Override
    public void run()
    {
        controller.setPIDF(P, I, D, F);

        webcam.goalId = useRedGoalId ? AprilTagWebcam.RED_GOAL_ID : AprilTagWebcam.BLUE_GOAL_ID;
        telemetry.addData("Goal Color", webcam.goalId == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");

        telemetry.addData("Mode", manualDrive ? "MANUAL" : "WEBCAM AUTO AIM");

        if (manualDrive)
        {
            drive.update(gamepads);
        }
        else
        {
            AprilTagDetection detection = webcam.getGoalDetection();

            if (detection == null)
            {
                drive.drive(0, 0, 0);
            }
            else
            {
                telemetry.addData("Range", detection.ftcPose.range);
                telemetry.addData("Bearing", detection.ftcPose.bearing);

                double rx = controller.calculate(detection.ftcPose.bearing, 0);
                drive.drive(0, 0, rx);

                telemetry.addData("RX", rx);
            }
        }
    }
}
