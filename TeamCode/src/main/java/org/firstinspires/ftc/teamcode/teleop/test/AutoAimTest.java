package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.teleop.tune.DashboardAngularPIDFTuner;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "Test")
public class AutoAimTest extends BaseTeleOp
{
    public static int webcamDebounceTime = 20; // milliseconds

    public Hardware hardware;
    public AbstractDrive drive;
    public PIDFController controller;
    public AprilTagWebcam webcam;
    public ElapsedTime webcamTimer;
    public boolean manualControl;

    public AprilTagDetection detection;

    @Override
    public void setup()
    {
        hardware = new Hardware(hardwareMap);
        drive = new FieldCentricDrive(hardware);
        controller = new PIDFController(
            DashboardAngularPIDFTuner.P,
            DashboardAngularPIDFTuner.I,
            DashboardAngularPIDFTuner.D,
            DashboardAngularPIDFTuner.F
        );
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
        webcamTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        manualControl = false;

        subsystems.add(webcam);
    }

    @Override
    public void start()
    {
        webcamTimer.reset();
    }

    @Override
    public void run()
    {
        telemetry.addData("Goal ID", webcam.getGoalId());
        telemetry.addData("Control Mode", manualControl ? "Manual" : "Webcam Auto Aim");

        if (gamepads.justPressed(Button.GP1_A))
        {
            manualControl = !manualControl;
        }

        if (gamepads.justPressed(Button.GP1_B))
        {
            webcam.toggleGoalId();
        }

        if (manualControl)
        {
            drive.update(gamepads);
        }
        else
        {
            telemetry.addData("Goal AprilTag Detected?", detection == null ? "NO" : "YES");

            if (webcamTimer.time() > webcamDebounceTime)
            {
                detection = webcam.getGoalDetection();
                if (detection == null)
                {
                    drive.drive(0, 0, 0);
                }
                else
                {
                    double bearingError = detection.ftcPose.bearing;
                    double rx = controller.calculate(bearingError, 0);
                    drive.drive(0, 0, rx);
                }

                webcamTimer.reset();
            }
        }
    }
}
