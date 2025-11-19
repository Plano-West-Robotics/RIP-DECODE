package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.subsystems.AbstractDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "Test")
public class WebcamAutoAimTest extends BaseTeleOp
{
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public AprilTagWebcam webcam;
    public AprilTagDetection detection;
    public AbstractDrive drive;
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
        if (gamepads.justPressed(Button.GP1_A))
        {
            webcam.toggleGoalId();
            detection = null;
        }

        if (gamepads.justPressed(Button.GP1_DPAD_RIGHT))
        {
            ((FieldCentricDrive) drive).imu.resetYaw();
        }

        controller.setPIDF(P, I, D, F);

        detection = webcam.getGoalDetection();

        telemetry.addData("Goal Color", webcam.getGoalId() == AprilTagWebcam.RED_GOAL_ID ? "RED" : "BLUE");
        if (detection != null)
        {
            telemetry.addLine("AprilTag Detected!");

            telemetry.addData("Yaw", detection.ftcPose.yaw);
            telemetry.addData("Pitch", detection.ftcPose.pitch);
            telemetry.addData("Roll", detection.ftcPose.roll);
            telemetry.addData("Range", detection.ftcPose.range);
            telemetry.addData("Bearing", detection.ftcPose.bearing);
            telemetry.addData("Elevation", detection.ftcPose.elevation);

            double rx = -controller.calculate(detection.ftcPose.bearing, 0);
            drive.drive(0, 0, rx);

            telemetry.addData("RX", rx);

            telemetry.addData("FR Power", drive.drivetrainMotors.fr.getPower());
            telemetry.addData("FL Power", drive.drivetrainMotors.fl.getPower());
            telemetry.addData("BR Power", drive.drivetrainMotors.br.getPower());
            telemetry.addData("BL Power", drive.drivetrainMotors.bl.getPower());
        }
    }
}
