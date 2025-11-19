package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "Test")
public class AprilTagWebcamTest extends BaseTeleOp
{
    public AprilTagWebcam webcam;
    public AprilTagDetection detection;

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware, AprilTagWebcam.RED_GOAL_ID);
    }

    @Override
    public void run()
    {
        if (gamepads.justPressed(Button.GP1_A))
        {
            webcam.toggleGoalId();
            detection = null;
        }

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
        }
    }
}
