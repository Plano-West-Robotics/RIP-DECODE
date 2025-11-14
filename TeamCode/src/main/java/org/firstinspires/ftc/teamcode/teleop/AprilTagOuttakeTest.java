package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(group = "Test")
public class AprilTagOuttakeTest extends BaseTeleOp
{
    public AprilTagWebcam webcam;
    public Outtake outtake;

    @Override
    public void setup()
    {
        webcam = new AprilTagWebcam(hardware);
        outtake = new Outtake(hardware);

        subsystems.add(webcam);
    }

    @Override
    public void init_loop()
    {
        boolean webcamIsReady = webcam.portal.getCameraState() != VisionPortal.CameraState.STREAMING;
        telemetry.addData("Camera", webcamIsReady ? "Ready" : "Not Ready");
        telemetry.update();
    }

    @Override
    public void run()
    {
        List<AprilTagDetection> detections = webcam.getDetections();
        AprilTagDetection target = detections.get(0); // will become the detection with the smallest range

        telemetry.addData("# AprilTags Detected", detections.size());

        int count = 0;
        for (AprilTagDetection detection : detections)
        {
            telemetry.addData(String.format("%d ID", count), "%d", detection.metadata.id);
            telemetry.addData(String.format("%d Range", count), "%5.5f", detection.ftcPose.range);
            telemetry.addData(String.format("%d Bearing", count), "%5.5f", detection.ftcPose.bearing);
            telemetry.addData(String.format("%d Elevation", count), "%5.5f", detection.ftcPose.elevation);

            if (detection.ftcPose.range < target.ftcPose.range)
            {
                target = detection;
            }

            count++;
        }

        double velocity = Outtake.calculateIdealFlywheelTangentialVelocity(target.ftcPose.range);
        double angularRate = Outtake.toAngularRate(velocity);
        ((DcMotorEx) outtake.motor).setVelocity(angularRate);
    }
}
