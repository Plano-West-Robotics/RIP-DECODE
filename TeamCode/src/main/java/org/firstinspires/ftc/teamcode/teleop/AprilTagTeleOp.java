package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class AprilTagTeleOp extends BaseTeleOp
{
    public Camera webcam;
    
    @Override
    public void setup()
    {
        webcam = new Camera(hardware);
    }

    @Override
    public void init_loop()
    {
        if (webcam.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            telemetry.addData("Camera", "Waiting");
        }
        else
        {
            telemetry.addData("Camera", "Ready");
        }
    }

    @Override
    public void start()
    {
        setManualExposure(Camera.EXPOSURE_MS, Camera.GAIN);  // Use low exposure time to reduce motion blur
    }

    @Override
    public void run()
    {
    }

    /**
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag(), and only works for Webcams;
     * @param exposureMS
     * @param gain
     */
    private void setManualExposure(int exposureMS, int gain) {
        ExposureControl exposureControl = webcam.visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = webcam.visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
