package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.templates.StageServo;
import org.firstinspires.ftc.teamcode.core.templates.StageServoMono;

public class RightStopper extends StageServoMono<RightStopper.Stage>
{
    public enum Stage
    {
        STOP,
        GO
    }

    public RightStopper(HardwareMap hardwareMap)
    {
        super(new StageServoMonoBuilder<>(hardwareMap, "stopR", Stage.class)
            .add(Stage.STOP, 0.3)
            .add(Stage.GO, 0.63)
        );
    }

    public void stop()
    {
        setStage(Stage.STOP);
    }

    public void go()
    {
        setStage(Stage.GO);
    }
}
