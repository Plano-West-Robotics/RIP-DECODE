package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.templates.StageServoPair;

public class StoppersServoPair extends StageServoPair<StoppersServoPair.Stage>
{
    public static final double POSITION_DIFF = 0;

    public enum Stage
    {
        STANDBY,
        STOP,
    }

    public StoppersServoPair(HardwareMap hardwareMap)
    {
        super(
            new StageServoPairBuilder<>(
            hardwareMap,
            "pl",
            "pr",
            Stage.class,
            POSITION_DIFF
            )
            .add(Stage.STANDBY, 0.5)
            .add(Stage.STOP, 1)
        );
    }

    public void standby()
    {
        setStage(Stage.STANDBY);
    }

    public void stop()
    {
        setStage(Stage.STOP);
    }
}
