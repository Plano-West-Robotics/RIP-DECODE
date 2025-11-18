package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.control.Analog;
import org.firstinspires.ftc.teamcode.core.control.Button;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@TeleOp(group = "Test")
public class GamepadsTest extends BaseTeleOp
{
    @Override
    public void setup()
    {
        super.setup();
    }

    @Override
    public void run()
    {
        for (Analog analogValue : Analog.values())
        {
            telemetry.addData(analogValue.name(), gamepads.getAnalogValue(analogValue));
        }

        for (Button buttonValue : Button.values())
        {
            telemetry.addData(buttonValue.name(), gamepads.isPressed(buttonValue));
        }
    }
}