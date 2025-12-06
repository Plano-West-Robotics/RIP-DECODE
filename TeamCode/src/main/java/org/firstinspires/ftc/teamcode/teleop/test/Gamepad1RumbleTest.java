package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.control.Gamepads;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

@Config
@TeleOp(group = "Test")
public class Gamepad1RumbleTest extends BaseTeleOp
{
    public Gamepads gamepads;
    public static boolean rumbleGP1;
    public boolean pastRumble = false;

    @Override
    public void setup()
    {
        gamepads = new Gamepads(gamepad1, gamepad2);
    }

    @Override
    public void run()
    {

        if (rumbleGP1 && !pastRumble && !gamepad1.isRumbling())
        {
            gamepad1.rumbleBlips(1);
        }

        pastRumble = rumbleGP1;
    }
}
