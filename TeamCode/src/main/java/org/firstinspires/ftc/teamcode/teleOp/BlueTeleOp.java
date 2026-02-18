package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MashedPotatoRobot;

@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode{

    MashedPotatoRobot robot = new MashedPotatoRobot();
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.curSide = MashedPotatoRobot.side.BLUE;
//        robot.setOffsets(0,0,0);
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2, telemetry);
    }


}
