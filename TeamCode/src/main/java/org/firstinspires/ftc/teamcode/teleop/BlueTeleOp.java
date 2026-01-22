package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODERobot;

@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.curSide = DECODERobot.side.BLUE;
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2);
    }
}
