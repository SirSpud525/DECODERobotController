package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MashedPotatoRobot;

@TeleOp(name = "Display", group = "TeleOp")
public class Display extends OpMode{

    MashedPotatoRobot robot = new MashedPotatoRobot();
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.setOffsets(0,0,0);
    }

    @Override
    public void loop() {
        robot.display(gamepad1);
    }


}
