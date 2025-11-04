package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MashedPotatoRobot;
import org.firstinspires.ftc.teamcode.util.Camera;

@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends LinearOpMode{

MashedPotatoRobot robot = new MashedPotatoRobot();

Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        waitForStart();
        //code here
        robot.donut();

        robot.sleep(1000);
        //code ends
        stop();
    }

}
