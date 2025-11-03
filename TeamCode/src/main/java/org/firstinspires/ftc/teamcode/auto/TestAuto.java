package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends LinearOpMode{

Robot robot = new Robot(telemetry);

Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        waitForStart();



        stop();
    }

}
