package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Camera;
import org.firstinspires.ftc.teamcode.DECODERobot;

@Autonomous(name = "ParkFromFront", group = "Auto")
public class ParkFromFront extends LinearOpMode {
    DECODERobot robot = new DECODERobot(telemetry);

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        waitForStart();

        robot.driveTo(100);
        //to shoot if running
        robot.turnToFromHere(-45);
        robot.shootControl(0.0);
        //go back and block
        robot.turnToFromHere(45);
        robot.driveTo(-50);
        //park if we can
        robot.strafe(50)

        stop();
    }
}
