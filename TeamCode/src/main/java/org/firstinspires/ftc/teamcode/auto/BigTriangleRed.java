package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MashedPotatoRobot;
import org.firstinspires.ftc.teamcode.util.Camera;

@Autonomous(name = "BigTriangleRed", group = "Auto")
public class BigTriangleRed extends LinearOpMode{

    MashedPotatoRobot robot = new MashedPotatoRobot();

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        robot.setOffsets(2755,2992,0);

        waitForStart();
        //code here
        robot.setCoords(2078,1896,148);


        //code ends
        stop();
    }

    public void shoot(){
        //shoot
    }

}