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
        robot.setShootPower(0.265);
        robot.setCoords(2198,1935,137);
        robot.sleep(7000);

        shoot();


        //code ends
        stop();
    }

    public void shoot(){
        //shoot
        //first - set power and spin up before calling this
        robot.transferPower(1.0);
        robot.intakePower(1.0);
        robot.handleMotorPower(600);

        //spin back up
        robot.transferPower(0.0);
        robot.intakePower(0.0);
        robot.handleMotorPower(1500);

        //second
        robot.transferPower(1.0);
        robot.intakePower(1.0);
        robot.handleMotorPower(1200);

        //spin back up
        robot.transferPower(0.0);
        robot.intakePower(0.0);
        robot.handleMotorPower(1000);

        //third
        robot.transferPower(1.0);
        robot.intakePower(1.0);
        robot.handleMotorPower(3000);

        robot.transferPower(0.0);
        robot.intakePower(0.0);
        robot.handleMotorPower();

    }

}