package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MashedPotatoRobot;
import org.firstinspires.ftc.teamcode.util.Camera;

@Autonomous(name = "BigTriangleBlue", group = "Auto")
public class BigTriangleBlue extends LinearOpMode{

    MashedPotatoRobot robot = new MashedPotatoRobot();

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        robot.setOffsets(401,3017,0);

        waitForStart();
        //code here
        robot.setShootPower(0.50);
        robot.setCoords(1264,1949,40);//1162,2160,-132.92
        robot.sleep(6000);

        shoot();
        robot.intakePower(-1.0);
        robot.transferPower(-1.0);
        robot.setShootPower(0.0);
        robot.handleMotorPower(1500);
        robot.setCoords(1111,1866,90);
        robot.intakePower(1.0);
        robot.transferPower(0.75);
        robot.driveTo(1400, 0.25);
        robot.turnToFromHere(0);

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
        robot.handleMotorPower(5000);

        //stop
        robot.transferPower(0.0);
        robot.intakePower(0.0);
        robot.handleMotorPower();

    }

}
