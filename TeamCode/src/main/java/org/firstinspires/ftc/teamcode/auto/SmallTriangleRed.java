package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MashedPotatoRobot;
import org.firstinspires.ftc.teamcode.util.Camera;

@Autonomous(name = "SmallTriangleRed", group = "Auto")
public class SmallTriangleRed extends LinearOpMode{

    MashedPotatoRobot robot = new MashedPotatoRobot();

    Camera camera;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        robot.setOffsets(2024,0,0);

        waitForStart();
        //code here
        robot.setShootPower(0.64);
        robot.setCoords(2210,144,-17.5);//2215,159,158.1
        robot.sleep(7000);
        shoot();

        robot.intakePower(-1.0);
        robot.transferPower(-1.0);
        robot.handleMotorPower(1500);

        //intake
        robot.setShootPower(0.0);
        robot.setCoords(2272, 773, -90);
        robot.intakePower(1.0);
        robot.transferPower(0.75);
        robot.driveTo(1250, 0.25);
        sleep(250);
        robot.intakePower(0.0);
        robot.transferPower(0.0);
        robot.turnToFromHere(0);

        //2nd shot
//        robot.setCoords(2215,159,158);
//        shoot();

//        robot.setCoords(2160,680,0);

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
        robot.handleMotorPower(1500);

        //third
        robot.transferPower(1.0);
        robot.intakePower(1.0);
        robot.handleMotorPower(3000);

        robot.transferPower(0.0);
        robot.intakePower(0.0);
        robot.handleMotorPower();

    }

}