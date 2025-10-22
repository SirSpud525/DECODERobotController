package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "TestAuto", group = "Auto")
public class TestAuto extends LinearOpMode{

MashedPotatoRobot robot = new MashedPotatoRobot();
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();
        //code here
        robot.donut();

        robot.sleep(1000);
        //code ends
        stop();
    }

}
