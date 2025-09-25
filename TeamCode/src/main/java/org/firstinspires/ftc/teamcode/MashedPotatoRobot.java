package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class MashedPotatoRobot {

//GLOBAL VARIABLES AND INIT
    
final String robotName = "Potato";
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public IMU imu;

    //potato
    //mashed potatoes
    //var
    //12345betA

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors using the driver/control hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set reverse motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        //zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        this.imu.resetYaw();
    }

    //UTILITY FUNCTIONS
    
        private void setDriveMode(final DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    //TELEOP FUNCTION

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry) {
            //Driving(gp1, telemetry);
            potatoDrive(gp1, telemetry);
    }

    public void Driving(Gamepad gp1, Telemetry telemetry){
        //uses driver input to speedup/slowdon as needed
        double multiplier = Math.max(0.3, 1 - gp1.right_trigger);
        double speedMode = (gp1.left_trigger);

        //gets joystick input
        double slowdown = 1.5;
        final double drive = (-gp1.left_stick_y);
        final double turn = (gp1.right_stick_x);
        final double strafe = (gp1.left_stick_x);

        //math to determine power - uses mecanum wheel equation
        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        double imuPos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        telemetry.addData("IMU-Angle", imuPos);
        telemetry.update();

        if (speedMode > 0.1){
            slowdown = 1;
        }

        //factor in slowdown and speedup
        frontLeft.setPower(flDrivePower * multiplier / slowdown);
        frontRight.setPower(frDrivePower * multiplier / slowdown);
        backLeft.setPower(blDrivePower * multiplier / slowdown);
        backRight.setPower(brDrivePower * multiplier / slowdown);
    }

    public void potatoDrive (Gamepad gp1, Telemetry telemetry){
        //gyro driving function
        //IMU setup
        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        //reset button - zeroes the gyro
        if (gp1.x == true){
            imu.resetYaw();
        }

        //get IMU
        double imuPos = (imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        telemetry.addData("IMU-Angle", imuPos);

        //math - uses trig functions to find the power for each motor
        double change = Math.cos(-imuPos);
        double sine = Math.sin(-imuPos);

        double drive = (-gp1.left_stick_y);
        double strafe = (gp1.left_stick_x);
        double turn = (gp1.right_stick_x);

        double driveCos = change * drive;
        double driveSin = sine * drive;
        double strafeSin = sine * strafe;
        double strafeCos = change * strafe;

        //divided by 1.5 is to account for strafing being slower than driving forward
        double actualDrive = (driveCos + strafeSin) / 1.5;
        double actualStrafe = -driveSin + strafeCos;

        //set power
        flDrivePower = (actualDrive + actualStrafe + turn);
        frDrivePower = (actualDrive - actualStrafe - turn);
        blDrivePower = (actualDrive - actualStrafe + turn);
        brDrivePower = (actualDrive + actualStrafe - turn);

        double slowdown = 0.85;

        //adds or removes to speed based on user input
        if (gp1.right_trigger >= 0.1){
            slowdown = 0.25;
        } else if (gp1.left_trigger >= 0.1) {
            slowdown = 1.25;
        }

        frontLeft.setPower(flDrivePower * slowdown);
        frontRight.setPower(frDrivePower * slowdown);
        backLeft.setPower(blDrivePower * slowdown);
        backRight.setPower(brDrivePower * slowdown);

    }

    
    //AUTO FUNCTIONS



    
}
