package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class MashedPotatoRobot {

//GLOBAL VARIABLES AND INIT
    Telemetry telemetry;
    final String robotName = "Potato";
    public ElapsedTime driveModeDebounce = new ElapsedTime();
    private ElapsedTime lockDebounce = new ElapsedTime();
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double shootPower;
    public double intakePower;
    public double upLeftPower;
    public double upRightPower;
    public double lowLeftPower;
    public double lowRightPower;
    public boolean gyroDrive = false;
    public boolean locked = false;
    public shooterStates shooterState = shooterStates.HIGH;
    public side curSide = side.BLUE;
    public Pose2D desiredPose = new Pose2D(DistanceUnit.MM, 0, 0, RADIANS, 0);
    public double targetAngle = 0.0;

    public double manualPower = 0.5;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shootMotor;
    private DcMotor shootMotor2;
    private DcMotor intake;
    private CRServo lowerRight;
    private CRServo lowerLeft;
    private CRServo upperRight;
    private CRServo upperLeft;
    public IMU imu;
    public GoBildaPinpointDriver odo;

    //potato
    //mashed potatoes
    //var
    //12345betA

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors using the driver/control hub
        frontLeft = hardwareMap.get(DcMotor.class, "flDrive"); //3
        frontRight = hardwareMap.get(DcMotor.class, "frDrive"); //2
        backLeft = hardwareMap.get(DcMotor.class, "blDrive"); //0
        backRight = hardwareMap.get(DcMotor.class, "brDrive"); //1
        shootMotor = hardwareMap.get(DcMotor.class, "leftShoot");
        shootMotor2 = hardwareMap.get(DcMotor.class, "rightShoot");

        intake = hardwareMap.get(DcMotor.class, "intake");
        lowerLeft = hardwareMap.get(CRServo.class, "blSpinner");
        lowerRight = hardwareMap.get(CRServo.class, "brSpinner");
        upperLeft = hardwareMap.get(CRServo.class, "tlSpinner");
        upperRight = hardwareMap.get(CRServo.class, "trSpinner");

        // Set reverse motors
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        upperRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        upperLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        //zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        this.imu.resetYaw();
        //Initializing odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(70, 315, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        setOdo(0, 0, 45);
    }

    //UTILITY FUNCTIONS
    public void setOdo(int initialX, int initialY, int initialDir){
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.MM, initialX, initialY, DEGREES, initialDir));
    }
    public void handleMotorPower() {
        frontLeft.setPower(flDrivePower);
        frontRight.setPower(frDrivePower);
        backLeft.setPower(blDrivePower);
        backRight.setPower(brDrivePower);

        intake.setPower(intakePower);

        shootMotor.setPower(shootPower);
        shootMotor2.setPower(shootPower);

        lowerLeft.setPower(lowLeftPower);
        upperLeft.setPower(upLeftPower);
        lowerRight.setPower(lowRightPower);
        upperRight.setPower(upRightPower);
    }

    public void handleMotorPower(int wait) {

        handleMotorPower();
        sleep(wait);

    }
    public void updateTelemetry(Telemetry telemetry) {
//        telemetry.addData("FL Drive Power: ", flDrivePower);
//        telemetry.addData("FR Drive Power: ", frDrivePower);
//        telemetry.addData("BL Drive Power: ", blDrivePower);
//        telemetry.addData("BR Drive Power: ", brDrivePower);
//        telemetry.addData("Drive Slowdown: ", driveSlowdown);
//        telemetry.addData("Drive Mode: ", driveMode);
        telemetry.addData("Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Camera: ", camera);
//        telemetry.addData("Intake Power: ", intakePower);
//        telemetry.addData("Shoot Power: ", shootPower);
//        telemetry.addData("Shoot Mode: ", shootingMode);
//        telemetry.addData("Top Spinner Power: ", topSpinnerPower);
//        telemetry.addData("Bottom Spinner Power: ", bottomSpinnerPower);
        telemetry.addData("Position: ", odo.getPosition());
//        telemetry.addData("Power for shoots: ", distToPower(45));
//        telemetry.addData("Actual power: ", leftShoot.getPower());
//        telemetry.addData("Target angle: ", getTargetAngle(305,305));

        telemetry.update();
    }
    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ignored) {
        }
    }
    public double distToAngle() {
        //Gives an angle based on distance to land the artifact in the basket
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        double dist = Math.sqrt(x * x + y * y);
        double v0 = (2 * 125 * 0.02) / (0.075); //initial velocity
        double constant = 9.8 / v0;
        double height = 0.9845;
        double discriminant = 1 - dist * dist * constant * constant - 2 * height * constant;
        double theta = Math.atan(v0 * (1 + Math.sqrt(discriminant)) / (9.8 * dist));
        return theta; //Returns angle
    }
    public double distToVelocity(double theta) {
        //Given theta (The angle at which the artifact shoots at) return the velocity needed to reach the basket
        double x = 1000 * odo.getPosX(DistanceUnit.MM);
        double y = 1000 * odo.getPosY(DistanceUnit.MM);
        double dist = Math.sqrt(x * x + y * y);
        double secant = 1 / Math.cos(Math.toRadians(theta));
        double numerator = -9.8 * dist * dist;
        double denominator = 2 * (.9845 - dist * Math.tan(Math.toRadians(theta)));
        return secant * Math.sqrt(numerator / denominator);
    }
    public double distToPower(){
        //Returns power based on distance
        double v = distToVelocity(45.0);
        double RPM = 60 * v / (3.1415926 * 2 * .062);
        return RPM / 6000; //Returns RPM needed divided by max RPM
    }
    public double getTargetAngle(int initialX, int initialY){
        //returns the angle needed to turn the robot
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        targetAngle = 180 * Math.atan((initialY + y)/(initialX + x)) / 3.1415926 - imu.getRobotYawPitchRollAngles().getYaw();
        return targetAngle;
    }
    private void setDriveMode(final DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
    public void drive(final double pow) {
        //Sets all motors to pow
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    public void setOffsets(int initialX, int initialY, int initialDir){
        odo.setOffsets(0, 0, DistanceUnit.MM);//was 70, 315 (smth is very wrong here - maybe the offsets are reversed? - -250, -175)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        sleep(300);
        odo.setPosition(new Pose2D(DistanceUnit.MM, initialX, initialY, DEGREES, initialDir));

//        odo.resetPosAndIMU();
    }
    public void resetOdo(){
        odo.setOffsets(0, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        this.imu.resetYaw();
        odo.resetPosAndIMU();
        sleep(300);
        if (curSide == side.RED){
            odo.setPosition(new Pose2D(DistanceUnit.MM, 1280, 0, DEGREES, 0));

        } else {
            odo.setPosition(new Pose2D(DistanceUnit.MM, 1955, 0, DEGREES, 0));
        }
    }

    //TELEOP FUNCTIONS

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry) {
        odo.update();
//        camera.update();
//        setLock(gp2);
//checks to see if lock should be used, stops everything from moving if it is
//otherwise run functions as normal
        if (locked){
            drive(0.0);
            intake.setPower(0.0);
            shootMotor.setPower(0.0);
            shootMotor2.setPower(0.0);
        }else {
            if (gp1.y) {
                aimbot();
            } else if (gp1.b){
                staybot();
            } else {
                if (gyroDrive) {
                    gyroDrive(gp1, telemetry);
                } else {
                    Driving(gp1, telemetry);
                }
            }
            toggleGyro(gp1);
            handleShootMotors(gp2);
            intake(gp2);
            transferSystem(gp2);
        }
        updateTelemetry(telemetry);
    }
    public void setLock(Gamepad gp) {
        if (gp.y) {
            if (lockDebounce.milliseconds() < 500) return;

            lockDebounce.reset();

            locked = !locked;
        }
    }
    public void toggleGyro(Gamepad gp1) {
        //Toggles gyro on or off

        //toggles GyroDrive boolean
        if (gp1.x) {
            if (driveModeDebounce.milliseconds() < 500) return;
            driveModeDebounce.reset();

            gyroDrive = !gyroDrive;
        }
    }
    public void intake(Gamepad gp2) {
        //sets intake power when b is pressed
        if (gp2.a == true || gp2.x == true) {
            intake.setPower(1.0);
        }
        else if (gp2.left_trigger > 0.1){
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }
    }
    public void transferSystem(Gamepad gp2) {
        double currentInput = -gp2.left_stick_y;
        if (currentInput > 0){
            upperRight.setPower(1.0);
            upperLeft.setPower(-1.0);
            lowerRight.setPower(1.0);
            lowerLeft.setPower(-1.0);
        }
        else if (currentInput < 0){
            upperRight.setPower(-1.0);
            upperLeft.setPower(1.0);
            lowerRight.setPower(-1.0);
            lowerLeft.setPower(1.0);
        } else if (gp2.left_trigger > 0.1){
            upperRight.setPower(-1.0);
            upperLeft.setPower(1.0);
            lowerRight.setPower(-1.0);
            lowerLeft.setPower(1.0);
        } else if (gp2.a){
            upperRight.setPower(1.0);
            upperLeft.setPower(-1.0);
            lowerRight.setPower(1.0);
            lowerLeft.setPower(-1.0);
        }else if (gp2.x){
            lowerRight.setPower(1.0);
            lowerLeft.setPower(-1.0);
        } else {
            upperRight.setPower(0);
            upperLeft.setPower(0);
            lowerRight.setPower(0);
            lowerLeft.setPower(0);
        }

    }

    public void handleShootMotors(Gamepad gp2) {
        if (gp2.dpad_up) {
            shooterState = shooterStates.HIGH; //enum name mentioned here - Saumya
        }
        if (gp2.dpad_down) {
            shooterState = shooterStates.LOW;//enum name mentioned here - Saumya
        }

        if (gp2.dpad_right || gp2.dpad_left) {
            shooterState = shooterStates.MID;//enum name mentioned here - Saumya
        }

        if(gp2.right_trigger > 0.1) {
            if (shooterState == shooterStates.HIGH) {
                shootMotor.setPower(0.35); // setting power fo the shoot motor based on what enum is used - Saumya
                shootMotor2.setPower(0.35);
            }
            if (shooterState == shooterStates.MID){
                shootMotor.setPower(0.32);
                shootMotor2.setPower(0.32);
            }
            if (shooterState == shooterStates.LOW) {
                shootMotor.setPower(0.29);
                shootMotor2.setPower(0.29);
            }
            if (shooterState == shooterStates.MANUAL) {
                shootMotor.setPower(manualPower);
                shootMotor2.setPower(manualPower);
            }

        } else if(gp2.left_trigger > 0.1){
            shootMotor.setPower(-0.8);
            shootMotor2.setPower(-0.8);
        }
        else {
            shootMotor.setPower(0); //setting power to 0 simple - Saumya
            shootMotor2.setPower(0.0);
        }

        if (gp2.left_bumper == true){
            manualPower = manualPower - 0.01;
        }
        if (gp2.right_bumper == true){
            manualPower = manualPower + 0.01;
        }

    }

    public void Driving(Gamepad gp1, Telemetry telemetry) {
        //uses driver input to speedup/slowdown as needed
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

//        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
//        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
//        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        double imuPos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
//        telemetry.addData("IMU-Angle", imuPos);
//        telemetry.update();

        if (speedMode > 0.1) {
            slowdown = 1;
        }

        //factor in slowdown and speedup
        frontLeft.setPower(flDrivePower * multiplier / slowdown);
        frontRight.setPower(frDrivePower * multiplier / slowdown);
        backLeft.setPower(blDrivePower * multiplier / slowdown);
        backRight.setPower(brDrivePower * multiplier / slowdown);
    }

    public void gyroDrive(Gamepad gp1, Telemetry telemetry) {
        //gyro driving function
        //IMU setup
        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        //reset button - zeroes the gyro
        if (gp1.y == true) {
            imu.resetYaw();
        }

        //get IMU
        double imuPos = (imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        telemetry.addData("IMU-Angle", imuPos);

        //return target angle
        telemetry.addData("Target angle: ", getTargetAngle(305,305));

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
        if (gp1.right_trigger >= 0.1) {
            slowdown = 0.25;
        } else if (gp1.left_trigger >= 0.1) {
            slowdown = 1.25;
        }

        frontLeft.setPower(flDrivePower * slowdown);
        frontRight.setPower(frDrivePower * slowdown);
        backLeft.setPower(blDrivePower * slowdown);
        backRight.setPower(brDrivePower * slowdown);

    }

    public void aimbot() {
        double targetAngle;

        if (curSide == side.BLUE) {
            targetAngle = -Math.PI + (Math.atan((odo.getPosX(DistanceUnit.CM) - 7) / (343 - odo.getPosY(DistanceUnit.CM))));
        }else{
            targetAngle = Math.PI + (Math.atan((odo.getPosX(DistanceUnit.CM) - 313) / (346 - odo.getPosY(DistanceUnit.CM))));
        }

        double error = (targetAngle - odo.getHeading(RADIANS)) * (180 / Math.PI);

        double kp = 1;

        double proportional = error * kp;
        double turn = proportional / (180);

        if (Math.abs(error) > 0.25) {
            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            frontLeft.setPower((flDrivePower) + (Math.signum(flDrivePower) * 0.1));
            frontRight.setPower((frDrivePower) + (Math.signum(frDrivePower) * 0.1));
            backLeft.setPower((blDrivePower) + (Math.signum(blDrivePower) * 0.1));
            backRight.setPower((brDrivePower) + (Math.signum(brDrivePower) * 0.1));

        } else {
            brake();
        }
    }

    public void staybot() {

        double xmm = desiredPose.getX(DistanceUnit.MM);
        double ymm = desiredPose.getY(DistanceUnit.MM);
        double error = 15.0;
        double errorButMore = 225.0 + error;

        if (Math.abs(xmm - odo.getPosX(DistanceUnit.MM)) > error || Math.abs(ymm - odo.getPosY(DistanceUnit.MM)) > error) {
            // Drive
            double xDist = xmm - odo.getPosX(DistanceUnit.MM);
            double yDist = xmm - odo.getPosY(DistanceUnit.MM);

            double drivePwr = yDist / (double) Math.abs(ymm);
            double strafePwr = xDist / (double) Math.abs(xmm);

            if (Math.abs(yDist) > error) {
                drivePwr = drivePwr + ((Math.signum(drivePwr) * 0.2));
            } else {
                drivePwr = 0.0;
            }
            if (Math.abs(xDist) > error) {
                strafePwr = strafePwr + ((Math.signum(strafePwr) * 0.2));
            } else {
                strafePwr = 0.0;
            }

            if (Math.abs(xDist) > errorButMore) {
                drivePwr = 1.2 * Math.signum(yDist);
            }
            if (Math.abs(yDist) > errorButMore) {
                strafePwr = 1.2 * Math.signum(xDist);
            }

            double angle = odo.getHeading(RADIANS);

            double cosine = Math.cos(-angle);
            double sine = Math.sin(-angle);

            double driveCos = cosine * drivePwr;
            double driveSin = sine * drivePwr;
            double strafeCos = cosine * strafePwr;
            double strafeSin = sine * strafePwr;

            drivePwr = (driveCos + strafeSin) / 1.25;
            strafePwr = (strafeCos - driveSin);

            frontLeft.setPower((drivePwr + strafePwr) / 1.5);
            frontRight.setPower((drivePwr - strafePwr) / 1.5);
            backLeft.setPower((drivePwr - strafePwr) / 1.5);
            backRight.setPower((drivePwr + strafePwr) / 1.5);

        } else {
            brake();
        }
    }

        //AUTO FUNCTIONS
    public void strafeTo(int pos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int mm = 10 * pos;

        int flBrPos = mm;
        int frBlPos = -mm;

        while (Math.abs(mm - odo.getPosX(DistanceUnit.MM)) > 5) {
            double flDistance = flBrPos - odo.getPosX(DistanceUnit.MM);
            double frDistance = frBlPos - odo.getPosX(DistanceUnit.MM);
            double blDistance = frBlPos - odo.getPosX(DistanceUnit.MM);
            double brDistance = flBrPos - odo.getPosX(DistanceUnit.MM);

            flDrivePower = flDistance / (double) Math.abs(flBrPos);
            blDrivePower = blDistance / (double) Math.abs(frBlPos);
            frDrivePower = frDistance / (double) Math.abs(frBlPos);
            brDrivePower = brDistance / (double) Math.abs(flBrPos);

            if (Math.abs(flBrPos - backRight.getCurrentPosition()) > 100 || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > 100) {
                flDrivePower = Math.signum(flDrivePower);
                frDrivePower = Math.signum(frDrivePower);
                blDrivePower = Math.signum(blDrivePower);
                brDrivePower = Math.signum(brDrivePower);
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);
            }

            // Correct angle
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / 180;

            double flDrivePowerCorrection = -turn;
            double frDrivePowerCorrection = turn;
            double blDrivePowerCorrection = -turn;
            double brDrivePowerCorrection = turn;

            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);

            flDrivePower += flDrivePowerCorrection;
            frDrivePower += frDrivePowerCorrection;
            blDrivePower += blDrivePowerCorrection;
            brDrivePower += brDrivePowerCorrection;

            handleMotorPower();
            odo.update();

            telemetry.addData("pos", odo.getPosX(DistanceUnit.MM));
            telemetry.update();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void enableAllMotors(final double p1, final double p2) {
        //abstraction, sets power to the TL & BR motors for p1, and TR & BL motors for p2
        frDrivePower = p2;
        brDrivePower = p1;
        flDrivePower = p1;
        blDrivePower = p2;
    }

    public void angularStrafe(int degrees, int distance) {
        //Strafes the robot distance in "degrees" direction
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Pitch = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES); //makes sure that it strafes relative to the starting IMU pos
        double angleInRadians = Math.toRadians(degrees + 315 - Pitch); //0 deg, would strafe right, 90 deg would strafe up, etc.
        double cosValue = Math.cos(angleInRadians);
        double sinValue = Math.sin(angleInRadians);
        double yDist = distance * sinValue + odo.getPosY(DistanceUnit.MM);
        double xDist = distance * cosValue + odo.getPosX(DistanceUnit.MM);

        final double error = 100;
        final double slowdown = 1.8;

        while (Math.abs(yDist - odo.getPosY(DistanceUnit.MM)) > error || Math.abs(xDist - odo.getPosX(DistanceUnit.MM)) > error) {

            enableAllMotors(cosValue / slowdown, sinValue / slowdown);
            handleMotorPower();
        }
        drive(0.0);

    }
    public void setCoords(double xPos, double yPos, double desAngle){
        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        double xmm = xPos * 10;
        double ymm = yPos * 10;
        final int delay = 10;
        double error = 15.0;
        double angError = 0.5;
        double errorButMore = 225.0 + error;
        double kp = 1;
        odo.update();

        while (Math.abs(xmm - odo.getPosX(DistanceUnit.MM)) > error || Math.abs(ymm - odo.getPosY(DistanceUnit.MM)) > error || Math.abs(desAngle - odo.getHeading(DEGREES)) > angError){
            // Drive
            double xDist = xmm - odo.getPosX(DistanceUnit.MM);
            double yDist = xmm - odo.getPosY(DistanceUnit.MM);

            double drivePwr = yDist / (double) Math.abs(ymm);
            double strafePwr = xDist / (double) Math.abs(xmm);

            if (Math.abs(yDist) > error) {
                drivePwr = drivePwr + ((Math.signum(drivePwr) * 0.2));
            } else {
                drivePwr = 0.0;
            }
            if (Math.abs(xDist) > error) {
                strafePwr = strafePwr + ((Math.signum(strafePwr) * 0.2));
            } else {
                strafePwr = 0.0;
            }

            if (Math.abs(xDist) > errorButMore){
                drivePwr = 1.2 * Math.signum(yDist);
            }
            if (Math.abs(yDist) > errorButMore){
                strafePwr = 1.2 * Math.signum(xDist);
            }

            double angle = odo.getHeading(RADIANS);

            double cosine = Math.cos(-angle);
            double sine = Math.sin(-angle);

            double driveCos = cosine * drivePwr;
            double driveSin = sine * drivePwr;
            double strafeCos = cosine * strafePwr;
            double strafeSin = sine * strafePwr;

            drivePwr = (driveCos + strafeSin) / 1.25;
            strafePwr = (strafeCos - driveSin);

            flDrivePower = (drivePwr + strafePwr) / 1.5;
            frDrivePower = (drivePwr - strafePwr) / 1.5;
            blDrivePower = (drivePwr - strafePwr) / 1.5;
            brDrivePower = (drivePwr + strafePwr) / 1.5;


            double curAngError = desAngle - odo.getHeading(DEGREES);

            if (Math.abs(curAngError) > angError) {
                double proportional = curAngError * kp;
                double turn = proportional / 180;
                turn = turn + (Math.signum(curAngError) * 0.1);

                flDrivePower -= turn;
                frDrivePower += turn;
                blDrivePower -= turn;
                brDrivePower += turn;

            }

            handleMotorPower();
            odo.update();

            telemetry.addData("angleOffset", angle * 180/3.14);
            telemetry.addData("heading", odo.getHeading(DEGREES));
            telemetry.addData("flDrive", flDrivePower);
            telemetry.addData("blDrive", blDrivePower);
            telemetry.addData("frDrive", frDrivePower);
            telemetry.addData("brDrive", brDrivePower);
            telemetry.addData("drivePwr", drivePwr);
            telemetry.addData("strafePwr", strafePwr);
            telemetry.addData("xPos", odo.getPosX(DistanceUnit.MM));
            telemetry.addData("yPos", odo.getPosY(DistanceUnit.MM));
            telemetry.update();

            sleep(delay);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void accurateStrafe(int pos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs((Math.abs(flBrPos - frontLeft.getCurrentPosition()) + Math.abs(frBlPos - frontRight.getCurrentPosition())) / 2) > 20) {
            int flDistance = flBrPos - frontLeft.getCurrentPosition();
            int frDistance = frBlPos - frontRight.getCurrentPosition();
            int blDistance = frBlPos - backLeft.getCurrentPosition();
            int brDistance = flBrPos - backRight.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(flBrPos);
            blDrivePower = (double) blDistance / (double) Math.abs(frBlPos);
            frDrivePower = (double) frDistance / (double) Math.abs(frBlPos);
            brDrivePower = (double) brDistance / (double) Math.abs(flBrPos);

            if (Math.abs(flBrPos - backRight.getCurrentPosition()) > 100 || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > 100) {
                flDrivePower = Math.signum(flDrivePower);
                frDrivePower = Math.signum(frDrivePower);
                blDrivePower = Math.signum(blDrivePower);
                brDrivePower = Math.signum(brDrivePower);
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);
            }

            // Correct angle
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / (180 * kp);

            double flDrivePowerCorrection = -turn;
            double frDrivePowerCorrection = turn;
            double blDrivePowerCorrection = -turn;
            double brDrivePowerCorrection = turn;

            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);

            flDrivePower += flDrivePowerCorrection;
            frDrivePower += frDrivePowerCorrection;
            blDrivePower += blDrivePowerCorrection;
            brDrivePower += brDrivePowerCorrection;

            // there was a slowdown here, I removed it cus not responsible, could be an issue later

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveTo(int pos, double slowdown) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        int mm = pos * 10;
        final int DELAY = 20;
        odo.update();

        while (Math.abs(mm - odo.getPosY(DistanceUnit.MM)) > 5) {
            // Drive
            double distance = mm - odo.getPosY(DistanceUnit.MM);

            double power = distance / (double) Math.abs(mm);

            power = (power / slowdown) + (Math.signum(power) * 0.1);

            flDrivePower = power;
            frDrivePower = power;
            blDrivePower = power;
            brDrivePower = power;

            handleMotorPower();
            odo.update();


            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(double target) {
        //Target is how many degrees, turns slowly but accurately
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 2) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);
            final double mult = 2.4;

            flDrivePower = -turn * mult;
            frDrivePower = turn * mult;
            blDrivePower = -turn * mult;
            brDrivePower = turn * mult;

            flDrivePower = flDrivePower / 2;
            frDrivePower = frDrivePower / 2;
            blDrivePower = blDrivePower / 2;
            brDrivePower = brDrivePower / 2;

            handleMotorPower();

            try {
                Thread.sleep(DELAY);
            } catch (InterruptedException e) {
            }
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
        }
    }

    public void fastTurn(int deg) {
        //a faster(but less accurate) version of the turn function
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


        this.imu.resetYaw();
        double pos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);

        //get within 15 degrees of target
        while (Math.abs(deg - pos) > 15) {
            pos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);

            if (deg - pos >= 0) {
                flDrivePower = 0.8;
                frDrivePower = -0.8;
                brDrivePower = -0.8;
                blDrivePower = 0.8;
            } else {
                flDrivePower = -0.8;
                frDrivePower = 0.8;
                brDrivePower = 0.8;
                blDrivePower = -0.8;
            }

            handleMotorPower();

        }

        //finishing turning with the more accurate function
        turn(deg - pos);

    }

    public void turnToFromHere(double target) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 3) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

            //removed slowdown stuff

            handleMotorPower();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
public void intakePower(double power){
        intakePower = power;
}

public void transferPower(double power){
        upLeftPower = power * -1 ;
        upRightPower = power;
        lowLeftPower = power * -1 ;
        lowRightPower = power ;
        // This function accepts power as a input and controls it(-1 -> left, 1-> right)
}
public void setShootPower(int variableA){
        shootPower = variableA;
}

public void donut() {

        double strafe = 1.0;
        double turn = 0.25;

        flDrivePower = (strafe + turn);
        frDrivePower = (-strafe - turn);
        blDrivePower = (-strafe + turn);
        brDrivePower = (strafe - turn);

        for (int i = 200; i >= 0; i--) {
            handleMotorPower();
            sleep(25);
        }

    }

    //ENUMS

    public enum shooterStates{ //enum is a list of states, usually created in all caps -Saumya
        HIGH, MID, LOW, MANUAL
    }

    public enum side {
        BLUE,
        RED
    }

}