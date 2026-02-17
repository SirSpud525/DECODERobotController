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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Camera;


public class DECODERobot {
    Telemetry telemetry;
    public side curSide = side.BLUE;
    private final ElapsedTime driveModeDebounce = new ElapsedTime();
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double driveSlowdown = 2;
    public Pose2D desiredPose = new Pose2D(DistanceUnit.MM, 0, 0, RADIANS, 0);
    public DriveMode driveMode = DriveMode.NORMAL;
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor blDrive;
    private DcMotor brDrive;


//    public double slidePower;
//    private DcMotor leftSlide;
//    private DcMotor rightSlide;

    public double intakePower;
    private DcMotor intake;

    public ShootingMode shootingMode = ShootingMode.LOW;
    public double shootPower;
    private DcMotor leftShoot;
    private DcMotor rightShoot;

    public double bottomSpinnerPower = 0.0;
    public double topSpinnerPower = 0.0;

    private CRServo tlSpinner;
    private CRServo trSpinner;
    private CRServo blSpinner;
    private CRServo brSpinner;

    private final ElapsedTime lockDebounce = new ElapsedTime();
    public boolean locked = false;

    public IMU imu;

    public Camera camera;

    public GoBildaPinpointDriver odo;

    public DECODERobot(Telemetry robotTelemetry) {
        telemetry = robotTelemetry;
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize hardware map - declares code version of physical motors as mapped in the driver hub
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");//1
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");//0
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");//3
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");//2
//        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
//        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");//1e
        leftShoot = hardwareMap.get(DcMotor.class, "leftShoot");//2e
        rightShoot = hardwareMap.get(DcMotor.class, "rightShoot");//3e

        tlSpinner = hardwareMap.get(CRServo.class, "tlSpinner");//0
        trSpinner = hardwareMap.get(CRServo.class, "trSpinner");//
        blSpinner = hardwareMap.get(CRServo.class, "blSpinner");//2
        brSpinner = hardwareMap.get(CRServo.class, "brSpinner");//

        // Reverse motors
        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        brDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        trSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        brSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset all motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake motors when they have zero power
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        //Initializing odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

//        odo.setPosition(new Pose2D(DistanceUnit.MM, -154, 104, RADIANS, 0));
        //135 * 3.141592653589793238462643383 / 180
    }

    public void updateMotorPowers(int waitAfter) {
        updateMotorPowers();

        // Wait (to allow the action done to be performed)
        sleep(waitAfter);
    }

    public void updateMotorPowers() {
        flDrive.setPower(flDrivePower / driveSlowdown);
        frDrive.setPower(frDrivePower / driveSlowdown);
        blDrive.setPower(blDrivePower / driveSlowdown);
        brDrive.setPower(brDrivePower / driveSlowdown);

//        leftSlide.setPower(slidePower);
//        rightSlide.setPower(slidePower);

        intake.setPower(intakePower);

        leftShoot.setPower(shootPower);
        rightShoot.setPower(-shootPower);

        tlSpinner.setPower(topSpinnerPower);
        trSpinner.setPower(topSpinnerPower);

        blSpinner.setPower(bottomSpinnerPower);
        brSpinner.setPower(bottomSpinnerPower);
    }

    public void updateTelemetry() {
//        telemetry.addData("FL Drive Power: ", flDrivePower);
//        telemetry.addData("FR Drive Power: ", frDrivePower);
//        telemetry.addData("BL Drive Power: ", blDrivePower);
//        telemetry.addData("BR Drive Power: ", brDrivePower);
//        telemetry.addData("Drive Slowdown: ", driveSlowdown);
//        telemetry.addData("Drive Mode: ", driveMode);
        telemetry.addData("Angle: ", imu.getRobotYawPitchRollAngles().getYaw(DEGREES));
        telemetry.addData("odoAngle: ", odo.getHeading(DEGREES));
//        telemetry.addData("Slide Power: ", slidePower);
//        telemetry.addData("Camera: ", camera);
//        telemetry.addData("Intake Power: ", intakePower);
//        telemetry.addData("Shoot Power: ", shootPower);
//        telemetry.addData("Shoot Mode: ", shootingMode);
//        telemetry.addData("Top Spinner Power: ", topSpinnerPower);
//        telemetry.addData("Bottom Spinner Power: ", bottomSpinnerPower);
        telemetry.addData("Position: ", odo.getPosition());
//        telemetry.addData("Power for shoots: ", distToPower(45));
//        telemetry.addData("Actual power: ", leftShoot.getPower());

        telemetry.update();
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        camera.update();
        odo.update();

        setLock(gp2);

        if (locked) {
            mainDrive(0, 0, 0, 1, 1);
            spinnerControl(0.0);
//            slideControl(-0.2);
            intakeControl(0.0);
            shootControl(0.0);
        } else {
            setDrivingMode(gp1);
            if (gp1.dpad_down){
                desiredPose = odo.getPosition();
            }
            if (gp1.y) {
                aimbot();
            } else if (gp1.b){
                staybot();
            } else {
                if (driveMode == DriveMode.NORMAL) {
                    mainDrive(gp1);
                } else {
                    mainGyroDrive(gp1);
                }
            }
            if (gp1.left_bumper && gp1.right_bumper){
                resetOdo();
            }
            spinnerControl(0.0);
            intakeControl(gp2);
            shootControl(gp2);
            spinnerControl(gp2);
            bottomSpinnerOverride(gp2);
        }

        // Update state
        updateMotorPowers();

        // Update telemetry
        updateTelemetry();
    }

    public void setDrivingMode(Gamepad gp) {
        if (gp.x) {
            if (driveModeDebounce.milliseconds() < 500) return;

            driveModeDebounce.reset();

            if (driveMode == DriveMode.NORMAL) {
                setDrivingMode(DriveMode.GYRO);
            } else {
                setDrivingMode(DriveMode.NORMAL);
            }
        }
    }

    public void setDrivingMode(DriveMode mode) {
        driveMode = mode;
    }

    public void mainDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = gp.left_stick_x;
        double slowDownFactor = gp.left_trigger;
        double speedUpFactor = gp.right_trigger;

        mainDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        flDrivePower = drive + strafe + turn;
        frDrivePower = drive - strafe - turn;
        blDrivePower = drive - strafe + turn;
        brDrivePower = drive + strafe - turn;

        if (slowDownFactor > 0.5 && speedUpFactor > 0.5) {
            driveSlowdown = 2;
        } else if (slowDownFactor > 0.5) {
            driveSlowdown = 4;
        } else if (speedUpFactor > 0.5) {
            driveSlowdown = 1;
        } else {
            driveSlowdown = 2;
        }
    }

    public void mainGyroDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = gp.left_stick_x;
        double slowDownFactor = gp.left_trigger;
        double speedUpFactor = gp.right_trigger;

        if (gp.a) {
            imu.resetYaw();
        }

        mainGyroDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainGyroDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        double imuPos = imu.getRobotYawPitchRollAngles().getYaw(RADIANS);

        double cosine = Math.cos(-imuPos);
        double sine = Math.sin(-imuPos);

        double driveCos = cosine * drive;
        double driveSin = sine * drive;
        double strafeCos = cosine * strafe;
        double strafeSin = sine * strafe;

        double actualDrive = (driveCos + strafeSin) / 1.5;
        double actualStrafe = strafeCos - driveSin;

        mainDrive(actualDrive, turn, actualStrafe, slowDownFactor, speedUpFactor);
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

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

//            flDrivePower = flDrivePower / driveSlowdown;
//            frDrivePower = frDrivePower / driveSlowdown;
//            blDrivePower = blDrivePower / driveSlowdown;
//            brDrivePower = brDrivePower / driveSlowdown;
        } else {
            brake();
        }
    }

    public void staybot(){

        double xmm = desiredPose.getX(DistanceUnit.MM);
        double ymm = desiredPose.getY(DistanceUnit.MM);
        double error = 15.0;
        double errorButMore = 225.0 + error;
        odo.update();

        if (Math.abs(xmm - odo.getPosX(DistanceUnit.MM)) > error || Math.abs(ymm - odo.getPosY(DistanceUnit.MM)) > error ) {
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

            flDrivePower = (drivePwr + strafePwr) / 1.5;
            frDrivePower = (drivePwr - strafePwr) / 1.5;
            blDrivePower = (drivePwr - strafePwr) / 1.5;
            brDrivePower = (drivePwr + strafePwr) / 1.5;

            // Slowdown
//            flDrivePower = flDrivePower / driveSlowdown;
//            frDrivePower = frDrivePower / driveSlowdown;
//            blDrivePower = blDrivePower / driveSlowdown;
//            brDrivePower = brDrivePower / driveSlowdown;
        } else {
            brake();
        }

    }

//    public void slideControl(Gamepad gp) {
//        boolean contract = gp.left_bumper && !gp.right_bumper;
//        boolean extend = gp.right_bumper && !gp.left_bumper;
//
//        if (contract) {
//            slideControl(1.0);
//        } else if (extend) {
//            slideControl(-1.0);
//        } else {
//            slideControl(0.0);
//        }
//    }

//    public void slideControl(double power) {
//        double clampedPower;
//
//        if ((power < 0 && leftSlide.getCurrentPosition() > -2140) || (power > 0 && leftSlide.getCurrentPosition() < 0)) {
//            clampedPower = power;
//        } else {
//            clampedPower = 0.0;
//        }
//
//        double shiftedPower = clampedPower + 0.2; // some contractive power always necessary
//
//        slidePower = Math.min(Math.max(shiftedPower, -1.0), 1.0); // clamp between -1 to 1
//    }

    public void intakeControl(Gamepad gp) {
        if (gp.a) {
            intakeControl(1.0);
            spinnerControl(1.0);
        } else if (gp.x) {
            intakeControl(1.0);
            spinnerControl(0.0, 1.0);
        } else {
            intakeControl(0.0);
        }
    }

    public void intakeControl(double power) {
        intakePower = power;
    }

    public void shootControl(Gamepad gp) {
        if (gp.dpad_down) {
            shootingMode = ShootingMode.LOW;
        } else if (gp.dpad_up) {
            shootingMode = ShootingMode.HIGH;
        } else if (gp.dpad_left || gp.dpad_right){
            shootingMode = ShootingMode.MID;
        }

        if (gp.right_trigger > 0.2) {
            if (shootingMode == ShootingMode.LOW) {
                shootControl(0.29);//was 0.37
//                shootControl(distToPower(46));
            } else if (shootingMode == ShootingMode.HIGH){
                shootControl(0.35);//was 0.45
            } else {
                shootControl(0.32);//was 0.42
            }
        } else if (gp.left_trigger > 0.2) {
            shootControl(-0.8);
            spinnerControl(-1.0);
            intakeControl(-1.0);
        } else {
            shootControl(0.0);
        }
    }

    public void shootControl(double power) {
        shootPower = power;
    }

    public void spinnerControl(Gamepad gp) {
        if (Math.abs(gp.left_stick_y) > 0.2) {
            spinnerControl(-gp.left_stick_y);
        }
    }

    public void bottomSpinnerOverride(Gamepad gp) {
        if (gp.y) {
            spinnerControl(-1.0, bottomSpinnerPower);
        }
    }

    public void spinnerControl(double power) {
        topSpinnerPower = power;
        bottomSpinnerPower = power;
    }

    public void spinnerControl(double top, double bottom) {
        topSpinnerPower = top;
        bottomSpinnerPower = bottom;
    }

    public void setLock(Gamepad gp) {
//        if (gp.y) {
//            if (lockDebounce.milliseconds() < 500) return;
//
//            lockDebounce.reset();
//
//            locked = !locked;
//        }
    }

    public void driveTo(int pos) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        while ((Math.abs(pos - flDrive.getCurrentPosition()) + Math.abs(pos - frDrive.getCurrentPosition()) / 2) > 20) {
            // Drive
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            if (Math.abs(pos - flDrive.getCurrentPosition()) > 150 && Math.abs(pos - frDrive.getCurrentPosition()) > 150) {
                flDrivePower = Math.signum(flDrivePower) * 0.9;
                frDrivePower = Math.signum(frDrivePower) * 0.9;
                blDrivePower = Math.signum(blDrivePower) * 0.9;
                brDrivePower = Math.signum(brDrivePower) * 0.9;
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);
            }

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

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

        final int DELAY = 20;

        while (((Math.abs(pos - flDrive.getCurrentPosition()) + Math.abs(pos - frDrive.getCurrentPosition())) / 2) > 20) {
            // Drive
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            flDrivePower = (flDrivePower / slowdown) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower / slowdown) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower / slowdown) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower / slowdown) + (Math.signum(brDrivePower) * 0.1);

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setY(int pos, double slowdown) {
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

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();
            odo.update();

            telemetry.addData("Pos", odo.getPosY(DistanceUnit.MM));
            telemetry.update();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeTo(int pos) {
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

        while (Math.abs((Math.abs(flBrPos - flDrive.getCurrentPosition()) + Math.abs(frBlPos - frDrive.getCurrentPosition())) / 2) > 20) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(flBrPos);
            blDrivePower = (double) blDistance / (double) Math.abs(frBlPos);
            frDrivePower = (double) frDistance / (double) Math.abs(frBlPos);
            brDrivePower = (double) brDistance / (double) Math.abs(flBrPos);

            if (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 100 || Math.abs(flBrPos - flDrive.getCurrentPosition()) > 100) {
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

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setX(int pos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
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

            if (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 100 || Math.abs(flBrPos - flDrive.getCurrentPosition()) > 100) {
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
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
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

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();
            odo.update();

            telemetry.addData("pos", odo.getPosX(DistanceUnit.MM));
            telemetry.update();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            updateMotorPowers();
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
    public void driveAndStrafe(int drivePos, int strafePos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int flBrPos = (int) (1.5 * (double) strafePos + (double) drivePos);
        int frBlPos = (int) (-1.5 * (double) strafePos + (double) drivePos);

        while (Math.abs((Math.abs(flBrPos - flDrive.getCurrentPosition()) + Math.abs(frBlPos - frDrive.getCurrentPosition())) / 2) > 15) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(flBrPos);
            blDrivePower = (double) blDistance / (double) Math.abs(frBlPos);
            frDrivePower = (double) frDistance / (double) Math.abs(frBlPos);
            brDrivePower = (double) brDistance / (double) Math.abs(flBrPos);

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);

            // Correct angle
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / (180);

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

            // Slowdown
            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void angularStrafe(int degrees, int distance) {
        //Strafes the robot distance in "degrees" direction
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angleInRadians = Math.toRadians(degrees + 315); //0 deg, would strafe right, 90 deg would strafe up, etc.
        double cosValue = Math.cos(angleInRadians);
        double sinValue = Math.sin(angleInRadians);
        double yDist = distance * sinValue + odo.getPosY(DistanceUnit.MM);
        double xDist = distance * cosValue + odo.getPosX(DistanceUnit.MM);

        final double error = 100;
        final double slowdown = 1.8;

        while (Math.abs(yDist - odo.getPosY(DistanceUnit.MM)) > error || Math.abs(xDist - odo.getPosX(DistanceUnit.MM)) > error) {

            enableAllMotors(cosValue / 1.5, sinValue);
            updateMotorPowers();
        }

    }
    private void enableAllMotors(final double p1, final double p2) {
        //abstraction, sets power to the TL & BR motors for p1, and TR & BL motors for p2
        frDrivePower = p2;
        brDrivePower = p1;
        flDrivePower = p1;
        blDrivePower = p2;
    }
    public void turnToFromHere(double target) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 3) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnTo(double target) {
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        double error = target - currentPosition;

        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            updateMotorPowers();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.imu.resetYaw();
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    public void resetGyro() {
        this.imu.resetYaw();
    }

    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        flDrive.setPower(0.0);
        blDrive.setPower(0.0);
        frDrive.setPower(0.0);
        brDrive.setPower(0.0);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException _ignored) {
        }
    }


    public double distToVelocity(double theta) {
        //Given theta (The angle at which the artifact shoots at) return the velocity needed to reach the basket
        double x = odo.getPosX(DistanceUnit.MM) / 1000 - 1.3;
        double y = odo.getPosY(DistanceUnit.MM) / 1000;
        double dist = Math.sqrt(x * x + y * y);
        double secant = 1 / Math.cos(Math.toRadians(theta));
        double numerator = -9.8 * dist * dist;
        double denominator = 2 * (.9845 - dist * Math.tan(Math.toRadians(theta)));
        return secant * Math.sqrt(numerator / denominator);
    }

    public double distToPower(double theta){
        //Given dist to goal, return power necessary to shoot it in
        int maxRPM = 6000;
        return 60 * distToVelocity(theta) / (3.1415926535 * 0.05 * maxRPM);
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

    public enum DriveMode {
        NORMAL,
        GYRO
    }

    public enum ShootingMode {
        LOW,
        MID,
        HIGH
    }

    public enum side {
        BLUE,
        RED
    }

}
