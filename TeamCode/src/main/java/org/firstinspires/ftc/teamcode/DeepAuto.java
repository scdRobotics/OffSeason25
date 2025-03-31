package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import android.bluetooth.BluetoothClass;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//this is a copy of the BasicAutonomous from Into the Deep
//I'll probably add to / edit the actual running part
@Autonomous
public class DeepAuto extends LinearOpMode{

    final double trackwidth =  124;
    double odometryHeading;

    double finalHeading;

    double differenceInPar0;
    double differenceInPar1;
    double differenceInPerp;
    double averageDifferenceInPar;
    double differenceInOdoHeading;


    double par0Pos = 0;
    double par1Pos = 0;
    double perpPos = 0;
    double par0LastPos = 0;
    double par1LastPos = 0;
    double perpLastPos = 0;

    Pose2D robotPose;

    @Override
    public void runOpMode() throws InterruptedException {

        //
        //Declare Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");


        // CRServo servo = hardwareMap.crservo.get("servo");

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        imu.initialize(parameters);

        imu.resetYaw();

        gyroSensorOrientation gyroSensorOrientation = new gyroSensorOrientation();

        Double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("robot Heading", robotHeading);
        telemetry.update();

        waitForStart();

        //first clip
        forwardAndMoveSlide(800 , 1, 2700, true);
        forward(450, 1);

        moveSlide(-800);

        backwards(250, 1);

        //push sample
        //turn(-45, false);
        turn(-40, false);
        strafe(2150);
        forward(600, 1);
        turn(-5.1, true);


        forward(900, 1);

        turn(6, true);

        backwards(1900, 1);

        //turn around
        forwardAndMoveSlide(500, 1, -1150, false);
        turn(-180, false);

        //take specimen
        forward(1300, 0.5);
        intake(0.8);
        moveSlide(1900);

        //go back
        forwardAndMoveSlide(-600, 1, 15, false);
        stopIntake();

        turn(-270, false);

        forward(1650, 1);
        turn(6, false);

        forward(1250, 1);

        moveSlide(-1000);

        //second clip

    }

    void forwardAndMoveSlide(int forwardWantedPosition, double forwardPower, int slideWantedPosition, boolean useTimer)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        ArrayList<DcMotor> slideMotors = new ArrayList<DcMotor>();
        ArrayList<CRServo> servos = new ArrayList<CRServo>();

        slideMotors.add(leftSlideMotor);
        slideMotors.add(rightSlideMotor);

        servos.add(leftSlideIntakeServo);
        servos.add(rightSlideIntakeServo);

        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(slideWantedPosition < 0)
        {
            leftSlideMotor.setTargetPosition(slideWantedPosition);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setPower(1);
            leftSlideMotor.setPower(-1);
        }
        else
        {
            leftSlideMotor.setTargetPosition(slideWantedPosition);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setPower(-1);
            leftSlideMotor.setPower(1);
        }

        boolean slideHasFinished = false;
        boolean wheelsHaveFinished = false;
        boolean hasStartedMotors = false;



        double startTime = System.currentTimeMillis();
        while(opModeIsActive())
        {
            trackOdometry();

            if((System.currentTimeMillis() - startTime < 1000 || !useTimer) && !hasStartedMotors) {
                for (DcMotor motor : motors) {

                    motor.setTargetPosition(-forwardWantedPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(forwardPower);
                }
                hasStartedMotors = true;
            }



            if((Math.abs(Math.abs(frontRightMotor.getCurrentPosition()) - Math.abs(frontRightMotor.getTargetPosition())) < 13))
            {
                for(DcMotor motor : motors)
                {
                    motor.setPower(0);
                }
                wheelsHaveFinished = true;
            }
            if(Math.abs(Math.abs(frontLeftMotor.getCurrentPosition()) - Math.abs(frontLeftMotor.getTargetPosition())) < 13)
            {
                for(DcMotor motor : motors)
                {
                    motor.setPower(0);
                }
                wheelsHaveFinished = true;
            }

            if(Math.abs(Math.abs(leftSlideMotor.getCurrentPosition()) - Math.abs(slideWantedPosition)) < 10 || Math.abs(leftSlideMotor.getCurrentPosition()) > Math.abs(slideWantedPosition))
            {
                rightSlideMotor.setPower(0);

                leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition());
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                stopIntake();
                slideHasFinished = true;
            }

            telemetry.addData("frontRightMotor position", frontRightMotor.getCurrentPosition());

            telemetry.addData("rightWheelPosition", frontRightMotor.getCurrentPosition());

            telemetry.addData("frontRightMotor desired position", frontRightMotor.getTargetPosition());

            telemetry.addData("left Slide position", leftSlideMotor.getCurrentPosition());
            telemetry.addData("left Slide wanted position", slideWantedPosition);

            telemetry.addData("wheelsHaveFinished", wheelsHaveFinished);
            telemetry.addData("slidesHaveFinished", slideHasFinished);
            telemetry.update();


            if(wheelsHaveFinished && slideHasFinished)
            {
                break;
            }
        }
    }

    void turn(double desiredOrientation, boolean useTimer)
    {
        double robotHeading;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        long startTime = System.currentTimeMillis();



        while(opModeIsActive())
        {
            trackOdometry();
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            gyroSensorOrientation.autoOrient(robotHeading, desiredOrientation, 0, frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor, telemetry);

            if (gyroSensorOrientation.phase == "normal")
            {
                break;
            }

            if(System.currentTimeMillis() - startTime > 3000 && useTimer)
            {
                break;
            }

            telemetry.addData("turn time",  System.currentTimeMillis() - startTime);
            telemetry.addData("time left", 3000 - (System.currentTimeMillis() - startTime));

            telemetry.addData("robotHeading", robotHeading);
        }
    }

    void strafe(int desiredPosition)
    {
        double robotHeading;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        frontLeftMotor.setTargetPosition(desiredPosition);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0.4);

        backLeftMotor.setTargetPosition(-desiredPosition);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(0.4);

        frontRightMotor.setTargetPosition(desiredPosition);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(0.4);

        backRightMotor.setTargetPosition(-desiredPosition);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(0.4);


        long startTime = System.currentTimeMillis();

        while(opModeIsActive())
        {
            telemetry.addData("Is Strafing", "");
            telemetry.update();

            trackOdometry();
            double frontRight = Math.abs(frontRightMotor.getCurrentPosition());
            double frontLeft = Math.abs(frontLeftMotor.getCurrentPosition());
            double backRight = Math.abs(backRightMotor.getCurrentPosition());
            double backLeft = Math.abs(backLeftMotor.getCurrentPosition());
            double desiredPos = Math.abs(desiredPosition);

            double currentOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if(frontRight - desiredPos > 5 && frontLeft - desiredPos > 5 && backRight - desiredPos > 5 && backLeft - desiredPos > 5)
            {
                break;
            }

            /*if(System.currentTimeMillis() - startTime > 2000)
            {
                break;
            }*/
        }
    }

    void forward(int desiredPosition, double power)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor.setTargetPosition(-desiredPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while(opModeIsActive())
        {
            trackOdometry();
            telemetry.addData("frontRightMotor position", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor desired position", frontRightMotor.getTargetPosition());
            telemetry.update();

            if(Math.abs(Math.abs(frontRightMotor.getCurrentPosition()) - Math.abs(frontRightMotor.getTargetPosition())) < 5)
            {
                break;
            }
            if(Math.abs(Math.abs(frontLeftMotor.getCurrentPosition()) - Math.abs(frontLeftMotor.getTargetPosition())) < 5)
            {
                break;
            }

        }

    }



    void intake(double power)
    {
        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        ArrayList<CRServo> servos = new ArrayList<CRServo>();

        servos.add(leftSlideIntakeServo);
        servos.add(rightSlideIntakeServo);

        leftSlideIntakeServo.setDirection(CRServo.Direction.REVERSE);
        rightSlideIntakeServo.setDirection(CRServo.Direction.FORWARD);

        for(CRServo servo : servos)
        {
            servo.setPower(power);
        }
    }

    void stopIntake()
    {
        trackOdometry();
        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        ArrayList<CRServo> servos = new ArrayList<CRServo>();

        servos.add(leftSlideIntakeServo);
        servos.add(rightSlideIntakeServo);

        for(CRServo servo : servos)
        {
            servo.setPower(0);
        }
    }

    void backwards(int desiredPosition, double power)
    {
        boolean isFinished = false;
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(frontLeftMotor);
        motors.add(frontRightMotor);
        motors.add(backLeftMotor);
        motors.add(backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            motor.setTargetPosition(desiredPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while(opModeIsActive())
        {
            trackOdometry();
            telemetry.addData("frontRightMotor position", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontRightMotor desired position", frontRightMotor.getTargetPosition());
            telemetry.addData("going backwards", "");
            telemetry.update();

            if(Math.abs(Math.abs(frontRightMotor.getCurrentPosition()) - Math.abs(frontRightMotor.getTargetPosition())) < 5)
            {
                break;
            }
            if(Math.abs(Math.abs(frontLeftMotor.getCurrentPosition()) - Math.abs(frontLeftMotor.getTargetPosition())) < 5)
            {
                break;
            }
        }
    }

    void moveSlide(int desiredPosition)
    {
        DcMotor leftSlideMotor = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlideMotor = hardwareMap.dcMotor.get("rightSlide");

        CRServo leftSlideIntakeServo = hardwareMap.crservo.get("leftSlideIntakeServo");
        CRServo rightSlideIntakeServo = hardwareMap.crservo.get("rightSlideIntakeServo");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        ArrayList<CRServo> servos = new ArrayList<CRServo>();

        motors.add(leftSlideMotor);
        motors.add(rightSlideMotor);

        servos.add(leftSlideIntakeServo);
        servos.add(rightSlideIntakeServo);

        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(desiredPosition < 0)
        {

            rightSlideMotor.setPower(0.7);

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlideMotor.setPower(-0.7);

            intake(0.15);
        }
        else
        {
            leftSlideMotor.setTargetPosition(desiredPosition);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setPower(-1);
            leftSlideMotor.setPower(1);
        }


        while(opModeIsActive())
        {
            trackOdometry();
            telemetry.addData("LeftSlideMotor position", leftSlideMotor.getCurrentPosition());
            telemetry.addData("LeftSlide desired Position", leftSlideMotor.getTargetPosition());
            telemetry.addData("Left Slide power", leftSlideMotor.getPower());


            telemetry.addData("leftSideDistance", Math.abs(Math.abs(leftSlideMotor.getCurrentPosition()) - Math.abs(leftSlideMotor.getTargetPosition())));
            telemetry.update();

            if(Math.abs(Math.abs(leftSlideMotor.getCurrentPosition()) - Math.abs(desiredPosition)) < 40)
            {
                rightSlideMotor.setPower(0);

                leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition());
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                stopIntake();
                break;
            }

        }
    }

    void GoToPose(Pose2D currentPose, Pose2D wantedPose)
    {
        // if(currentPose.getX())
    }
    int i = 0;
    void trackOdometry()
    {

        DcMotor par0 = hardwareMap.dcMotor.get("bucketSlideMotor");
        DcMotor par1 = hardwareMap.dcMotor.get("par1");
        DcMotor perp = hardwareMap.dcMotor.get("rightSlide");
        if(i==0)
        {
            par0.setDirection(DcMotorSimple.Direction.REVERSE);
            par1.setDirection(DcMotorSimple.Direction.REVERSE);

            par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robotPose = new Pose2D(INCH, 0, 0, DEGREES, 0);
        }
        else
        {
            telemetry.addData("odometry has been set", "");

            i++;
        }

        par0Pos = par0.getCurrentPosition();
        par1Pos = par1.getCurrentPosition();
        perpPos = perp.getCurrentPosition();

        odometryHeading = (par0Pos - par1Pos) / trackwidth;

        differenceInPar0 = par0Pos - par0LastPos;
        differenceInPar1 = par1Pos - par1LastPos;

        averageDifferenceInPar = (differenceInPar0 - differenceInPar1) * 2;

        differenceInOdoHeading = (differenceInPar0 - differenceInPar1) / trackwidth;

        differenceInPerp = perpPos - perpLastPos;

        robotPose = new Pose2D(INCH, robotPose.getX(INCH) + (differenceInPar0 * differenceInPar1 < 0 ? 0 : averageDifferenceInPar), robotPose.getY(INCH) + (differenceInPar0 * differenceInPar1 < 0 ? 0 : differenceInPerp), DEGREES, robotPose.getHeading(DEGREES) + differenceInOdoHeading);

        par0LastPos = par0Pos;
        par1LastPos = par1Pos;
        perpLastPos = perpPos;

        telemetry.addData("RobotPose x val", robotPose.getX(INCH));
        telemetry.addData("RobotPose y val", robotPose.getY(INCH));
        telemetry.addData("RobotPose Heading", robotPose.getHeading(DEGREES));
    }
}
