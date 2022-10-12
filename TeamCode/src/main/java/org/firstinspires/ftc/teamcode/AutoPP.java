package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;


@Autonomous(name = "AUTO", group = "")

public class AutoPP extends LinearOpMode {
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;
    private static final String VUFORIA_KEY =
            "Ae2mEyz/////AAABmQBmoTE94ki5quwzTT/OlIIeOueUfjuHL/5k1VNWN943meU2RmiXCJ9eX3rUR/2CkwguvbBU45e1SzrbTAwz3ZzJXc7XN1ObKk/7yPHQeulWpyJgpeZx+EqmZW6VE6yG4mNI1mshKI7vOgOtYxqdR8Yf7YwBPd4Ruy3NVK01BwBl1F8V/ndY26skaSlnWqpibCR3XIvVG0LXHTdNn/ftZyAFmCedLgLi1UtNhr2eXZdr6ioikyRYEe7qsWZPlnwVn5DaQoTcgccZV4bR1/PEvDLn7jn1YNwSimTC8glK+5gnNpO+X7BiZa5LcqtYEpvk/QNQda0Fd+wHQDXA8ojeMUagawtkQGJvpPpz9c6p4fad";
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Servo claw;
    private CRServo Flipper;
    public double grabberClose = 0;
    public double grabberOpen = 1;
    double currentX, currentY;
    private RevBlinkinLedDriver blinkin;
    private String position;


    @Override
    public void runOpMode() {
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        claw = hardwareMap.get(Servo.class, "claw");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        waitForStart();

        if(isStopRequested()) {
            //run code
        }

        /*private void MecanumFunction(double YL, double XL, double XR){
            leftFront.setPower(0.8 * (YL - XL + XR));
            rightFront.setPower(0.8 * (YL + XL - XR));
            leftRear.setPower(0.8 * (YL + XL + XR));
            rightRear.setPower(0.8 * (YL - XL - XR));
        }*/
    }
}
