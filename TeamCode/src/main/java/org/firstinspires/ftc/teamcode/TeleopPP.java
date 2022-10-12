package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.checkerframework.checker.units.qual.Current;

@TeleOp(name = "MainTeleopPP", group = "")

public class TeleopPP extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private double FilteredXR, LastFilteredXR, FilteredXL, LastFilteredXL, FilteredYL, LastFilteredYL;
    private Servo claw;
    public static final double NEW_P = 20;
    public static final double NEW_I = 15;
    public static final double NEW_D = 0;
    public static final double NEW_F = 5;
    boolean gamepad2xIsPressed;
    boolean gamepad2yIsPressed;
    public boolean slowmode = true;
    boolean gamepad1aispressed;
    private State CurrentState;
    private RevBlinkinLedDriver blinkin;
    private DigitalChannel RedLED2;
    private DigitalChannel GreenLED2;
    private DigitalChannel RedLED;
    private DigitalChannel GreenLED;
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;
    private int level = 0;
    private boolean palm;
    private ElapsedTime TimerB;
    private double grabberClose = 0.3;
    private double grabberOpen = 0.5;
    private boolean lifterHome = false;




    private enum State {
        DEFAULT,
        DROP,

    }

    @Override
    public void runOpMode() {

        TimerB = new ElapsedTime();

        palm = false;

        CurrentState = State.DEFAULT;

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        LastFilteredXL = 0;
        LastFilteredXR = 0;
        LastFilteredYL = 0;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        claw = hardwareMap.get(Servo.class, "claw");

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");

        GreenLED = hardwareMap.get(DigitalChannel.class, "green");
        RedLED = hardwareMap.get(DigitalChannel.class, "red");
        GreenLED2 = hardwareMap.get(DigitalChannel.class, "green2");
        RedLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        RedLED.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLED.setMode(DigitalChannel.Mode.OUTPUT);
        RedLED2.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLED2.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLED.setState(false);
        GreenLED2.setState(false);
        RedLED.setState(false);
        RedLED2.setState(false);

        waitForStart();


        while (opModeIsActive()) {

            SmoothingFunction();

            if (gamepad2.a && !gamepad1aispressed){
                if (slowmode) {
                    slowmode = false;
                } else {
                    slowmode = true;
                }
            }
            gamepad1aispressed = gamepad2.a;

            if (slowmode) {

                FilteredYL = 0.3 * gamepad1.left_stick_y + 0.7 * LastFilteredYL;
                FilteredXL = 0.3 * gamepad1.left_stick_x + 0.7 * LastFilteredXL;
                FilteredXR = 0.3 * gamepad1.right_stick_x + 0.7 * LastFilteredXR;


                leftFront.setPower(0.6 * ((FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR))));
                rightFront.setPower(0.6 * ((-(FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR)))));
                leftRear.setPower(0.6 * (((FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR)))));
                rightRear.setPower(0.6 * (-(FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));

                LastFilteredYL = FilteredYL;
                LastFilteredXL = FilteredXL;
                LastFilteredXR = FilteredXR;

            } else {
                leftFront.setPower(1 * ((gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y) - (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x) - (0.75 * (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x))));
                rightFront.setPower(1 * (-(gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y) - (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x) - (0.75 * (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x))));
                leftRear.setPower(1 * ((gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y) + (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x) - (0.75 * (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x))));
                rightRear.setPower(1 * (-(gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y) + (gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x) - (0.75 * (gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x))));

            }

            andrewState();

        }

    }

    /*private void leapFrog(){
        if (gamepad2.dpad_up) {

        }
    }*/

    private void SmoothingFunction(){
        if (gamepad2.x && !gamepad2xIsPressed){
            FilteredYL = 0.1 * gamepad1.left_stick_y + 0.9 * LastFilteredYL;
            FilteredXL = 0.1 * gamepad1.left_stick_x + 0.9 * LastFilteredXL;
            FilteredXR = 0.1 * gamepad1.right_stick_x + 0.9 * LastFilteredXR;


            leftFront.setPower(1 * (FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR)));
            rightFront.setPower(1 * (-(FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR))));
            leftRear.setPower(1 * ((FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));
            rightRear.setPower(1 * (-(FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));

            LastFilteredYL = FilteredYL;
            LastFilteredXL = FilteredXL;
            LastFilteredXR = FilteredXR;

        } else if (gamepad2.y && !gamepad2yIsPressed){
            FilteredYL = 0.1 * gamepad1.left_stick_y + 0.9 * LastFilteredYL;
            FilteredXL = 0.1 * gamepad1.left_stick_x + 0.9 * LastFilteredXL;
            FilteredXR = 0.1 * gamepad1.right_stick_x + 0.9 * LastFilteredXR;

            leftFront.setPower(-1 * (FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR)));
            rightFront.setPower(-1 * (-(FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR))));
            leftRear.setPower(-1 * ((FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));
            rightRear.setPower(-1 * (-(FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));

            LastFilteredYL = FilteredYL;
            LastFilteredXL = FilteredXL;
            LastFilteredXR = FilteredXR;

        }
        gamepad2xIsPressed = gamepad2.x;
        gamepad2yIsPressed = gamepad2.y;
    }

    private void MecanumFunction(double YL, double XL, double XR){
        leftFront.setPower((YL - XL + XR));
        rightFront.setPower((YL + XL - XR));
        leftRear.setPower((YL + XL + XR));
        rightRear.setPower((YL - XL - XR));

        FilteredYL = 0.1 * gamepad1.left_stick_y + 0.9 * LastFilteredYL;
        FilteredXL = 0.1 * gamepad1.left_stick_x + 0.9 * LastFilteredXL;
        FilteredXR = 0.1 * gamepad1.right_stick_x + 0.9 * LastFilteredXR;

        LastFilteredYL = FilteredYL;
        LastFilteredXL = FilteredXL;
        LastFilteredXR = FilteredXR;
    }


    private void andrewState() {
        switch (CurrentState) {
            case DEFAULT:
                if (!palm) {

                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                    claw.setPosition(grabberOpen);

                    telemetry.addData("distance", ((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
                    telemetry.addData("timer", TimerB);
                    telemetry.update();

                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 6)) {
                        TimerB.reset();

                        telemetry.addData("cone?!?!?!?!?!??!?!?!??!", "None");
                    }

                    if (TimerB.milliseconds() >= 500) {
                        palm = true;
                        lifterHome = false;
                    }

                    if (TimerB.milliseconds() >= 250) {
                        claw.setPosition(grabberClose);
                    } else {
                        claw.setPosition(grabberOpen);
                    }

                } else {
                    CurrentState = State.DROP;
                }
            case DROP:
                if (palm) {

                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                    /*if (gamepad2.dpad_up) {
                        level = level + 1;
                    } else if (gamepad2.dpad_down) {
                        level = level - 1;
                    }

                    if (level == 0) {
                        lifter.setPosition(groundLevel);
                    } else if (level == 1) {
                        lifter.setPosition(low);
                    } else if (level == 2) {
                        lifter.setPosition(mid);
                    } else if (level == 3) {
                        lifter.setPosition(high);
                    }*/

                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 6)) {
                        TimerB.reset();

                        telemetry.addData("Palm?", "None");
                    }
                    if (TimerB.milliseconds() >= 500) {
                        palm = false;

                    }
                    if (TimerB.milliseconds() >= 250) {
                        palm = false;
                    }

                    if (gamepad2.b) {
                        claw.setPosition(grabberOpen);
                    }


                } else {
                    CurrentState = State.DEFAULT;
                }
        }
    }


}
