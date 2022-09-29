package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.Current;

@TeleOp(name = "MainTeleop", group = "")

public class TeleopPP extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;
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

    private enum State {
        DEFAULT,
        DROP,

    }

    //hope this works

    @Override
    public void runOpMode() {

        CurrentState = State.DEFAULT;

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

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        LastFilteredXL = 0;
        LastFilteredXR = 0;
        LastFilteredYL = 0;

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

            //awesomeState();

        }

    }

    private void leapFrog(){
        if (gamepad2.dpad_up) {

        }
    }

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

    private void MoveUp() {

    }

    /*private void andrewState() {
        switch (CurrentState) {
            case DEFAULT:
                if (claw.getPosition() == 0) {
                    //have claw open, lifter at home position, led strip in green
                } else {
                    CurrentState = State.DROP;
                }
            case DROP:
                if () {
                    //have something similar to last years, where we had the bumpers determine the side it would drop (no sides u drop in the same spot
                    //BUT have it so the dPad? will determine the level, so dpadUp could be high, dpadRight would be middle, and dpadDown would be low
                    //^^dpad down can lower the arm one level and dpad up can raise it one with a max of 3, and if you hit dpad down enough it returns it to home
                } else {
                    CurrentState = State.DEFAULT;
                }
        }
    }*/


}
