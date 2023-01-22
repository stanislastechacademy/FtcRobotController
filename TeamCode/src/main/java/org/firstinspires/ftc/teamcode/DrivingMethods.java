package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "methods", group = "Concept")

public class DrivingMethods extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor;
    private Servo intakeServo;

    public void initAutonomous(HardwareMap hardwareMap) {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor1");
        intakeServo = hardwareMap.get(Servo.class, "Intake");

        leftBack.setDirection(Direction.FORWARD);
        rightBack.setDirection(Direction.FORWARD);
        leftFront.setDirection(Direction.FORWARD);
        rightFront.setDirection(Direction.FORWARD);
        armMotor.setDirection(Direction.REVERSE);
    }
    public void drive(int leftBackTicks, int rightBackTicks, int leftFrontTicks, int rightFrontTicks) {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(leftBackTicks);
        rightBack.setTargetPosition(rightBackTicks);
        leftFront.setTargetPosition(leftFrontTicks);
        rightFront.setTargetPosition(rightFrontTicks);

        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
            idle();
        }
    }
    public void driveLeft(int Ticks) {
        drive(Ticks, Ticks, -Ticks, -Ticks);
    }
    public void driveRight(int Ticks) {
        drive(-Ticks, -Ticks, Ticks, Ticks);
    }
    public void driveForward(int Ticks) {
        drive(Ticks, -Ticks, Ticks, -Ticks);
    }
    public void driveBackward(int Ticks) {
        drive(-Ticks, Ticks, -Ticks, Ticks);
    }
    public void turnLeft (int Ticks) {drive(Ticks, Ticks, Ticks, Ticks); }
    public void turnRight (int Ticks) {drive (-Ticks, -Ticks, -Ticks, -Ticks); }
    public void armMovement(int armTicks) {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(armTicks);
        armMotor.setPower(0.5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()) {
            idle();
        }
    }
    public void servoPositioning(double servoPosition) {
        intakeServo.setPosition(servoPosition);
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}