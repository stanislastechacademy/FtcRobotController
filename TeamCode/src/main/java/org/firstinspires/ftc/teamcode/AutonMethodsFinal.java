package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.DrivingMethods;

@Autonomous(name = "Autonomous Dean", group = "Concept")

public class AutonMethodsFinal extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor;
    private Servo intakeServo;

    public void initAutonomous() {
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
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

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
    public void turnLeft (int degrees) {int Ticks = degrees * 70 / 6; drive(-Ticks, -Ticks, -Ticks, -Ticks); }
    public void turnRight (int degrees) {int Ticks = degrees * 70 / 6; drive (Ticks, Ticks, Ticks, Ticks); }
    public void armMovement(int armTicks) {
        armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        armMotor.setTargetPosition(armTicks);
        armMotor.setPower(0.5);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor.isBusy()) {
            idle();
        }
    }
    public void servoPositioning(double servoPosition) {intakeServo.setPosition(servoPosition);} /*pos 0.3 = closed, pos 0.5 = open*/
    public void armGet (int stacknum) {
        int[][] stack = { {400, 350}, {400, 320} };
        servoPositioning(0.5);
        armMovement(stack[stacknum][0]);
        driveForward(stack[stacknum][1]);
        servoPositioning(0.3);
        armMovement(stack[stacknum][0]);
        driveBackward(stack[stacknum][1]);
        armMovement(-(stack[stacknum][0] + 400));
    }
    public void armDrop (int height) {
        armMovement(height);
        driveForward(350);
        armMovement(-height);
        servoPositioning(0.5);
        driveBackward(380);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initAutonomous();
        waitForStart();
        servoPositioning(0.3);
        armGet(1);
//        driveForward(1600);
//        driveLeft(750);
//        armMovement(2100);
//        driveForward(350);
//        armMovement(-2100);
//        servoPositioning(0.5);
//        driveBackward(380);
//        driveRight(2400);
//        driveForward(600);
//        turnRight(37);
    }
}