package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class UnifiedMethods {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor1;
    private DcMotor armMotor2;
    private Servo intakeServo;
    private LinearOpMode opmode;

    //HardwareMap hardwareMap = new HardwareMap();

    int currentHeight = 0;
    int[] stackHeight = { 0, 400, 420, 525, 625};
    int[] junction = {150, 800, 1300, 1800};
    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    SampleMecanumDrive drivetrain = new SampleMecanumDrive(HardwareMap);

    TrajectorySequence slightMove = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
            .lineTo(new Vector2d(5, 0))
            .build();

    TrajectorySequence slightMoveBack = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
            .lineTo(new Vector2d(-5, 0))
            .build();

    TrajectorySequence turn90 = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
            .lineToLinearHeading(new Pose2d(0, 0, 90))
            .build();

    public void initAutonomous(HardwareMap hardwareMap, LinearOpMode opmode) {
        this.opmode = opmode;

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armMotor1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armMotor2= hardwareMap.get(DcMotor.class, "armmotor2");

        intakeServo = hardwareMap.get(Servo.class, "Intake");

        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(int leftBackTicks, int rightBackTicks, int leftFrontTicks, int rightFrontTicks, double speed) {
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

            leftBack.setPower(speed);
            rightBack.setPower(speed);
            leftFront.setPower(speed);
            rightFront.setPower(speed);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(leftBack.isBusy() || rightBack.isBusy() || leftFront.isBusy() || rightFront.isBusy()) {
                opmode.idle();
            }
        }
        public void driveBackward(int Ticks, double speed) {
            drive(Ticks, Ticks, -Ticks, -Ticks, speed);
        }
        public void driveForward(int Ticks, double speed) {
            drive(-Ticks, -Ticks, Ticks, Ticks, speed);
        }
        public void driveLeft(int Ticks, double speed) {
            drive(Ticks, -Ticks, Ticks, -Ticks, speed);
        }
        public void driveRight(int Ticks, double speed) {
            drive(-Ticks, Ticks, -Ticks, Ticks, speed);
        }
        public void turnLeft (int degrees) {int Ticks = degrees * 70 / 6; drive(-Ticks, -Ticks, -Ticks, -Ticks, 0.5); }
        public void turnRight (int degrees) {int Ticks = degrees * 70 / 6; drive (Ticks, Ticks, Ticks, Ticks, 0.5);
    }

    public void armMovement(int armTicks) {
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setTargetPosition(armTicks);
        armMotor1.setPower(0.5);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setTargetPosition(armTicks);
        armMotor2.setPower(0.5);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor1.isBusy()) {
            opmode.idle();
        }
    }

    /** input ibtergers beschrijving. Output: real
     * beschrijf kort en krachtig de werking van de functoe
     * @param servoPosition stiff
     */

    public void servoPositioning(double servoPosition) {intakeServo.setPosition(servoPosition);} /*pos 0.3 = closed, pos 0.5 = open*/
    public int armGet (int stackNum) {
        servoPositioning(0.5);
        armMovement(stackHeight[stackNum] - currentHeight);

        servoPositioning(0.3);
        armMovement(400);

        return armMotor1.getCurrentPosition();
    }

    public int armGetFirst (int stackNum) {
        servoPositioning(0.5);
        armMovement(stackHeight[stackNum] - currentHeight);
        return armMotor1.getCurrentPosition();
    }

    public int armGetSecond () {
        servoPositioning(0.3);
        armMovement(400);
        return armMotor1.getCurrentPosition();
    }

    public int armDrop (int junctionNum) {
        armMovement(junction[junctionNum] - currentHeight);
        armMovement(-400);
        servoPositioning(0.5);
        return armMotor1.getCurrentPosition();
    }

    public int armDropSecond () {
        armMovement(-400);
        servoPositioning(0.5);
        return armMotor1.getCurrentPosition();
    }

    public int armSequence (int sequenceNum) {
        armGetFirst(5 - sequenceNum);
        drivetrain.followTrajectorySequence(slightMove);
        armGetSecond();
        drivetrain.followTrajectorySequence(slightMoveBack);
        drivetrain.followTrajectorySequence(turn90);
        armMovement(junction[3] - currentHeight);
        drivetrain.followTrajectorySequence(slightMove);
        armDropSecond();
        drivetrain.followTrajectorySequence(slightMoveBack);
        drivetrain.followTrajectorySequence(turn90);
        return armMotor1.getCurrentPosition();
    }
}