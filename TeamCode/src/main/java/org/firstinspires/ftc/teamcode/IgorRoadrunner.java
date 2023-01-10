package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "IgorRoadrunner", group = "Concept")
public class IgorRoadrunner extends LinearOpMode{

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor;
    private Servo intakeServo;
    int currentHeight = 0;

    public void initAutonomous() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor1");
        intakeServo = hardwareMap.get(Servo.class, "Intake");

        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

    /** input ibtergers beschrijving. Output: real
     * beschrijf kort en krachtig de werking van de functoe
     * @param servoPosition
     */
    public void servoPositioning(double servoPosition) {intakeServo.setPosition(servoPosition);} /*pos 0.3 = closed, pos 0.5 = open*/
    public int armGet (int stackNum) {
        int[][] stack = { {0, 400}, {400, 350}, {420, 320}, {525, 300}, {625, 280} };
        servoPositioning(0.5);
        armMovement(stack[stackNum][0] - currentHeight);
        driveForward(stack[stackNum][1]);
        servoPositioning(0.3);
        armMovement(400);
        driveBackward(stack[stackNum][1]);
        return armMotor.getCurrentPosition();
    }
    public int armDrop (int junctionNum) {
        int[] junction = {150, 1100, 2100, 3000};
        armMovement(junction[junctionNum] - currentHeight);
        driveForward(350);
        armMovement(-400);
        servoPositioning(0.5);
        driveBackward(380);
        return armMotor.getCurrentPosition();
    }

    @Override
    public  void runOpMode() throws InterruptedException {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        initAutonomous();
        waitForStart();
        servoPositioning(0.3);

        /*
        TrajectorySequence PreloadedConePlusFirstCone = drivetrain.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                .lineTo(new Vector2d(36, 0))
                .lineTo(new Vector2d(34, 0))
                .waitSeconds(1)
                .lineTo(new Vector2d(36, 0))
                .lineTo(new Vector2d(36, -12))
                .lineTo(new Vector2d(56, -12))
                .waitSeconds(1)
                .build();

        TrajectorySequence ConeStack = drivetrain.trajectorySequenceBuilder(new Pose2d(56, -12, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47.5, -12, Math.toRadians(270)))
                .lineTo(new Vector2d(47.5, -14))
                //.waitSeconds(1)
                .lineTo(new Vector2d(47.5, -10))
                .lineToLinearHeading(new Pose2d(56, -12, Math.toRadians(0)))
                //.waitSeconds(1)
                .build();

        TrajectorySequence Park = drivetrain.trajectorySequenceBuilder(new Pose2d(0, -0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(180)))
                .build(); */

        TrajectorySequence test = drivetrain.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)))
                .build();

        drivetrain.followTrajectorySequence(test);
    }
}