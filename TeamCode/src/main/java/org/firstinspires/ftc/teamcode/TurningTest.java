package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Turning", group = "Concept")

public class TurningTest extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(1000);
        rightBack.setTargetPosition(1000);
        leftFront.setTargetPosition(1000);
        rightFront.setTargetPosition(1000);

        leftBack.setPower(Math.sin(-Math.PI/4)+1);
        rightBack.setPower(Math.cos(-Math.PI/4)-1);
        leftFront.setPower(Math.cos(-Math.PI/4)+1);
        rightFront.setPower(Math.sin(-Math.PI/4)-1);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
            idle();
        }
    }
}