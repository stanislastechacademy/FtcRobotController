// mr_stoffer was here
// hello there
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Gengar Driving", group="Gengar")

public class GengarDwiveTwain extends OpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor armMotor1 = null;
    private Servo intake = null;
    double intakePosition = 0.3;

    public void armMovement(int armTicks) {
        armMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        armMotor1.setTargetPosition(armTicks);
        armMotor1.setPower(0.5);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armMotor1.isBusy()) {
        }
    }
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armMotor1 = hardwareMap.get(DcMotor.class, "armmotor1");
        intake = hardwareMap.get(Servo.class, "Intake");
        
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(Servo.Direction.FORWARD);
        
        
        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop() {
    }
    
    @Override
    public void start() {
        runtime.reset();
    }
    
    @Override
    public void loop() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double armPower1;

        boolean sensitivity = gamepad1.right_bumper;
        boolean armLowJunction = gamepad1.a;

        double x = gamepad1.left_stick_x; // 1.5;
        double y = -gamepad1.left_stick_y; // 1.5;
        
        double rightX = gamepad1.right_stick_x;
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double leftTrigger = gamepad2.left_trigger;
        double rightTrigger = gamepad2.right_trigger;

        if (armLowJunction) {
            armMovement(1800);
        }
        if (sensitivity) {
            rightX = gamepad1.right_stick_x / 2;
        }
        if (gamepad2.dpad_down) {
            intakePosition = 0.3;
        }
        if (gamepad2.dpad_up) {
            intakePosition = 0.5;
        }
        leftFrontPower = (r * Math.cos(robotAngle) + rightX);
        rightFrontPower = (r * Math.sin(robotAngle) - rightX);
        leftBackPower = (r * Math.sin(robotAngle) + rightX);
        rightBackPower = (r * Math.cos(robotAngle) - rightX);
        armPower1 = rightTrigger - leftTrigger;
       
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        armMotor1.setPower(armPower1*0.65);
        intake.setPosition(intakePosition);
        
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LeftTrigger", leftTrigger);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
}
