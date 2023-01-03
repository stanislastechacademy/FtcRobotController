package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Failure 3 Electric Boogaloo", group = "Concept")

public class AutonFinalMethodTest extends LinearOpMode {
    
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor;
    private Servo intakeServo;


    @Override
    public void runOpMode() {
        DrivingMethods drivingMethods = new DrivingMethods();
        if (drivingMethods == null) {
            idle();
        } else {
            drivingMethods.driveForward(2000);
        }
    }
}
