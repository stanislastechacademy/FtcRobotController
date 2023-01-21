import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic af code", group="Gengar")

public class basicCode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor armMotor1 = null;
    private DcMotor armMotor2 = null;
    private Servo intake = null;
    double intakePosition = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armMotor1 = hardwareMap.get(DcMotor.class, "armmotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "armmotor2");
        intake = hardwareMap.get(Servo.class, "Intake");

        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double armPower;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        double rightX = gamepad1.right_stick_x;
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double leftTrigger = gamepad1.left_trigger;
        double rightTrigger = gamepad1.right_trigger;

        if (gamepad1.dpad_down) {
            intakePosition = 0.3;
        }
        if (gamepad1.dpad_up) {
            intakePosition = 0.5;
        }

        leftFrontPower = (r * Math.cos(robotAngle) + rightX);
        rightFrontPower = (r * Math.sin(robotAngle) - rightX);
        leftBackPower = (r * Math.sin(robotAngle) + rightX);
        rightBackPower = (r * Math.cos(robotAngle) - rightX);

        armPower = rightTrigger - leftTrigger;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        armMotor1.setPower(armPower);
        armMotor2.setPower(armPower);
        intake.setPosition(intakePosition);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("LeftTrigger", leftTrigger);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
}
