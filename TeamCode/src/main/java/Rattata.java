import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Rotom", group="Gengar")

public class Rattata extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor Left = null;
    private DcMotor Right = null;
    private DcMotor Arm = null;
    private Servo intake = null;
    double intakePosition = 0.5;

    public void init() {
        telemetry.addData("Status", "Initialized");

        Left  = hardwareMap.get(DcMotor.class, "left");
        Right = hardwareMap.get(DcMotor.class, "right");
        Arm = hardwareMap.get(DcMotor.class, "arm1");
        intake = hardwareMap.get(Servo.class, "Intake");


        Left.setDirection(DcMotorSimple.Direction.FORWARD);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(Servo.Direction.FORWARD);

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
        double rightFrontPower;
        double armPower;

        double x = -gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.left_trigger;
        double t = gamepad1.right_trigger;

        leftFrontPower = (x);
        rightFrontPower = (y);
        armPower = z - t;

        Left.setPower(leftFrontPower + rightFrontPower);
        Right.setPower(rightFrontPower - leftFrontPower);
        Arm.setPower(armPower);

        if (gamepad1.dpad_up) {
            intakePosition = 1.0;
        }
        if (gamepad1.dpad_down) {
            intakePosition = 0.0;
        }
    }
}