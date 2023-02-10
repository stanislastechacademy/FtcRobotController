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

    public void init() {
        telemetry.addData("Status", "Initialized");

        Left  = hardwareMap.get(DcMotor.class, "left");
        Right = hardwareMap.get(DcMotor.class, "right");

        Left.setDirection(DcMotorSimple.Direction.FORWARD);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);
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

        double x = -gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;

            leftFrontPower = (x);
            rightFrontPower = (y);


        Left.setPower(leftFrontPower + rightFrontPower);
        Right.setPower(rightFrontPower - leftFrontPower);
    }
}
