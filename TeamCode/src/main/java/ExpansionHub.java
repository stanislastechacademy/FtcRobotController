import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ExpansionHub", group="Gengar")

public class ExpansionHub extends OpMode {

    private DcMotor armMotor1 = null;
    private DcMotor armMotor2 = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        armMotor1 = hardwareMap.get(DcMotor.class, "Left");
        armMotor2 = hardwareMap.get(DcMotor.class, "Right");

        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        armMotor1.setPower(x);
        armMotor2.setPower(y);

        telemetry.addData("Status", "Run Time: ");
        telemetry.addData("Motors", "left (%.2f), right (%.2f)");
    }
}
