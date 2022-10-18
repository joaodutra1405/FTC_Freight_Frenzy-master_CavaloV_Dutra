package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name="Bacana", group="Linear Opmode")
public class Supimpa extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx esquerda = null;
    private DcMotorEx direita = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        esquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        direita = hardwareMap.get(DcMotorEx.class, "direita");
        esquerda.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double velocity = (gamepad1.right_trigger * 0.7) + 0.3;
            double x = gamepad1.left_stick_x * velocity;
            double y = gamepad1.left_stick_y * velocity;
            double esquerdaPower = y+x;
            double direitaPower = y-x;
            esquerda.setPower(esquerdaPower);
            direita.setPower(direitaPower);

            if (gamepad1.a == true) {
                esquerda.setVelocity(1000);
                direita.setVelocity(1000);
            }
            else if (gamepad1.b == true){
                esquerda.setVelocity(-1000);
                direita.setVelocity(-1000);

            }
            else {
                esquerda.setVelocity(0);
                direita.setVelocity(0);
            }
            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocidade",velocity);
            telemetry.addData("Current Esquerda",esquerda.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current Direita", direita.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
