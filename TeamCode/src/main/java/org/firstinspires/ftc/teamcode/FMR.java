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
@TeleOp(name="FMR", group="Linear Opmode")
public class FMR extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx Carrossel = null;
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx motorOmbro = null;
    private DcMotorEx motorCotovelo = null;
    private Servo servoPulso = null;
    private Servo servoGarra = null;
    private Servo servobandeira = null;
    private double PosY;
    private double PosX;
    private double phi;
    private DigitalChannel botao1ombro;
    private DigitalChannel botao2cotovelo;
    private int Velocidade_Carrossel;
    NormalizedColorSensor colorSensor;
    DistanceSensor distSensor;
    private ElapsedTime tempodegiro = new ElapsedTime();


    private double fatorCotovelo = (-264 / Math.toRadians(180)); //-264 = 180°
    private double fatorOmbro = (5600 / Math.toRadians(180)); //5600 = 180°
    private double fatorPulso = (1 / Math.toRadians(180));
    private boolean lastDetectedCube = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Carrossel = hardwareMap.get(DcMotorEx.class, "motorCarrossel");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        botao1ombro = hardwareMap.get(DigitalChannel.class, "Botao1Ombro");
        botao2cotovelo = hardwareMap.get(DigitalChannel.class, "Botao2Cotovelo");
        botao1ombro.setMode(DigitalChannel.Mode.INPUT);
        botao2cotovelo.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        distSensor = ((DistanceSensor) colorSensor);

        motorCotovelo = hardwareMap.get(DcMotorEx.class, "Cotovelo");
        motorOmbro = hardwareMap.get(DcMotorEx.class, "Ombro");

        motorOmbro.setTargetPosition(0);
        motorCotovelo.setTargetPosition(0);
        motorCotovelo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOmbro.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOmbro.setVelocity(1500); //(MAX VEL 2800)
        motorCotovelo.setVelocity(500); //(MAX VEL 600)
        motorCotovelo.setCurrentAlert(4.4, CurrentUnit.AMPS);//current limit
        motorCotovelo.setVelocityPIDFCoefficients(40.00, 0, 0, 42);
        motorOmbro.setVelocityPIDFCoefficients(1, 0, 1, 25.6);

        servoPulso = hardwareMap.get(Servo.class, "ServoPunho");
        servoGarra = hardwareMap.get(Servo.class, "ServoGarra");
        servobandeira = hardwareMap.get(Servo.class, "ServoBandeira");
        servoPulso.setPosition(1);

        Carrossel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorOmbro.setDirection(DcMotorEx.Direction.REVERSE);


        Cinematica_5 braco2 = new Cinematica_5();

        //a partir daqui o codigo é iniciado no celular
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)


        PosX = -0.137;
        PosY = -0.029;
        phi = Math.toRadians(200);
        while (opModeIsActive()) {
            double velocity = (gamepad1.right_trigger * 0.80) + 0.20;
            double y = gamepad1.left_stick_y * velocity;
            double x = gamepad1.left_stick_x * -1.1 * velocity;
            double rx = -gamepad1.right_stick_x * velocity;

            /*
            Codigo responsavel pela movimentação do robô
             */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            PosY = PosY + gamepad2.right_stick_y * -0.005;
            PosX = PosX + gamepad2.right_stick_x * -0.005;
            phi = phi + (gamepad2.left_stick_x * 0.05);
            if (phi < Math.toRadians(90)){
                phi = Math.toRadians(90);
            }
            if (phi > Math.toRadians(360)){
                phi = Math.toRadians(360);
            }

            if (PosY > 2){
                PosY = 2;
            }
            if (PosY < -1){
                PosY = -1;
            }

            if (PosX > 2) {
                PosX = 2;
            }
            if (PosX < -2) {
                PosX = -2;
            }
            if (phi > 280) {
                phi = 280;
            }


            if (gamepad2.x == true){
                phi = Math.toRadians(260.5);
                PosX = -0.1;
                PosY = 0.022;
            }

            if (gamepad2.b == true){
                phi = Math.toRadians(310.59);
                PosX = -0.083;
                PosY = -0.022;
            }
            if (gamepad2.y == true){
                phi = Math.toRadians(325.77);
                PosX = -0.087;
                PosY = 0.03;
            }
            if (gamepad2.a == true){
                phi = Math.toRadians(322.29);
                PosX = -0.042;
                PosY = 0.0266;
            }
            if (gamepad2.dpad_down == true) {
                PosX = 0.068;
                PosY = -0.03;
                phi = Math.toRadians(242.017);
            }
            if (gamepad2.dpad_right == true) {
                phi = Math.toRadians(222.3);
                PosX = -0.087;
                PosY = -0.021;
            }
            if(gamepad2.dpad_up == true){
                PosX = -0.137;
                PosY = 0.029;
                phi = Math.toRadians(200);
            }
            if(gamepad2.dpad_left == true){
                PosX = 0.161;
                PosY = 0.004;
                phi = Math.toRadians(222.3);
            }
            braco2.setPos(PosX, PosY, phi);

            double dist = distSensor.getDistance(DistanceUnit.CM);
            if((!Double.isNaN(dist)) && dist < 15){ //detectou cubo?
                //erguida
                servobandeira.setPosition(1);
                if (lastDetectedCube == false)
                {
                    gamepad2.rumble(0,1.0,300);
                }
                lastDetectedCube = true;
            }else{
                lastDetectedCube = false;
                //abaixada
                servobandeira.setPosition(0);
            }

            double ombro = braco2.getTe1();
            double cotovelo = braco2.getTe2();
            double pulso = Math.toRadians(180) - braco2.getTe3();
            if(pulso < Math.toRadians(0)){
                pulso = Math.toRadians(0);
            }
            if(pulso > Math.toRadians(180)){
                pulso = Math.toRadians(180);
            }
            if (cotovelo != Double.NaN && cotovelo > Math.toRadians(0) && cotovelo < Math.toRadians(180)) {
                motorCotovelo.setTargetPosition((int) (cotovelo * fatorCotovelo));
            }

            if (ombro != Double.NaN && ombro > Math.toRadians(0) && ombro < Math.toRadians(180)) {
                motorOmbro.setTargetPosition((int) (ombro * fatorOmbro));
            }
            if (pulso != Double.NaN && pulso*fatorPulso >= 0 && pulso*fatorPulso <= 1){
                servoPulso.setPosition(pulso*fatorPulso);
            }

            if (gamepad2.right_bumper == true){
                servoGarra.setPosition(0);
            }
            if (gamepad2.left_bumper == true){
                servoGarra.setPosition(1);
            }

            // Sequencia responsavel por exibir no monitor os valores importantes do codigo.
            telemetry.addLine("Opmode");
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addLine("Cinemática INPUT");
            telemetry.addData("Posição X", PosX);
            telemetry.addData("Posição Y", PosY);
            telemetry.addData("PHI", Math.toDegrees(phi));

            telemetry.addLine("Cinemática OUTPUT");
            telemetry.addData("Te1", Math.toDegrees(braco2.getTe1()));
            telemetry.addData("Te2", Math.toDegrees(braco2.getTe2()));
            telemetry.addData("Te3", Math.toDegrees(braco2.getTe3()));

            telemetry.addLine("Motores - OMBRO");
            telemetry.addData("Posição Motor Ombro",motorOmbro.getCurrentPosition());
            telemetry.addData("Target Position Ombro", motorOmbro.getTargetPosition());
            telemetry.addData("Current Ombro", motorOmbro.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("Motores - COTOVELO");
            telemetry.addData("Posição Motor Cotovelo",motorCotovelo.getCurrentPosition());
            telemetry.addData("Target Position Cotovelo", motorCotovelo.getTargetPosition());
            telemetry.addData("Current Cotovelo", motorCotovelo.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }
}
