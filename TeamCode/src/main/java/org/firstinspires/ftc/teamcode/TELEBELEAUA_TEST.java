package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp(name = "TELEBELEAUA_FINAL", group = "Robo")
public class TELEBELEAUA_TEST extends OpMode {
    // Mecanum drive motors
    private Servo gheara;
    private Servo pozitionare;
    private Servo rotareR;
    private Servo rotareL;
    private boolean isClawOpen = false;
    private boolean previousA = false;
    private DcMotorEx armMotor;
    private DcMotorEx elbowMotor;
    private double delayTranzitie = 2.0;

    private int armTargetPosition;
    private int elbowTargetPosition;

    //    private int armTicksPerDegree;
//    private int elbowTicksPerDegree;
    private int armCheckPos;
    private int elbowCheckPos;
    private boolean check;
    private boolean check1 = true;
    private double wrist2Angle = 134;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private int valoare = 0;

    private boolean isSlowMode = false;
    private boolean previousY = false;
    private boolean previousX2 = false;
    private boolean previousB2 = false;
    private PIDController controller1;
    private PIDController controller2;
    public static int target = 0;
    public static int target2 = 0;
    private final double ticks_in_degrees = 1440 / 80;
    public static double p1 = 0.04, i1 = 0.015, d1 = 0.005; // 0.04 0.015 0.005
    public static double f1 = 0.07;
    public static double p2 = 0.06, i2 = 0.03, d2 = 0.001;
    public static double f2 = 0.14;

    @Override
    public void init() {
        // Initialize drive motors

        controller1 = new PIDController(p1, i1,d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller2 = new PIDController(p2, i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        gheara = hardwareMap.get(Servo.class, "wrist1");
        gheara.setPosition(degreesToServoPosition(63));
        pozitionare = hardwareMap.get(Servo.class, "wrist2");
        pozitionare.setPosition((degreesToServoPositionPro(132)));
        rotareR = hardwareMap.get(Servo.class, "rotareR");
        rotareR.setPosition(degreesToServoPositionPro(120));
        rotareL = hardwareMap.get(Servo.class, "rotareL");
        rotareL.setPosition(degreesToServoPositionPro(-120));
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");
        //elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Asigură-te că motorul se oprește la poziția dorită
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Asigură-te că motorul se oprește la poziția dorită

        // Setarea modului de motor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setTargetPosition(0);
        elbowMotor.setTargetPosition(0);

        // Obținem numărul de "ticks" per grad pentru fiecare motor
//        armTicksPerDegree = (int) (armMotor.getMotorType().getTicksPerRev() / 360.0);
//        elbowTicksPerDegree = (int) (elbowMotor.getMotorType().getTicksPerRev() / 360.0);

        // Modificăm direcția motorului elbow pentru a inversa mișcarea
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.REVERSE); // Schimbăm direcția motorului elbow
        // Inițializare poziții țintă
        // Setare putere inițială pentru motoare
    }

    @Override
    public void loop() {

        controller1.setPID(p1, i1, d1);
        controller2.setPID(p2, i2, d2);


        int armPos = armMotor.getCurrentPosition();
        int elbowPos = elbowMotor.getCurrentPosition();

        double pid = controller1.calculate(elbowPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f1;

        double power = pid * ff;


        double pid2 = controller2.calculate(armPos, target2);
        double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degrees)) * f2;

        double power2 = pid2 * ff2;

        elbowMotor.setPower(power);
        armMotor.setPower(power2);

        // Drive system controls
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double speedMultiplier = isSlowMode ? 0.55 : 0.88;

        // Mecanum drive calculations (fără field-centric)
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Normalize powers to avoid exceeding max value
        double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftMotor.setPower((frontLeftPower / maxPower) * speedMultiplier);
        frontRightMotor.setPower((frontRightPower / maxPower) * speedMultiplier);
        backLeftMotor.setPower((backLeftPower / maxPower) * speedMultiplier);
        backRightMotor.setPower((backRightPower / maxPower) * speedMultiplier);

        if (gamepad2.dpad_down) {
            // Poziția țintă pentru cot (elbow) este setată pentru 90 de grade
            target = 508;  // Ajustează după necesitate 900 - poz buna
            speedMultiplier = 0.2;
            frontLeftMotor.setPower((frontLeftPower / maxPower) * speedMultiplier);
            frontRightMotor.setPower((frontRightPower / maxPower) * speedMultiplier);
            backLeftMotor.setPower((backLeftPower / maxPower) * speedMultiplier);
            backRightMotor.setPower((backRightPower / maxPower) * speedMultiplier);
        }
        if (gamepad2.a) {
            target = 0;
            pozitionare.setPosition((degreesToServoPositionPro(132)));
        }
        if (gamepad2.dpad_left) {
            target = 512 ;
        }
        if (gamepad2.dpad_up){
            target = 560;
            target2 = 1400;
            speedMultiplier = 0.2;
            frontLeftMotor.setPower((frontLeftPower / maxPower) * speedMultiplier);
            frontRightMotor.setPower((frontRightPower / maxPower) * speedMultiplier);
            backLeftMotor.setPower((backLeftPower / maxPower) * speedMultiplier);
            backRightMotor.setPower((backRightPower / maxPower) * speedMultiplier);
        }
        if (gamepad2.y) {
            double elbowError = Math.abs(elbowMotor.getCurrentPosition() - target);
            double armError = Math.abs(armMotor.getCurrentPosition() - target2);

            double elbowPower = Math.max(0.02, Math.min(0.1, elbowError * 0.0003)); // Ajustează factorul de scalare
            double armPower = Math.max(0.02, Math.min(0.1, armError * 0.0003));

            elbowMotor.setPower(elbowPower);
            armMotor.setPower(armPower);

            target = 0;
            target2 = 0;
        }
    if (gamepad1.dpad_down )
    {
        target = target + 5;
    }
        if (gamepad1.dpad_up)
    {
        target = target - 3;
    }

        if (gamepad1.y && !previousY) {
            isSlowMode = !isSlowMode;
        }
        previousY = gamepad1.y;

        elbowCheckPos = elbowMotor.getCurrentPosition();
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armCheckPos = armMotor.getCurrentPosition();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Verificăm ce buton a fost apăsat pentru a schimba poziția țintă

        if (gamepad2.right_bumper && !previousX2) {
            wrist2Angle += 30.0; // Crește unghiul cu 30 de grade
            if (wrist2Angle > 225.0) wrist2Angle = 225.0; // Limitează la 225 de grade
            pozitionare.setPosition(degreesToServoPositionPro(wrist2Angle));
        }
        previousX2 = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !previousB2) {
            wrist2Angle -= 30.0;
            if (wrist2Angle < 0) wrist2Angle = 0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));
        }
        previousB2 = gamepad2.left_bumper;


        if (gamepad2.b) {
            wrist2Angle = 132.0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));

        }


        if (gamepad1.a && !previousA) {
            isClawOpen = !isClawOpen;
        }

        previousA = gamepad1.a;

        if (isClawOpen) {
            gheara.setPosition(degreesToServoPosition(0));
        } else {
            gheara.setPosition(degreesToServoPosition(65));
        }

        // Telemetry
//        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
//        telemetry.addData("Front Right Power", frontRightMotor.getPower());
//        telemetry.addData("Back Left Power", backLeftMotor.getPower());
//        telemetry.addData("Back Right Power", backRightMotor.getPower());
//        telemetry.addData("Slow Mode Active", isSlowMode);
//        telemetry.addData("Arm Target Position (ticks)", armTargetPosition);
//        telemetry.addData("Elbow Target Position (ticks)", elbowTargetPosition);
//        telemetry.addData("Arm Current Position (ticks)", armMotor.getCurrentPosition());
//        telemetry.addData("Elbow Current Position (ticks)", elbowMotor.getCurrentPosition());
//        telemetry.addData("Motor Ticks/Rev ", armMotor.getMotorType().getTicksPerRev());
//        telemetry.addData("Check :", check);
//        telemetry.addData("Check1", check1);
//        telemetry.addData("Tranzitie :", delayTranzitie);
//        telemetry.addData("Servo Position", gheara.getPosition());
//        telemetry.addData("Servo Position 2", isClawOpen);
//        telemetry.addData("pozitie servo 2:", pozitionare.getPosition());
//        telemetry.addData("Button A Pressed", gamepad2.a);
//        telemetry.addData("Servo Target Value", valoare);
//        telemetry.update();
        telemetry.addData("posArm", armMotor.getCurrentPosition());
        telemetry.addData("target", target2);
        telemetry.update();
    }

    private boolean check(int currentPos, int targetPos){
        if (currentPos > targetPos) return true;
        else return false;
    }
    private double degreesToServoPosition(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 180.0;
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }
    private double degreesToServoPositionPro(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 270.0;
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }
}