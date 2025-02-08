package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TELEBELEAUA", group = "Robo")
public class TELEBELEAUA extends OpMode {
    // Mecanum drive motors
    private Servo gheara;
    private Servo pozitionare;
    private boolean isClawOpen = false;
    private boolean previousA = false;
    private DcMotor armMotor;
    private DcMotor elbowMotor;
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
    private boolean wasRotatingRight = false;
    private boolean wasRotatingLeft = false;



    @Override
    public void init() {
        // Initialize drive motors
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
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Asigură-te că motorul se oprește la poziția dorită
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Asigură-te că motorul se oprește la poziția dorită

        // Setarea modului de motor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        armTargetPosition = armMotor.getCurrentPosition();
        elbowTargetPosition = elbowMotor.getCurrentPosition();

        // Setare putere inițială pentru motoare
        armMotor.setPower(0.05);
        elbowMotor.setPower(0.05);


    }

    @Override
    public void loop() {
        // Drive system controls
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double rotatie = gamepad2.right_stick_x;
        double speedMultiplier = isSlowMode ? 0.55 : 0.85;

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

        if (gamepad1.y && !previousY) {
            isSlowMode = !isSlowMode;
        }
        previousY = gamepad1.y;

        elbowCheckPos = elbowMotor.getCurrentPosition();
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armCheckPos = armMotor.getCurrentPosition();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Verificăm ce buton a fost apăsat pentru a schimba poziția țintă
        if (gamepad2.dpad_down) {
            // Poziția țintă pentru cot (elbow) este setată pentru 90 de grade
            elbowMotor.setPower(0.06);
            elbowTargetPosition = 600;  // Ajustează după necesitate 900 - poz buna
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbowMotor.setTargetPosition(elbowTargetPosition);
            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            check = check(elbowCheckPos, elbowTargetPosition);
            check1 = false;
        }
        if (gamepad2.a) {
            // Poziția țintă pentru cot (elbow) este setată pentru -360 de grade
            elbowMotor.setPower(0.15);
            elbowTargetPosition = 0;  // Ajustează după necesitate
            elbowMotor.setTargetPosition(elbowTargetPosition);
            check = check(elbowCheckPos, elbowTargetPosition);
            check1 = false;
            wrist2Angle = 132.0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));
        }
        if (gamepad2.dpad_up) {
            // Poziția țintă pentru cot (elbow) este setată pentru 90 de grade
            armMotor.setPower(0.85);
            armTargetPosition = 1299;  // Ajustează după necesitate 900 - poz buna
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(armTargetPosition);
            check = check(armCheckPos, armTargetPosition);
            check1 = false;
            elbowMotor.setPower(0.14);
            elbowTargetPosition = 865;  // Ajustează după necesitate 900 - poz buna
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbowMotor.setTargetPosition(elbowTargetPosition);
            check = check(elbowCheckPos, elbowTargetPosition);
            check1 = false;
//            if (gamepad2.right_trigger > 0.5) {
//                double power = -gamepad2.right_trigger * 0.2; // Ajustează viteza de coborâre
//                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                armMotor.setPower(power);
//            } else {
//                armMotor.setPower(0);
//                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
        }
        if (gamepad2.y) {
            // Poziția țintă pentru cot (elbow) este setată pentru 90 de grade
            armMotor.setPower(0.3);
            elbowMotor.setPower(0.20);
            armTargetPosition = 0;  // Ajustează după necesitate 900 - poz buna
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(armTargetPosition);
            check = check(armCheckPos, armTargetPosition);
            check1 = false;
            elbowTargetPosition = 0;  // Ajustează după necesitate 900 - poz buna
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbowMotor.setTargetPosition(elbowTargetPosition);
            check = check(elbowCheckPos, elbowTargetPosition);
            check1 = false;
        }

/*
        if(gamepad2.dpad_up){
            armMotor.setPower(0.2);
            armTargetPosition = 90;
            armMotor.setTargetPosition(armTargetPosition);
        }

        if(Math.abs(armTargetPosition - armMotor.getCurrentPosition()) < 10){
            armMotor.setPower(0.05);
        }
*/

//        if (gamepad2.right_bumper && !previousX2) {
//            wrist2Angle += 30.0; // Crește unghiul cu 30 de grade
//            if (wrist2Angle > 225.0) wrist2Angle = 225.0; // Limitează la 225 de grade
//            pozitionare.setPosition(degreesToServoPositionPro(wrist2Angle));
//        }
//        previousX2 = gamepad2.right_bumper;
//
//        if (gamepad2.left_bumper && !previousB2) {
//            wrist2Angle -= 30.0;
//            if (wrist2Angle < 0) wrist2Angle = 0;
//            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));
//        }
//        previousB2 = gamepad2.left_bumper;
        if (rotatie >= 0.4 && !wasRotatingRight) {
            wrist2Angle += 30;
            if (wrist2Angle > 225.0) wrist2Angle = 225.0;
            pozitionare.setPosition(degreesToServoPositionPro(wrist2Angle));
            wasRotatingRight = true; // Marchează că joystick-ul este în poziție de rotație dreapta
        }
        if (rotatie < 0.4) {
            wasRotatingRight = false; // Reset la revenirea joystick-ului
        }

        if (rotatie <= -0.4 && !wasRotatingLeft) {
            wrist2Angle -= 30;
            if (wrist2Angle < 0) wrist2Angle = 0;
            pozitionare.setPosition(degreesToServoPositionPro(wrist2Angle));
            wasRotatingLeft = true; // Marchează că joystick-ul este în poziție de rotație stânga
        }
        if (rotatie > -0.4) {
            wasRotatingLeft = false; // Reset la revenirea joystick-ului
        }

        if (gamepad2.b) {
            wrist2Angle = 132.0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));

        }

        if (gamepad2.x) {
            armMotor.setPower(1.0);
            armTargetPosition = 1000;  // Ajustează după necesitate 900 - poz buna
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(armTargetPosition);
            check = check(armCheckPos, armTargetPosition);
            check1 = false;
            //elbowMotor.setPower(0.15);
            elbowTargetPosition = 166;  // Ajustează după necesitate 900 - poz buna
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elbowMotor.setTargetPosition(elbowTargetPosition);
            check = check(elbowCheckPos, elbowTargetPosition);
            check1 = false;
            wrist2Angle = 132.0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));
        }


        if(check) {
            if (elbowCheckPos <= elbowTargetPosition) {
                if(elbowCheckPos>70){
                    elbowMotor.setPower(0.015);
                }
                else{
                    elbowMotor.setPower(0.02);
                }
                check1 = true;
            }
        } else if (!check) {
            if (elbowCheckPos >= elbowTargetPosition) {
                if(elbowCheckPos>70){
                    elbowMotor.setPower(0.015);
                }
                else{
                    elbowMotor.setPower(0.017);
                }
                check1 = true;
            }
        }

        if(check1){
            elbowMotor.setPower(delayTranzitie);
            if(delayTranzitie>0.01) delayTranzitie -= 0.005;
/*
            if(delayTranzitie>0){
                elbowTargetPosition=0;
                elbowMotor.setTargetPosition(elbowTargetPosition);
                elbowMotor.setPower(1);
                delayTranzitie -= 0.01;
            }else{
                elbowMotor.setPower(0.1);
                elbowTargetPosition = elbowMotor.getCurrentPosition();
                elbowMotor.setTargetPosition(elbowTargetPosition);
            }*/
        }

        if (gamepad1.a && !previousA) {
            isClawOpen = !isClawOpen;
        }

        previousA = gamepad1.a;

        if (isClawOpen) {
            gheara.setPosition(degreesToServoPosition(0));
        } else {
            gheara.setPosition(degreesToServoPosition(60));
        }

        // Telemetry
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.addData("Slow Mode Active", isSlowMode);
        telemetry.addData("Arm Target Position (ticks)", armTargetPosition);
        telemetry.addData("Elbow Target Position (ticks)", elbowTargetPosition);
        telemetry.addData("Arm Current Position (ticks)", armMotor.getCurrentPosition());
        telemetry.addData("Elbow Current Position (ticks)", elbowMotor.getCurrentPosition());
        telemetry.addData("Motor Ticks/Rev ", armMotor.getMotorType().getTicksPerRev());
        telemetry.addData("Check :", check);
        telemetry.addData("Check1", check1);
        telemetry.addData("Tranzitie :", delayTranzitie);
        telemetry.addData("Servo Position", gheara.getPosition());
        telemetry.addData("Servo Position 2", isClawOpen);
        telemetry.addData("pozitie servo 2:", pozitionare.getPosition());
        telemetry.addData("Button A Pressed", gamepad2.a);
        telemetry.addData("Servo Target Value", valoare);
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