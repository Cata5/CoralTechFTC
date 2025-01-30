package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AUTO_HUMAN", group = "Autonomie")
public class Auto extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Puterea pentru motoare
    private final double putere = 0.5;
    // Timpul pentru fiecare miscare (în secunde)
//    private double timpFata = 0.8;
//    private double timpFata2 = 1;
//    private double timpSpate = 2.8;
//    private double timpSpate2 = 1.55;
//    private double timpStanga = 2.5;
//    private double timpDreapta = 0.2;
//    private double timpDreapta2 = 0.35;
//    private double timpDreapta3 = 0.3;
//    private double fataAlign = 0.5;
//    private double dreaptaAlign = 0.5;
//    private double timpDiag = 2.5;
//    private int armTargetPosition;
//    private int elbowTargetPosition;
//    private int armCheckPos;
//    private int elbowCheckPos;
//    private boolean check;
//    private boolean check1 = true;
//    private double wrist2Angle = 134;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializare hardware
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        Servo gheara = hardwareMap.get(Servo.class, "wrist1");
        gheara.setPosition(degreesToServoPosition(63));
        Servo pozitionare = hardwareMap.get(Servo.class, "wrist2");
        pozitionare.setPosition((degreesToServoPositionPro(132)));
        //    private boolean isClawOpen = false;
        //    private boolean previousA = false;
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        DcMotor elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setare directie motoare
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Asteapta start
        waitForStart();

        if (opModeIsActive()) {
            moveForwardForTime(0.59);
            strafeRightForTime(0.9);
            moveForwardForTime(0.5);
            strafeRightForTime(0.44);
            moveBackwardForTime(1.6);
            moveForwardForTime(1.3); // 1.3
            strafeRightForTime(0.56);
            moveBackwardForTime(1.7);

            moveForwardForTime(1.45);
            strafeRightForTime(0.5);
            moveBackwardForTime(1.7);
        }
    }

    // Functie pentru deplasare fata pentru un anumit timp (în secunde)
    private void moveForwardForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(putere);
        backLeft.setPower(putere);
        frontRight.setPower(putere);
        backRight.setPower(putere);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Functie pentru deplasare spate pentru un anumit timp (în secunde)
    private void moveBackwardForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(-putere);
        backLeft.setPower(-putere);
        frontRight.setPower(-putere);
        backRight.setPower(-putere);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Functie pentru deplasare laterala dreapta pentru un anumit timp (în secunde)
    private void strafeRightForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(putere);
        backLeft.setPower(-putere);
        frontRight.setPower(-putere);
        backRight.setPower(putere);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    private void strafeLeftForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(-putere);
        backLeft.setPower(putere);
        frontRight.setPower(putere);
        backRight.setPower(-putere);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înainte-stânga
    private void diagonalForwardLeftForTime(double timeInSeconds) throws InterruptedException {
        frontRight.setPower(putere);
        backLeft.setPower(putere);
        frontLeft.setPower(0);
        backRight.setPower(0);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înainte-dreapta
    private void diagonalForwardRightForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(putere);
        backRight.setPower(putere);
        frontRight.setPower(0);
        backLeft.setPower(0);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înapoi-stânga
    private void diagonalBackwardLeftForTime(double timeInSeconds) throws InterruptedException {
        frontRight.setPower(-putere);
        backLeft.setPower(-putere);
        frontLeft.setPower(0);
        backRight.setPower(0);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înapoi-dreapta
    private void diagonalBackwardRightForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(-putere);
        backRight.setPower(-putere);
        frontRight.setPower(0);
        backLeft.setPower(0);
        sleep((long) (timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Functie pentru oprirea treptata a motoarelor
    private void gradualStopMotors() throws InterruptedException {
        double currentPower = frontLeft.getPower();

        if (currentPower > 0) {
            for (double p = currentPower; p > 0; p -= 0.3) {
                frontLeft.setPower(p);
                frontRight.setPower(p);
                backLeft.setPower(p);
                backRight.setPower(p);
                sleep(50);
            }
        } else if (currentPower < 0) {
            for (double p = currentPower; p < 0; p += 0.3) {
                frontLeft.setPower(p);
                frontRight.setPower(p);
                backLeft.setPower(p);
                backRight.setPower(p);
                sleep(50);
            }
        }

        stopMotors();
    }

    // Functie pentru oprirea motoarelor
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private boolean check(int currentPos, int targetPos) {
        return currentPos > targetPos;
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