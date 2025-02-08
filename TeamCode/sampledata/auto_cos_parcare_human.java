//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Autonomous(name = "AUTO_COS", group = "Autonomie")
//public class autoParcare extends LinearOpMode {
//
//    private Servo gheara;
//    private Servo pozitionare;
//    private boolean isClawOpen = false;
//    private boolean previousA = false;
//    private DcMotor armMotor;
//    private DcMotor elbowMotor;
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    // Puterea pentru motoare
//    private double putere = 0.5;
//    // Timpul pentru fiecare miscare (în secunde)
//    private int armTargetPosition;
//    private int elbowTargetPosition;
//    private int armCheckPos;
//    private int elbowCheckPos;
//    private boolean check;
//    private boolean check1 = true;
//    private double wrist2Angle = 134;
//    private PIDController controller1;
//    private PIDController controller2;
//    public static int target = 0;
//    public static int target2 = 0;
//    private final double ticks_in_degrees = 1440 / 80;
//    public static double p1 = 0.029, i1 = 0.0013, d1 = 0.0045; // 0.04 0.015 0.005
//    public static double f1 = 0.1;
//    public static double p2 = 0.04, i2 = 0.03, d2 = 0.0011;
//    public static double f2 = 0.16;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initializare hardware
//        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
//        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
//        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
//        backRight = hardwareMap.get(DcMotor.class, "rightBack");
//
//        controller1 = new PIDController(p1, i1,d1);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        controller2 = new PIDController(p2, i2,d2);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        controller1.setPID(p1, i1, d1);
//        controller2.setPID(p2, i2, d2);
//
//        int armPos = armMotor.getCurrentPosition();
//        int elbowPos = elbowMotor.getCurrentPosition();
//
//        double pid = controller1.calculate(elbowPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f1;
//
//        double power = pid * ff;
//
//
//        double pid2 = controller2.calculate(armPos, target2);
//        double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degrees)) * f2;
//
//        double power2 = pid2 * ff2;
//
//        elbowMotor.setPower(power);
//        armMotor.setPower(power2);
//
//        gheara = hardwareMap.get(Servo.class, "wrist1");
//        gheara.setPosition(degreesToServoPosition(63));
//        pozitionare = hardwareMap.get(Servo.class, "wrist2");
//        pozitionare.setPosition((degreesToServoPositionPro(132)));
//        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
//        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
//        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        armMotor.setTargetPosition(0);
//        elbowMotor.setTargetPosition(0);
//
//
//        target = 0;
//        target2 = 0;
//
//        // Setare directie motoare
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//
//        armMotor.setDirection(DcMotor.Direction.FORWARD);
//        elbowMotor.setDirection(DcMotor.Direction.REVERSE); // Schimbăm direcția motorului elbow
//
//        // Asteapta start
//        waitForStart();
//
//        if (opModeIsActive()) {
//            moveForwardForTime(0.5);
//            target = 30;
//            wait(1);
//            gheara.setPosition(degreesToServoPosition(0));
//        }
//    }
//
//    // Functie pentru deplasare fata pentru un anumit timp (în secunde)
//    private void moveForwardForTime(double timeInSeconds) throws InterruptedException {
//        frontLeft.setPower(putere);
//        backLeft.setPower(putere);
//        frontRight.setPower(putere);
//        backRight.setPower(putere);
//        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
//        gradualStopMotors(); // Aplicăm frânarea treptată
//    }
//
//    // Functie pentru deplasare spate pentru un anumit timp (în secunde)
//    private void moveBackwardForTime(double timeInSeconds) throws InterruptedException {
//        frontLeft.setPower(-putere);
//        backLeft.setPower(-putere);
//        frontRight.setPower(-putere);
//        backRight.setPower(-putere);
//        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
//        gradualStopMotors(); // Aplicăm frânarea treptată
//    }
//
//    // Functie pentru deplasare laterala dreapta pentru un anumit timp (în secunde)
//    private void strafeRightForTime(double timeInSeconds) throws InterruptedException {
//        frontLeft.setPower(putere);
//        backLeft.setPower(-putere);
//        frontRight.setPower(-putere);
//        backRight.setPower(putere);
//        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
//        gradualStopMotors(); // Aplicăm frânarea treptată
//    }
//    private void strafeLeftForTime(double timeInSeconds) throws InterruptedException{
//        frontLeft.setPower(-putere);
//        backLeft.setPower(putere);
//        frontRight.setPower(putere);
//        backRight.setPower(-putere);
//        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
//        gradualStopMotors(); // Aplicăm frânarea treptată
//    }
//
//    // Funcție pentru deplasare diagonală înainte-stânga
//    private void diagonalForwardLeftForTime(double timeInSeconds) throws InterruptedException {
//        frontRight.setPower(putere);
//        backLeft.setPower(putere);
//        frontLeft.setPower(0);
//        backRight.setPower(0);
//        sleep((long)(timeInSeconds * 1000));
//        gradualStopMotors();
//    }
//
//    // Funcție pentru deplasare diagonală înainte-dreapta
//    private void diagonalForwardRightForTime(double timeInSeconds) throws InterruptedException {
//        frontLeft.setPower(putere);
//        backRight.setPower(putere);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        sleep((long)(timeInSeconds * 1000));
//        gradualStopMotors();
//    }
//
//    // Funcție pentru deplasare diagonală înapoi-stânga
//    private void diagonalBackwardLeftForTime(double timeInSeconds) throws InterruptedException {
//        frontRight.setPower(-putere);
//        backLeft.setPower(-putere);
//        frontLeft.setPower(0);
//        backRight.setPower(0);
//        sleep((long)(timeInSeconds * 1000));
//        gradualStopMotors();
//    }
//
//    // Funcție pentru deplasare diagonală înapoi-dreapta
//    private void diagonalBackwardRightForTime(double timeInSeconds) throws InterruptedException {
//        frontLeft.setPower(-putere);
//        backRight.setPower(-putere);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        sleep((long)(timeInSeconds * 1000));
//        gradualStopMotors();
//    }
//    // Functie pentru oprirea treptata a motoarelor
//    private void gradualStopMotors() throws InterruptedException {
//        for (double i = putere; i > 0; i -= 0.30) {
//            frontLeft.setPower(i);
//            frontRight.setPower(i);
//            backLeft.setPower(i);
//            backRight.setPower(i);
//            sleep(50);  // Pauză între fiecare scădere de putere (50 ms)
//        }
//        stopMotors();  // Oprirea completă a motoarelor după ce puterea ajunge la 0
//    }
//
//    // Functie pentru oprirea motoarelor
//    private void stopMotors() {
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
//    private boolean check(int currentPos, int targetPos){
//        if (currentPos > targetPos) return true;
//        else return false;
//    }
//    private double degreesToServoPosition(double degrees) {
//        double minDegrees = 0.0;
//        double maxDegrees = 180.0;
//        return (degrees - minDegrees) / (maxDegrees - minDegrees);
//    }
//    private double degreesToServoPositionPro(double degrees) {
//        double minDegrees = 0.0;
//        double maxDegrees = 270.0;
//        return (degrees - minDegrees) / (maxDegrees - minDegrees);
//    }
//}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AUTO_COS_PARCARE_HUMAN", group = "Autonomie")
public class auto_cos_parcare_human extends LinearOpMode {

    private Servo gheara;
    private Servo pozitionare;
    private boolean isClawOpen = false;
    private boolean previousA = false;
    private DcMotor armMotor;
    private DcMotor elbowMotor;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Puterea pentru motoare
    private double putere = 0.3;
    // Timpul pentru fiecare miscare (în secunde)
    private double timpFata = 0.8;
    private double timpFata2 = 1;
    private double timpSpate = 2.8;
    private double timpSpate2 = 1.55;
    private double timpStanga = 2.5;
    private double timpDreapta = 0.2;
    private double timpDreapta2 = 0.35;
    private double timpDreapta3 = 0.3;
    private double fataAlign = 0.5;
    private double dreaptaAlign = 0.5;
    private double timpDiag = 2.5;
    private int armTargetPosition;
    private int elbowTargetPosition;
    private int armCheckPos;
    private int elbowCheckPos;
    private boolean check;
    private boolean check1 = true;
    private double wrist2Angle = 134;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializare hardware
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        gheara = hardwareMap.get(Servo.class, "wrist1");
        gheara.setPosition(degreesToServoPosition(63));
        pozitionare = hardwareMap.get(Servo.class, "wrist2");
        pozitionare.setPosition((degreesToServoPositionPro(132)));
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setare directie motoare
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");

        // Reset encoderi și setare mod de utilizare
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Setare zero power behavior pentru a menține poziția
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Așteptare pentru Start
        telemetry.addLine("Ready for start");
        telemetry.update();
        waitForStart();

        // Asteapta start
        waitForStart();

        if (opModeIsActive()) {
//            strafeLeftForTime(1.2);
//            strafeRightForTime(0.8);
//            moveForwardForTime(2.75);
//            strafeLeftForTime(0.75);
//            moveBackwardForTime(2.9);
//            moveForwardForTime(2.5);
//            strafeLeftForTime(0.87);
//            moveBackwardForTime(2.6);
//            moveForwardForTime(2.4);
//            strafeLeftForTime(0.6);
//            moveBackwardForTime(2.45);
        }
    }

    // Functie pentru deplasare fata pentru un anumit timp (în secunde)
    private void moveToPosition(DcMotorEx motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setPower(power);

        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Motor", motor.getDeviceName());
            telemetry.addData("Target", position);
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0); // Oprire motor după ce poziția este atinsă
    }
    private void moveForwardForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(putere);
        backLeft.setPower(putere);
        frontRight.setPower(putere);
        backRight.setPower(putere);
        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
        gradualStopMotors(); // Aplicăm frânarea treptată
    }

    // Functie pentru deplasare spate pentru un anumit timp (în secunde)
    private void moveBackwardForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(-putere);
        backLeft.setPower(-putere);
        frontRight.setPower(-putere);
        backRight.setPower(-putere);
        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
        gradualStopMotors(); // Aplicăm frânarea treptată
    }

    // Functie pentru deplasare laterala dreapta pentru un anumit timp (în secunde)
    private void strafeRightForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(putere);
        backLeft.setPower(-putere);
        frontRight.setPower(-putere);
        backRight.setPower(putere);
        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
        gradualStopMotors(); // Aplicăm frânarea treptată
    }
    private void strafeLeftForTime(double timeInSeconds) throws InterruptedException{
        frontLeft.setPower(-putere);
        backLeft.setPower(putere);
        frontRight.setPower(putere);
        backRight.setPower(-putere);
        sleep((long)(timeInSeconds * 1000)); // Convertește secunde în milisecunde
        gradualStopMotors(); // Aplicăm frânarea treptată
    }
    private void rotire(double timeInSeconds) throws InterruptedException{
        frontLeft.setPower(putere);
        backLeft.setPower(putere);
        frontRight.setPower(-putere);
        backRight.setPower(-putere);
        sleep((long)(timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înainte-stânga
    private void diagonalForwardLeftForTime(double timeInSeconds) throws InterruptedException {
        frontRight.setPower(putere);
        backLeft.setPower(putere);
        frontLeft.setPower(0);
        backRight.setPower(0);
        sleep((long)(timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înainte-dreapta
    private void diagonalForwardRightForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(putere);
        backRight.setPower(putere);
        frontRight.setPower(0);
        backLeft.setPower(0);
        sleep((long)(timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înapoi-stânga
    private void diagonalBackwardLeftForTime(double timeInSeconds) throws InterruptedException {
        frontRight.setPower(-putere);
        backLeft.setPower(-putere);
        frontLeft.setPower(0);
        backRight.setPower(0);
        sleep((long)(timeInSeconds * 1000));
        gradualStopMotors();
    }

    // Funcție pentru deplasare diagonală înapoi-dreapta
    private void diagonalBackwardRightForTime(double timeInSeconds) throws InterruptedException {
        frontLeft.setPower(-putere);
        backRight.setPower(-putere);
        frontRight.setPower(0);
        backLeft.setPower(0);
        sleep((long)(timeInSeconds * 1000));
        gradualStopMotors();
    }
    // Functie pentru oprirea treptata a motoarelor
    private void gradualStopMotors() throws InterruptedException {
        for (double i = putere; i > 0; i -= 0.30) {
            frontLeft.setPower(i);
            frontRight.setPower(i);
            backLeft.setPower(i);
            backRight.setPower(i);
            sleep(50);  // Pauză între fiecare scădere de putere (50 ms)
        }
        stopMotors();  // Oprirea completă a motoarelor după ce puterea ajunge la 0
    }

    // Functie pentru oprirea motoarelor
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
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

