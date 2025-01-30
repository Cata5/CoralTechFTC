//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@Config
//@TeleOp(name = "COX", group = "Robo")
//public class glisiere_in_pula_mea {
//    private Servo gheara;
//    private Servo pozitionare;
//    private Servo rotireR;
//    private Servo rotireL;
//    private boolean isClawOpen = true;
//    private boolean previousA = false;
//    private DcMotorEx armMotor;
//    private DcMotorEx elbowMotor;
//    private double delayTranzitie = 2.0;
//
//    private int armTargetPosition;
//    private int elbowTargetPosition;
//
//    private int armCheckPos;
//    private int elbowCheckPos;
//    private boolean check;
//    private boolean check1 = true;
//    private double wrist2Angle = 134;
//    private DcMotor frontLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor backRightMotor;
//    private int valoare = 0;
//
//    private boolean isSlowMode = false;
//    private boolean previousY = false;
//    private boolean previousX2 = false;
//    private boolean previousB2 = false;
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
//    public void init() {
//        controller1 = new PIDController(p1, i1,d1);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        controller2 = new PIDController(p2, i2,d2);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
//        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
//
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        gheara = hardwareMap.get(Servo.class, "wrist1");
//        gheara.setPosition(degreesToServoPosition(63));
//        pozitionare = hardwareMap.get(Servo.class, "wrist2");
//        pozitionare.setPosition((degreesToServoPositionPro(132)));
//        rotireR = hardwareMap.get(Servo.class, "rotareR");
//        rotireR.setPosition(degreesToServoPositionPro(120));
//        rotireL = hardwareMap.get(Servo.class, "rotareL");
//        rotireL.setPosition(degreesToServoPositionPro(-120));
//        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
//        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");
//        target = 0;
//        target2 = 0;
//        //elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Asigură-te că motorul se oprește la poziția dorită
//        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Asigură-te că motorul se oprește la poziția dorită
//
//        // Setarea modului de motor
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//
////        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        armMotor.setTargetPosition(0);
//        elbowMotor.setTargetPosition(0);
//
//        // Obținem numărul de "ticks" per grad pentru fiecare motor
////        armTicksPerDegree = (int) (armMotor.getMotorType().getTicksPerRev() / 360.0);
////        elbowTicksPerDegree = (int) (elbowMotor.getMotorType().getTicksPerRev() / 360.0);
//
//        // Modificăm direcția motorului elbow pentru a inversa mișcarea
//        armMotor.setDirection(DcMotor.Direction.FORWARD);
//        elbowMotor.setDirection(DcMotor.Direction.REVERSE); // Schimbăm direcția motorului elbow
//        // Inițializare poziții țintă
//        // Setare putere inițială pentru motoare
//    }
//
//    @Override
//    public void loop(){
//        telemetry.addData("posArm", armMotor.getCurrentPosition());
//        telemetry.addData("target", target2);
//        telemetry.addData("posElbow", elbowMotor.getCurrentPosition());
//        telemetry.addData("target2", target);
//        telemetry.update();
//    }
//}
//
//private boolean check(int currentPos, int targetPos){
//    if (currentPos > targetPos) return true;
//    else return false;
//}
//private double degreesToServoPosition(double degrees) {
//    double minDegrees = 0.0;
//    double maxDegrees = 180.0;
//    return (degrees - minDegrees) / (maxDegrees - minDegrees);
//}
//private double degreesToServoPositionPro(double degrees) {
//    double minDegrees = 0.0;
//    double maxDegrees = 270.0;
//    return (degrees - minDegrees) / (maxDegrees - minDegrees);
//}
//
