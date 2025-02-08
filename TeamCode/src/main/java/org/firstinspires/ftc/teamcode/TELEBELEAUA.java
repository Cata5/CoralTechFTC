package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "TELEBELEAUAAAAA", group = "FINAL")
public class TELEBELEAUA extends OpMode {
    private DcMotorEx motorGlisiere;
    private DcMotorEx motorBratStanga, motorBratDreapta;
    private Servo gheara, pozitionare_gheara;
    private PIDController controller_unghi, controler_glisiere;

    private final int GLISIERE_LIMIT_MIN = 0;
    private final int GLISIERE_LIMIT_MAX = 3000;
    private double wrist2Angle = 134;
    private boolean previousX2 = false;
    private boolean previousB2 = false;
    private final int BRAT_LIMIT_MIN = 0;
    private final int BRAT_LIMIT_MAX = 2700;
    private final double ticks_in_degrees = 700/180.0;
    private boolean isClawOpen = true;
    private boolean previousA = false;

    ///GHEARA UNGHI
    public Servo rotatieL;
    public boolean previousLT = false;
    public boolean previousRT = false;
    private double pozitieRotatieL = 0;
    private double pozitieRotatieR = 270;

    ///sasiu

    private boolean isSlowMode = false;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    public static double DEADZONE = 0.1;   // Minimum joystick input to move the motors.
    public static double MAX_POWER = 1.0;    // Maximum motor power.
    private Servo pozitionare;
    double position = 0;

    public static int target_unghi = 0;
    public static double p_unghi = 0, i_unghi = 0, d_unghi = 0, f_unghi = 0;


    @Override
    public void init() {
        controller_unghi = new PIDController(p_unghi, i_unghi, d_unghi);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gheara = hardwareMap.get(Servo.class, "wrist1");
        gheara.setPosition(degreesToServoPositionPro(70));
        pozitionare = hardwareMap.get(Servo.class, "wrist2");
        pozitionare.setPosition((degreesToServoPositionPro(100)));
///trebuiesc inversate motoarele pt brat
        motorGlisiere = hardwareMap.get(DcMotorEx.class, "motorGlisiere");
        motorBratDreapta = hardwareMap.get(DcMotorEx.class, "motorBratDreapta");
        motorBratStanga = hardwareMap.get(DcMotorEx.class, "motorBratStanga");

        motorGlisiere.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        motorGlisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBratStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBratDreapta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        motorGlisiere.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBratDreapta.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBratStanga.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rotatieL = hardwareMap.get(Servo.class, "rotatie");

        rotatieL.setPosition(degreesToServoPositionPro(110));
        motorGlisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        motorGlisiere.setDirection(DcMotor.Direction.FORWARD);
        motorBratStanga.setDirection(DcMotor.Direction.REVERSE);
        motorBratDreapta.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        controller_unghi.setPID(p_unghi, i_unghi, d_unghi);
        int unghi_pos = motorBratDreapta.getCurrentPosition();

        double pid_unghi = controller_unghi.calculate(unghi_pos, target_unghi);
        double ff_unghi = Math.cos(Math.toRadians(target_unghi / ticks_in_degrees)) * f_unghi;
        double power_unghi = pid_unghi + ff_unghi;
        motorBratDreapta.setPower(power_unghi);
        motorBratStanga.setPower(power_unghi);
        double joystickY = gamepad2.left_stick_y;
        double power = joystickY;
        double joystickY2 = gamepad2.right_trigger;
        int arm_pos = motorBratDreapta.getCurrentPosition();
//        if (Math.abs(arm_pos) > BRAT_LIMIT_MAX) {
//            motorBratDreapta.setPower(0); // Oprește motorul dacă depășește limita maximă
//            motorBratStanga.setPower(0); // Oprește motorul dacă depășește limita maximă
//
//        } else if (Math.abs(arm_pos) < BRAT_LIMIT_MIN) {
//            motorGlisiere.setPower(0); // Oprește motorul dacă ajunge sub limita minimă
//            motorBratStanga.setPower(0); // Oprește motorul dacă depășește limita maximă
//        } else {
//            double clippedPower2 = Range.scale(power, -1, 1, -0.8, 0.8);
//            motorBratDreapta.setPower(clippedPower2);  // Setează puterea motorului dacă nu a atins limitele
//            motorBratStanga.setPower(clippedPower2);  // Setează puterea motorului dacă nu a atins limitele
//        }
        // Check if the joystick input is within the deadzone

//            if (Math.abs(power) == 0) {
//                motorGlisiere.setPower(0);
//            } else {
//                double clippedPower = Range.scale(power,-1,1, -0.8, 0.8);
//                position = position+clippedPower;
//                int powerINT = (int) clippedPower;
////                motorGlisiere.setPower(position);
//                motorGlisiere.setPower(powerINT);
//
//            }
        if (Math.abs(power) < DEADZONE) {
            motorGlisiere.setPower(0);
        } else {
            // Clip the power to ensure it does not exceed the maximum allowed power.
            double clippedPower = Range.clip(power, -MAX_POWER, MAX_POWER);
            motorGlisiere.setPower(clippedPower);
        }
        int glisiere_pos = motorGlisiere.getCurrentPosition();
        if (Math.abs(glisiere_pos) > GLISIERE_LIMIT_MAX) {
            motorGlisiere.setPower(0); // Oprește motorul dacă depășește limita maximă
        } else if (Math.abs(glisiere_pos) < GLISIERE_LIMIT_MIN) {
            motorGlisiere.setPower(0); // Oprește motorul dacă ajunge sub limita minimă
        } else {
            double clippedPower = Range.scale(power, -1, 1, -0.8, 0.8);
            motorGlisiere.setPower(clippedPower);  // Setează puterea motorului dacă nu a atins limitele
        }
        if(gamepad2.b){
            setBratGlisieraPosition(200,0);
        }

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        double speedMultiplier = isSlowMode ? 0.55 : 1.0;

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftMotor.setPower((frontLeftPower / maxPower) * speedMultiplier);
        frontRightMotor.setPower((frontRightPower / maxPower) * speedMultiplier);
        backLeftMotor.setPower((backLeftPower / maxPower) * speedMultiplier);
        backRightMotor.setPower((backRightPower / maxPower) * speedMultiplier);

        boolean currentLT = gamepad2.left_trigger > 0.5;
        boolean currentRT = gamepad2.right_trigger > 0.5;

//        if (currentLT && !previousLT) {
//            pozitieRotatieL += 90;
//            pozitieRotatieR -= 90;
//
//            if (pozitieRotatieL > 180 && pozitieRotatieR < 90) {
//                pozitieRotatieL = 270;
//                pozitieRotatieR = 0;
//            }
//
//            rotatieL.setPosition(degreesToServoPositionPro(pozitieRotatieL));
//        }
//
//        if (currentRT && !previousRT) {
//            pozitieRotatieL -= 90;
//            pozitieRotatieR += 90;
//
//            if (pozitieRotatieL < 90 && pozitieRotatieR > 180) {
//                pozitieRotatieL = 0;
//                pozitieRotatieR = 270;
//            }
//
//            rotatieL.setPosition(degreesToServoPositionPro(pozitieRotatieL));
//        }
        if (gamepad1.a && !previousA) {
            isClawOpen = !isClawOpen;
        }
        if (isClawOpen) {
            gheara.setPosition(degreesToServoPositionPro(0)); // Deschide gheara
        } else {
            gheara.setPosition(degreesToServoPositionPro(70)); // Închide gheara
        }
        previousA = gamepad1.a;

        if (gamepad2.right_bumper && !previousX2) {
            wrist2Angle += 30.0;
            if (wrist2Angle > 225.0) wrist2Angle = 225.0;
            pozitionare.setPosition(degreesToServoPositionPro(wrist2Angle));
        }
        previousX2 = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !previousB2) {
            wrist2Angle -= 30.0;
            if (wrist2Angle < 0) wrist2Angle = 0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));
        }
        previousB2 = gamepad2.left_bumper;

        previousLT = currentLT;
        previousRT = currentRT;
        telemetry.addData("Joystick Y", joystickY);
        telemetry.addData("Motor Power", motorGlisiere.getPower());
        telemetry.addData("pos" ,unghi_pos);
        telemetry.addData("target", target_unghi);
        telemetry.update();


    }

    //    private void setBratGlisieraPosition(int unghi, int glisiere){
//        position=glisiere;
//        motorBratDreapta.setTargetPosition(unghi);
//        motorBratStanga.setTargetPosition(unghi);
//
//        motorBratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motorBratDreapta.setPower(0.5); // Ajustează viteza după nevoie
//        motorBratStanga.setPower(0.5);
//    }
    private void setBratGlisieraPosition(int unghi, int glisiere){
        motorGlisiere.setTargetPosition(glisiere);
        motorGlisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiere.setPower(0.8); // Ajustează viteza după nevoie

        // Setăm poziția brațului
        motorBratDreapta.setTargetPosition(unghi);
        motorBratStanga.setTargetPosition(unghi);

        motorBratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
    private double degreesToServoPositionPro(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 270.0;
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }


}