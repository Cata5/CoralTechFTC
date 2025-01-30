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
@TeleOp(name = "o_persoana", group = "Robo")
public class teleop_1_person extends OpMode {
    // Mecanum drive motors
    private Servo gheara;
    private Servo pozitionare;
    private Servo rotireR;
    private Servo rotireL;
    private boolean isClawOpen = true;
    private boolean previousA = false;
    private DcMotorEx armMotor;
    private DcMotorEx elbowMotor;
    private double delayTranzitie = 2.0;

    private int armTargetPosition;
    private int elbowTargetPosition;

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
    public static double p1 = 0.029, i1 = 0.0013, d1 = 0.0045; // 0.04 0.015 0.005
    public static double f1 = 0.1;
    public static double p2 = 0.04, i2 = 0.03, d2 = 0.0011;
    public static double f2 = 0.16;
    public boolean pula = false;

    @Override
    public void init() {
        controller1 = new PIDController(p1, i1, d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller2 = new PIDController(p2, i2, d2);
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
        rotireR = hardwareMap.get(Servo.class, "rotareR");
        rotireR.setPosition(degreesToServoPositionPro(120));
        rotireL = hardwareMap.get(Servo.class, "rotareL");
        rotireL.setPosition(degreesToServoPositionPro(-120));
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");
        target = 0;
        target2 = 0;

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        armMotor.setTargetPosition(0);
        elbowMotor.setTargetPosition(0);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.REVERSE);
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

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double speedMultiplier = isSlowMode ? 0.55 : 0.95;

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

        if (gamepad1.dpad_down) {
            target2 = 0;
            target = 460;
            pula = true;
        }
        if (gamepad1.dpad_up) {
            target = 640;
            target2 = 1500;
            pula = true;
            pozitionare.setPosition(degreesToServoPositionPro(222));
        }
        if (gamepad1.dpad_right) {
            target = 485;
            target2 = 950;
            pula = true;
        }
        if (gamepad1.dpad_left) {
            target2 = 550;
            pula = true;
        }
        if (gamepad1.b) {
            pula = false;
            double elbowError = Math.abs(elbowMotor.getCurrentPosition() - target);
            double armError = Math.abs(armMotor.getCurrentPosition() - target2);

            double elbowPower = Math.max(0.02, Math.min(0.1, elbowError * 0.0003));
            double armPower = Math.max(0.02, Math.min(0.1, armError * 0.0003));

            elbowMotor.setPower(elbowPower);
            armMotor.setPower(armPower);

            target = 0;
            target2 = 0;
            pozitionare.setPosition(degreesToServoPositionPro(132));
        }

        if (gamepad1.b) {
            pula = false;
        } else if (gamepad2.left_bumper || gamepad2.right_bumper || gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.b || gamepad2.x || gamepad2.y) {
            pula = true;
        }

        if (pula) {
            isSlowMode = true;
        } else {
            isSlowMode = false;
        }

        if (gamepad1.right_trigger > 0.2) {
            target = target + 7;
        }
        if (gamepad1.left_trigger > 0.2) {
            target = target - 5;
        }

        elbowCheckPos = elbowMotor.getCurrentPosition();
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armCheckPos = armMotor.getCurrentPosition();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (gamepad1.right_bumper && !previousX2) {
            wrist2Angle += 30.0;
            if (wrist2Angle > 225.0) wrist2Angle = 225.0;
            pozitionare.setPosition(degreesToServoPositionPro(wrist2Angle));
        }
        previousX2 = gamepad2.right_bumper;

        if (gamepad1.left_bumper && !previousB2) {
            wrist2Angle -= 30.0;
            if (wrist2Angle < 0) wrist2Angle = 0;
            pozitionare.setPosition((degreesToServoPositionPro(wrist2Angle)));
        }
        previousB2 = gamepad2.left_bumper;

        if (gamepad1.a && !previousA) {
            isClawOpen = !isClawOpen;
        }

        previousA = gamepad1.a;

        if (isClawOpen) {
            gheara.setPosition(degreesToServoPosition(0));
        } else {
            gheara.setPosition(degreesToServoPosition(65));
        }

        telemetry.addData("posArm", armMotor.getCurrentPosition());
        telemetry.addData("target", target2);
        telemetry.addData("posElbow", elbowMotor.getCurrentPosition());
        telemetry.addData("target2", target);
        telemetry.addData("pula", pula);
        telemetry.addData("putere", speedMultiplier);
        telemetry.update();
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
