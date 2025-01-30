package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "brat+sasiu", group = "Robo")
public class Robot_final extends OpMode {
    private Servo gheara;
    private boolean isClawOpen = false;
    private boolean previousA = false;
    // Mecanum drive motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Arm motors
    private DcMotor armMotor;
    private DcMotor elbowMotor;

    private boolean isSlowMode = false;
    private boolean previousY = false;
    private IMU imu;

    private boolean previousLB = false;
    private boolean previousRB = false;
    private boolean fieldcentric = false;

    private int degreeArmMotor;
    private int degreeElbowMotor;
    private int armError = 0;
    private int elbowError = 0;

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

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        // Initialize arm motors
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        gheara = hardwareMap.get(Servo.class, "wrist1");
        gheara.setPosition(degreesToServoPosition(90));

        // Set directions for arm and elbow motors
        armMotor.setDirection(DcMotor.Direction.FORWARD); // Sau REVERSE dacă mișcarea e inversă
        elbowMotor.setDirection(DcMotor.Direction.FORWARD); // Sau REVERSE dacă mișcarea e inversă

        degreeArmMotor = (int) (armMotor.getMotorType().getTicksPerRev() / 360.0);
        degreeElbowMotor = (int) (elbowMotor.getMotorType().getTicksPerRev() / 360.0);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Drive system controls
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double speedMultiplier = isSlowMode ? 0.55 : 0.85;

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        double botHeading = fieldcentric ? angles.firstAngle : 0.0;

        double rotatedX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotatedY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        double frontLeftPower = rotatedY + rotatedX + rotate;
        double frontRightPower = rotatedY - rotatedX - rotate;
        double backLeftPower = rotatedY - rotatedX + rotate;
        double backRightPower = rotatedY + rotatedX - rotate;

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

        if (gamepad1.left_bumper && !previousLB && fieldcentric) {
            imu.resetYaw();
        }
        previousLB = gamepad1.left_bumper;

//        if (gamepad1.right_bumper && !previousRB) {
//            fieldcentric = !fieldcentric;
//        }
//        previousRB = gamepad1.right_bumper;

        // Arm controls for preset positions
        if (gamepad2.y) {
            setPos(90, 80); // poziție default (sus mediu)
        } else if (gamepad2.x) {
            setPos(70, 35); // poziție default (sus jos)
        } else if (gamepad2.b) {
            setPos(-35, 0); // jos braț
        } else if (gamepad2.a) {
            setPos(0, 60); // poziție la sol
        } else if (gamepad2.dpad_up) {
            armMotor.setDirection(DcMotor.Direction.REVERSE); // Sau REVERSE dacă mișcarea e inversă
            elbowMotor.setDirection(DcMotor.Direction.FORWARD); // Sau REVERSE dacă mișcarea e inversă
            setPos(120, 100); // *Poziția High Basket (sus coș)*
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-gamepad2.right_stick_y / 3);
            elbowMotor.setPower(-gamepad2.left_stick_y / 3);
        }
        if (gamepad1.a && !previousA) {
            isClawOpen = !isClawOpen;
        }

        previousA = gamepad1.a;

        if (isClawOpen) {
            gheara.setPosition(degreesToServoPosition(90));
        } else {
            gheara.setPosition(degreesToServoPosition(40));
        }


        // Telemetry
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.addData("Slow Mode Active", isSlowMode);
        telemetry.addData("Current Heading", botHeading);
        telemetry.addData("Fieldcentric", fieldcentric);
        telemetry.update();
    }

    private boolean isAnyMotorRunning() {
        return armMotor.isBusy() || elbowMotor.isBusy();
    }
    private double degreesToServoPosition(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 180.0;
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }

    private void setPos(int armDegrees, int elbowDegrees) {
        int armTarget = degreeArmMotor * armDegrees + armError;
        int elbowTarget = degreeElbowMotor * elbowDegrees + elbowError;

        armMotor.setTargetPosition(armTarget);
        elbowMotor.setTargetPosition(elbowTarget);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.5);
        elbowMotor.setPower(0.2);
    }
}