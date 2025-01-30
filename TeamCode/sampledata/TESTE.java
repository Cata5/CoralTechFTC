package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TESTE cu Field-Centric", group="Robo")
public class TESTE extends OpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private boolean isSlowMode = false; // Mod viteză redusă
    private boolean previousY = false; // Stare anterioară a butonului Y
    private IMU imu;  // Sensorul IMU pentru orientarea robotului

    private double headingOffset = 0; // Offset pentru resetarea orientării
    private double robotHeading = 0; // Orientarea actuală a robotului

    @Override
    public void init() {
        // Initializam fiecare motor cu hardware map
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Initializare IMU pentru field-centric
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        // Directia motoarelor
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Resetare orientare cu butonul X
        if (gamepad1.x) {
            headingOffset = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
        }

        // Actualizăm orientarea robotului
        robotHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle - headingOffset;

        // Toggle pentru butonul Y
        if (gamepad1.y && !previousY) { // Detectăm o apăsare unică
            isSlowMode = !isSlowMode; // Inversăm starea slow mode
        }
        previousY = gamepad1.y; // Actualizăm starea anterioară

        // Valorile de la joystick
        double drive = -gamepad1.left_stick_y;    // Forward/backward
        double strafe = gamepad1.left_stick_x;    // Left/right
        double rotate = gamepad1.right_stick_x;   // Rotation (turning)
        double speedMultiplier = isSlowMode ? 0.4 : 2.0; // Ajustăm multiplier-ul în funcție de slow mode

        // Transformare coordonate pentru field-centric
        double rotatedX = strafe * Math.cos(-robotHeading) - drive * Math.sin(-robotHeading);
        double rotatedY = strafe * Math.sin(-robotHeading) + drive * Math.cos(-robotHeading);

        // Calculăm puterea fiecărui motor cu amplificare
        double frontLeftPower = rotatedY + rotatedX + rotate;
        double frontRightPower = rotatedY - rotatedX - rotate;
        double backLeftPower = rotatedY - rotatedX + rotate;
        double backRightPower = rotatedY + rotatedX - rotate;

        // Normalizam pentru a asigura că valorile puterii motoarelor nu depășesc 1.0
        double maxPower = Math.max(2.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftMotor.setPower((frontLeftPower / maxPower) * speedMultiplier);
        frontRightMotor.setPower((frontRightPower / maxPower) * speedMultiplier);
        backLeftMotor.setPower((backLeftPower / maxPower) * speedMultiplier);
        backRightMotor.setPower((backRightPower / maxPower) * speedMultiplier);

        // Telemetry pentru debug
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.addData("Slow Mode Active", isSlowMode);
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Heading Offset", headingOffset);
        telemetry.update();
    }
}
