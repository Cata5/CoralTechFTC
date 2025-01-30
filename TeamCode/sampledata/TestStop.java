///Sa va bag optimizarea adanc in cur pana va iese prin gura si ajunge in ma ta
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

@TeleOp(name="TESTE cu Field-Centric si PID", group="Robo")
public class TestStop extends OpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private boolean isSlowMode = false; // Mod viteză redusă
    private boolean previousY = false; // Stare anterioară a butonului Y
    private IMU imu;  // Sensorul IMU pentru orientarea robotului

    private boolean previousLB = false; // Stare anterioară pentru butonul LB (Left Bumper)
    private boolean previousRB = false; // Stare anterioară pentru butonul LB (Left Bumper)
    private boolean fieldcentric = true;

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
        // PID constants declared as local variables inside loop()
        double kP = 0.0003;
        double kI = 0.0001;
        double kD = 0.0005;
        double previousError = 0;
        double integral = 0;

        // Toggle pentru butonul Y
        if (gamepad1.y && !previousY) { // Detectăm o apăsare unică
            isSlowMode = !isSlowMode; // Inversăm starea slow mode
        }
        previousY = gamepad1.y; // Actualizăm starea anterioară

        // Resetarea orientării robotului la apăsarea butonului LB
        if (gamepad1.left_bumper && !previousLB && fieldcentric) { // Detectăm o apăsare unică a butonului LB
            imu.resetYaw(); // Resetăm orientarea robotului (Yaw)
        }
        previousLB = gamepad1.left_bumper; // Actualizăm starea anterioară pentru LB

        if (gamepad1.right_bumper && !previousRB) {
            fieldcentric = !fieldcentric;
        }

        // Valorile de la joystick
        double drive = -gamepad1.left_stick_y;    // Forward/backward
        double strafe = -gamepad1.left_stick_x;    // Left/right
        double rotate = gamepad1.right_stick_x;   // Rotation (turning)
        double speedMultiplier = isSlowMode ? 0.7 : 0.9; // Ajustăm multiplier-ul în funcție de slow mode + putere mai mare daca schimb 1.5

        // Obținem orientarea curentă a robotului din IMU
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        double botHeading;
        if (fieldcentric){
            botHeading = angles.firstAngle;
        } else {
            botHeading = 0.0;
        }
        // Transformare coordonate pentru field-centric
        double rotatedX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotatedY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        // Calculăm puterea fiecărui motor cu amplificare
        double frontLeftPower = rotatedY + rotatedX + rotate;
        double frontRightPower = rotatedY - rotatedX - rotate;
        double backLeftPower = rotatedY - rotatedX + rotate;
        double backRightPower = rotatedY + rotatedX - rotate;

        // Normalizam pentru a asigura că valorile puterii motoarelor nu depășesc 1.0 și că robotul funcționează corect
        double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        // Aplicăm PID pentru a corecta erorile de mișcare dreaptă
        if (Math.abs(rotate) < 0.1 && (Math.abs(drive) > 0.1 || Math.abs(strafe) > 0.1)) {
            // Calculam eroarea direct în formula PID
            double error = (frontLeftMotor.getCurrentPosition() - frontRightMotor.getCurrentPosition()) / 2.0;
            integral += error;
            double derivative = error - previousError;

            double adjustment = (kP * error) + (kI * integral) + (kD * derivative);

            frontLeftPower -= adjustment;
            frontRightPower += adjustment;
            backLeftPower -= adjustment;
            backRightPower += adjustment;

            previousError = error;
        }

        // Aplicăm puterea motoarelor cu multiplier-ul pentru viteză
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
        telemetry.addData("Current Heading", botHeading);  // Adăugăm unghiul curent
        telemetry.addData("PID Error", previousError);  // Adăugăm eroarea PID
        telemetry.update();
    }
}

