///Ma cheama Rares si vreau sa incep prin a spune ca imi bag toata pula in acest concurs, m-a lasat fara familie, prieteni, relatie, si cu
///  ganduri sinucigase, plang in fiecare seara ca nu merge autonomia si PID-ul, si nici roadrunner-ul, asa ca cine citeste asta
/// ma suge de cioaca, imi bag toata ciocarlia in scorbura, in concursul vostru si in tot ce tine de ftc, daca o sa ma intrebe cineva
/// ce inseamna ftc-ul pt mine am sa ii zic ca e o pula, e un concurs de cacat care te face sa te tai pe vene, sa te certi cu ma-ta
/// ca ai o autonomie de cacat, tu lucrezi la ea si ea te cheama la masa, dar tu ii zici sa iti manance toata pula, iar apoi dormi afara.
/// IN CONCLUZIE, FTC-ul e un concurs de cacat. un concurs de comunisti, realizat de Tovarasul Ceausescu, pentru a scapa de populatia
/// tarii noastre democrate !!!!
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

@TeleOp(name="PID_V1", group="Robo")
public class RobotPID extends OpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private boolean isSlowMode = false; // Mod viteza redusa
    private boolean previousY = false; // Stare anterioara a butonului Y
    private IMU imu;  // Sensorul IMU pentru orientarea robotului

    private boolean previousLB = false; // Stare anterioara pentru butonul LB (Left Bumper)

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
        if (gamepad1.y && !previousY) { // Detectam o apasare unica
            isSlowMode = !isSlowMode; // Inversam starea slow mode
        }
        previousY = gamepad1.y; // Actualizam starea anterioara

        // Resetarea orientarii robotului la apasarea butonului LB
        if (gamepad1.left_bumper && !previousLB) { // Detectam o apasare unica a butonului LB
            imu.resetYaw(); // Resetam orientarea robotului (Yaw)
        }
        previousLB = gamepad1.left_bumper; // Actualizam starea anterioara pentru LB

        // Valorile de la joystick
        boolean Brake = !gamepad1.right_bumper; // boolean pentru brake (se tine apasat)
        int BrakeCheck = Brake ? 1 : 0;
        double drive = -gamepad1.left_stick_y * BrakeCheck;    // Forward/backward
        double strafe = -gamepad1.left_stick_x * BrakeCheck;    // Left/right
        double rotate = gamepad1.right_stick_x * BrakeCheck;   // Rotation (turning)
        double speedMultiplier = isSlowMode ? 0.4 : 0.9; // Ajustam multiplier-ul in functie de slow mode + putere mai mare daca schimb 1.5

        // Obtinem orientarea curenta a robotului din IMU
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        double botHeading = angles.firstAngle;  // Folosim `firstAngle` pentru unghiul pe axa Z

        // Transformare coordonate pentru field-centric
        double rotatedX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotatedY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        // Calculam puterea fiecarui motor cu amplificare
        double frontLeftPower = rotatedY + rotatedX + rotate;
        double frontRightPower = rotatedY - rotatedX - rotate;
        double backLeftPower = rotatedY - rotatedX + rotate;
        double backRightPower = rotatedY + rotatedX - rotate;

        // Normalizam pentru a asigura ca valorile puterii motoarelor nu depasesc 1.0 si ca robotul functioneaza corect
        double maxPower = Math.max(0.4, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        // Aplicam PID pentru a corecta erorile de miscare dreapta
        if (Math.abs(rotate) < 0.1 && (Math.abs(drive) > 0.1 || Math.abs(strafe) > 0.1)) {
            // Calculam eroarea direct in formula PID
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

        // Aplicam puterea motoarelor cu multiplier-ul pentru viteza
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
        telemetry.addData("Current Heading", botHeading);  // Adaugam unghiul curent
        telemetry.addData("PID Error", previousError);  // Adaugam eroarea PID
        telemetry.update();
    }
}
