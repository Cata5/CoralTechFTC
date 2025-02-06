///VREAU SA INCEP PRIN A SPUNE CA DORESC SA MA OMOR DIN CAUZA ACESTEI COMPETITII NENOROCITE CARE MI A
/// MANCAT TOT SUFLETUL, AM AJUNS SA MERG LA PSIHOLOG DE 2 ORI PE SAPTAMANA, AM AJUNS SA DORESC SA MA TAI PE VENE
/// SA IMI IAU VIATA SI SA MA ARUNC IN CAP, AM INCEPUT SA IMI DORESC SA REVINA PERIOADA TOVARASULUI CEAUSESCU
/// IN CARE ACEASTA COMPETITIE NU AR FI PUTUT FI REALIZATA PE TERIORIUL ROMANIEI.
/// VIATA MEA A DEVENIT DIN CE IN CE MAI REA DIN CAUZA ACESTUI CONCURS, CODUL MEU ESTE PLIN DE LACRIMI DE DURERE
/// SI DE SUFERINTA DIN CAUZA CA NU MERGE AUTONOMIA, MI SE DA ROBOTUL CU BRATUL DE PAMANT, FACE CERCULETE DIN CAUZA
/// PID ULUI, AM IN CAMERA O MICA POZA CU CEAUSESCU, VESNICUL MEU PRIETEN
/// DRAG FTC ROMANIA, VA DORESC ZILE PLINE DE FERICIRE SI IUBIREA, CEEA CE EU NU AM AVUT PARTE DE CAND M AM BAGAT IN CONCURS!!!
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp(name = "28260", group = "FINAL")
public class COD_FINAL_28260 extends OpMode {
    private DcMotorEx glisieraStanga, glisieraDreapta;
    private DcMotorEx motorBratStanga, motorBratDreapta;
    private Servo gheara, pozitionare_gheara;
    private PIDController controller_unghi, controler_glisiere;

    private final int GLISIERE_LIMIT_MIN = 0;
    private final int GLISIERE_LIMIT_MAX = 3000;
    private final int BRAT_LIMIT_MIN = 0;
    private final int BRAT_LIMIT_MAX = 3000;
    private final double ticks_in_degrees = 1;

    public static int target_unghi = 0;
    public static double p_unghi = 0, i_unghi = 0, d_unghi = 0, f_unghi = 0;
    public static int target_glisiere = 0;
    public static double p_glisiere = 0, i_glisiere = 0, d_glisiere = 0, f_glisiere = 0;

    ///GHEARA UNGHI
    public Servo rotatieL, rotatieR;
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
    @Override
    public void init() {
        controler_glisiere = new PIDController(p_glisiere, i_glisiere, d_glisiere);
        controller_unghi = new PIDController(p_unghi, i_unghi, d_unghi);

        glisieraStanga = hardwareMap.get(DcMotorEx.class, "glisieraStanga");
        glisieraDreapta = hardwareMap.get(DcMotorEx.class, "glisieraDreapta");
        motorBratStanga = hardwareMap.get(DcMotorEx.class, "motorBratStanga");
        motorBratDreapta = hardwareMap.get(DcMotorEx.class, "motorBratDreapta");

        glisieraStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        glisieraDreapta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBratDreapta.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBratStanga.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        glisieraStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        glisieraDreapta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBratStanga.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBratDreapta.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rotatieL = hardwareMap.get(Servo.class, "rotatieL");
        rotatieR = hardwareMap.get(Servo.class, "rotatieR");

        rotatieL.setPosition(degreesToServoPositionPro(0));
        rotatieR.setPosition(degreesToServoPositionPro(270));

        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        target_glisiere = 0;
        target_unghi = 0;
    }

    @Override
    public void loop() {
        controler_glisiere.setPID(p_glisiere, i_glisiere, d_glisiere);
        controller_unghi.setPID(p_unghi, i_unghi, d_unghi);

        int unghi_pos = motorBratDreapta.getCurrentPosition();
        int glisiere_pos = glisieraDreapta.getCurrentPosition();

        double pid_unghi = controller_unghi.calculate(unghi_pos, target_unghi);
        double pid_glisiere = controler_glisiere.calculate(glisiere_pos, target_glisiere);

        double ff_unghi = Math.cos(Math.toRadians(target_unghi / ticks_in_degrees)) * f_unghi;
        double ff_glisiere = Math.cos(Math.toRadians(target_glisiere / ticks_in_degrees)) * f_glisiere;

        double power_unghi = pid_unghi * ff_unghi;
        double power_glisiere = pid_glisiere * ff_glisiere;

        motorBratDreapta.setPower(power_unghi);
        motorBratStanga.setPower(power_unghi);
        glisieraStanga.setPower(power_glisiere);
        glisieraDreapta.setPower(power_glisiere);

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

        // Control manual PIDF prin touchpad
        double touchpadInputGlisiere = gamepad2.touchpad_finger_2_x;
        if (Math.abs(touchpadInputGlisiere) > 0.1) {
            target_glisiere += touchpadInputGlisiere * 10;
            target_glisiere = Math.max(GLISIERE_LIMIT_MIN, Math.min(target_glisiere, GLISIERE_LIMIT_MAX));
        }

        double touchpadInputBrat = gamepad2.touchpad_finger_2_y;
        if (Math.abs(touchpadInputBrat) > 0.1) {
            target_unghi += touchpadInputBrat * 10;
            target_unghi = Math.max(BRAT_LIMIT_MIN, Math.min(target_unghi, BRAT_LIMIT_MAX));
        }
        boolean currentLT = gamepad2.left_trigger > 0.5;
        boolean currentRT = gamepad2.right_trigger > 0.5;

        if (currentLT && !previousLT) {
            pozitieRotatieL += 90;
            pozitieRotatieR -= 90;

            if (pozitieRotatieL > 180 && pozitieRotatieR < 90) {
                pozitieRotatieL = 270;
                pozitieRotatieR = 0;
            }

            rotatieL.setPosition(degreesToServoPositionPro(pozitieRotatieL));
            rotatieR.setPosition(degreesToServoPositionPro(pozitieRotatieR));
        }

        if (currentRT && !previousRT) {
            pozitieRotatieL -= 90;
            pozitieRotatieR += 90;

            if (pozitieRotatieL < 90 && pozitieRotatieR > 180) {
                pozitieRotatieL = 0;
                pozitieRotatieR = 270;
            }

            rotatieL.setPosition(degreesToServoPositionPro(pozitieRotatieL));
            rotatieR.setPosition(degreesToServoPositionPro(pozitieRotatieR));
        }

        previousLT = currentLT;
        previousRT = currentRT;
    }

    private void setBratGlisieraPosition(int newTargetUnghi, int newTargetGlisiere) {
        target_unghi = Math.max(BRAT_LIMIT_MIN, Math.min(newTargetUnghi, BRAT_LIMIT_MAX));
        target_glisiere = Math.max(GLISIERE_LIMIT_MIN, Math.min(newTargetGlisiere, GLISIERE_LIMIT_MAX));
    }
    private double degreesToServoPositionPro(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 270.0;
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }
}
