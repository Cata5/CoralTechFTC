package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmControl extends OpMode {
    // Definirea componentelor hardware
    private DcMotor motionMotor;    // Motor pentru mișcarea brațului (rotire)
    private DcMotor extendMotor;    // Motor pentru extinderea brațului (lungire)
    private Servo clawServo;        // Servo pentru controlul ghearei

    // Poziții predefinite pentru gheară
    private static final double CLAW_OPEN_POSITION = 0.5;  // Poziția deschisă a ghearei (ajustează după necesități)
    private static final double CLAW_CLOSED_POSITION = 0.2; // Poziția închisă a ghearei

    // Variabilă pentru a verifica dacă robotul este în mișcare
    private boolean isMoving = false;

    @Override
    public void init() {
        // Inițializarea hardware-ului, asocierea componentelor cu hardware-ul definit în robot
        motionMotor = hardwareMap.get(DcMotor.class, "ArmMotionMotor"); // Găsește motorul de mișcare al brațului
        extendMotor = hardwareMap.get(DcMotor.class, "ArmExtendMotor"); // Găsește motorul de extindere al brațului
        clawServo = hardwareMap.get(Servo.class, "ClawServo"); // Găsește servo-ul pentru gheară

        // Configurarea motoarelor pentru a răspunde direct la mișcarea joystick-ului
        motionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Permite motorului să răspundă la puterea dată continuu, folosind encoderul
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Permite motorului să răspundă la puterea dată continuu, folosind encoderul

        telemetry.addData("Status", "Initialized"); // Afișează pe ecranul driver-ului statusul de inițializare
    }

    @Override
    public void loop() {
        // Controle pentru mișcarea brațului
        double armPower = -gamepad2.left_stick_y; // Preia valoarea de pe joystick-ul stâng pentru mișcarea brațului (negativ pentru direcția corectă)
        motionMotor.setPower(armPower); // Setează puterea motorului de mișcare a brațului conform joystick-ului

        // Controle pentru extinderea brațului
        double extendPower = gamepad2.right_stick_y; // Preia valoarea de pe joystick-ul drept pentru extinderea brațului
        extendMotor.setPower(extendPower); // Setează puterea motorului de extindere a brațului conform joystick-ului

        // Controle pentru gheară
        if (gamepad2.right_trigger > 0.1) { // Dacă este apăsat trigger-ul din dreapta mai mult de 10%
            clawServo.setPosition(CLAW_OPEN_POSITION); // Setează gheara la poziția deschisă
        } else if (gamepad2.left_trigger > 0.1) { // Dacă este apăsat trigger-ul din stânga mai mult de 10%
            clawServo.setPosition(CLAW_CLOSED_POSITION); // Setează gheara la poziția închisă
        }

        // Adăugăm un nou comportament pentru butonul "Y" de pe gamepad2
        if (gamepad2.y && !isMoving) { // Dacă butonul Y este apăsat și robotul nu este în mișcare
            isMoving = true; // Setează că robotul este în mișcare

            // Setează pozițiile țintă pentru motorul de mișcare și extindere
            motionMotor.setTargetPosition(1500); // Poziția dorită pentru motorul de mișcare
            motionMotor.setPower(0.8); // Setează puterea motorului de mișcare

            extendMotor.setTargetPosition(800); // Poziția dorită pentru motorul de extindere
            extendMotor.setPower(0.8); // Setează puterea motorului de extindere

            // Poziția servo-ului (de exemplu, pentru a închide gheara)
            clawServo.setPosition(CLAW_OPEN_POSITION); // Setează gheara la poziția intermediară
        }

        // Verificăm dacă brațul a ajuns la pozițiile țintă
        if (isMoving && !motionMotor.isBusy() && !extendMotor.isBusy()) {
            isMoving = false; // Oprește mișcarea după ce motoarele au ajuns la țintă
        }

        // Telemetrie pentru a vizualiza statusul pe ecranul driver-ului
        telemetry.addData("Arm Power", armPower); // Afișează puterea aplicată motorului brațului
        telemetry.addData("Extend Power", extendPower); // Afișează puterea aplicată motorului de extindere
        telemetry.addData("Claw Position", clawServo.getPosition()); // Afișează poziția curentă a ghearei
        telemetry.addData("Is Moving", isMoving ? "Yes" : "No"); // Afișează dacă robotul este în mișcare
        telemetry.update(); // Actualizează afișajul telemetriei pe ecranul driver-ului
    }
}
