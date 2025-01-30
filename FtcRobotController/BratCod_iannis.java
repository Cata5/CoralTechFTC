package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Braț Menținere Poziție", group = "TeleOp")
public class BratCod_iannis extends OpMode {
    private DcMotor armMotor;
    private DcMotor elbowMotor;

    private int armTargetPosition;
    private int elbowTargetPosition;

    private int armTicksPerDegree;
    private int elbowTicksPerDegree;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");

        armMotor.setTargetPosition(0);
        elbowMotor.setTargetPosition(0);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armTicksPerDegree = (int) (armMotor.getMotorType().getTicksPerRev() / 360.0);
        elbowTicksPerDegree = (int) (elbowMotor.getMotorType().getTicksPerRev() / 360.0);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        elbowMotor.setDirection(DcMotor.Direction.FORWARD);

        armTargetPosition = armMotor.getCurrentPosition();
        elbowTargetPosition = elbowMotor.getCurrentPosition();

        armMotor.setPower(0.5);
        elbowMotor.setPower(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            elbowTargetPosition = elbowTicksPerDegree * 360; // Mută doar elbow la 30°
            elbowMotor.setTargetPosition(elbowTargetPosition);
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.b) {
            armTargetPosition = armTicksPerDegree * 120;  // Mută arm la 120°
            elbowTargetPosition = elbowTicksPerDegree * 120;  // Mută elbow la 120°
        } else {
            // Menține poziția actuală pentru armMotor și elbowMotor
            armTargetPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(armTargetPosition);
            elbowTargetPosition = elbowMotor.getCurrentPosition();
            elbowMotor.setTargetPosition((elbowTargetPosition));
        }

        // Updatează pozițiile țintă
        armMotor.setTargetPosition(armTargetPosition);
        elbowMotor.setTargetPosition(elbowTargetPosition);

        telemetry.addData("Arm Target Position (ticks)", armTargetPosition);
        telemetry.addData("Elbow Target Position (ticks)", elbowTargetPosition);
        telemetry.addData("Arm Current Position (ticks)", armMotor.getCurrentPosition());
        telemetry.addData("Elbow Current Position (ticks)", elbowMotor.getCurrentPosition());
        telemetry.update();
    }
}
