package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Robot");
public class RobotCoral extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Hardware: ", "functional");


    }

    @Override
    public void loop() {
        //telemetry.addData("Initializare:", "is a succes");
        float y = gamepad1.left_stick_y;
        //se poate modifica in if de jos x sau y in functie de axa pe care o dorim pe controler
        if (gamepad1.left_stick_y > 0) {
            //telemetry.addData("Test: ", "Merge");
            //telemetry.update();
            motor.setPower(y);
        }
        motor.setPower(0);
    }
}