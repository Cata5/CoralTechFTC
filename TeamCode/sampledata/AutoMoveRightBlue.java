package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoMoveRightBlue extends OpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private double speed;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Inițializăm fiecare motor cu hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Setăm direcțiile motoarelor
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        speed = 1.0;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        performMovement(this::turnRight, 500, "Strafing right");
        stopAll();
        performMovement(this::forward, 1500, "Going forward");
        stopAll();
        performMovement(this::turnRight, 500, "Going right for sample 1");
        stopAll();
        performMovement(this::backwards, 1500, "Going backwords to the human zone");
        stopAll();
        performMovement(this::forward, 1500, "Going forward for sample no 2");
        stopAll();
        performMovement(this::turnRight, 350, "Going right to position the robot " +
                "for the sample no 2");
        stopAll();
        performMovement(this::backwards, 1500, "Going backwords to the human zone");
        stopAll();
    }

    @Override
    public void loop() {
        telemetry.addData("Runtime", runtime.seconds());
    }

    private void performMovement(Runnable movement, int durationMs, String actionDescription) {
        telemetry.addData("Action", actionDescription);
        telemetry.update();

        runtime.reset();
        movement.run();

        // Așteaptă durata specificată
        while (runtime.milliseconds() < durationMs) {
            // Continuă bucla pentru a păstra mișcarea activă
            telemetry.addData("Action", actionDescription);
            telemetry.addData("Time Remaining", durationMs - runtime.milliseconds());
            telemetry.update();
        }

        stopAll(); // Oprește motoarele după ce timpul a expirat
    }

    private void forward() {
        setMotorPower(speed, speed, speed, speed);
    }

    private void backwards() {
        setMotorPower(-speed, -speed, -speed, -speed);
    }

    private void turnRight() {
        setMotorPower(speed, -speed, speed, -speed);
    }

    private void turnLeft() {
        setMotorPower(-speed, speed, -speed, speed);
    }

    private void strafeLeft() {
        setMotorPower(-speed, speed, speed, -speed);
    }

    private void strafeRight() {
        setMotorPower(speed, -speed, -speed, speed);
    }

    private void stopAll() {
        setMotorPower(0, 0, 0, 0);
    }

    private void setMotorPower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
