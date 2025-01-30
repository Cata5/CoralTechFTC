package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test extends OpMode {
    private DcMotor glisiereL;
    private DcMotor glisiereR;
    private DcMotor armMotor;
    private Servo servoClaw;
    private Servo servoClawPos;
    private double servoClawPosSpeed = 0.05;
    private double ticksPerRotationGL = glisiereL.getMotorType().getTicksPerRev();
    private double ticksPerRotationGR = glisiereR.getMotorType().getTicksPerRev();
    private double ticksPerRotationArm = armMotor.getMotorType().getTicksPerRev();
    private double DegreesPerMM = 1.7;
    private double CLAW_OPEN = 0.7;
    private double CLAW_CLOSE = 0.3;
    private int MAX_EXTEND = 500;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        glisiereL = hardwareMap.get(DcMotor.class, "glisiereL");
        glisiereR = hardwareMap.get(DcMotor.class, "glisiereR");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        glisiereR.setDirection(DcMotorSimple.Direction.REVERSE);
        glisiereL.setDirection(DcMotorSimple.Direction.FORWARD);

        setModePreset();
        setPosition0();
    }

    @Override
    public void loop() {
        //high basket
        if(gamepad2.y && !isAnyMotorRunning()){
            setModePreset();
            armMotor.setTargetPosition(0);
            glisiereR.setTargetPosition(MMConverter(MAX_EXTEND));
            glisiereL.setTargetPosition(MMConverter(MAX_EXTEND));
            servoClawPos.setPosition(ServoConverter(80));
        }
        //low basket
        if(gamepad2.x && !isAnyMotorRunning()){
            setModePreset();
            armMotor.setTargetPosition((TickConverter(ticksPerRotationArm, 40)));
            glisiereL.setTargetPosition(MMConverter(400));
            glisiereR.setTargetPosition(MMConverter(400));
            servoClawPos.setPosition(ServoConverter(50));
        }
        //Ground sample
        if(gamepad2.b && !isAnyMotorRunning()){
            setModePreset();
            armMotor.setTargetPosition(TickConverter(ticksPerRotationArm, 90));
            glisiereL.setTargetPosition(MMConverter(0));
            glisiereR.setTargetPosition(MMConverter(0));
            servoClawPos.setPosition(ServoConverter(25));
        }
        //Pit sample
        if(gamepad2.a && !isAnyMotorRunning()) {
            setModePreset();
            armMotor.setTargetPosition(TickConverter(ticksPerRotationArm, 90));
            glisiereL.setTargetPosition(MMConverter(150));
            glisiereR.setTargetPosition(MMConverter(150));
            servoClawPos.setPosition(ServoConverter(90));
        }
        else { //Manual controls
            setModeDynamic();
            //Manual arm
            double armPower = -gamepad2.left_stick_y; // Preia valoarea de pe joystick-ul stâng pentru mișcarea brațului (negativ pentru direcția corectă)
            armMotor.setPower(armPower); // Setează puterea motorului de mișcare a brațului conform joystick-ului
            //Claw
            if (gamepad2.right_trigger > 0.1) { // Dacă este apăsat trigger-ul din dreapta mai mult de 10%
                servoClaw.setPosition(CLAW_OPEN); // Setează gheara la poziția deschisă
            } else if (gamepad2.left_trigger > 0.1) { // Dacă este apăsat trigger-ul din stânga mai mult de 10%
                servoClaw.setPosition(CLAW_CLOSE); // Setează gheara la poziția închisă
            }
            //Manual extend
            double extend = -gamepad2.right_stick_y;
            glisiereL.setPower(extend);
            glisiereR.setPower(extend);
            //Manual claw position
            if(gamepad2.left_bumper){
                servoClawPos.setPosition(servoClawPos.getPosition()+servoClawPosSpeed);
            }if(gamepad2.right_bumper){
                servoClawPos.setPosition(servoClawPos.getPosition()-servoClawPosSpeed);
            }
        }
    }

    private void setModePreset(){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiereL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiereR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower();
    }

    private void setPower(){
        armMotor.setPower(0.2);
        glisiereR.setPower(0.1);
        glisiereL.setPower(0.1);
    }

    private void setPosition0(){
        armMotor.setTargetPosition(0);
        glisiereL.setTargetPosition(0);
        glisiereR.setTargetPosition(0);
    }

    private int TickConverter(double ticks, double degrees){
        int n = (int)(ticks / 360 * degrees);
        return n;
    }
    private int MMConverter(double mm){
        return (int)(DegreesPerMM * mm);
    }

    private double ServoConverter(int degrees)
    {
        return 1 / 360 * degrees;
    }
    private void setModeDynamic(){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glisiereL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glisiereR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean isAnyMotorRunning(){
        if(armMotor.isBusy() || glisiereL.isBusy() || glisiereR.isBusy()){ return true;}
        else{ return false;}
    }
}