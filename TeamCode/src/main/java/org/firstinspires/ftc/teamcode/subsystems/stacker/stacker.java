package org.firstinspires.ftc.teamcode.subsystems.stacker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

public class stacker {

    //Gripper
    private Servo gripper;
    private double OPEN_POS = 0.45;
    private double HALF_POS = 0.52;
    private double CLOSED_POS = 0.55;
//    private double CAP_POS = 0.05;

    //Cantilever left
    private Servo cantiliverLeft;
    private double EXTEND_POS_LEFT = 0.0;
    private double INTAKE_POS_LEFT = 0.75;
    private double GRAB_POS_LEFT = 0.85;

    //Cantilever right
    private Servo cantiliverRight;
    private double EXTEND_POS_RIGHT = 1.0;
    private double INTAKE_POS_RIGHT = 0.25;
    private double GRAB_POS_RIGHT = 0.15;

    //Color Sensor V2s
    private DistanceSensor stoneDistLeft;
    private DistanceSensor stoneDistRight;
    private DistanceSensor stoneDistLow;

    private HardwareMap hardwareMap;

    private LinearSlides slides;
    private skystoneChassis chassis;

    Gamepad gamepad1, gamepad2;


    public stacker(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {

        this.hardwareMap = hardwareMap;

        gripper = hardwareMap.servo.get("gripper");
        cantiliverLeft = hardwareMap.servo.get("cantileverLeft");
        cantiliverRight = hardwareMap.servo.get("cantileverRight");


        //Back distance sensors
        stoneDistLeft = hardwareMap.get(DistanceSensor.class, "stoneDistLeft");
        stoneDistRight = hardwareMap.get(DistanceSensor.class, "stoneDistRight");
        stoneDistLow = hardwareMap.get(DistanceSensor.class, "stoneDistLow");

        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        init();

    }

    public void init() throws InterruptedException
    {
        cantiliverLeft.setPosition(INTAKE_POS_LEFT);
        cantiliverRight.setPosition(INTAKE_POS_RIGHT);
        gripper.setPosition(OPEN_POS);
    }

    public void grab() throws InterruptedException
    {
        cantiliverLeft.setPosition(GRAB_POS_LEFT);
        cantiliverRight.setPosition(GRAB_POS_RIGHT);
        Thread.sleep(200);
        gripper.setPosition(CLOSED_POS);

    }

    public void extend() throws InterruptedException
    {
        cantiliverLeft.setPosition(EXTEND_POS_LEFT);
        cantiliverRight.setPosition(EXTEND_POS_RIGHT);
    }

    public void retract() throws InterruptedException
    {
        gripper.setPosition(HALF_POS);
        Thread.sleep(100);
        cantiliverLeft.setPosition(INTAKE_POS_LEFT);
        cantiliverRight.setPosition(INTAKE_POS_RIGHT);
        gripper.setPosition(OPEN_POS);
    }

//    public void capDrop() throws InterruptedException
//    {
//        gripper.setPosition(CAP_POS);
//    }

    public void stoneShiftLeft() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        double SHIFT_POWER = 0.3;
        runTime.reset();

        while (!(stoneDistRight.getDistance(DistanceUnit.INCH)<5) && runTime.time()<4 || gamepad1.back){
            chassis.chassisTeleOp(gamepad1, gamepad2, SHIFT_POWER);
            chassis.shiftTeleop(SHIFT_POWER);
        }
        chassis.stopDriving();
    }

    public void stoneShiftRight() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        double SHIFT_POWER = -0.3;
        runTime.reset();

        while (!(stoneDistLeft.getDistance(DistanceUnit.INCH)<5) && runTime.time()<4 || gamepad1.back){
            chassis.chassisTeleOp(gamepad1, gamepad2, SHIFT_POWER);
            chassis.shiftTeleop(SHIFT_POWER);
        }
        chassis.stopDriving();
    }

    public void platformReverse() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        double drivePower = -0.2;

        while (!(stoneDistLow.getDistance(DistanceUnit.INCH)<2.5) && runTime.time()<4 || gamepad1.back){
            chassis.chassisTeleOp(gamepad1, gamepad2, -drivePower);
            chassis.driveTeleop(-drivePower);
        }
        chassis.stopDriving();
    }
}