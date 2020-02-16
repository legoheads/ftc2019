package org.firstinspires.ftc.teamcode.subsystems.stacker;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

public class stacker {

    //Gripper
    private Servo gripper;
    private double OPEN_POS = 0.55;
    private double CLOSED_POS = 0.1;
//    private double CAP_POS = 0.05;

    //Cantilever left
    private Servo cantileverLeft;
    private double EXTEND_POS_LEFT = 0.0;
    private double INTAKE_POS_LEFT = 0.95;

    //Cantilever right
    private Servo cantileverRight;
    private double EXTEND_POS_RIGHT = 0.9;
    private double INTAKE_POS_RIGHT = 0.2;

    //Capstone
    private Servo capstone;
    private double INIT_POS = 0.45;
    private double CAP_POS = 0.0;

    //Color Sensor V2s
    private DistanceSensor stoneDistLeft;
    private DistanceSensor stoneDistRight;
    private DistanceSensor stoneDistLow;

    private HardwareMap hardwareMap;

    private LinearSlides slides;
    private skystoneChassis chassis;

    Gamepad gamepad1, gamepad2;


    public stacker(HardwareMap hardwareMap) throws InterruptedException {

        this.hardwareMap = hardwareMap;

        gripper = hardwareMap.servo.get("gripper");
        cantileverLeft = hardwareMap.servo.get("cantileverLeft");
        cantileverRight = hardwareMap.servo.get("cantileverRight");
        capstone = hardwareMap.servo.get("capstone");


        //Back distance sensors
        stoneDistLeft = hardwareMap.get(DistanceSensor.class, "stoneDistLeft");
        stoneDistRight = hardwareMap.get(DistanceSensor.class, "stoneDistRight");
        stoneDistLow = hardwareMap.get(DistanceSensor.class, "stoneDistLow");

        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);

        init();

    }

    public void init() throws InterruptedException
    {
        gripper.setPosition(OPEN_POS);
        cantileverLeft.setPosition(INTAKE_POS_LEFT);
        cantileverRight.setPosition(INTAKE_POS_RIGHT);
        capstone.setPosition(INIT_POS);
    }

    public void grab() throws InterruptedException {
        cantileverLeft.setPosition(INTAKE_POS_LEFT);
        cantileverRight.setPosition(INTAKE_POS_RIGHT);
        Thread.sleep(200);
        gripper.setPosition(CLOSED_POS);
    }

    public void ungrab()
    {
        gripper.setPosition(OPEN_POS);
    }

    public void extend() throws InterruptedException
    {
        cantileverLeft.setPosition(EXTEND_POS_LEFT);
        cantileverRight.setPosition(EXTEND_POS_RIGHT);
    }

    public void retract() throws InterruptedException
    {
        gripper.setPosition(OPEN_POS);
        Thread.sleep(100);
        slides.spoolEncoder(0.8, 40);
        chassis.driveForwardsAutonomous(0.4, 400);
        cantileverLeft.setPosition(INTAKE_POS_LEFT);
        cantileverRight.setPosition(INTAKE_POS_RIGHT);
        slides.spoolEncoder(-0.8, -40);
    }

    public void cap() throws InterruptedException
    {
        grab();
        capstone.setPosition(CAP_POS);
    }
}