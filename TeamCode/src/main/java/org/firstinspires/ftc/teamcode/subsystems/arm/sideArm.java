package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class sideArm implements Arm
{
    private Servo sideLift;
    private Servo twister;
    private Servo sideGrab;

    private HardwareMap hardwareMap;

    //Positions for Lifter
    private final double AUTO_START = 0.5;
    private final double TELE_START = 0.15;
    private final double LIFT_POSITION = 0.3;
    private final double DOWN_POSITION = 0.1;

    //Positions for Twister
    private final double PERPENDICULAR_POSITION = 0.9;
    private final double PARALLEL_POSITION = 0.5;

    //Positions for Grabber
    private final double OPEN_POSITION = 0.5;
    private final double GRAB_POSITION = 0.7;
    private final double CLOSED_POSITION = 0.9;


    public sideArm(HardwareMap hardwareMap, Servo sideLift, Servo twister, Servo sideGrab)
    {
        this.sideLift = sideLift;
        this.twister = twister;
        this.sideGrab = sideGrab;

        this.hardwareMap = hardwareMap;

        sideLift = hardwareMap.servo.get("sideLift");
        twister = hardwareMap.servo.get("twister");
        sideGrab = hardwareMap.servo.get("sideGrab");
    }

    @Override
    public void init() throws InterruptedException
    {
        sideLift.setPosition(TELE_START);
        twister.setPosition(PARALLEL_POSITION);
        sideGrab.setPosition(CLOSED_POSITION);
    }

    @Override
    public void initAuto() throws InterruptedException
    {
        sideLift.setPosition(AUTO_START);
        twister.setPosition(PERPENDICULAR_POSITION);
        sideGrab.setPosition(OPEN_POSITION);

    }

    @Override
    public void down() throws InterruptedException{
        sideLift.setPosition(DOWN_POSITION);
        Thread.sleep(500);
    }

    @Override
    public void up() throws InterruptedException{
        sideLift.setPosition(AUTO_START);
        Thread.sleep(500);
    }

    @Override
    public void grab() throws InterruptedException{
        sideGrab.setPosition(CLOSED_POSITION);
        Thread.sleep(500);
    }

    @Override
    public void open() throws InterruptedException{
        sideGrab.setPosition(OPEN_POSITION);
        Thread.sleep(500);
    }

    public void twist() throws InterruptedException{
        twister.setPosition(PERPENDICULAR_POSITION);
    }

    public void parallel() throws InterruptedException{
        twister.setPosition(PARALLEL_POSITION);
    }

    public void partial() throws InterruptedException{
        sideLift.setPosition(LIFT_POSITION);
    }




    @Override
    public void openSkystone() throws InterruptedException {
        open();
        twister.setPosition(PERPENDICULAR_POSITION);
        Thread.sleep(500);
        down();
    }
}
