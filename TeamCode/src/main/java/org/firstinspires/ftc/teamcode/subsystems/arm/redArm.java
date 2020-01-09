package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class redArm implements Arm {
    private Servo sideLift;
    private Servo sideGrab;

    private HardwareMap hardwareMap;

    //Positions for Lifter
    private final double AUTO_START = 0.5;
    private final double TELE_START = 0.15;
    private final double LIFT_POS = 0.1;
    private final double DOWN_POS= 0.0;

    //Positions for Grabber
    private final double  OPEN_POS= 0.5;
    private final double GRAB_POS = 1.0;


    public redArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        sideLift = hardwareMap.servo.get("sideLiftRed");
        sideGrab = hardwareMap.servo.get("sideGrabRed");
    }

    @Override
    public void up() throws InterruptedException {
        sideLift.setPosition(AUTO_START);
        Thread.sleep(500);
    }

    @Override
    public void down() throws InterruptedException {
        sideLift.setPosition(DOWN_POS);
//        Thread.sleep(500);
    }

    @Override
    public void partial() throws InterruptedException {
        sideLift.setPosition(LIFT_POS);
//        Thread.sleep(500);
    }


    public void partial2() throws InterruptedException {
        sideLift.setPosition(0.3);
//        Thread.sleep(500);
    }


    @Override
    public void grab() throws InterruptedException {
        sideGrab.setPosition(GRAB_POS);
        Thread.sleep(1000);
    }

    @Override
    public void open() throws InterruptedException {
        sideGrab.setPosition(OPEN_POS);
        Thread.sleep(500);
    }

    @Override
    public void init() throws InterruptedException {
        sideLift.setPosition(TELE_START);
        sideGrab.setPosition(GRAB_POS);
    }

    @Override
    public void initAuto() throws InterruptedException {
        up();
        grab();
    }
}
