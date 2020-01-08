package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class blueArm implements Arm {
    private Servo sideLift;
    private Servo sideGrab;

    private HardwareMap hardwareMap;

    //Positions for Lifter
    private final double AUTO_START = 0.5;
    private final double TELE_START = 0.15;
    private final double LIFT_POS = 0.6;
    private final double DOWN_POS= 0.85;

    //Positions for Grabber
    private final double  OPEN_POS= 0.5;
    private final double GRAB_POS = 0.1;


    public blueArm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        sideLift = hardwareMap.servo.get("sideLiftBlue");
        sideGrab = hardwareMap.servo.get("sideGrabBlue");
    }

    @Override
    public void up() throws InterruptedException {
        sideLift.setPosition(AUTO_START);
//        Thread.sleep(500);
    }

    @Override
    public void down() throws InterruptedException {
        sideLift.setPosition(DOWN_POS);
        Thread.sleep(500);
    }

    @Override
    public void partial() throws InterruptedException {
        sideLift.setPosition(LIFT_POS);
        Thread.sleep(500);
    }


    @Override
    public void grab() throws InterruptedException {
        sideGrab.setPosition(GRAB_POS);
        Thread.sleep(500);
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

    public void setPos(double pos) throws InterruptedException{
        sideGrab.setPosition(pos);
    }
}
