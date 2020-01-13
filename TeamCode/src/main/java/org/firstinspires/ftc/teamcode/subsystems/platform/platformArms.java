package org.firstinspires.ftc.teamcode.subsystems.platform;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class platformArms implements Platform{

    private HardwareMap hardwareMap;

    //Left and right servo arms to grab platform
    private Servo platformLeft;
    private Servo platformRight;

    //Servo positions when up
    private double LEFT_DOWN = 0.1;
    private double RIGHT_DOWN = 0.9;

    //Servo positions when latched onto platform
    private double LEFT_UP = 0.6;
    private double RIGHT_UP = 0.4;

    private double LEFT_MIDDLE = 0.5;
    private double RIGHT_MIDDLE = 0.5;

    public platformArms(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        platformLeft = hardwareMap.servo.get("platformLeft");
        platformRight = hardwareMap.servo.get("platformRight");

        init();
    }

    public void init(){
        platformLeft.setPosition(LEFT_UP);
        platformRight.setPosition(RIGHT_UP);
    }

    public void grab() throws InterruptedException{
        platformLeft.setPosition(LEFT_DOWN);
        platformRight.setPosition(RIGHT_DOWN);
        Thread.sleep(400);
    }

    @Override
    public void middle() throws InterruptedException {
        platformLeft.setPosition(LEFT_MIDDLE);
        platformRight.setPosition(RIGHT_MIDDLE);

    }

    public void up() throws InterruptedException{
        platformLeft.setPosition(LEFT_UP);
        platformRight.setPosition(RIGHT_UP);
        Thread.sleep(400);
    }

    public Servo getPlatformLeft(){
        return platformLeft;
    }

    public Servo getPlatformRight(){
        return platformRight;
    }
}