package org.firstinspires.ftc.teamcode.subsystems.platform;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class platformArms implements Platform{

    private HardwareMap hardwareMap;

    //Left and right servo arms to grab platform
    private Servo platformLeft;
    private Servo platformRight;

    //Servo positions when up
    private double LEFT_DOWN = 0.0;
    private double RIGHT_DOWN = 1.0;

    //Servo positions when latched onto platform
    private double LEFT_UP = 0.5;
    private double RIGHT_UP = 0.5;

    public platformArms(HardwareMap hardwareMap, Servo leftArm, Servo rightArm){
        this.hardwareMap = hardwareMap;

        this.platformLeft = leftArm;
        this.platformRight = rightArm;

        platformLeft = hardwareMap.servo.get("platformLeft");
        platformRight = hardwareMap.servo.get("platformRight");

    }

    public void grab() throws InterruptedException{
        platformLeft.setPosition(LEFT_DOWN);
        platformRight.setPosition(RIGHT_DOWN);
        Thread.sleep(400);
    }

    public void up() throws InterruptedException{
        platformLeft.setPosition(LEFT_UP);
        platformRight.setPosition(RIGHT_UP);
        Thread.sleep(400);
    }
}