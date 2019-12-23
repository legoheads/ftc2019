package org.firstinspires.ftc.teamcode.subsystems.platform;

import com.qualcomm.robotcore.hardware.Servo;

public class platformArms implements Platform{

    //Left and right servo arms to grab platform
    private Servo leftArm;
    private Servo rightArm;

    //Servo positions when up
    private double LEFT_DOWN = 0.0;
    private double RIGHT_DOWN = 1.0;

    //Servo positions when latched onto platform
    private double LEFT_UP = 0.4;
    private double RIGHT_UP = 0.6;

    public platformArms(Servo leftArm, Servo rightArm){
        this.leftArm = leftArm;
        this.rightArm = rightArm;
    }

    public void grab() throws InterruptedException{
        leftArm.setPosition(LEFT_DOWN);
        rightArm.setPosition(RIGHT_DOWN);
        Thread.sleep(400);
    }

    public void up() throws InterruptedException{
        leftArm.setPosition(LEFT_UP);
        rightArm.setPosition(RIGHT_UP);
        Thread.sleep(400);
    }
}