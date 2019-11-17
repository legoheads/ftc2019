package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;


public class armSkystone implements Arm
{
    private Servo sideLift;
    private Servo twister;
    private Servo sideGrab;

    private final double UP_POSITION = 0.15;
    private final double LIFT_POSITION = 0.4;
    private final double DOWN_POSITION = 0.8;

    private final double PERPENDICULAR_POSITION = 0.9;
    private final double PARALLEL_POSITION = 0.5;

    private final double OPEN_POSITION = 0.3;
    private final double GRAB_POSITION = 0.7;
    private final double CLOSED_POSITION = 1.0;


    public armSkystone (Servo sideLift, Servo twister, Servo sideGrab)
    {
        this.sideLift = sideLift;
        this.twister = twister;
        this.sideGrab = sideGrab;
    }

    @Override
    public void initTele() throws InterruptedException
    {
        sideLift.setPosition(UP_POSITION);
        twister.setPosition(PARALLEL_POSITION);
        sideGrab.setPosition(CLOSED_POSITION);
    }

    @Override
    public void initAuto() throws InterruptedException
    {
        sideLift.setPosition(UP_POSITION);
        twister.setPosition(PARALLEL_POSITION);
        sideGrab.setPosition(CLOSED_POSITION);

    }

    @Override
    public void grabSkystone() throws InterruptedException
    {

        sideGrab.setPosition(OPEN_POSITION);
        Thread.sleep(500);
        twister.setPosition(PERPENDICULAR_POSITION);
        Thread.sleep(500);

        sideLift.setPosition(DOWN_POSITION);
        Thread.sleep(500);

        sideGrab.setPosition(GRAB_POSITION);
        Thread.sleep(500);

        sideLift.setPosition(LIFT_POSITION);

    }

    @Override
    public void dropSkystone() throws InterruptedException
    {
        sideGrab.setPosition(OPEN_POSITION);
    }
}
