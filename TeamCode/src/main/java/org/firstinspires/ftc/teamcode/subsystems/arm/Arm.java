package org.firstinspires.ftc.teamcode.subsystems.arm;

public interface Arm
{
    //Dunk
    void initTele() throws InterruptedException;
    void initAuto() throws InterruptedException;
    void grabSkystone() throws InterruptedException;
    void dropSkystone() throws InterruptedException;
}
