package org.firstinspires.ftc.teamcode.subsystems.arm;

public interface Arm
{
    //Init the arm for teleOp
    void initTele() throws InterruptedException;
    //Init the arm for autonomous
    void initAuto() throws InterruptedException;
    //Put the arm down
    void down() throws InterruptedException;
    //Put the arm up
    void up() throws InterruptedException;
    //Grab the block
    void grab() throws InterruptedException;
    //Open the grabber
    void open() throws InterruptedException;
    void grabSkystone() throws InterruptedException;
    void dropSkystone() throws InterruptedException;

}
