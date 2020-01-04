package org.firstinspires.ftc.teamcode.subsystems.arm;

public interface Arm
{
    //Init the arm for teleOp
    void init() throws InterruptedException;
    //Init the arm for autonomous
//    void initAuto() throws InterruptedException;
    //Put the arm down
    void down() throws InterruptedException;
    //Put the arm up
    void up() throws InterruptedException;
    //Grab the block
    void grab() throws InterruptedException;
    //Open the grabber
    void open() throws InterruptedException;
    //Twist the block to prepare for drop
    void twist() throws InterruptedException;

    void openSkystone() throws InterruptedException;
    void parallel() throws InterruptedException;
    void partial() throws InterruptedException;


}
