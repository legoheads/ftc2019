package org.firstinspires.ftc.teamcode.subsystems.arm;

public interface Arm {
    //Put the arm up
    void up() throws InterruptedException;
    //Put the arm down
    void down() throws InterruptedException;
    //Put the arm between up and down
    void partial() throws InterruptedException;

    //Grab the block
    void grab() throws InterruptedException;
    //Open the grabber
    void open() throws InterruptedException;

    //Init the arm for teleOp
    void init() throws InterruptedException;
    //Init the arm for autonomous
    void initAuto() throws InterruptedException;
}
