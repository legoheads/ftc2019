package org.firstinspires.ftc.teamcode.CV;

public interface ComputerVision {


    enum stone { SKY, STONE, UNKNOWN }

    public stone left = stone.UNKNOWN;
    public stone mid = stone.UNKNOWN;
    public stone right = stone.UNKNOWN;

    enum location { LEFT, MID, RIGHT}

//    /**
//     * Initialize the Vuforia localization engine.
//     */
//    public void initVuforia();
//
//
//    /**
//     * Initialize the Tensor Flow Object Detection engine.
//     */
//    public void initTfod();
//
//
//    /**
//     * Activate TFod once it has been instialized
//     */
//    public void tFodActivate();
//
//    /**
//     * Checks the position of the two silver minerals and the gold mineral and determines
//     * where the gold mineral is relative to the silvers
//     */
//    public void lookForMinerals();
//
//    /**
//     * goldMineral runs the lookForMinerals() function for a specified time and returns the
//     * position of the gold mineral
//     * @return position of gold mineral (LEFT, RIGHT, CENTER, or UNKNOWN)
//     * @throws InterruptedException
//     */
//    public skyStone getMineralTime() throws InterruptedException;
//
//    public skyStone getMineral() throws InterruptedException;

}
