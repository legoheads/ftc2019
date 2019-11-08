package org.firstinspires.ftc.teamcode.subsystems.tensorFlow;

public interface TensorFlow {


    enum skyStone {ONE, TWO, THREE, FOUR, FIVE, SIX, UNKNOWN}
    public skyStone location1 = skyStone.UNKNOWN;
    public skyStone location2 = skyStone.UNKNOWN;

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia();


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod();


    /**
     * Activate TFod once it has been instialized
     */
    public void tFodActivate();

    public void findSkystones();

    public skyStone getSkystones() throws InterruptedException;
}




