//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CV.ComputerVision;
import org.firstinspires.ftc.teamcode.CV.skystoneDetectorClass;


@Autonomous(name="CVTEST") //Name the class
public class CVTest extends LinearOpMode {

    int RUNTIME = 10000;
    ComputerVision.location skystoneLocation;
    Boolean isFound = false;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {

        skystoneDetectorClass cv = new skystoneDetectorClass(hardwareMap, telemetry);

        while(!isStarted()){

            //Runs for an amount of time
//            skystoneLocation = cv.getSkystoneTime(10000);

            //Runs until play is pressed
            skystoneLocation = cv.getSkystoneInfinite();

        }

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {


            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
