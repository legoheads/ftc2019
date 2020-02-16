package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;

@Autonomous(name="Auto Park") //Name the class
public class autoPark extends LinearOpMode
{
    private skystoneChassis chassis;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        //Intialize subsystems
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);


        //Wait for start button to be clicked
        waitForStart();


//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            chassis.driveForwardsAutonomous(0.3, 200);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program

