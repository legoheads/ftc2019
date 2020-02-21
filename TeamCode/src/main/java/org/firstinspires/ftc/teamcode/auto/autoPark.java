package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;

@TeleOp(name="PID Test") //Name the class
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
            if (gamepad1.dpad_up)
            {
                chassis.driveForwardsAutonomousPID(2000);
            }
            if (gamepad1.dpad_down)
            {
                chassis.driveBackwardsAutonomousPID(2000);
            }
            if (gamepad1.dpad_left)
            {
                chassis.leftShiftAutonomousPID(2000);
            }
            if (gamepad1.dpad_right)
            {
                chassis.rightShiftAutonomousPID(2000);
            }

            if (gamepad1.left_bumper)
            {
                chassis.leftTurnIMUPID(90);
            }

            if (gamepad1.right_bumper)
            {
                chassis.rightTurnIMUPID(-90);
            }


            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program

