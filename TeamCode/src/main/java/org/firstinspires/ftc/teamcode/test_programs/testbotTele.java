//Run from the necessary package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="PushBot") //Name the class
public class testbotTele extends LinearOpMode {
    //Drivetrain
    private DcMotor LF, RF, LB, RB;

    //Define floats to be used as joystick inputs and trigger inputs
    private float drivePower, shiftPower, leftTurnPower, rightTurnPower;

    //Define a function to use to set motor powers
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        LF.setPower(leftFrontPower);
        LB.setPower(leftBackPower);
        RF.setPower(rightFrontPower);
        RB.setPower(rightBackPower);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC Motors from the hardware map
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");


        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the drive motors to brake mode to prevent rolling due to chain
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

            //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = ((gamepad1.left_stick_y + gamepad2.left_stick_y));
            shiftPower = -((gamepad1.left_stick_x + gamepad2.left_stick_x));
            leftTurnPower = ((gamepad1.left_trigger + gamepad2.left_trigger));
            rightTurnPower = ((gamepad1.right_trigger + gamepad2.right_trigger));


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            }


            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                setDriveMotorPowers(-shiftPower, shiftPower, shiftPower, -shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
                setDriveMotorPowers(-leftTurnPower, -leftTurnPower, leftTurnPower, leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
