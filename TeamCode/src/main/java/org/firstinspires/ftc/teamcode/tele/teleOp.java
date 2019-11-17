//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.armSkystone;

@TeleOp(name="teleOp") //Name the class
public class teleOp extends LinearOpMode
{
    //Drivetrain
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Intake
    DcMotor intakeLeft;
    DcMotor intakeRight;

    //Outtake
    DcMotor dumper;
    Servo gripper;
    DcMotor spool;

    //Platform mover
    Servo platformLeft;
    Servo platformRight;

    //Sidearm
    Servo sideLift;
    Servo twister;
    Servo sideGrab;


    //Define floats to be used as joystick inputs and trigger inputs
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;
    float spoolPower;

    float flipUpPower = (float) 0.5;
    float flipDownPower = (float) 0.3;
    float maxPower = (float) 0.8;

    int xPress = 0;
    int aPress= 0;
    int yPress = 0;

    Arm arm;

    String intakeState = "Stop";

    //Define a function to use to set motor powers
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
        rightMotorBack.setPower(rightBackPower);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC Motors from the hardware map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        //Get references to the Servo Motors from the hardware map
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        dumper = hardwareMap.dcMotor.get("dumper");
        gripper = hardwareMap.servo.get("twister");

        sideLift = hardwareMap.servo.get("sideLift");
        twister = hardwareMap.servo.get("twister");
        sideGrab = hardwareMap.servo.get("sideGrab");
        
        spool = hardwareMap.dcMotor.get("spool");

        platformLeft = hardwareMap.servo.get("platformLeft");
        platformRight = hardwareMap.servo.get("platformRight");

        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightMotorFront goes in wrong direction. Gearbox is messed up
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spool.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = new armSkystone(sideLift, twister, sideGrab);

        arm.initTele();

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

            //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = (float) -((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) -((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);
            spoolPower = (float) ((gamepad1.right_stick_y)*0.5);


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            }

            spool.setPower(spoolPower);
            if (Math.abs(spoolPower)<0.1){
                spool.setPower(0.0);
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


            if (gamepad1.a){
                aPress++;
                if (aPress%2==0){
                    intakeLeft.setPower(maxPower);
                    intakeRight.setPower(maxPower);
                    intakeState = "In";
                }
                else if (aPress%2==1){
                    intakeLeft.setPower(-maxPower);
                    intakeRight.setPower(-maxPower);
                    intakeState = "Out";
                }
//                telemetry.addData("BPress = ", bPress);
            }
            if (gamepad1.b){
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                intakeState = "Stop";
            }

            telemetry.addData("Intake: ", intakeState);


            if (gamepad1.dpad_up){
                gripper.setPosition(0.8);
                Thread.sleep((150));
                dumper.setPower(-flipUpPower);
                Thread.sleep(1000);
                dumper.setPower(0.0);
                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);
            }
            if (gamepad1.dpad_down){
                gripper.setPosition(0.9);
                dumper.setPower(flipDownPower);
                Thread.sleep(1000);
                dumper.setPower(0.0);
                intakeLeft.setPower(maxPower);
                intakeRight.setPower(maxPower);
            }

            if (gamepad1.x){
                xPress++;
                if (xPress%2==1){
                    sideLift.setPosition(0.65);
                }
                else if (xPress%2==0){
                    sideLift.setPosition(0.91);
                }
                Thread.sleep(300);
                telemetry.addData("BPress = ", xPress);
            }

            if (gamepad1.y){
                yPress++;
                if (yPress%2== 1){
                    sideGrab.setPosition(0.3);
                }
                else if (yPress%2==0){
                    sideGrab.setPosition(0.7);
                }
                Thread.sleep(300);
            }

            if (gamepad1.left_bumper){
                platformLeft.setPosition(0.4);
                platformRight.setPosition((0.6));
            }
            if (gamepad1.right_bumper){
                platformLeft.setPosition(0.0);
                platformRight.setPosition((1.0));
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
