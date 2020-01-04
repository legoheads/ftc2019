//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.arm.*;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

@TeleOp(name="ARMTEST") //Name the class
public class armTest extends LinearOpMode
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
//    DcMotor dumper;
    Servo gripper;
    DcMotor spool;

    CRServo extend;

//    //Platform mover
//    Servo platformLeft;
//    Servo platformRight;

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

    ElapsedTime extendTime = new ElapsedTime();

    int xPress = 0;
    int yPress = 0;

    BNO055IMU boschIMU;
    IIMU imu;

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
//        dumper = hardwareMap.dcMotor.get("dumper");
        gripper = hardwareMap.servo.get("gripper");

        extend = hardwareMap.crservo.get("extend");

        sideLift = hardwareMap.servo.get("sideLift");
        twister = hardwareMap.servo.get("twister");
        sideGrab = hardwareMap.servo.get("sideGrab");
//
        spool = hardwareMap.dcMotor.get("spool");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");
//
//        platformLeft = hardwareMap.servo.get("platformLeft");
//        platformRight = hardwareMap.servo.get("platformRight");

        Chassis driveTrain = new Chassis(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);




        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

//        spool.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm = new sideArm(sideLift, twister, sideGrab);

//        arm.init();
//        gripper.setPosition(0.95);
//        platformLeft.setPosition(0.0);
//        platformRight.setPosition((1.0));

//        gripper.setPosition(0.6);

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
            drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y);
            shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.5);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.5);
            spoolPower = gamepad1.right_stick_y + gamepad2.right_stick_y;


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
//                setDriveMotorPowers(drivePower, drivePower, drivePower* (float)0.92, drivePower* (float)0.92);

                driveTrain.driveTeleop(drivePower);
            }

//            spool.setPower(spoolPower);
//            if (Math.abs(spoolPower)<0.1){
//                spool.setPower(0.0);
//            }

            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
//                setDriveMotorPowers(-shiftPower*(float)0.95, shiftPower, shiftPower*(float)0.95, -shiftPower);
                driveTrain.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
//                setDriveMotorPowers(-leftTurnPower, -leftTurnPower, leftTurnPower, leftTurnPower);
                driveTrain.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
//                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
                driveTrain.rightTurnTeleop(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                driveTrain.stopDriving();
            }

            spool.setPower(spoolPower);


            if (gamepad1.a){
                arm.open();
            }
            if (gamepad1.x){
                arm.grab();
            }

            if (gamepad1.b){
                arm.down();
            }

            if (gamepad1.y){
                arm.up();
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
