//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.sideArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;

@Autonomous(name="AUTOARM") //Name the class
public class autonomousArm extends LinearOpMode
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
//    Servo gripper;
    DcMotor spool;

    //Platform mover
    Servo platformLeft;
    Servo platformRight;

    //Sidearm
    Servo sideLift;
    Servo twister;
    Servo sideGrab;

    float drivePower = (float) 0.6;
    float turnPower = (float) 0.4;
    float shiftPower = (float) 0.4;

    BNO055IMU boschIMU;

    int xPress = 0;
    int aPress= 0;
    int yPress = 0;

    Arm arm;

    String intakeState = "Stop";

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
//        gripper = hardwareMap.servo.get("gripper");

        sideLift = hardwareMap.servo.get("sideLift");
        twister = hardwareMap.servo.get("twister");
        sideGrab = hardwareMap.servo.get("sideGrab");

        spool = hardwareMap.dcMotor.get("spool");

//        platformLeft = hardwareMap.servo.get("platformLeft");/
//        platformRight = hardwareMap.servo.get("platformRight");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spool.setDirection(DcMotorSimple.Direction.REVERSE);

        Chassis chassis = new Chassis(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);


        arm = new sideArm(sideLift, twister, sideGrab);


//        arm.init();

        arm.twist();
        arm.up();
        arm.grab();


        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

//            arm.parallel();
//            Thread.sleep(2000);
            arm.partial();
            Thread.sleep(500);
            arm.parallel();
            arm.open();
            Thread.sleep(500);
            arm.down();
            Thread.sleep(500);
            arm.grab();
            Thread.sleep(500);
            arm.partial();
            Thread.sleep(200);
            arm.twist();
            Thread.sleep(500);
            arm.up();

            chassis.driveAutonomous(0.3, 3000);

            chassis.rightShiftAutonomous(0.3, 300);

            sideLift.setPosition(0.15);
            Thread.sleep(500);
            arm.open();
            Thread.sleep(500);
            arm.up();

            chassis.leftShiftAutonomous(0.3,300);

            chassis.driveAutonomous(-0.3, -3000);

//            chassis.driveAutonomous(-0.5, -1000);








//            leftMotorFront.setPower(0.5);
//            rightMotorFront.setPower(0.5);
//            leftMotorBack.setPower(0.5);
//            rightMotorBack.setPower(0.5);
//
//            Thread.sleep(3000);
//
//            leftMotorFront.setPower(0.0);
//            rightMotorFront.setPower(0.0);
//            leftMotorBack.setPower(0.0);
//            rightMotorBack.setPower(0.0);

//            chassis.leftTurnIMU();

//
//
//            intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetry.addData("Encoders Reset", "good");
////            telemetry.update();
//
//            intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            telemetry.addData("Encoders Run", "good");
////            telemetry.update();
////
////
//            telemetry.update();
//            chassis.odometryDrive(spool, drivePower, 1000, telemetry);






//            arm.down();
//            Thread.sleep(500);
//            arm.open();
//            Thread.sleep(500);
//            chassis.odometryRightShift(intakeLeft, shiftPower, 300, telemetry);
//            arm.grab();
//            Thread.sleep(500);
//            arm.up();

//            arm.grabSkystone();

//            chassis.odometryDrive(spool, -drivePower, -500, telemetry);
//
//            chassis.odometryLeftShift(intakeLeft, shiftPower, 300, telemetry);

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
