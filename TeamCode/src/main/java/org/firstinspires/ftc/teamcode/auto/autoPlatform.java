//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.sideArm;
import org.firstinspires.ftc.teamcode.subsystems.platform.*;

@Disabled
@Autonomous(name="autoPlatform")
public class autoPlatform extends LinearOpMode
{
    //Drivetrain
    DcMotor leftMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorFront;
    DcMotor rightMotorBack;

    //Intake
    DcMotor intakeLeft;
    DcMotor intakeRight;

    //Outtake
    DcMotor dumper;
    Servo gripper;
    DcMotor spool;

    //Platform mover
    Platform platform;
    Servo platformLeft;
    Servo platformRight;

    //Sidearm
    Arm arm;
    Servo sideLift;
    Servo twister;
    Servo sideGrab;

    //IMU
    BNO055IMU boschIMU;

    //Move powers
    float drivePower = (float) 0.7;
    float turnPower = (float) 0.4;
    float shiftPower = (float) 0.9;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the hardware map

        //Drive Motors
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        //Intake
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        dumper = hardwareMap.dcMotor.get("dumper");
        gripper = hardwareMap.servo.get("gripper");

        sideLift = hardwareMap.servo.get("sideLift");
        twister = hardwareMap.servo.get("twister");
        sideGrab = hardwareMap.servo.get("sideGrab");

        spool = hardwareMap.dcMotor.get("spool");

        platformLeft = hardwareMap.servo.get("platformLeft");
        platformRight = hardwareMap.servo.get("platformRight");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spool.setDirection(DcMotorSimple.Direction.REVERSE);

        Chassis chassis = new Chassis(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack, boschIMU);
        arm = new sideArm(sideLift, twister, sideGrab);
        arm.init();

        Platform platform = new platformArms(platformLeft, platformRight);

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            platform.up();

            intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//            chassis.odometryDrive(spool, drivePower, 5500, telemetry);

            chassis.driveTeleop(0.8);
            Thread.sleep(2500);
//            chassis.odometryLeftShift(intakeLeft, (float)0.3, 300, telemetry);

            platform.grab();

            chassis.coeffShiftTeleop(0.8);
            Thread.sleep(3500);

//            chassis.odometryLeftShift(intakeLeft, (float)shiftPower, 3000, telemetry);

//            chassis.leftTurnIMU(-0.9, 90);
//            chassis.odometryDrive(spool, -drivePower, -500, telemetry);





            //Update the data
            telemetry.update();
            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
