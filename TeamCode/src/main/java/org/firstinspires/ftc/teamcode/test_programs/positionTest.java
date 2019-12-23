package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

@Disabled
@TeleOp(name="Track position") //Name the class
public class positionTest extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    DcMotor spool;

    DcMotor intakeLeft;
    DcMotor intakeRight;

    BNO055IMU boschIMU;

    IIMU imu;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;
    int degrees = 1000;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        spool = hardwareMap.dcMotor.get("spool");


        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        imu = new BoschIMU(boschIMU);
        imu.initialize();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            telemetry.addData("x displacement (port 0): ", intakeLeft.getCurrentPosition());
            telemetry.addData("y displacement (port 3): ", spool.getCurrentPosition());
            telemetry.addData("x angle (use this one):", boschIMU.getAngularOrientation().firstAngle);
            telemetry.addData("y angle: ", boschIMU.getAngularOrientation().secondAngle);
            telemetry.addData("z angle: ", boschIMU.getAngularOrientation().thirdAngle);
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
