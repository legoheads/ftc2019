package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

@Disabled
@TeleOp(name="Track position") //Name the class
public class positionTest extends LinearOpMode {

    //Odometer ports
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor spoolLeft;

    private BNO055IMU boschIMU;

    IIMU imu;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;
    int degrees = 1000;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        spoolLeft = hardwareMap.dcMotor.get("spoolLeft");


        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        imu = new BoschIMU(hardwareMap);
        imu.init();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            telemetry.addData("y displacement left (port 0): ", intakeLeft.getCurrentPosition());
            telemetry.addData("y displacement right (port 1): ", intakeRight.getCurrentPosition());
            telemetry.addData("x displacement right (port 2): ", spoolLeft.getCurrentPosition());
            telemetry.addData("x angle (use this one):", boschIMU.getAngularOrientation().firstAngle);
            telemetry.addData("y angle: ", boschIMU.getAngularOrientation().secondAngle);
            telemetry.addData("z angle: ", boschIMU.getAngularOrientation().thirdAngle);

            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
