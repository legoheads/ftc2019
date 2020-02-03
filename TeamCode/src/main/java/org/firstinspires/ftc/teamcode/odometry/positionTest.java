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
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor backOdometer;


    IIMU imu;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;
    int degrees = 1000;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {

        LF = hardwareMap.dcMotor.get("LF");
        RB = hardwareMap.dcMotor.get("RB");
        backOdometer = hardwareMap.dcMotor.get("backOdometer");


        imu = new BoschIMU(hardwareMap);

        imu.init();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            if (gamepad1.b)
            {
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("y displacement left (port 0): ", LF.getCurrentPosition());
            telemetry.addData("y displacement right (port 1): ", RB.getCurrentPosition());
            telemetry.addData("x displacement right (port 2): ", backOdometer.getCurrentPosition());
            telemetry.addData("x angle :", imu.getXAngle());
            telemetry.addData("y angle: ", imu.getYAngle());
            telemetry.addData("z angle (use this one): ", imu.getZAngle());

            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
