//Run from the necessary package
package org.firstinspires.ftc.teamcode.odometry;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

@Disabled
@TeleOp(name="Odometry method test") //Name the class
public class odometryMethodTest extends LinearOpMode
{
    //Drivetrain
    private DcMotor LF, RF, LB, RB, backOdometer;

    private IIMU imu;

    private double slowPower = 0.5;


    //Define a function to use to set motor powers
    public void setDriveMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        LF.setPower(leftFrontPower);
        LB.setPower(leftBackPower);
        RF.setPower(rightFrontPower);
        RB.setPower(rightBackPower);
    }

    public void stopDriving()
    {
        setDriveMotorPowers(0.0, 0.0, 0.0, 0.0);
    }

    public void goToIMU(double power, double degrees, Telemetry telemetry) throws InterruptedException
    {
        if (Math.abs(imu.getZAngle() - degrees) > 2.0)
        {
            while (imu.getZAngle() < degrees)
            {
                telemetry.addData("Angle", imu.getZAngle());
                telemetry.update();
                setDriveMotorPowers(-power, -power, power, power);
            }
            while (imu.getZAngle() > degrees)
            {
                telemetry.addData("Angle", imu.getZAngle());
                telemetry.update();
                setDriveMotorPowers(power, power, -power, -power);
            }
        }
        stopDriving();
    }

    public void goToPosition(DcMotor yMotor, DcMotor xMotor, double targetYPos, double targetXPos, double power, Telemetry telemetry) throws InterruptedException {
        double distanceToY = targetYPos - yMotor.getCurrentPosition();
        double distanceToX = targetXPos - xMotor.getCurrentPosition();
        double coeff;
        while ((distanceToY > 20) || (distanceToX > 20))
        {
            distanceToY = targetYPos - yMotor.getCurrentPosition();
            distanceToX = targetXPos - xMotor.getCurrentPosition();
            coeff = distanceToX / (distanceToX + distanceToY);

            if (targetYPos > yMotor.getCurrentPosition() && targetXPos > xMotor.getCurrentPosition())
            {
                setDriveMotorPowers(-power, -power, -coeff * power, -coeff * power);
            }
            if (targetYPos > yMotor.getCurrentPosition() && targetXPos < xMotor.getCurrentPosition())
            {
                setDriveMotorPowers(-coeff * power, -coeff * power, -power, -power);
            }
            if (targetYPos < yMotor.getCurrentPosition() && targetXPos < xMotor.getCurrentPosition())
            {
                setDriveMotorPowers(coeff * power, coeff * power, power, power);
            }
            if (targetYPos < yMotor.getCurrentPosition() && targetXPos > xMotor.getCurrentPosition())
            {
                setDriveMotorPowers(power, power, coeff * power, coeff * power);

            }


            telemetry.addData("Y encoder position: ", yMotor.getCurrentPosition());
            telemetry.addData("X encoder position: ", xMotor.getCurrentPosition());
            telemetry.addData("Angle: ", imu.getZAngle());
            telemetry.update();
        }

        stopDriving();
        Thread.sleep(5000);

        goToIMU(power, 170, telemetry);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        backOdometer = hardwareMap.dcMotor.get("backOdometer");

        //Reverse right side motors
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = new BoschIMU(hardwareMap);

        imu.calibrate();
        imu.init();



        //Wait for start button to be clicked
        waitForStart();


//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            if (gamepad1.b)
            {
                backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                backOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad1.a)
            {
                goToPosition(LF, backOdometer,3000, 1200, slowPower, telemetry);
            }

            telemetry.addData("Y encoder position: ", LF.getCurrentPosition());
            telemetry.addData("X encoder position: ", backOdometer.getCurrentPosition());
            telemetry.addData("Angle: ", imu.getZAngle());
            telemetry.update();

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
