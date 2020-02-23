//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto.Red;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.CV.CV;
import org.firstinspires.ftc.teamcode.subsystems.CV.skystoneDetector;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.Distance;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.distanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

@Autonomous(name="AutoRed Intake", group = "Red") //Name the class
public class autoRedIntake extends LinearOpMode {

    private float DRIVE_POWER = (float) 0.5;
    private float TURN_POWER = (float) 0.3;
    private float SHIFT_POWER = (float) 0.5;

    private float DRIFT_POWER = (float) 0.5;

    int driveDistance;
    int driveDistance2;
    int shiftDistance;

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private skystoneChassis chassis;
    private skystoneDetector detector;
    private Platform platform;
    private IIMU imu;
    private IntakeWheels intake;
    private stacker stacker;
    private Distance distanceSensor;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        detector = new skystoneDetector(hardwareMap, telemetry, true);
        intake = new intake(hardwareMap);

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        platform = new platformArms(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap);
        imu = new BoschIMU(hardwareMap);
        stacker = new stacker(hardwareMap);

        //Look for Skystone until play is pressed
        while(!isStarted()){ skystoneLocation = detector.getSkystoneInfinite(); }

        //Wait for start button to be clicked
        waitForStart();

        telemetry.addData("Skystone", skystoneLocation);

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            if (skystoneLocation == CV.location.LEFT)
            {
                shiftDistance = 1250;
                driveDistance = 4050;
            }

            if (skystoneLocation == CV.location.MID)
            {
                shiftDistance = 925;
                driveDistance = 3750;
                driveDistance2 = 500;
            }

            if (skystoneLocation == CV.location.RIGHT)
            {
                shiftDistance = 525;
                driveDistance = 3450;
            }

            intake.intake();

            chassis.driveForwardsAutonomousPID(1050);

            //CV
//            chassis.leftShiftAutonomous(SHIFT_POWER, shiftDistance);
            chassis.leftShiftAutonomousPID(shiftDistance);

            while (imu.getZAngle() < 35)
            {
                platform.up();
                chassis.setDriveMotorPowers(0, 0, TURN_POWER, TURN_POWER);
            }

            chassis.stopDriving();

            chassis.driveBackwardsAutonomousPID(-650);

            stacker.grab();

            chassis.leftTurnIMUPID(87);

            chassis.driveBackwardsAutonomousPID(-driveDistance);

            chassis.leftTurnIMUPID(175);

            while (imu.getZAngle() > 0)
            {
                chassis.leftTurnTeleop(TURN_POWER / 2);
            }

            distanceSensor.platformReverse();

            Thread.sleep(200);

            platform.grab();

            Thread.sleep(200);

            chassis.setDriveMotorPowers(DRIFT_POWER * 1.5,DRIFT_POWER * 1.5, DRIFT_POWER / 4, DRIFT_POWER / 4);
            Thread.sleep(100);

            //drift turn
            while (imu.getZAngle() > 90)
            {
                stacker.extend();
                chassis.setDriveMotorPowers(DRIFT_POWER * 1.5,DRIFT_POWER * 1.5, DRIFT_POWER / 4, DRIFT_POWER / 4);
            }
            chassis.stopDriving();

            platform.up();

            chassis.goToIMU(90);

            stacker.retract();

            chassis.rightShiftAutonomous(SHIFT_POWER, 350);

            intake.intake();


            chassis.driveForwardsAutonomous(DRIVE_POWER, 500);

            chassis.rightTurnIMU(TURN_POWER, 35);

            chassis.driveForwardsAutonomous(DRIVE_POWER / 1.5, 900);

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -900);

            stacker.grab();

            chassis.leftTurnIMU(TURN_POWER, 90);

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -500);

            stacker.extend();

            stacker.ungrab();

            chassis.driveForwardsAutonomous(DRIVE_POWER, 600);

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
