//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto.Red;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.CV.CV;
import org.firstinspires.ftc.teamcode.subsystems.CV.skystoneDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.leftArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.rightArm;
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

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private Arm arm;
    private Arm arm2;
    private skystoneChassis chassis;
    private skystoneDetector detector;
    private Platform platform;
    private IIMU imu;
    private IntakeWheels intake;
    private stacker stacker;

    private Distance distanceSensor;

    private Servo shortSaber;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        //Intialize subsystems

        shortSaber = hardwareMap.servo.get("shortSaber");

        detector = new skystoneDetector(hardwareMap, telemetry);
        intake = new intake(hardwareMap);

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        arm = new leftArm(hardwareMap);
        arm2 = new rightArm(hardwareMap);
        platform = new platformArms(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap, gamepad1, gamepad2);
        imu = new BoschIMU(hardwareMap);
        stacker = new stacker(hardwareMap, gamepad1, gamepad2);

        shortSaber.setPosition(0.45);

        //Look for Skystone until play is pressed
        while(!isStarted()){ skystoneLocation = detector.getSkystoneInfinite(); }

        //Wait for start button to be clicked
        waitForStart();

        telemetry.addData("Skystone", skystoneLocation);

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            intake.intake();

            chassis.driveForwardsAutonomous(DRIVE_POWER, 950);

            //CV
            chassis.leftShiftAutonomous(SHIFT_POWER, 450);

            chassis.useEncoder(false);
            while (imu.getZAngle() < 45)
            {
                platform.up();
                chassis.setDriveMotorPowers(0, 0, TURN_POWER, TURN_POWER);
            }
            chassis.stopDriving();

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -600);

            chassis.useEncoder(false);
            while (imu.getZAngle() < 90)
            {
                chassis.setDriveMotorPowers(-TURN_POWER, -TURN_POWER, 0, 0);
                stacker.grab();
            }
            intake.stop();
            chassis.stopDriving();

            chassis.rightTurnIMU(TURN_POWER, 85.5);

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -2700);

            while (imu.getZAngle() > 0)
            {
                chassis.leftTurnTeleop(TURN_POWER);
            }
            chassis.stopDriving();

            while (imu.getZAngle() < 0)
            {
                chassis.rightTurnTeleop(TURN_POWER / 2);
            }
            chassis.stopDriving();

            //ADD PLATS INTO HERE
            distanceSensor.platformReverse();
//            chassis.driveBackwardsAutonomous(-DRIVE_POWER/2, 250);

            Thread.sleep(200);

            platform.grab();

            Thread.sleep(200);

            //drift turn
            while (imu.getZAngle() > 90 || imu.getZAngle() < 0)
            {
                stacker.extend();
                chassis.setDriveMotorPowers(DRIFT_POWER * 1.8,DRIFT_POWER * 1.8, DRIFT_POWER / 4, DRIFT_POWER / 4);
            }

            chassis.leftTurnIMU(TURN_POWER, 90);

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -500);

            chassis.rightShiftAutonomous(SHIFT_POWER, 200);

            platform.up();

            stacker.retract();

            chassis.driveForwardsAutonomous(DRIVE_POWER, 1500);

            intake.intake();

            chassis.rightTurnIMU(TURN_POWER, 45);

            chassis.driveForwardsAutonomous(DRIVE_POWER, 900);

            Thread.sleep(1000);

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -900);

            chassis.leftTurnIMU(TURN_POWER, 90);

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -2000);

            chassis.driveForwardsAutonomous(DRIVE_POWER, 1000);

            shortSaber.setPosition(0.8);

            Thread.sleep(2000);
            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
