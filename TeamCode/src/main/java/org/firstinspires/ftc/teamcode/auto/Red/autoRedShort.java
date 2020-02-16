//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto.Red;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;

@Disabled
@Autonomous(name="AutoRed Short", group = "Red") //Name the class
public class autoRedShort extends LinearOpMode {

    private float DRIVE_POWER = (float) 0.5;
    private float TURN_POWER = (float) 0.3;
    private float SHIFT_POWER = (float) 0.5;

    private float DRIFT_POWER = (float) 0.3;

    int driveDistance;

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private Arm arm;
    private Arm arm2;
    private skystoneChassis chassis;
    private skystoneDetector detector;
    private Platform platform;
    private IIMU imu;

    private Distance distanceSensor;

    private Servo shortSaber;

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        //Intialize subsystems

        shortSaber = hardwareMap.servo.get("shortSaber");

        detector = new skystoneDetector(hardwareMap, telemetry, true);

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        arm = new leftArm(hardwareMap);
        arm2 = new rightArm(hardwareMap);
        platform = new platformArms(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap);
        imu = new BoschIMU(hardwareMap);

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
            arm.releaseAuto();

            if (skystoneLocation == CV.location.LEFT)
            {
                driveDistance = 0;
                distanceSensor.distLeftShift(SHIFT_POWER, 8.5);
            }
            if (skystoneLocation == CV.location.MID)
            {
                driveDistance = 225;
                distanceSensor.distLeftShift(SHIFT_POWER, 9);

            }
            if (skystoneLocation == CV.location.RIGHT)
            {
                driveDistance = 550;
                distanceSensor.distLeftShift(SHIFT_POWER, 9);
            }

            chassis.driveForwardsAutonomous(DRIVE_POWER, driveDistance);

            arm.grabAuto();

            Thread.sleep(100);

            arm.lift();

            chassis.rightShiftAutonomous(SHIFT_POWER / 2, 250);

            imu.init();

            chassis.driveForwardsAutonomous(DRIVE_POWER, 3550 - driveDistance);

            Thread.sleep(300);

            distanceSensor.distLeftShift(SHIFT_POWER, 6);

            arm.releaseAuto();
            arm.init();

            chassis.rightShiftAutonomous(SHIFT_POWER,150);

            chassis.rightTurnIMU(TURN_POWER,-90);

            imu.init();

            distanceSensor.platformReverse();

            chassis.driveBackwardsAutonomous(-DRIVE_POWER/2, -20);

            Thread.sleep(100);

            platform.grab();

            Thread.sleep(500);


            while (imu.getZAngle() > -90)
            {
                platform.grab();
                chassis.setDriveMotorPowers(DRIFT_POWER * 1.5,DRIFT_POWER * 1.5, DRIFT_POWER / 4, DRIFT_POWER / 4);
            }

            chassis.stopDriving();

            imu.init();

            platform.up();

            chassis.driveBackwardsAutonomous(-DRIVE_POWER, -750);

            chassis.rightShiftAutonomous(SHIFT_POWER,400);

            chassis.driveForwardsAutonomous(DRIVE_POWER, 1100);

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
