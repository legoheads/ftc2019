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

@Autonomous(name="AutoRed Intake", group = "Red") //Name the class
public class autoRedIntake extends LinearOpMode {

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
    private IntakeWheels intake;

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

            chassis.driveAutonomous(DRIVE_POWER, 800);

            //CV
            chassis.leftShiftAutonomous(SHIFT_POWER, 500);

            chassis.useEncoder(false);

            while (imu.getZAngle() < 45)
            {
                chassis.setDriveMotorPowers(0, 0, TURN_POWER, TURN_POWER);
            }
            chassis.stopDriving();

            chassis.driveAutonomous(-DRIVE_POWER, -400);


            chassis.useEncoder(false);

            while (imu.getZAngle() < 90)
            {
                chassis.setDriveMotorPowers(-TURN_POWER, -TURN_POWER, 0, 0);
            }

            intake.stop();

            chassis.stopDriving();

            while (imu.getZAngle()>90){
                chassis.rightTurnTeleop(TURN_POWER/2);
            }

//            chassis.rightTurnIMU(TURN_POWER/2, 90);


            chassis.driveAutonomous(-DRIVE_POWER, -2800);


            while(imu.getZAngle()>0){
                chassis.leftTurnTeleop(TURN_POWER);
            }

            chassis.stopDriving();

            while(imu.getZAngle()<0){
                chassis.rightTurnTeleop(TURN_POWER/2);
            }

            chassis.stopDriving();

            imu.init();

            distanceSensor.platformReverse();

            Thread.sleep(200);

            platform.grab();

            Thread.sleep(500);


            while (imu.getZAngle() > -90) {
//                platform.grab();
                chassis.setDriveMotorPowers(DRIFT_POWER * 1.5,DRIFT_POWER * 1.5, DRIFT_POWER / 4, DRIFT_POWER / 4);
            }

            platform.up();

            chassis.driveAutonomous(DRIVE_POWER, 300);
//
//            chassis.rightShiftAutonomous(SHIFT_POWER, 200);
//
//            //CV
//            chassis.driveAutonomous(DRIVE_POWER, 500);
//
//            intake.intake();
//
//            chassis.rightTurnIMU(TURN_POWER, 45);
//
//            chassis.driveAutonomous(DRIVE_POWER, 500);
//
//            chassis.driveAutonomous(-DRIVE_POWER, -500);
//
//            chassis.rightTurnIMU(TURN_POWER, 90);
//
//            chassis.driveAutonomous(-DRIVE_POWER, 2000);
//
//            chassis.driveAutonomous(DRIVE_POWER, 1000);
//
//            shortSaber.setPosition(0.8);
//
//            Thread.sleep(2000);
            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
