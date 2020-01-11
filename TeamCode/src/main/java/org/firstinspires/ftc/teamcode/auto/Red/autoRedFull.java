//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto.Red;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.CV.CV;
import org.firstinspires.ftc.teamcode.subsystems.CV.skystoneDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.redArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

@Autonomous(name="AutoRed Full", group = "Red") //Name the class
public class autoRedFull extends LinearOpMode {

    private float DRIVE_POWER = (float) 0.5;
    private float TURN_POWER = (float) 0.5;
    private float SHIFT_POWER = (float) 0.5;

    private DistanceSensor distSensor;

    int driveDistance;

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private Arm arm;
    private IntakeWheels intake;
    private LinearSlides slides;
    private skystoneChassis chassis;
    private skystoneDetector detector;
    private Platform platform;
    private IIMU imu;
    private stacker stacker;

    private int STONE_SPACE = 300;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        //Intialize subsystems

        distSensor = hardwareMap.get(DistanceSensor.class, "distLeft");

        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        arm = new redArm(hardwareMap);
        detector = new skystoneDetector(hardwareMap, telemetry);
        platform = new platformArms(hardwareMap);
        imu = new BoschIMU(hardwareMap);
        stacker = new stacker(hardwareMap, gamepad1, gamepad2);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distSensor;

        //Initialize sidearm servos
//        arm.twist();
        arm.up();
        arm.grab();



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

            if (skystoneLocation== CV.location.LEFT) {
                driveDistance = 0;
            }
            if (skystoneLocation== CV.location.MID) {
                driveDistance = 160;
            }
            if (skystoneLocation== CV.location.RIGHT) {
                driveDistance = 320;
            }

            chassis.driveAutonomous(DRIVE_POWER, driveDistance);


            chassis.shiftTeleop(SHIFT_POWER);

            arm.down();

            arm.open();

            double startAngle = imu.getZAngle();
            double COEFF = 0.94;



            while((!(distSensor.getDistance(DistanceUnit.INCH)<10)))
            {
                chassis.shiftTeleop(SHIFT_POWER);
                if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
                {
                    if (imu.getZAngle() > startAngle) {
                        chassis.setDriveMotorPowers(COEFF * SHIFT_POWER, SHIFT_POWER, SHIFT_POWER, COEFF * SHIFT_POWER);
                    }

                    if (imu.getZAngle() < startAngle) {
                        chassis.setDriveMotorPowers(SHIFT_POWER, COEFF * SHIFT_POWER, COEFF * SHIFT_POWER, SHIFT_POWER);
                    }
                }


            }
            chassis.stopDriving();

            arm.grab();

            Thread.sleep(500);

            arm.lift();

            chassis.rightShiftAutonomous(SHIFT_POWER, 300);

            chassis.driveAutonomous(DRIVE_POWER, 3100 - driveDistance);

            chassis.leftShiftAutonomous(SHIFT_POWER, 400);

            arm.down();

            Thread.sleep(500);
            arm.open();
            arm.up();
            arm.grab();

            chassis.rightShiftAutonomous(SHIFT_POWER,200);

            chassis.rightTurnIMU(TURN_POWER,-90);

            chassis.driveAutonomous(-DRIVE_POWER, -300);

            stacker.stoneReverse();

            platform.grab();

            Thread.sleep(500);

            chassis.driveAutonomous(DRIVE_POWER, 500);

            platform.up();

            chassis.rightShiftAutonomous(SHIFT_POWER,1000);

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
