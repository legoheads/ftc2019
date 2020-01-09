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
import org.firstinspires.ftc.teamcode.subsystems.arm.blueArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

@Autonomous(name="AutoTest", group = "Red") //Name the class
public class autoTest extends LinearOpMode {

    private float DRIVE_POWER = (float) 0.5;
    private float TURN_POWER = (float) 0.5;
    private float SHIFT_POWER = (float) 0.5;

    private BNO055IMU boschIMU;

    private DistanceSensor distSensor;

    int driveDistance;

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private Arm arm;
    private IntakeWheels intake;
    private LinearSlides slides;
    private skystoneChassis chassis;
    private skystoneDetector detector;

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
        arm = new blueArm(hardwareMap);
        detector = new skystoneDetector(hardwareMap, telemetry);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distSensor;

        //Initialize sidearm servos
//        arm.twist();
        arm.up();
        arm.grab();

        //Look for skystone until play is pressed
        while(!isStarted()){ skystoneLocation = detector.getSkystoneInfinite(); }

        //Wait for start button to be clicked
        waitForStart();

        telemetry.addData("Skystone", skystoneLocation);



//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            while((!(distSensor.getDistance(DistanceUnit.INCH)<8))){
                chassis.shiftTeleop(0.5);
            }
//
//            chassis.stopDriving();
//
//            arm.grab();
//
//            Thread.sleep(1000);
//
//            arm.partial2();
//
//            Thread.sleep(500);
//
//            chassis.rightShiftAutonomous(SHIFT_POWER, 400);
//
////            arm.up();
//
//            chassis.driveAutonomous(DRIVE_POWER, 2500);
//
//            chassis.leftShiftAutonomous(SHIFT_POWER, 400);
//
//            arm.down();
//
//            Thread.sleep(500);
//            arm.open();
//            Thread.sleep(500);
//            arm.up();
//            arm.grab();
//
//            chassis.rightShiftAutonomous(SHIFT_POWER,400);
//
//            chassis.driveAutonomous(-DRIVE_POWER, -3000);

//            chassis.rightShiftAutonomous(SHIFT_POWER, 1050);
//
//            if (skystoneLocation== CV.location.LEFT) {
//                driveDistance = 200;
//            }
//            if (skystoneLocation== CV.location.MID) {
//                driveDistance = 100;
//            }
//            if (skystoneLocation== CV.location.RIGHT) {
//                driveDistance = 0;
//            }






//            chassis.driveAutonomous(0.5, 1000);

//            arm.partial();
//
//            arm.open();
//
//            chassis.rightShiftAutonomous(SHIFT_POWER, 1050);
//
//            if (skystoneLocation== CV.location.LEFT) {
//                driveDistance = 200;
//            }
//            if (skystoneLocation== CV.location.MID) {
//                driveDistance = 100;
//            }
//            if (skystoneLocation== CV.location.RIGHT) {
//                driveDistance = 0;
//            }
//
//            chassis.driveAutonomous(DRIVE_POWER, driveDistance);
//
//            arm.down();
//
//            arm.grab();
//
//            Thread.sleep(1000);
//
//            arm.partial2();
//
//            Thread.sleep(500);
//
//            chassis.leftShiftAutonomous(SHIFT_POWER, 400);
//
////            arm.up();
//
//            chassis.driveAutonomous(-DRIVE_POWER, -2500);
//
//            chassis.rightShiftAutonomous(SHIFT_POWER, 400);
//
//            arm.down();
//
//            Thread.sleep(500);
//            arm.open();
//            Thread.sleep(500);
//            arm.up();
//            arm.grab();
//
//            chassis.leftShiftAutonomous(SHIFT_POWER,400);
//
//            chassis.driveAutonomous(DRIVE_POWER, 3000);
//
//
//
//
////            }



            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
