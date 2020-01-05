//Run from the necessary package
package org.firstinspires.ftc.teamcode.auto.Blue;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.CV.CV;
import org.firstinspires.ftc.teamcode.subsystems.CV.skystoneDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.sideArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

@Autonomous(name="AutoBlue Full", group = "blue") //Name the class
public class autoBlueFull extends LinearOpMode {

    //Drivetrain motors
    private DcMotor LF, LB, RF, RB;

    //Intake motors
    private DcMotor intakeLeft, intakeRight;

    //Linear slide spool motors
    private DcMotor spoolLeft, spoolRight;

    //Sidearm
    private Servo sideLift, twister, sideGrab;

    private float DRIVE_POWER = (float) 0.3;
    private float TURN_POWER = (float) 0.3;
    private float SHIFT_POWER = (float) 0.3;

    private BNO055IMU boschIMU;

    //Skystone location variable
    private CV.location skystoneLocation = CV.location.MID;

    private Arm arm;
    private IntakeWheels intake;
    private LinearSlides slides;
    private skystoneChassis chassis;
    private skystoneDetector detector;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        //Intialize subsystems

        intake = new intake(hardwareMap, intakeLeft, intakeRight);
        slides = new slides(hardwareMap, spoolLeft, spoolRight);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE, LF, LB, RF, RB, boschIMU);
        arm = new sideArm(hardwareMap, sideLift, twister, sideGrab);
        detector = new skystoneDetector(hardwareMap, telemetry);

        //Initialize sidearm servos
        arm.twist();
        arm.up();
        arm.grab();


        //Look for skystone until play is pressed
        while(!isStarted()){ skystoneLocation = detector.getSkystoneInfinite(); }

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {


            if (skystoneLocation== CV.location.LEFT){

            }

            if (skystoneLocation== CV.location.MID){

            }

            if (skystoneLocation== CV.location.RIGHT){

            }

            arm.partial();
            Thread.sleep(500);
            arm.parallel();
            arm.open();
            Thread.sleep(500);
            arm.down();
            Thread.sleep(500);
            arm.grab();
            Thread.sleep(500);
            arm.partial();
            Thread.sleep(200);
            arm.twist();
            Thread.sleep(500);
            arm.up();

            chassis.driveAutonomous(DRIVE_POWER, 3000);

            chassis.rightShiftAutonomous(SHIFT_POWER, 300);

            sideLift.setPosition(0.15);
            Thread.sleep(500);
            arm.open();
            Thread.sleep(500);
            arm.up();

            chassis.leftShiftAutonomous(SHIFT_POWER,300);

            chassis.driveAutonomous(-DRIVE_POWER, -3000);

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
