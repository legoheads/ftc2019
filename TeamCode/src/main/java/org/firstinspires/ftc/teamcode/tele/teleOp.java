//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.sideArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.*;
import org.firstinspires.ftc.teamcode.subsystems.slides.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.*;
import org.firstinspires.ftc.teamcode.subsystems.platform.*;

@TeleOp(name="teleOp") //Name the class
public class teleOp extends LinearOpMode
{
    //Drivetrain
    private DcMotor LF, LB, RF, RB;

    //Intake
    private DcMotor intakeLeft, intakeRight;

    private DcMotor spoolLeft, spoolRight;

    //Outtake
    Servo gripper;
    DcMotor spool;
    CRServo extend;

    //Platform mover
    private Servo platformLeft, platformRight;

    //Sidearm
    private Servo sideLift, twister, sideGrab;


    //Define floats to be used as joystick inputs and trigger inputs
    private float drivePower, shiftPower, leftTurnPower, rightTurnPower, spoolPower;

    float flipUpPower = (float) 0.5;
    float flipDownPower = (float) 0.3;
    float maxPower = (float) 0.8;

    ElapsedTime extendTime = new ElapsedTime();

    int xPress = 0;
    int yPress = 0;

    BNO055IMU boschIMU;

    private Arm arm;
    private Platform platform;
    private skystoneChassis chassis;
    private IntakeWheels intake;
    private LinearSlides slides;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {


        intake = new intake(hardwareMap, intakeLeft, intakeRight);
        slides = new slides(hardwareMap, spoolLeft, spoolRight);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE, LF, LB, RF, RB, boschIMU);
        arm = new sideArm(hardwareMap, sideLift, twister, sideGrab);
        platform = new platformArms(hardwareMap, platformLeft, platformRight);

        //Get references to the Servo Motors from the hardware map
        gripper = hardwareMap.servo.get("gripper");
        extend = hardwareMap.crservo.get("extend");

        gripper.setPosition(0.6);

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //DRIVE MOTOR CONTROLS
            drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y);
            shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.5);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.5);
            spoolPower = gamepad1.right_stick_y + gamepad2.right_stick_y;


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
               chassis.driveTeleop(drivePower);
            }

            if (Math.abs(spoolPower)<0.1){
                slides.moveSpool(spoolPower);
            }

            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
              chassis.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
//              chassis.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
//                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
                chassis.rightTurnTeleop(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                chassis.stopDriving();
            }

            //Intake
            if (gamepad1.right_bumper) { intake.intake(); }
            //Eject
            if (gamepad1.left_bumper) { intake.eject(); }
            telemetry.addData("Intake: ", intake.getIntakeState());


            if(gamepad1.dpad_up) { platform.up(); }

            if(gamepad1.dpad_down){ platform.grab(); }


            if (gamepad2.dpad_right)
            {
                gripper.setPosition(0.45);
                extend.setPower(1.0);
            }

            if (gamepad2.dpad_left)
            {
                gripper.setPosition(0.6);
                Thread.sleep(100);
                gripper.setPosition(0.525);
                extend.setPower(-1.0);
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
