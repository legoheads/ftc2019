//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.leftArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.rightArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.*;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.slides.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.*;
import org.firstinspires.ftc.teamcode.subsystems.platform.*;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

@TeleOp(name="teleOp") //Name the class
public class teleOp extends LinearOpMode {

    //Define floats to be used as joystick inputs and trigger inputs
    private double drivePower, shiftPower, leftTurnPower, rightTurnPower, spoolPower;

    private final double MAX_POWER = 0.8;
    private final double CHASSIS_POWER = 0.7;
    private final double STOP_POWER = 0.0;

    private final double ERROR_MARGIN = 0.1;

    private final int LIFT_DISTANCE = 420;

//    private String team = "red";
    private String team = "blue";

    int two = 2;

    private skystoneChassis chassis;
    private Platform platform;
    private IntakeWheels intake;
    private LinearSlides slides;
    private stacker stacker;
    private Arm leftArm;
    private Arm rightArm;
    private IIMU imu;

    private Servo saber;
    private Servo shortSaber;

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        saber = hardwareMap.servo.get("saber");
        shortSaber = hardwareMap.servo.get("shortSaber");

        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        platform = new platformArms(hardwareMap);
        stacker = new stacker(hardwareMap, gamepad1, gamepad2);
        leftArm = new leftArm(hardwareMap);
        rightArm = new rightArm(hardwareMap);
        imu = new BoschIMU(hardwareMap);

        platform.teleInit();
        shortSaber.setPosition(0.45);

        telemetry.addData("init complete", two);
        telemetry.update();

        //Wait for start button to be clicked
        waitForStart();

        intake.intake();

    //***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //DRIVE MOTOR CONTROLS
            drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y) * CHASSIS_POWER;
            shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x) * CHASSIS_POWER;
            leftTurnPower = (gamepad1.left_trigger) * CHASSIS_POWER;
            rightTurnPower = (gamepad1.right_trigger) * CHASSIS_POWER;
            spoolPower = -(gamepad1.right_stick_y + gamepad2.right_stick_y);


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
               chassis.driveTeleop(drivePower);
            }

            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < ERROR_MARGIN)
            {
                chassis.stopDriving();
            }

            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                chassis.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > STOP_POWER)
            {
                chassis.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > STOP_POWER)
            {
                chassis.rightTurnTeleop(rightTurnPower);
            }

            if (Math.abs(spoolPower) > ERROR_MARGIN)
            {
                slides.moveSpool(spoolPower);
            }

            slides.stop();

            if(gamepad1.y)
            {
                platform.up();
            }

            if(gamepad1.a)
            {
//                stacker.platformReverse();
                platform.grab();
            }

            //Saber red
            if (gamepad2.y)
            {
                leftArm.open();
                rightArm.open();
                saber.setPosition(0.0);
//                saberRight.setPosition(0.0);
            }

            //Saber blue
            if (gamepad2.a)
            {
                leftArm.open();
                rightArm.open();
                saber.setPosition(0.9);
            }

            if (gamepad1.dpad_up)
            {
                chassis.setDriveMotorPowers(0.3, 0.3, 0.3, 0.3);
            }



            if (gamepad1.dpad_left)
            {
                stacker.stoneShiftLeft();
            }

            if (gamepad1.dpad_right)
            {
                stacker.stoneShiftRight();
            }

            if (gamepad1.dpad_down)
            {
                stacker.platformReverse();
            }

            if (gamepad2.right_bumper)
            {
                slides.spoolEncoder(MAX_POWER, LIFT_DISTANCE);
            }

            if (gamepad2.left_bumper)
            {
                slides.spoolEncoder(-MAX_POWER, -LIFT_DISTANCE);
            }

            //Eject
            if (gamepad1.left_bumper)
            {
                intake.eject();
                chassis.setDriveMotorPowers(- CHASSIS_POWER, - CHASSIS_POWER, - CHASSIS_POWER, - CHASSIS_POWER);
                Thread.sleep(500);
                chassis.stopDriving();
            }
            //Intake
            else
            {
                intake.intake();
            }

            telemetry.addData("Intake: ", intake.getIntakeState());

            //Killswitch
            if (gamepad1.b)
            {
                chassis.stopDriving();
                slides.stop();
                intake.stop();
            }

            //Stacker System
            if (gamepad2.dpad_right)
            {
                stacker.extend();
                intake.stop();
            }

            if (gamepad2.dpad_left)
            {
                stacker.retract();
                intake.intake();
            }

            if (gamepad2.x)
            {
                stacker.grab();
            }

            if (gamepad2.b)
            {
                chassis.stopDriving();
                intake.stop();
            }

            if(gamepad2.back)
            {
                //                stacker.capDrop();

                platform.grab();
                rightArm.releaseAuto();
                leftArm.releaseAuto();
                Thread.sleep(1000);

                //Do not open arms
                leftArm.up();
                rightArm.up();

                platform.up();
            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
}//Close class and end program