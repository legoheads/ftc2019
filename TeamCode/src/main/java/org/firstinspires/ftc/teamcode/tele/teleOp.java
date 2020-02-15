//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.Distance;
import org.firstinspires.ftc.teamcode.subsystems.distanceSensor.distanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeWheels;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake;
import org.firstinspires.ftc.teamcode.subsystems.platform.Platform;
import org.firstinspires.ftc.teamcode.subsystems.platform.platformArms;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
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

    int two = 2;

    private skystoneChassis chassis;
    private Platform platform;
    private IntakeWheels intake;
    private LinearSlides slides;
    private stacker stacker;
    private IIMU imu;

    private Servo saber;

    private Distance distanceSensor;

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        saber = hardwareMap.servo.get("saber");

        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        platform = new platformArms(hardwareMap);
        stacker = new stacker(hardwareMap);
        distanceSensor = new distanceSensor(hardwareMap);

        imu = new BoschIMU(hardwareMap);

        telemetry.addData("init complete", two);
        telemetry.update();

        //Wait for start button to be clicked
        waitForStart();

        intake.intake();

    //***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //DRIVE MOTOR CONTROLS
            drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y) * CHASSIS_POWER;
            shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x) * CHASSIS_POWER;
            leftTurnPower = (gamepad1.left_trigger + gamepad2.left_trigger) * CHASSIS_POWER;
            rightTurnPower = (gamepad1.right_trigger + gamepad2.right_trigger) * CHASSIS_POWER;
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

            if(gamepad1.y)
            {
                platform.up();
            }

            if(gamepad1.a)
            {
                platform.grab();
            }

            //Saber red
            if (gamepad2.y)
            {
                saber.setPosition(0.25);
            }

            //Saber blue
            if (gamepad2.a)
            {
                saber.setPosition(1.0);
            }

            if (gamepad1.dpad_up)
            {
                chassis.setDriveMotorPowers(0.3, 0.3, 0.3, 0.3);
            }

            if (gamepad1.dpad_left)
            {
                distanceSensor.stoneShiftLeft();
            }

            if (gamepad1.dpad_right)
            {
                distanceSensor.stoneShiftRight();
            }

            if (gamepad1.dpad_down)
            {
                distanceSensor.platformReverse();
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
                intake.stop();
                stacker.grab();
            }

            if (gamepad2.b)
            {
                stacker.ungrab();
            }


            //Add cap here
            if(gamepad2.back)
            {

            }

            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
}//Close class and end program