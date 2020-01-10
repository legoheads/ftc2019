//Run from the necessary package
package org.firstinspires.ftc.teamcode.tele;

//Import necessary items
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.arm.redArm;
import org.firstinspires.ftc.teamcode.subsystems.chassis.*;
import org.firstinspires.ftc.teamcode.subsystems.slides.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.*;
import org.firstinspires.ftc.teamcode.subsystems.platform.*;
import org.firstinspires.ftc.teamcode.subsystems.stacker.stacker;

import java.sql.Time;
import java.util.Locale;

@TeleOp(name="teleOp") //Name the class
public class teleOp extends LinearOpMode {

    //Define floats to be used as joystick inputs and trigger inputs
    private float drivePower, shiftPower, leftTurnPower, rightTurnPower, spoolPower;

    float flipUpPower = (float) 0.5;
    float flipDownPower = (float) 0.3;
    float maxPower = (float) 0.8;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    ElapsedTime extendTime = new ElapsedTime();

    private skystoneChassis chassis;
    private redArm arm;
    private Platform platform;
    private IntakeWheels intake;
    private LinearSlides slides;
    private stacker stacker;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {


        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        arm = new redArm(hardwareMap);
        platform = new platformArms(hardwareMap);
        stacker = new stacker(hardwareMap, gamepad1, gamepad2);

        //Wait for start button to be clicked
        waitForStart();

        intake.intake();

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
            spoolPower = -(gamepad1.right_stick_y + gamepad2.right_stick_y);

            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
               chassis.driveTeleop(drivePower);
            }

            if (Math.abs(spoolPower)>0.1){

                slides.moveSpool(spoolPower);
            }
            slides.stop();


            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
              chassis.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
              chassis.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
//                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
                chassis.rightTurnTeleop(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15) {
                chassis.stopDriving();
            }

            //Intake
            if (gamepad1.right_bumper) { intake.intake(); }
            //Eject
            if (gamepad1.left_bumper) { intake.eject(); }
            telemetry.addData("Intake: ", intake.getIntakeState());


//            if(gamepad1.dpad_up) { platform.up(); }
//
//            if(gamepad1.dpad_down){ platform.grab(); }


            if (gamepad2.dpad_right) {
                stacker.extend();
                intake.stop();
            }

            if (gamepad2.x) { stacker.drop(); }

            if (gamepad2.dpad_left) {
                stacker.retract();
                intake.intake();
            }

            if (gamepad2.b){
                chassis.stopDriving();
                intake.stop();
            }

            if(gamepad1.back || gamepad2.back){
                stacker.capDrop();
            }

            if (gamepad1.y){
                slides.spoolEncoder();
            }

            if (gamepad1.dpad_left){
                stacker.stoneShiftLeft();
            }

            if (gamepad1.dpad_right){
                stacker.stoneShiftRight();
            }

            if (gamepad1.dpad_down){
                stacker.stoneReverse();
            }


            telemetry.addData("Runtime", runtime);
            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program