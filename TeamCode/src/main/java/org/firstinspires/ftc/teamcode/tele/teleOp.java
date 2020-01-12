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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
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

    private int raiseClick = 0;

    private Arm blueArm;
    private Arm redArm;

    private Servo saberLeft;
    private Servo saberRight;


    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {


        saberLeft = hardwareMap.servo.get("saberLeft");
        saberLeft = hardwareMap.servo.get("saberRight");


        intake = new intake(hardwareMap);
        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.FLOAT);
        arm = new redArm(hardwareMap);
        platform = new platformArms(hardwareMap);
        stacker = new stacker(hardwareMap, gamepad1, gamepad2);
        redArm = new redArm(hardwareMap);
        blueArm = new redArm(hardwareMap);

        slides.getSpoolLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        //Wait for start button to be clicked
        waitForStart();

        redArm.up();
        redArm.grab();

        blueArm.up();
        blueArm.grab();

        platform.up();

        intake.intake();

        DcMotor spoolLeft = slides.getSpoolLeft();



    //***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            //DRIVE MOTOR CONTROLS
            drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y)*(float)0.5;
            shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x)*(float)0.5;
            leftTurnPower = ((gamepad1.left_trigger) *(float) 0.5);
            rightTurnPower = ((gamepad1.right_trigger) *(float) 0.5);
            spoolPower = -(gamepad1.right_stick_y + gamepad2.right_stick_y);


            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower)) {
               chassis.driveTeleop(drivePower);
            }

            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15) {
                chassis.stopDriving();
            }

            if (spoolPower>0.1){

                slides.moveSpool(spoolPower*(float)0.6);
            }
            if (spoolPower<0.1){

                slides.moveSpool(spoolPower);
            }

            slides.stop();

            if(gamepad1.y) {
                platform.up();
            }

            if(gamepad1.a){
                stacker.stoneReverse();
                platform.grab();
            }

            if (gamepad1.x){
                saberLeft.setPosition(1.0);
                saberRight.setPosition(0.0);
            }

            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower)) {
              chassis.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0) {
              chassis.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0) {
                chassis.rightTurnTeleop(rightTurnPower);
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

            if (gamepad2.right_bumper) {
                slides.spoolEncoder(0.8, 420);
            }

            if (gamepad2.left_bumper) {
                slides.spoolEncoder(-0.8, -420);
            }





            //Eject
            if (gamepad1.left_bumper) {
                intake.eject();
                chassis.setDriveMotorPowers(-0.5, -0.5,-0.5, -0.5);
                Thread.sleep(500);
                chassis.stopDriving();
            }
            else{
                //Intake
                intake.intake();
            }


            telemetry.addData("Intake: ", intake.getIntakeState());

            //Killswitch
            if (gamepad1.b || gamepad2.b){
                chassis.stopDriving();
                slides.stop();
                intake.stop();
            }



            //Stacker System
            if (gamepad2.dpad_right) {
                stacker.extend();
                intake.stop();
            }

            if (gamepad2.dpad_left) {
                stacker.retract();
                intake.intake();
            }

            if (gamepad2.x) { stacker.drop(); }



            if (gamepad2.b){
                chassis.stopDriving();
                intake.stop();
            }



            if(gamepad2.back){
                stacker.capDrop();
            }







            telemetry.addData("Runtime", runtime);
            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
}//Close class and end program