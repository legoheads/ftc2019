//Run from the necessary package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

@TeleOp(name="NEWBOT TELE") //Name the class
public class testbotTele2 extends LinearOpMode {
    //Drivetrain
    private DcMotor LF, RF, LB, RB;

    private DcMotor intakeLeft, intakeRight;

    private Servo platLeft, platRight;

    //Define floats to be used as joystick inputs and trigger inputs
    private double drivePower, shiftPower, leftTurnPower, rightTurnPower;

    private IIMU imu;

    private skystoneChassis chassis;

    private double slowPower = 0.3;


    //Define a function to use to set motor powers
    public void setDriveMotorPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        //Use the entered powers and feed them to the motors
        LF.setPower(leftFrontPower);
        LB.setPower(leftBackPower);
        RF.setPower(rightFrontPower);
        RB.setPower(rightBackPower);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        //Hardware mapping
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        platLeft = hardwareMap.servo.get("platLeft");
        platRight = hardwareMap.servo.get("platRight");


        imu = new BoschIMU(hardwareMap);

        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOWF
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

            //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = -((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.3);
            shiftPower = -((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.3);
            leftTurnPower = ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.3);
            rightTurnPower = ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.3);

            if (gamepad1.a){
                intakeLeft.setPower(0.5);
                intakeRight.setPower(-0.5);
            }

            if (gamepad1.x){
                intakeLeft.setPower(-0.5);
                intakeRight.setPower(0.5);
            }


            if (gamepad1.dpad_up){
                platLeft.setPosition(0.5);
                platRight.setPosition(0.5);
            }

            if (gamepad1.dpad_down){
                platLeft.setPosition(1.0);
                platRight.setPosition(0.0);
            }

            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower)) {
                setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            }


            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower)) {
                setDriveMotorPowers(-shiftPower, shiftPower, shiftPower, -shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0) {
                setDriveMotorPowers(-leftTurnPower, -leftTurnPower, leftTurnPower, leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0) {
                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15) {
                setDriveMotorPowers(0.0, 0.0, 0.0, 0.0);
            }

            if (gamepad1.b) {
                LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


//            if (gamepad1.dpad_up)
//            {
//                chassis.odometryDrive(LF, RB, slowPower, 500, telemetry);
//            }
//
//            if (gamepad1.dpad_down)
//            {
//                chassis.odometryDrive(LF, RB, -slowPower, -500, telemetry);
//            }


            telemetry.addData("y displacement left (port 0): ", LF.getCurrentPosition());
            telemetry.addData("y displacement right (port 1): ", RB.getCurrentPosition());
            telemetry.addData("z angle (use this one): ", imu.getZAngle());
            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
