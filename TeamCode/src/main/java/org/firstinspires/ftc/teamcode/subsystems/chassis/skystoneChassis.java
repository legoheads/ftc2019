//Run from the package
package org.firstinspires.ftc.teamcode.subsystems.chassis;

//Import necessary items

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.imu.*;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class skystoneChassis implements DriveTrain {
    //Define drive motors
    private DcMotor LF, LB, RF, RB;

    private IIMU imu;
    private HardwareMap hardwareMap;

    private LinearSlides slides;

    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public skystoneChassis(HardwareMap hardwareMap, DcMotor.ZeroPowerBehavior type) {

        this.hardwareMap = hardwareMap;

        slides = new slides(hardwareMap);

        //Hardware mapping
        this.LF = hardwareMap.dcMotor.get("LF");
        this.LB = hardwareMap.dcMotor.get("LB");
        this.RF = hardwareMap.dcMotor.get("RF");
        this.RB = hardwareMap.dcMotor.get("RB");

        //Reverse right side motors
        this.LF.setDirection(DcMotorSimple.Direction.FORWARD);
        this.LB.setDirection(DcMotorSimple.Direction.FORWARD);
        this.RF.setDirection(DcMotorSimple.Direction.REVERSE);
        this.RB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the drive motors either Brake or Float
        this.LF.setZeroPowerBehavior(type);
        this.LB.setZeroPowerBehavior(type);
        this.RF.setZeroPowerBehavior(type);
        this.RB.setZeroPowerBehavior(type);

        imu = new BoschIMU(hardwareMap);
        imu.init();
    }

    /**
     * Takes in motor powers for 4 drive motors
     */
    public void setDriveMotorPowers(double LFPower, double LBPower, double RFPower, double RBPower) {
        //Use the entered powers and feed them to the motors
        LF.setPower((float) LFPower);
        LB.setPower((float) LBPower);
        RF.setPower((float) RFPower);
        RB.setPower((float) RBPower);
    }

    /**
     * If this function is called, stop the drive motors
     */
    public void stopDriving() {
        //Set all drive motor powers as zero
        setDriveMotorPowers(0.0, 0.0, 0.0, 0.0);
    }


    /**
     * If this function is called, turn on the drive motors at the given powers to make it drive forward or backwards
     */
    public void driveTeleop(double power) throws InterruptedException {
        //Send all the motors in the same direction
        setDriveMotorPowers(power, power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn left
     */
    public void leftTurnTeleop(double power) throws InterruptedException {
        //Turn the left motors backwards and the right motors forward so that it turns left
        setDriveMotorPowers(-power, -power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurnTeleop(double power) throws InterruptedException {
        //Turn the right motors backwards and the left motors forward so that it turns right
        setDriveMotorPowers(power, power, -power, -power);
    }

    /**
     * If this function is called, turn on the drive motors at the
     * given powers, to make it shift in the desired direction
     */
    public void shiftTeleop(double power) throws InterruptedException {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift
        setDriveMotorPowers(-power, power, power, -power);
    }


    public void stopResetEncoders() throws InterruptedException {
        //Reset the encoders
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoder(Boolean status){
        if (status){
            //Use the encoders
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Takes in powers for 4 drive motors, as well as 4 encoder distances
     * Allows us to run at the entered power, for the entered distance
     */
    public void driveAutonomous(double power, int degrees) throws InterruptedException {
        //Reset the encoders
        stopResetEncoders();

        useEncoder(true);

        //Sets the target position as the corresponding values entered
        LF.setTargetPosition(degrees);
        LB.setTargetPosition(degrees);
        RF.setTargetPosition(degrees);
        RB.setTargetPosition(degrees);

        //Set up the motors to run to the given position
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double startAngle = imu.getZAngle();
        double COEFF = 0.94;

        ElapsedTime timer = new ElapsedTime();

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(power, power, power, power);

        timer.reset();

        //Empty while loop while the motors are moving
        while (((LF.isBusy()) && (RF.isBusy()) && (LB.isBusy()) && (RB.isBusy())) || (timer.seconds() < 4.0))
        {
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0) {
                if (imu.getZAngle() > startAngle) {
                    setDriveMotorPowers(power, power, COEFF * power, COEFF * power);
                }

                if (imu.getZAngle() < startAngle) {
                    setDriveMotorPowers(COEFF * power, COEFF * power, power, power);
                }
            }
        }

        //Stop driving
        stopDriving();

        useEncoder(true);
    }

    public void driveAutoTest(double power, int degrees) throws InterruptedException {
        //Reset the encoders
        stopResetEncoders();

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets the target position as the corresponding values entered
        LF.setTargetPosition(degrees);

        //Set up the motors to run to the given position
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double startAngle = imu.getZAngle();
        double COEFF = 0.94;

        ElapsedTime timer = new ElapsedTime();

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(power, power, power, power);

        timer.reset();

        //Empty while loop while the motors are moving
        while (LF.isBusy()) {
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0) {
                if (imu.getZAngle() > startAngle) {
                    setDriveMotorPowers(power, power, COEFF * power, COEFF * power);
                }

                if (imu.getZAngle() < startAngle) {
                    setDriveMotorPowers(COEFF * power, COEFF * power, power, power);
                }
            }
        }

        //Stop driving
        stopDriving();

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        useEncoder(true);
    }


    /**
     * Shift right for the given distance at the given power
     *
     * @param degrees distance
     */
    public void rightShiftAutonomous( double power, int degrees) throws InterruptedException {
        //Reset the encoders
        stopResetEncoders();

        //Set up the motors to run to the given position
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the target position as the corresponding values entered
        LF.setTargetPosition(degrees);
        LB.setTargetPosition(-degrees);
        RF.setTargetPosition(-degrees);
        RB.setTargetPosition(degrees);

        double startAngle = imu.getZAngle();
        double COEFF = 0.94;

        ElapsedTime timer = new ElapsedTime();

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(power, -power, -power, power);

        timer.reset();

        //Empty while loop while the motors are moving
        while ((LF.isBusy()) && (RF.isBusy()) && (LB.isBusy()) && (RB.isBusy()) || (timer.seconds() < 4.0))
        {
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0) {
                if (imu.getZAngle() > startAngle) {
                    setDriveMotorPowers(power, COEFF * power, power, COEFF * power);
                }

                if (imu.getZAngle() < startAngle) {
                    setDriveMotorPowers(COEFF * power, power, COEFF * power, power);
                }
            }
        }

        //Stop driving
        stopDriving();

        useEncoder(true);
    }

    /**
     * Shift right for the given distance at the given power
     *
     * @param degrees distance
     */
    public void leftShiftAutonomous( double power, int degrees) throws InterruptedException {
        //Reset the encoders
        stopResetEncoders();

        useEncoder(true);

        //Set up the motors to run to the given position
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the target position as the corresponding values entered
        LF.setTargetPosition(-degrees);
        LB.setTargetPosition(degrees);
        RF.setTargetPosition(degrees);
        RB.setTargetPosition(-degrees);

        double startAngle = imu.getZAngle();
        double COEFF = 0.94;

        ElapsedTime timer = new ElapsedTime();

        //Turn on the motors at the corresponding powers
        setDriveMotorPowers(-power, power, power, -power);

        timer.reset();

        //Empty while loop while the motors are moving
        while ((LF.isBusy()) && (RF.isBusy()) && (LB.isBusy()) && (RB.isBusy()) || (timer.seconds() < 4.0))
        {
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0) {
                if (imu.getZAngle() > startAngle) {
                    setDriveMotorPowers(COEFF * power, power, COEFF * power, power);
                }

                if (imu.getZAngle() < startAngle) {
                    setDriveMotorPowers(power, COEFF * power, power, COEFF * power);
                }
            }
        }

        //Stop driving
        stopDriving();

        useEncoder(true);
    }


    public void rightTurnIMU(double power, int target) throws InterruptedException {
        while (imu.getZAngle() > target) {
            rightTurnTeleop(power);
        }
        stopDriving();
        while (imu.getZAngle() < target) {
            leftTurnTeleop(0.2);
        }
        stopDriving();
    }

    public void leftTurnIMU(double power, int target) throws InterruptedException {
        while (imu.getZAngle() < target) {
            leftTurnTeleop(power);
        }

        stopDriving();
        while (imu.getZAngle() > target) {
            rightTurnTeleop(0.2);
        }
        stopDriving();

    }

    public void pidIMULeft(double power, int degrees) throws InterruptedException {
        while (Math.abs((double) degrees - imu.getZAngle()) > 1) {
            while (imu.getZAngle() < degrees) {
                leftTurnTeleop(power);
            }
            stopDriving();
            while (imu.getZAngle() > degrees) {
                rightTurnTeleop(power);
            }
            stopDriving();
        }
        stopDriving();
    }

    public void pidIMURight(double power, int degrees) throws InterruptedException {
        while (Math.abs((double) degrees - imu.getZAngle()) > 1) {
            while (imu.getZAngle() > degrees) {
                rightTurnTeleop(power);
            }
            stopDriving();
            while (imu.getZAngle() < degrees) {
                leftTurnTeleop(power);
            }
            stopDriving();
        }
        stopDriving();
    }


    public void odometryMotion(DcMotor motor, double LFPower, double LBPower, double RFPower, double RBPower, int degrees, Telemetry telemetry) {
        degrees *= -1;
        //Empty while loop while the motors are moving
        while (Math.abs(motor.getCurrentPosition() - degrees) > 20) {
            telemetry.addData("enc", motor.getCurrentPosition());
            setDriveMotorPowers(LFPower, LBPower, RFPower, RBPower);
            telemetry.update();
        }

        //Stop driving
        stopDriving();
    }

    public void odometryDrive(DcMotor motor, double power, int degrees, Telemetry telemetry) throws InterruptedException {
        odometryMotion(motor, power, power, power * 0.92, power * 0.92, degrees, telemetry);
    }

    public void odometryLeftShift(DcMotor motor, double power, int degrees, Telemetry telemetry) throws InterruptedException {
        odometryMotion(motor, -power * 0.95, power, power * 0.95, -power, degrees, telemetry);
    }

    public void odometryRightShift(DcMotor motor, double power, int degrees, Telemetry telemetry) throws InterruptedException {
        odometryMotion(motor, power * 0.95, -power, -power * 0.95, power, degrees, telemetry);
    }


    public void chassisTeleOp(Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        float drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y)*(float)0.7;
        float shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x)*(float)0.7;
        float leftTurnPower = (gamepad1.left_trigger + gamepad2.left_trigger)*(float)0.5;
        float rightTurnPower = (gamepad1.right_trigger + gamepad2.right_trigger)*(float)0.5;
        float spoolPower = -(gamepad2.right_stick_y);

        //Drive if joystick pushed more Y than X on gamepad1 (fast)
        if (Math.abs(drivePower) > Math.abs(shiftPower)) {
            driveTeleop(drivePower);
        }

        if (Math.abs(spoolPower)>0.1){

            slides.moveSpool(spoolPower);
        }
        slides.stop();

        //Shift if pushed more on X than Y on gamepad1 (fast)
        if (Math.abs(shiftPower) > Math.abs(drivePower)) {
            shiftTeleop(shiftPower);
        }

        //If the left trigger is pushed on gamepad1, turn left at that power (fast)
        if (leftTurnPower > 0) {
            leftTurnTeleop(leftTurnPower);
        }

        //If the right trigger is pushed on gamepad1, turn right at that power (fast)
        if (rightTurnPower > 0) {
            rightTurnTeleop(rightTurnPower);
        }

        //If the joysticks are not pushed significantly shut off the wheels
        if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15) {
            stopDriving();
        }

        if (gamepad2.right_bumper) {
            slides.spoolEncoder(0.8, 370);
        }

        if (gamepad2.left_bumper) {
            slides.spoolEncoder(-0.8, -370);
        }
    }

    /**
     * If this function is called, it enables us to run one DC motor to a specific distance
     */
    public static void oneMotorEncoder(DcMotor motor, double power, int degrees) throws InterruptedException {
        int firstPos, secondPos;

        //Use the encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up the motor to run to the given position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        motor.setTargetPosition(motor.getCurrentPosition() + degrees);

        //Turn the motor on at the corresponding power
        motor.setPower((float) power);

        //Empty while loop while the motor is moving
        while ((motor.isBusy())) {
            firstPos = motor.getCurrentPosition();
            Thread.sleep(75);
            secondPos = motor.getCurrentPosition();

            if (Math.abs(firstPos - secondPos) < 5) {
                break;
            }
        }

        //Stop the motor
        motor.setPower(0.0);

        //Use the encoder in the future
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void omeWithDriveMotors(DcMotor motor, double power, int degrees, Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        runTime.reset();

        //Use the encoder
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up the motor to run to the given position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        motor.setTargetPosition(motor.getCurrentPosition() + degrees);

        //Turn the motor on at the corresponding power
        motor.setPower((float) power);

        //Empty while loop while the motor is moving
        while ((motor.isBusy()) && runTime.time() < 3000) {
            chassisTeleOp(gamepad1, gamepad2);
        }
        stopDriving();

        //Stop the motor
        motor.setPower(0.0);

        //Use the encoder in the future
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

