//Run from the package
package org.firstinspires.ftc.teamcode.subsystems.chassis;

//Import necessary items

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;

public class skystoneChassis implements DriveTrain
{
    //Define drive motors
    private DcMotor LF, LB, RF, RB;

    private IIMU imu;
    private HardwareMap hardwareMap;

    private LinearSlides slides;

    private double MAX_POWER = 1.0;
    private double SLOW_POWER = 0.2;

    private double initial;
    private double target;
    private double startAngle;
    private double COEFF;

    //PID variables
    private double error = 0.0;
    private double oldError = 0.0;
    private double currentTime = 0.0;
    private double deltaTime = 0.0;
    private double integral = 0.0;
    private double derivative = 0.0;



    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public skystoneChassis(HardwareMap hardwareMap, DcMotor.ZeroPowerBehavior type)
    {
        this.hardwareMap = hardwareMap;

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
        slides = new slides(hardwareMap);
    }

    /**
     * Takes in motor powers for 4 drive motors
     */
    public void setDriveMotorPowers(double LFPower, double LBPower, double RFPower, double RBPower)
    {
        //Use the entered powers and feed them to the motors
        LF.setPower(LFPower);
        LB.setPower(LBPower);
        RF.setPower(RFPower);
        RB.setPower(RBPower);
    }

    /**
     * If this function is called, stop the drive motors
     */
    public void stopDriving()
    {
        //Set all drive motor powers as zero
        setDriveMotorPowers(0.0, 0.0, 0.0, 0.0);
    }


    /**
     * If this function is called, turn on the drive motors at the given powers to make it drive forward or backwards
     */
    public void driveTeleop(double power) throws InterruptedException
    {
        //Send all the motors in the same direction
        setDriveMotorPowers(power, power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn left
     */
    public void leftTurnTeleop(double power) throws InterruptedException
    {
        //Turn the left motors backwards and the right motors forward so that it turns left
        setDriveMotorPowers(-power, -power, power, power);
    }

    /**
     * If this function is called, turn on the drive motors at the given powers, to make it tank turn right
     */
    public void rightTurnTeleop(double power) throws InterruptedException
    {
        //Turn the right motors backwards and the left motors forward so that it turns right
        setDriveMotorPowers(power, power, -power, -power);
    }

    /**
     * If this function is called, turn on the drive motors at the
     * given powers, to make it shift in the desired direction
     */
    public void shiftTeleop(double power) throws InterruptedException
    {
        //This sequence of backwards, forwards, forwards, backwards makes the robot shift
        setDriveMotorPowers(-power, power, power, -power);
    }


    public void stopResetEncoders() throws InterruptedException
    {
        //Reset the encoders
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoder(Boolean status)
    {
        if (status)
        {
            //Use the encoders
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else
        {
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //START FIXING HERE

    /**
     * Takes in powers for 4 drive motors, as well as 4 encoder distances
     * Allows us to run at the entered power, for the entered distance
     */
    public void driveForwardsAutonomous(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = Math.abs(power);
        degrees = Math.abs(degrees);

        initial = LF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;

        while (LF.getCurrentPosition() < target)
        {
            driveTeleop(power);
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    public void driveBackwardsAutonomous(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = -Math.abs(power);
        degrees = -Math.abs(degrees);

        initial = LF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;

        while (LF.getCurrentPosition() > target)
        {
            driveTeleop(power);
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    /**
     * Shift right for the given distance at the given power
     *
     * @param degrees distance
     */
    public void leftShiftAutonomous(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = Math.abs(power);
        degrees = Math.abs(degrees);

        initial = RF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;

        while (RF.getCurrentPosition() < target)
        {
            shiftTeleop(power);
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    /**
     * Shift right for the given distance at the given power
     *
     * @param degrees distance
     */
    public void rightShiftAutonomous(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = Math.abs(power);
        degrees = Math.abs(degrees);

        initial = LF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;

        while (LF.getCurrentPosition() < target)
        {
            shiftTeleop(-power);
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    //START HERE

    public void driveForwardsAutonomousIMU(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = Math.abs(power);
        degrees = Math.abs(degrees);

        initial = LF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;
        COEFF = 1.0;

        driveTeleop(power);
        while (LF.getCurrentPosition() < degrees)
        {
            driveTeleop(power);
            COEFF = 1 - ((Math.abs(imu.getZAngle() - startAngle)/100));
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                if (imu.getZAngle() > startAngle)
                {
                    setDriveMotorPowers(power, power, COEFF * power, COEFF * power);
                }

                if (imu.getZAngle() < startAngle)
                {
                    setDriveMotorPowers(COEFF * power, COEFF * power, power, power);
                }
            }
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    public void driveBackwardsAutonomousIMU(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = -Math.abs(power);
        degrees = -Math.abs(degrees);

        initial = LF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;
        COEFF = 1.0;

        driveTeleop(power);
        while (LF.getCurrentPosition() < degrees)
        {
            driveTeleop(power);
            COEFF = 1 - ((Math.abs(imu.getZAngle() - startAngle)/100));
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                if (imu.getZAngle() > startAngle)
                {
                    setDriveMotorPowers(COEFF * power, COEFF * power, power, power);
                }

                if (imu.getZAngle() < startAngle)
                {
                    setDriveMotorPowers(power, power, COEFF * power, COEFF * power);
                }
            }
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    public void leftShiftAutonomousIMU(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = Math.abs(power);
        degrees = Math.abs(degrees);

        initial = RF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;
        COEFF = 1.0;

        shiftTeleop(power);
        while (RF.getCurrentPosition() < target)
        {
            shiftTeleop(power);
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                COEFF = 1 - ((Math.abs(imu.getZAngle() - startAngle)/100));
                if (imu.getZAngle() > startAngle)
                {
                    setDriveMotorPowers(-COEFF * power, power, power, -COEFF * power);
                }

                if (imu.getZAngle() < startAngle)
                {
                    setDriveMotorPowers(-power, COEFF * power, COEFF * power, -power);
                }
            }
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    public void rightShiftAutonomousIMU(double power, double degrees) throws InterruptedException
    {
        useEncoder(false);
        power = Math.abs(power);
        degrees = Math.abs(degrees);

        initial = LF.getCurrentPosition();
        startAngle = imu.getZAngle();
        target = initial + degrees;
        COEFF = 1.0;

        shiftTeleop(-power);
        while (LF.getCurrentPosition() < target)
        {
            shiftTeleop(-power);
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                COEFF = 1 - ((Math.abs(imu.getZAngle() - startAngle)/100));
                if (imu.getZAngle() > startAngle)
                {
                    setDriveMotorPowers(-power, COEFF * power, COEFF * power, -power);
                }

                if (imu.getZAngle() < startAngle)
                {
                    setDriveMotorPowers(-COEFF * power, power, power, -COEFF * power);
                }
            }
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }

    public void leftTurnIMU(double power, double targetAngle) throws InterruptedException
    {
        while (imu.getZAngle() < targetAngle)
        {
            leftTurnTeleop(power);
        }
        stopDriving();

        while (imu.getZAngle() > targetAngle)
        {
            rightTurnTeleop(power / 2);
        }
        stopDriving();

    }

    public void rightTurnIMU(double power, double targetAngle) throws InterruptedException
    {
        while (imu.getZAngle() > targetAngle)
        {
            rightTurnTeleop(power);
        }
        stopDriving();

        while (imu.getZAngle() < targetAngle)
        {
            leftTurnTeleop(power / 2);
        }
        stopDriving();
    }

    public void chassisTeleOp(Gamepad gamepad1, Gamepad gamepad2, double startPower) throws InterruptedException
    {
        float drivePower = -(gamepad1.left_stick_y + gamepad2.left_stick_y)*(float)0.7;
        float shiftPower = -(gamepad1.left_stick_x + gamepad2.left_stick_x)*(float)0.7;
        float leftTurnPower = (float) ((gamepad1.left_trigger) * 0.7);
        float rightTurnPower = (float) ((gamepad1.right_trigger) * 0.7);
        float spoolPower = -(gamepad2.right_stick_y);

        //Drive if joystick pushed more Y than X on gamepad1 (fast)
        if (Math.abs(drivePower) > Math.abs(shiftPower)) {
            driveTeleop(drivePower);
        }

        if (spoolPower>0.1){

            slides.moveSpool(spoolPower*(float)0.8);
        }
        if (spoolPower<0.1){

            slides.moveSpool(spoolPower);
        }

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
        if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) + Math.abs(startPower) < 0.15) {
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

    public void driveForwardsAutonomousPID(double degrees, Telemetry telemetry) throws InterruptedException
    {
        degrees = Math.abs(degrees);

        //Need to tune constants
        double KP = 0.5, KI = 0.2, KD = 0.1;
        double movementPower = MAX_POWER;

        stopResetEncoders();
        useEncoder(false);

        startAngle = imu.getZAngle();

        error = 0.0;
        oldError = 0.0;
        integral = 0.0;
        derivative = 0.0;

        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        driveTeleop(movementPower);
        while (LF.getCurrentPosition() < degrees)
        {
            telemetry.addData("error: ", error);
            telemetry.addData("oldError: ", oldError);
            telemetry.addData("currentTime: ", currentTime);
            telemetry.addData("deltaTime: ", deltaTime);
            telemetry.addData("integral: ", integral);
            telemetry.addData("derivative:", derivative);
            telemetry.update();

            error = degrees - LF.getCurrentPosition();
            deltaTime = runTime.seconds() - currentTime;
            integral = integral + (deltaTime * error);
            derivative = (error - oldError) / deltaTime;
            if (Math.abs(error) < 10)
            {
                integral = 0;
            }
            if (Math.abs(error) > 200)
            {
                integral = 0;
            }

            movementPower = KP * error + KI * integral + KD * derivative;

            driveTeleop(movementPower);

            currentTime = runTime.seconds();
            oldError = error;

            telemetry.addData("error: ", error);
            telemetry.addData("oldError: ", oldError);
            telemetry.addData("currentTime: ", currentTime);
            telemetry.addData("deltaTime: ", deltaTime);
            telemetry.addData("integral: ", integral);
            telemetry.addData("derivative:", derivative);
            telemetry.update();
        }
        stopDriving();

        leftTurnIMU(SLOW_POWER, startAngle);
        stopDriving();
        Thread.sleep(5);
    }
}

