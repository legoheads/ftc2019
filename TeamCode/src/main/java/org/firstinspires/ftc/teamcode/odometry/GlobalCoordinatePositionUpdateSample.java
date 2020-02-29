package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;

/**
 * Created by Samedh on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //Odometry encoder wheels
    DcMotor LF, LB, RF, RB, backOdometer;


    IIMU imu;

    //The amount of encoder ticks for each inch the robot moves. This will change for each robot and needs to be changed here
    final double COUNTS_PER_INCH = 200;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "LF", verticalRightEncoderName = "RB", horizontalEncoderName = "backOdometer";

    @Override
    public void runOpMode() throws InterruptedException {

        //Assign the hardware map to the odometry wheels
        LF = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get(verticalRightEncoderName);
        backOdometer = hardwareMap.dcMotor.get(horizontalEncoderName);

        imu = new BoschIMU(hardwareMap);

        //Reset the encoders
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the LF and RB encoders spin forward, they return positive values, and when the
        backOdometer encoder travels to the right, it returns positive value
        */

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Reverse left side motors
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
         globalPositionUpdate = new OdometryGlobalCoordinatePosition(LF, RB, backOdometer, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        goToPosition(20 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.4, -90, 4 * COUNTS_PER_INCH);

        while(opModeIsActive())
        {
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Orientation (IMU)", imu.getZAngle());

            telemetry.addData("Vertical Left Encoder Position", LF.getCurrentPosition());
            telemetry.addData("Vertical Right Encoder Position", RB.getCurrentPosition());
            telemetry.addData("Horizontal Encoder Position", backOdometer.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }

    public void goToPosition(double targetYPosition, double targetXPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError)
    {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);

        while (opModeIsActive() && distance > allowableDistanceError)
        {
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double pivotCorrection = AngleWrap(robotMovementAngle - globalPositionUpdate.returnOrientation());
            double relativeTurnAngle1 = pivotCorrection + desiredRobotOrientation;

            double robotMovementTurnComponent = Range.clip(relativeTurnAngle1 / 30, -1, 1) * robotPower;

            //These are the power variables to feed into mecanum kinematics
            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            //Call mecanum kinematics 1 or 2 here
            mecanumKinematics1(robotMovementYComponent, robotMovementXComponent, robotMovementTurnComponent);
        }
        setDriveMotorPowers(0,0,0,0);


    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    public void setDriveMotorPowers(double LFPower, double LBPower, double RFPower, double RBPower)
    {
        LF.setPower(-LFPower);
        LB.setPower(-LBPower);
        RF.setPower(-RFPower * 0.95);
        RB.setPower(-RBPower * 0.95);
    }

    public void mecanumKinematics1(double yPower, double xPower, double turnPower)
    {
        double LFPower = yPower + xPower - turnPower;
        double LBPower = yPower - xPower - turnPower;
        double RFPower = yPower - xPower + turnPower;
        double RBPower = yPower + xPower + turnPower;
        setDriveMotorPowers(LFPower, LBPower, RFPower, RBPower);
    }

    public void mecanumKinematics2(double yPower, double xPower, double turnPower)
    {
        double LFPower = yPower + xPower + turnPower;
        double LBPower = yPower - xPower + turnPower;
        double RFPower = yPower - xPower - turnPower;
        double RBPower = yPower + xPower - turnPower;
        setDriveMotorPowers(LFPower, LBPower, RFPower, RBPower);
    }

    public static double AngleWrap(double angle)
    {
        while (angle < -180)
        {
            angle += 360;
        }

        while (angle > 180)
        {
            angle -= 360;
        }
        return angle;
    }

}