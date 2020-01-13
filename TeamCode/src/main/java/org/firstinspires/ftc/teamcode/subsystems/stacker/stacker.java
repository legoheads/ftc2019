package org.firstinspires.ftc.teamcode.subsystems.stacker;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;

public class stacker {

    //Gripper
    private Servo gripper;
    private double GRIP_OPEN = 0.525;
    private double GRIP_GRAB = 0.45;
    private double CAP_POS = 0.95;

    //Pusher
    private Servo pusher;
    private double PUSH = 0.9;
    private double UNPUSH = 0.2;

    //Extender
    private CRServo extend;
    private double EXTEND_TIME = 3.3;
    private double MAX_POWER = 1.0;

    //Spool
    private double STOP_POWER = 0.0;
    private double SPOOL_POWER = 0.3;

    //Color Sensor V2s
    private DistanceSensor stoneDistLeft;
    private DistanceSensor stoneDistRight;
    private DistanceSensor stoneDistLow;
    private DistanceSensor platformLeftSensor;
    private DistanceSensor platformRightSensor;

    private HardwareMap hardwareMap;

    private LinearSlides slides;
    private skystoneChassis chassis;
    private IIMU imu;


    Gamepad gamepad1, gamepad2;


    public stacker(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){

        this.hardwareMap = hardwareMap;

        pusher = hardwareMap.servo.get("pusher");
        gripper = hardwareMap.servo.get("gripper");
        extend = hardwareMap.crservo.get("extend");

        //Back distance sensors
        stoneDistLeft = hardwareMap.get(DistanceSensor.class, "stoneDistLeft");
        stoneDistRight = hardwareMap.get(DistanceSensor.class, "stoneDistRight");
        stoneDistLow = hardwareMap.get(DistanceSensor.class, "stoneDistLow");
        platformLeftSensor = hardwareMap.get(DistanceSensor.class, "platformLeftSensor");
        platformRightSensor = hardwareMap.get(DistanceSensor.class, "platformRightSensor");

        gripper.setPosition(GRIP_OPEN);

        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
        imu = new BoschIMU(hardwareMap);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }


    public void extend() throws InterruptedException{
        ElapsedTime extensionTimer = new ElapsedTime();

        chassis.stopDriving();

        pusher.setPosition(PUSH);
        Thread.sleep(700);
        gripper.setPosition(GRIP_GRAB);

        extensionTimer.reset();
        while (extensionTimer.seconds() < EXTEND_TIME)
        {
            extend.setPower(MAX_POWER);
            chassis.chassisTeleOp(gamepad1, gamepad2);
        }
        chassis.stopDriving();
        extend.setPower(STOP_POWER);
        pusher.setPosition(UNPUSH);
    }

    public void drop() throws InterruptedException{
        gripper.setPosition(GRIP_OPEN);
    }

    public void retract() throws InterruptedException {
        ElapsedTime extensionTimer = new ElapsedTime();

        drop();

        Thread.sleep(300);

        chassis.stopDriving();

        slides.spoolEncoder(0.8, 350);

        extensionTimer.reset();
        while (extensionTimer.seconds() < EXTEND_TIME) {
//            chassis.stopDriving();
            extend.setPower(-MAX_POWER);
            chassis.chassisTeleOp(gamepad1, gamepad2);
        }
        chassis.stopDriving();

        extend.setPower(STOP_POWER);

//        slides.spoolReturn(-0.8);

//        slides.spoolEncoder(-0.8, -350);




    }

    public void capDrop() throws InterruptedException{
//        ElapsedTime extensionTimer = new ElapsedTime();

        gripper.setPosition(CAP_POS);

//
//        slides.spoolEncoder(0.8, 370);
//        slides.stop();
//
//        extensionTimer.reset();
//        while (extensionTimer.seconds() < EXTEND_TIME){
//            chassis.stopDriving();
//            extend.setPower(-MAX_POWER);
//            chassis.chassisTeleOp(gamepad1, gamepad2);
//        }
//        chassis.stopDriving ();
//        extend.setPower(STOP_POWER);
    }

    public void stoneShiftLeft() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        while (!(stoneDistRight.getDistance(DistanceUnit.INCH)<5) && runTime.time()<4 || gamepad1.back){
            chassis.chassisTeleOp(gamepad1, gamepad2);
            chassis.shiftTeleop(0.3);
        }
        chassis.stopDriving();
    }

    public void stoneShiftRight() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        while (!(stoneDistLeft.getDistance(DistanceUnit.INCH)<5) && runTime.time()<4 || gamepad1.back){
            chassis.chassisTeleOp(gamepad1, gamepad2);
            chassis.shiftTeleop(-0.3);
        }
        chassis.stopDriving();
    }

    public void stoneReverse() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        while (!(stoneDistLow.getDistance(DistanceUnit.INCH)<2.5) && runTime.time()<4 || gamepad1.back){
            chassis.chassisTeleOp(gamepad1, gamepad2);
            chassis.driveTeleop(-0.2);
        }
        chassis.stopDriving();
    }

    public void stoneReverseAuto() throws InterruptedException
    {
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        while (!(stoneDistLow.getDistance(DistanceUnit.INCH)<2.5) && runTime.time()<3 || gamepad1.back){
            chassis.driveTeleop(-0.3);
        }
        chassis.stopDriving();
    }

    public void platformLeftShift() throws InterruptedException{
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        double startAngle = 0;
        double COEFF = 0.94;
        while (!(platformLeftSensor.getDistance(DistanceUnit.INCH)<12) && runTime.time()<6 || gamepad1.back)
        {
            chassis.shiftTeleop(0.4);
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                if (imu.getZAngle() > startAngle) {
                    chassis.setDriveMotorPowers(-COEFF * 0.5, 0.5, COEFF * 0.5, - 0.5);
                }

                if (imu.getZAngle() < startAngle) {
                    chassis.setDriveMotorPowers(-0.5, COEFF * 0.5, 0.5, -COEFF * 0.5);
                }
            }
        }
        chassis.stopDriving();
    }

    public void platformRightShift() throws InterruptedException{
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        runTime.reset();

        double startAngle = 0;
        double COEFF = 0.94;
        while (!(platformRightSensor.getDistance(DistanceUnit.INCH)<12) && runTime.time()<6 || gamepad1.back)
        {
            chassis.shiftTeleop(-0.4);
            if (Math.abs(imu.getZAngle() - startAngle) > 2.0)
            {
                if (imu.getZAngle() > startAngle) {
                    chassis.setDriveMotorPowers(COEFF * 0.5, -0.5, -COEFF * 0.5, 0.5);
                }

                if (imu.getZAngle() < startAngle) {
                    chassis.setDriveMotorPowers(0.5, -COEFF * 0.5, -0.5, COEFF * 0.5);
                }
            }
        }
        chassis.stopDriving();
    }
}