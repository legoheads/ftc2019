package org.firstinspires.ftc.teamcode.subsystems.stacker;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;

public class stacker {

    //Gripper
    private Servo gripper;
    private double GRIP_OPEN = 0.525;
    private double GRIP_GRAB = 0.45;
    private double CAP_POS = 0.7;

    //Pusher
    private Servo pusher;
    private double PUSH = 0.9;
    private double UNPUSH = 0.2;

    //Extender
    private CRServo extend;
    private double EXTEND_TIME = 3.0;
    private double MAX_POWER = 1.0;

    //Spool
    private double STOP_POWER = 0.0;
    private double SPOOL_POWER = 0.3;

    //Color Sensor V2s
    private DistanceSensor stoneDistLeft;
    private DistanceSensor stoneDistRight;
    private DistanceSensor stoneDistLow;

    private HardwareMap hardwareMap;

    private LinearSlides slides;
    private skystoneChassis chassis;



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

        gripper.setPosition(GRIP_OPEN);

        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }


    public void extend() throws InterruptedException{
        ElapsedTime extensionTimer = new ElapsedTime();

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
        slides.moveSpool(SPOOL_POWER);
        Thread.sleep(1000);
        slides.stop();

        extensionTimer.reset();
        while (extensionTimer.seconds() < EXTEND_TIME)
        {
            chassis.stopDriving();
            extend.setPower(-MAX_POWER);
            chassis.chassisTeleOp(gamepad1, gamepad2);
        }
        chassis.stopDriving();
        extend.setPower(STOP_POWER);
    }

    public void capDrop() throws InterruptedException{
        ElapsedTime extensionTimer = new ElapsedTime();

        gripper.setPosition(CAP_POS);
        drop();
        slides.moveSpool(SPOOL_POWER);
        Thread.sleep(1000);
        slides.stop();

        extensionTimer.reset();
        while (extensionTimer.seconds() < EXTEND_TIME){
            chassis.stopDriving();
            extend.setPower(-MAX_POWER);
            chassis.chassisTeleOp(gamepad1, gamepad2);
        }
        chassis.stopDriving ();
        extend.setPower(STOP_POWER);
    }

    public void stoneShiftLeft() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (!(stoneDistRight.getDistance(DistanceUnit.INCH)<5) || runTime.time()>4){
            chassis.shiftTeleop(0.3);
        }
        chassis.stopDriving();
    }

    public void stoneShiftRight() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (!(stoneDistLeft.getDistance(DistanceUnit.INCH)<5) || runTime.time()>4){
            chassis.shiftTeleop(-0.3);
        }
        chassis.stopDriving();
    }

    public void stoneReverse() throws InterruptedException{

        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (!(stoneDistLow.getDistance(DistanceUnit.INCH)<2.5)){
            chassis.driveTeleop(-0.2);
        }
        chassis.stopDriving();
    }
}
