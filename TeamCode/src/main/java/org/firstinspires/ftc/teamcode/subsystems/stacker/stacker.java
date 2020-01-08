package org.firstinspires.ftc.teamcode.subsystems.stacker;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;
import org.firstinspires.ftc.teamcode.subsystems.chassis.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.chassis.skystoneChassis;

public class stacker extends LinearOpMode {

    private Servo pusher;
    private Servo gripper;
    private CRServo extend;


    private double GRIP_OPEN = 0.525;
    private double GRIP_GRAB = 0.45;

    private double PUSH = 0.9;
    private double UNPUSH = 0.2;

    private double CAP_POS = 0.7;

    private double EXTEND_TIME = 3.0;

    private double MAX_POWER = 1.0;
    private double STOP_POWER = 0.0;
    private double SPOOL_POWER = 0.3;

    private HardwareMap hardwareMap;

    private LinearSlides slides;

    private skystoneChassis chassis;

    private ElapsedTime extensionTimer = new ElapsedTime();


    public stacker(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        pusher = hardwareMap.servo.get("pusher");
        gripper = hardwareMap.servo.get("gripper");
        extend = hardwareMap.crservo.get("extend");

        gripper.setPosition(GRIP_OPEN);

        slides = new slides(hardwareMap);
        chassis = new skystoneChassis(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void extend() throws InterruptedException{
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
//        extend.setPower(1.0);
//        Thread.sleep(EXTEND_TIME);
//        extend.setPower(0.0);
    }

    public void drop() throws InterruptedException{
        gripper.setPosition(GRIP_OPEN);
    }

    public void retract() throws InterruptedException
    {
        drop();
        slides.moveSpool(SPOOL_POWER);
        Thread.sleep(1000);
        slides.stop();

        extensionTimer.reset();
        while (extensionTimer.seconds() < EXTEND_TIME)
        {
            extend.setPower(-MAX_POWER);
            chassis.chassisTeleOp(gamepad1, gamepad2);
        }
        chassis.stopDriving();
        extend.setPower(STOP_POWER);
//        Thread.sleep(EXTEND_TIME);
//        long time = System.nanoTime();
//        if (System.nanoTime() - time > 1000)
    }

    public void capDrop() throws InterruptedException{
        gripper.setPosition(CAP_POS);
    }

    public void hello(){
        for (long stop=System.nanoTime()+ 10000; stop>System.nanoTime();) {
            /*
             * Hammer the JVM with junk
             */
        }
    }



    public void sleeper(Servo servo, double pos, long runtime) {
        long setTime = System.nanoTime();
        boolean hasRun = false;
        if(System.nanoTime() - setTime > runtime && !hasRun) {
            //Will only run after 10 seconds, and will only run once
            hasRun = true;
            servo.setPosition(pos);
        }
    }

    public void sleeper(CRServo servo, double power, long runtime){
        long setTime = System.nanoTime();
        boolean hasRun = false;
        if(System.nanoTime() - setTime > runtime && !hasRun) {
            //Will only run after 10 seconds, and will only run once
            hasRun = true;
            servo.setPower(power);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
