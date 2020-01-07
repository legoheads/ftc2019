package org.firstinspires.ftc.teamcode.subsystems.stoner;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.slides.LinearSlides;
import org.firstinspires.ftc.teamcode.subsystems.slides.slides;



import java.sql.Time;
import java.util.concurrent.TimeUnit;

public class stoner {

    private Servo pusher;
    private Servo gripper;
    private CRServo extend;


    private double GRIP_OPEN = 0.525;
    private double GRIP_GRAB = 0.45;

    private double PUSH = 0.9;
    private double UNPUSH = 0.2;

    private double CAP_POS = 0.7;

    private long EXTEND_TIME = 3000;

    private HardwareMap hardwareMap;

    private LinearSlides slides;

    public stoner(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        pusher = hardwareMap.servo.get("pusher");
        gripper = hardwareMap.servo.get("gripper");
        extend = hardwareMap.crservo.get("extend");

        gripper.setPosition(0.525);

        slides = new slides(hardwareMap);
    }


    public void extend() throws InterruptedException{
        pusher.setPosition(PUSH);
        Thread.sleep(700);
        gripper.setPosition(GRIP_GRAB);
        extend.setPower(1.0);
        Thread.sleep(EXTEND_TIME);
        extend.setPower(0.0);
        pusher.setPosition(UNPUSH);
    }

    public void drop() throws InterruptedException{
        gripper.setPosition(GRIP_OPEN);
    }

    public void retract() throws InterruptedException{
        drop();
        slides.moveSpool(0.3);
        Thread.sleep(1000);

        slides.stop();
        extend.setPower(-1.0);
        Thread.sleep(EXTEND_TIME);

        long time = System.nanoTime();

        if (System.nanoTime() - time > 1000)

        extend.setPower(0.0);
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
}
