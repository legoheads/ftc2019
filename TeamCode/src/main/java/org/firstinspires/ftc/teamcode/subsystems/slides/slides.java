package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class slides implements LinearSlides{

    private DcMotor spoolLeft, spoolRight;

    HardwareMap hardwareMap;

    //Add variables for distance needed to turn to go up 1 skystone distance

    public slides(HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;

        spoolLeft = hardwareMap.dcMotor.get("spoolLeft");
        spoolRight = hardwareMap.dcMotor.get("spoolRight");

        //Reverse right spool motor, both turn in same direction
        spoolLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void moveSpool(double spoolPower) throws InterruptedException{
        spoolLeft.setPower(spoolPower);
        spoolRight.setPower(spoolPower);
    }

    public void stop() throws InterruptedException{
        spoolLeft.setPower(0.0);
        spoolRight.setPower(0.0);
    }

    public DcMotor getSpoolLeft(){
        return spoolLeft;
    }

    public DcMotor getSpoolRight(){
        return spoolRight;
    }

    public void spoolEncoder() throws InterruptedException{
//        spoolLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //Set up the motor to run to the given position
//        spoolLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the target position as the value entered
        spoolLeft.setTargetPosition(spoolLeft.getCurrentPosition() + 100);

        while (spoolLeft.getCurrentPosition() < spoolLeft.getTargetPosition()){
            moveSpool(0.2);
        }

        stop();

        //Use the encoder in the future
        spoolRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
