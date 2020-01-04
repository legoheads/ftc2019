package org.firstinspires.ftc.teamcode.subsystems.slides;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class slides implements LinearSlides{

    private DcMotor spoolLeft, spoolRight;

    HardwareMap hardwareMap;

    //Add variables for distance needed to turn to go up 1 skystone distance

    public slides(HardwareMap hardwareMap, DcMotor spoolLeft, DcMotor spoolRight){

        this.hardwareMap = hardwareMap;

        this.spoolLeft = spoolLeft;
        this.spoolRight = spoolRight;

        //Reverse right spool motor, both turn in same direction
        this.spoolRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.spoolLeft = hardwareMap.dcMotor.get("spoolLeft");
        this.spoolRight = hardwareMap.dcMotor.get("spoolRight");
    }

    @Override
    public void moveSpool(double spoolPower){
        spoolLeft.setPower(spoolPower);
        spoolRight.setPower(spoolPower);
    }

}
