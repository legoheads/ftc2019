package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class intake implements IntakeWheels{

    private HardwareMap hardwareMap;

    private DcMotor intakeLeft, intakeRight;

    private double INTAKE_POWER = 1.0;
    private double EJECT_POWER = 0.3;

    private String intakeState = "Stop";


    public intake(HardwareMap hardwareMap, DcMotor intakeLeft, DcMotor intakeRight){
        this.hardwareMap = hardwareMap;

        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;

        this.intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        this.intakeRight = hardwareMap.dcMotor.get("intakeRight");

        //Reverse left intake motor
        this.intakeLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void intake(){
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);

        intakeState = "In";
    }

    @Override
    public void eject(){
        intakeLeft.setPower(-EJECT_POWER);
        intakeRight.setPower(-EJECT_POWER);

        intakeState = "Out";
    }

    @Override
    public String getIntakeState(){
        return intakeState;
    }
}
