package org.firstinspires.ftc.teamcode.subsystems.distanceSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public interface Distance
{
    void distLeftShift(double power, double distance) throws InterruptedException;
    void distRightShift(double power, double distance) throws InterruptedException;

    void stoneShiftLeft() throws InterruptedException;
    void stoneShiftRight() throws InterruptedException;
    void platformReverse() throws InterruptedException;
}
