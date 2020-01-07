package org.firstinspires.ftc.teamcode.subsystems.slides;

public interface LinearSlides {

    void moveSpool(double spoolPower) throws InterruptedException;
    void stop() throws InterruptedException;
    void spoolEncoder() throws InterruptedException;
}
