package org.firstinspires.ftc.teamcode.subsystems.platform;

import com.qualcomm.robotcore.hardware.Servo;

public interface Platform {
    void init();

    void up() throws InterruptedException;
    void grab() throws InterruptedException;
    void middle() throws InterruptedException;

    Servo getPlatformLeft();
    Servo getPlatformRight();
}
