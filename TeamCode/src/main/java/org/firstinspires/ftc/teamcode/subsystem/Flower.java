package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo-driven arm with two positions.
 *
 * Robot Controller config: add a servo named "arm" on the port it is plugged into.
 * Tune DOWN_POSITION / UP_POSITION (0.0-1.0) on the real robot.
 */
public class Flower {

    public static final String SERVO_LEFT = "servo1";
    public static final String SERVO_RIGHT = "servo2";
    public static final double DOWN_POSITION = 0.5;
    public static final double UP_POSITION = 0.1;

    private final Servo servol;
    private final Servo servor;
    private boolean up;

    public Flower(HardwareMap hardwareMap) {
        servor = hardwareMap.get(Servo.class, SERVO_RIGHT);
        servol = hardwareMap.get(Servo.class, SERVO_LEFT);
    }

    public void up() {
        servor.setPosition(DOWN_POSITION);
        servol.setPosition(UP_POSITION);
        up = true;
    }

    public void down() {
        servor.setPosition(UP_POSITION);
        servol.setPosition(DOWN_POSITION);
        up = false;
    }

    public void toggle() {
        if (up) down(); else up();
    }

    public boolean isUp() {
        return up;
    }
}
