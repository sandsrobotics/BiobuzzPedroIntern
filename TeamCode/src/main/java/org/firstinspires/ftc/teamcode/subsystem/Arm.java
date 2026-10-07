package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo-driven arm with two positions.
 *
 * Robot Controller config: add a servo named "arm" on the port it is plugged into.
 * Tune DOWN_POSITION / UP_POSITION (0.0-1.0) on the real robot.
 */
public class Arm {

    public static final String SERVO_NAME = "servo0";
    public static final double DOWN_POSITION = 0.5;
    public static final double UP_POSITION = 0.1;

    private final Servo servo;
    private boolean up;

    public Arm(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
    }

    public void up() {
        servo.setPosition(UP_POSITION);
        up = true;
    }

    public void down() {
        servo.setPosition(DOWN_POSITION);
        up = false;
    }

    public void toggle() {
        if (up) down(); else up();
    }

    public boolean isUp() {
        return up;
    }
}
