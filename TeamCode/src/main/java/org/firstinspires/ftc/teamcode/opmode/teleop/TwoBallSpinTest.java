package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ZZ.ZZ_ServoSSR;

/**
 * Bench test: turn one motor on/off and set its target RPM from the Driver Station gamepad.
 *
 * Controls (gamepad1):
 *   A           - toggle motor on / off
 *   dpad up/down    - target RPM +/- 100
 *   dpad right/left - target RPM +/- 10
 *   B           - reverse direction
 *   back        - stop and reset target to 0
 *
 * Color sensor: REV Color Sensor V3 on I2C bus 1, configured as "color". Servo 0 is
 * configured as "servo0". Yellow (pollen) -> POLLEN_POS; blue or red (nectar) -> NECTAR_POS;
 * any other reading leaves the servo where it was.
 *
 * Edit MOTOR_NAME and TICKS_PER_REV below to match the motor under test.
 */
@TeleOp(name = "Motor RPM Test", group = "Test")
public class TwoBallSpinTest extends LinearOpMode {

    static final String MOTOR_NAME = "motor0";
    // Encoder ticks per output-shaft revolution (28 = bare goBILDA 6000 RPM motor)
    static final double TICKS_PER_REV = 28.0;
    // Approximate no-load max RPM; the target is clamped to this
    static final double MAX_RPM = 6000;

    // Color sensor (I2C bus 1) and servo (servo port 0) configuration names
    static final String COLOR_NAME = "color";
    static final String SERVO_NAME = "servo0";

    // Time (ms) for the servo to travel its full range, used by ZZ_ServoSSR to estimate move completion
    static final int SERVO_SWEEP_MS = 1000;  // TBD

    // TBD servo positions (0..1) - tune on the robot
    static final double POLLEN_POS = 0.327;
    static final double NECTAR_POS = 0.126;  // TBD

    // Color classification (hue in degrees, 0-360). Tune on the robot.
    static final float MIN_SATURATION = 0.4f;  // below this the reading is treated as "no color"
    static final float MIN_VALUE = 0.05f;      // below this (too dark / nothing in view) = none
    static final float YELLOW_MIN = 61, YELLOW_MAX = 90;
    static final float BLUE_MIN = 180, BLUE_MAX = 260;
    static final float RED_MAX = 60, RED_MIN = 40;  // red wraps around 0

    static final int COLOR_LOOP_INTERVAL = 10;  // read the color sensor every Nth loop

    enum Seen { POLLEN, NECTAR, NONE }

    static final int LARGE_STEP = 100;
    static final int SMALL_STEP = 10;

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorEx.Direction.FORWARD);

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, COLOR_NAME);
        colorSensor.setGain(2);
        ZZ_ServoSSR servo = new ZZ_ServoSSR(hardwareMap.get(Servo.class, SERVO_NAME))
                .setSweepTime(SERVO_SWEEP_MS);
        String servoState = "unset";
        Seen seen = Seen.NONE;
        float[] hsv = new float[3];
        int loopCount = 0;

        boolean on = false;
        boolean reversed = false;
        int targetRpm = 0;

        // previous button states for edge detection
        boolean pA = false, pB = false, pUp = false, pDown = false, pLeft = false, pRight = false;

        telemetry.addLine("Motor RPM Test: press START");
        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a, b = gamepad1.b;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;

            if (a && !pA) on = !on;
            if (b && !pB) {
                reversed = !reversed;
                motor.setDirection(reversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
            }
            if (up && !pUp) targetRpm += LARGE_STEP;
            if (down && !pDown) targetRpm -= LARGE_STEP;
            if (right && !pRight) targetRpm += SMALL_STEP;
            if (left && !pLeft) targetRpm -= SMALL_STEP;
            if (gamepad1.back) {
                on = false;
                targetRpm = 0;
            }
            pA = a; pB = b; pUp = up; pDown = down; pLeft = left; pRight = right;

            targetRpm = (int) Math.max(0, Math.min(MAX_RPM, targetRpm));

            if (on) {
                motor.setVelocity(targetRpm * TICKS_PER_REV / 60.0);
            } else {
                motor.setPower(0);
            }

            // Color -> servo. I2C reads are slow, so only read every COLOR_LOOP_INTERVAL loops,
            // and only while the motor is on. If nothing recognizable is seen, leave the servo as it was.
            loopCount++;
            if (on && loopCount % COLOR_LOOP_INTERVAL == 0) {
                NormalizedRGBA rgba = colorSensor.getNormalizedColors();
                Color.colorToHSV(rgba.toColor(), hsv);
                seen = classify(hsv,seen);
                if (seen == Seen.POLLEN) {
                    servo.setPosition(POLLEN_POS);
                    servoState = "POLLEN";
                } else if (seen == Seen.NECTAR) {
                    servo.setPosition(NECTAR_POS);
                    servoState = "NECTAR";
                }
            }

            telemetry.addLine("=== Motor RPM Test ===");
            telemetry.addLine("A on/off | dpad U/D +-100, R/L +-10 | B reverse | back stop");
            telemetry.addData("Motor", MOTOR_NAME);
            telemetry.addData("State", on ? "ON" : "OFF");
            telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Actual RPM", "%.0f", Math.abs(motor.getVelocity()) * 60.0 / TICKS_PER_REV);
            telemetry.addData("Ticks/rev", TICKS_PER_REV);
            telemetry.addData("Color seen", seen);
            telemetry.addData("HSV", "%.0f / %.2f / %.2f", hsv[0], hsv[1], hsv[2]);
            telemetry.addData("Servo set to", servoState + (servo.isDone() ? " (done)" : " (moving)"));
            telemetry.update();
        }

        motor.setPower(0);
    }

    static Seen classify(float[] hsv, Seen seen) {
        float h = hsv[0], sat = hsv[1], val = hsv[2];
        if (sat > MIN_SATURATION) {
            if (h >= YELLOW_MIN && h <= YELLOW_MAX) return Seen.POLLEN;
            if (h >= BLUE_MIN && h <= BLUE_MAX) return Seen.NECTAR;
            if (h <= RED_MAX && h >= RED_MIN) return Seen.NECTAR;
        }
        return seen;
    }
}
