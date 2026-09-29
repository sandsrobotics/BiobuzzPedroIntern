package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PanelsDrawing;

/**
 * Field-centric omni (mecanum) drive using the goBILDA Pinpoint for heading and position.
 *
 * Controls (gamepad1):
 *   left stick   - translate (field-centric: "up" is the direction the robot faced at start)
 *   right stick  - rotate
 *   right bumper - slow mode
 *   back         - re-zero driver heading to the robot's current facing
 *   start        - toggle field-centric / robot-centric
 *
 * Telemetry shows the Pinpoint pose (Pedro field coordinates: 0-144 in, origin bottom-left,
 * heading 0 = +x) and a small map of the field with the robot's location and facing.
 * The robot and its trail are also drawn on the Panels dashboard field (http://192.168.43.1:8001).
 */
@TeleOp(name = "Pinpoint Omni Drive", group = "TeleOp")
public class PinpointOmniDrive extends LinearOpMode {

    private static final double FIELD_SIZE = 144.0;
    private static final int MAP_COLS = 24;   // 6 in per column
    private static final int MAP_ROWS = 12;   // 12 in per row (characters are ~2x taller than wide)
    private static final double SLOW_SCALE = 0.35;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(0, 0, 0);

    @Override
    public void runOpMode() {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        PanelsDrawing.init();

        localizer.setPose(start);
        localizer.update();

        while (opModeInInit()) {
            localizer.update();
            telemetry.addLine("Ready. Place robot at start pose.");
            PanelsDrawing.draw(localizer.pose());
            addPoseTelemetry(localizer.pose());
            telemetry.update();
        }

        // Re-seed after init in case the robot was nudged while waiting.
        localizer.setPose(start);
        localizer.update();

        double driverHeadingOffset = start.heading();
        boolean fieldCentric = false;
        boolean lastBack = false, lastStart = false;

        while (opModeIsActive()) {
            localizer.update();

            Pose pose = localizer.pose();

            if (gamepad1.back && !lastBack) driverHeadingOffset = pose.heading();
            if (gamepad1.start && !lastStart) fieldCentric = !fieldCentric;
            lastBack = gamepad1.back;
            lastStart = gamepad1.start;

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;   // positive = left
            double turn = -gamepad1.right_stick_x;    // positive = counter-clockwise

            if (fieldCentric) {
                // Rotate the stick vector from the driver's frame into the robot's frame.
                double h = pose.heading() - driverHeadingOffset;
                double cos = Math.cos(h), sin = Math.sin(h);
                double robotForward = forward * cos + strafe * sin;
                double robotStrafe = -forward * sin + strafe * cos;
                forward = robotForward;
                strafe = robotStrafe;
            }

            double scale = gamepad1.right_bumper ? SLOW_SCALE : 1.0;
            drivetrain.drive(new DrivePowers(forward * scale, strafe * scale, turn * scale), true);

            telemetry.addData("Mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
            telemetry.addData("Slow", gamepad1.right_bumper);
            PanelsDrawing.draw(pose);
            addPoseTelemetry(pose);
            telemetry.update();
        }

        drivetrain.stop();
    }

    private void addPoseTelemetry(Pose pose) {
        Pose ftc = PanelsDrawing.toFtc(pose);
        telemetry.addLine(String.format("FTC: X %6.1f Y %6.1f H %6.1f deg", ftc.x(), ftc.y(), Math.toDegrees(normalize(ftc.heading()))));
//        for (String row : fieldMap(ftc)) telemetry.addLine(row);  // map grid uses Pedro 0-144
    }

    /** ASCII top-down field with the robot drawn as an arrow pointing along its heading. */
//    private String[] fieldMap(Pose pose) {
//        int col = clamp((int) (pose.x() / FIELD_SIZE * MAP_COLS), 0, MAP_COLS - 1);
//        int row = clamp((int) ((FIELD_SIZE - pose.y()) / FIELD_SIZE * MAP_ROWS), 0, MAP_ROWS - 1);
//
//        String border = "+" + repeat('-', MAP_COLS) + "+";
//        String[] lines = new String[MAP_ROWS + 2];
//        lines[0] = border;
//        for (int r = 0; r < MAP_ROWS; r++) {
//            StringBuilder sb = new StringBuilder("|");
//            for (int c = 0; c < MAP_COLS; c++) {
//                sb.append(r == row && c == col ? headingArrow(pose.heading()) : '.');
//            }
//            lines[r + 1] = sb.append('|').toString();
//        }
//        lines[MAP_ROWS + 1] = border;
//        return lines;
//    }

    private static char headingArrow(double heading) {
        final char[] arrows = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};
        int i = (int) Math.round(normalize(heading) / (Math.PI / 4));
        return arrows[((i % 8) + 8) % 8];
    }

    private static double normalize(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String repeat(char ch, int n) {
        StringBuilder sb = new StringBuilder(n);
        for (int i = 0; i < n; i++) sb.append(ch);
        return sb.toString();
    }
}
