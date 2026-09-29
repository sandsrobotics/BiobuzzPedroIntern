package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

import java.util.ArrayDeque;

/**
 * Draws the robot and a trail of where it has been on the Panels dashboard field
 * (http://192.168.43.1:8001 while connected to the robot's Wi-Fi).
 *
 * Call {@link #init()} once, then {@link #draw(Pose)} every loop.
 */
public class PanelsDrawing {
    public static final double ROBOT_RADIUS = 9;
    private static final int TRAIL_LENGTH = 300;
    private static final double TRAIL_SPACING = 1.0; // inches between trail points

    private static final FieldManager field = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
    private static final Style trailLook = new Style("", "#4CAF50", 0.75);

    private static final ArrayDeque<Pose> trail = new ArrayDeque<>();

    /** Panels uses FTC field coordinates: (0, 0) at field center, -72 to +72 in. */
    public static void init() {
        field.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());
        field.setBackground(PanelsField.INSTANCE.getImages().getBIOBUZZ().getDARK());
        trail.clear();
    }

    public static void draw(Pose pose) {
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return;
        }
        pose = toFtc(pose);
        recordTrail(pose);
        drawTrail();
        drawRobot(pose);
        field.update();
    }

    /**
     * Converts a Pedro pose (0-144 in, origin bottom-left) to FTC field coordinates
     * (origin at field center). Use for display only; the follower and paths stay in Pedro.
     */
    public static Pose toFtc(Pose p) {
//        return new Pose(72 - p.y() - 72, p.x() - 72, p.heading()+ Math.PI / 2.0);
        return p;
    }

    private static void recordTrail(Pose pose) {
        Pose last = trail.peekLast();
        if (last != null && last.distance(pose) < TRAIL_SPACING) return;
        trail.addLast(pose);
        if (trail.size() > TRAIL_LENGTH) trail.removeFirst();
    }

    private static void drawTrail() {
        field.setStyle(trailLook);
        Pose prev = null;
        for (Pose p : trail) {
            if (prev != null) {
                field.moveCursor(prev.x(), prev.y());
                field.line(p.x(), p.y());
            }
            prev = p;
        }
    }

    private static void drawRobot(Pose pose) {
        field.setStyle(robotLook);
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS);

        // Heading tick from half-radius to the edge of the circle.
        double dx = Math.cos(pose.heading()) * ROBOT_RADIUS;
        double dy = Math.sin(pose.heading()) * ROBOT_RADIUS;
        field.moveCursor(pose.x() + dx / 2, pose.y() + dy / 2);
        field.line(pose.x() + dx, pose.y() + dy);
    }
}
