package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PanelsDrawing;
import org.firstinspires.ftc.teamcode.subsystem.Arm;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    private Follower follower;
    private Arm arm;

    /************************************************************************
    / Coordinates only paste here ******************************************/
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.2086, 8, 90);
    private final Pose path1 = poseFactory.of(28.6405, 48.893, 175.5848);
    private final Pose path1Control1 = poseFactory.of(52.5876, 47.2585, 0);
    private final Pose point2 = poseFactory.of(46.5407, 107.336, 0);
    private final Pose point2Control1 = poseFactory.of(1.9433, 139.5845, 0);
    private final Pose point3 = poseFactory.of(71.0157, 98.0818, 0);
    private final Pose point3Control1 = poseFactory.of(70.2735, 134.2716, 0);
    private final Pose point3Segment1Start = poseFactory.of(71.0157, 98.0818, 0);
    private final Pose point3Segment1End = poseFactory.of(71.0157, 98.0818, 0);
    private final Pose point4 = poseFactory.of(71.2296, 12.6397, -89.8566);
    private final Pose point5 = poseFactory.of(101.1971, 20.2673, 151.4428);
    private final Pose point5Control1 = poseFactory.of(80.8044, 31.4869, 0);
    private final Pose point6 = poseFactory.of(119.5331, 92.9146, -104.1655);
    private final Pose point7 = poseFactory.of(99.3576, 122.8662, -56.0357);

    public Path path1() {
        return curve(start, path1Control1, path1).tangent();
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).constant(point2);
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(1, Interpolator.linear(point3Segment1Start, point3Segment1End)));
    }

    public Path path4() {
        return line(point3, point4).tangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path path6() {
        return line(point5, point6).reverseTangent();
    }

    public Path path7() {
        return line(point6, point7).reverseTangent();
    }

    /************************************************************************/

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                raiseArm(),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    // Arm commands
    public Command raiseArm() {
        return instant(() -> arm.up());
    }

    public Command lowerArm() {
        return instant(() -> arm.down());
    }

    // Raise, give the servo time to move, then lower again
    public Command armCycle() {
        return sequential(
                raiseArm(),
                waitMs(800),
                lowerArm(),
                waitMs(500)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.down();                 // known start position
        follower.setPose(start);
        follower.update();
        PanelsDrawing.init();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose ftc = PanelsDrawing.toFtc(follower.pose());
            telemetry.addData("x", ftc.x());
            telemetry.addData("y", ftc.y());
            telemetry.addData("heading", Math.toDegrees(Math.atan2(Math.sin(ftc.heading()), Math.cos(ftc.heading()))));
            PanelsDrawing.draw(follower.pose());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
