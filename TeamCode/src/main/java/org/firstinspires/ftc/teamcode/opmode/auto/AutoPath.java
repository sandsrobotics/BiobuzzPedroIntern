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

    private final Pose start = poseFactory.of(8.5834, 8, 90);
    private final Pose path1 = poseFactory.of(36.6995, 12.5341, 0);
    private final Pose path1Control1 = poseFactory.of(14.1671, 45.9466, 0);
    private final Pose path1Segment1Start = poseFactory.of(36.6995, 12.5341, 0);
    private final Pose path1Segment1End = poseFactory.of(36.6995, 12.5341, 0);
    private final Pose point2 = poseFactory.of(73.9037, 109.0227, -22.6285);
    private final Pose point2Control1 = poseFactory.of(1.9433, 139.5845, 0);
    private final Pose point3 = poseFactory.of(124.8044, 98.8314, 0);
    private final Pose point3Control1 = poseFactory.of(128.1854, 136.5206, 0);
    private final Pose point3Segment1Start = poseFactory.of(124.8044, 98.8314, 0);
    private final Pose point3Segment1End = poseFactory.of(124.8044, 98.8314, 0);
    private final Pose point4 = poseFactory.of(123.8938, 54.2463, -91.1701);
    private final Pose point5 = poseFactory.of(93.513, 14.6448, 89.6449);
    private final Pose point5Control1 = poseFactory.of(93.5488, 44.9809, 0);

    public Path path1() {
        return curve(start, path1Control1, path1).tangent();
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).tangent();
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
