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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PanelsDrawing;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.5834, 8, 90);
    private final Pose path1 = poseFactory.of(44.8961, 19.2208, 90);
    private final Pose path1Control1 = poseFactory.of(14.1671, 45.9466, 0);
    private final Pose point2 = poseFactory.of(27.0568, 104.765, 58.2402);
    private final Pose point2Control1 = poseFactory.of(5.4405, 70.4244, 0);
    private final Pose point3 = poseFactory.of(77.5033, 116.0417, -25.4711);
    private final Pose point3Control1 = poseFactory.of(46.2649, 131.1007, 0);
    private final Pose point4 = poseFactory.of(73.4675, 64.3192, 157.3025);
    private final Pose point4Control1 = poseFactory.of(40.0013, 78.047, 0);
    private final Pose point5 = poseFactory.of(47.3199, 23.8325, -122.8557);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
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

    public Path path1() {
        return curve(start, path1Control1, path1).constant(path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).tangent();
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).tangent();
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).reverseTangent();
    }

    public Path path5() {
        return line(point4, point5).tangent();
    }
}
