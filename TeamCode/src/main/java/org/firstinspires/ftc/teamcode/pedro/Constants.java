package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
//    public static Follower create(HardwareMap h) {
//        // return new Follower(Drivetrain, Localizer, Foresight);
//        return null;
//    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motor0");
        c.frontRightName.set("motor1");
        c.backLeftName.set("motor2");
        c.backRightName.set("motor3");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(-3.75);
        c.yPodOffset.set(-0.125);
//        c.xPodOffset.set(3.7617219339205525);
//        c.yPodOffset.set(-1.4632172471894993);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22162309875747868);
                Controller secondaryTranslationalForward = Controller.proportional(0.08188381311262477);
                Controller primaryTranslationalLateral = Controller.proportional(0.33187638469311553);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12261945624378474);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017441355560333823));
                c.brake.set(Controller.proportionalFeedforward(0.01482515222628375));

                c.headingFeedback.set(Controller.proportional(3.252297634845086));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03282845604883762, 0.012264859536452595));

                c.linearBrakeCoefficients.set(Matrix.diag(0.060043707791918234, 0.06014303558668025));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025356503410465417, 0.0022424458654683103));

                c.maxAchievableForwardVelocity.set(59.03330876182463);
                c.maxAchievableStrafeVelocity.set(47.32356633594674);
                c.naturalForwardDeceleration.set(26.935090768417297);
                c.naturalStrafeDeceleration.set(51.02197743223549);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}