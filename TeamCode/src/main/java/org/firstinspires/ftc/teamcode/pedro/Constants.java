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
        c.xPodOffset.set(-3.8125);
        c.yPodOffset.set(-0.125);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.24455504296386282);
                Controller secondaryTranslationalForward = Controller.proportional(0.09035655374404929);
                Controller primaryTranslationalLateral = Controller.proportional(0.4224036879614898);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15606687586735554);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018048988011404095));
                c.brake.set(Controller.proportionalFeedforward(0.01534163980969348));

                c.headingFeedback.set(Controller.proportional(2.628407551009833));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.036431776203959196, 0.013582460985761447));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0449214352152518, 0.10252926781380846));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0034572232618271547, 0.0014553233695085534));

                c.maxAchievableForwardVelocity.set(58.74226587089903);
                c.maxAchievableStrafeVelocity.set(48.227169766316095);
                c.naturalForwardDeceleration.set(32.58936072097718);
                c.naturalStrafeDeceleration.set(48.327238065268915);
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