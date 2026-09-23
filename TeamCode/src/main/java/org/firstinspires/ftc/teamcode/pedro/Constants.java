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

//public class Constants {
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
        c.xPodOffset.set(3.37729116124431);
        c.yPodOffset.set(-0.798816230353408);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2403987917406908);
                Controller secondaryTranslationalForward = Controller.proportional(0.08882092997416527);
                Controller primaryTranslationalLateral = Controller.proportional(0.3730919828107286);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1378475184471781);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01801590557307128));
                c.brake.set(Controller.proportionalFeedforward(0.015313519737110587));

                c.headingFeedback.set(Controller.proportional(3.148287188724061));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03871286312008036, 0.014841795965194622));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07486543654085254, 0.09475344654165695));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0026688831992552854, 0.0017690243423229772));

                c.maxAchievableForwardVelocity.set(59.12008877740116);
                c.maxAchievableStrafeVelocity.set(46.99954481629859);
                c.naturalForwardDeceleration.set(37.507270081147155);
                c.naturalStrafeDeceleration.set(54.285772328841176);
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