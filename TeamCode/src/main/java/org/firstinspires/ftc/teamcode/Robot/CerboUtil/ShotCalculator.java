package org.firstinspires.ftc.teamcode.Robot.CerboUtil;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot.CerboUtil.Interpolation.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.Robot.CerboUtil.Interpolation.InterpolatingTreeMap;

public class ShotCalculator {
    private static ShotCalculator instance;

    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelRPMMap = new InterpolatingTreeMap<>();
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> timeOfFlightMap = new InterpolatingTreeMap<>();

    public static class ShootingParameters {
        public final boolean isValid;
        public final double targetFieldHeading;
        public final double flywheelRPM;
        public final double distanceToTarget;

        public ShootingParameters(boolean isValid, double targetFieldHeading, double flywheelRPM, double distanceToTarget) {
            this.isValid = isValid;
            this.targetFieldHeading = targetFieldHeading;
            this.flywheelRPM = flywheelRPM;
            this.distanceToTarget = distanceToTarget;
        }
    }

    //TUNE INTERPOLATION
    static {
        flywheelRPMMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));

        timeOfFlightMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
    }

    public ShotCalculator() {}

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public ShootingParameters calculate(Pose robotPose, Vector robotVel, Pose2d targetPos) {
        double currentDistance = Math.hypot(targetPos.getX() - robotPose.getX(), targetPos.getY() - robotPose.getY());
        double tof = timeOfFlightMap.getInterpolated(new InterpolatingDouble(currentDistance)).value;

        double virtualTargetX = targetPos.getX() - (robotVel.getXComponent() * tof);
        double virtualTargetY = targetPos.getY() - (robotVel.getYComponent() * tof);

        double lookaheadDistance = Math.hypot(virtualTargetX - robotPose.getX(), virtualTargetY - robotPose.getY());
        double fieldHeading = Math.atan2(virtualTargetY - robotPose.getY(), virtualTargetX - robotPose.getX());
        double rpm = flywheelRPMMap.getInterpolated(new InterpolatingDouble(lookaheadDistance)).value;

        boolean valid = lookaheadDistance > 10.0 && lookaheadDistance < 150.0;

        return new ShootingParameters(valid, fieldHeading, rpm, lookaheadDistance);
    }
}