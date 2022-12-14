package org.firstinspires.ftc.teamcode.common.purepursuit.path;


import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public class PurePursuitController {

    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }

    public static Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
//        System.out.println("error = " + deltaPose.toString());
        Pose powers = new Pose(
                PurePursuitConfig.xController.calculate(0, deltaPose.x),
                PurePursuitConfig.yController.calculate(0, deltaPose.y),
                PurePursuitConfig.hController.calculate(0, deltaPose.heading)
        );
        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
        double x_power = -x_rotated < -PurePursuitConfig.max_power ? -PurePursuitConfig.max_power :
                Math.min(-x_rotated, PurePursuitConfig.max_power);
        double y_power = -y_rotated < -PurePursuitConfig.max_power ? -PurePursuitConfig.max_power :
                Math.min(-y_rotated, PurePursuitConfig.max_power);
        double heading_power = powers.heading;
        System.out.println("x-power: " + x_power);
        System.out.println("y-power: " + y_power);
        System.out.println("h-power: " + heading_power + "\n");
        return new Pose(x_power, y_power, heading_power);
    }

    public static Pose goToPosition(Pose robotPose, Pose targetPose, Pose pCoefficients) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
        System.out.println("error = " + deltaPose.toString());
        Pose powers = deltaPose.divide(pCoefficients);
        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
        double x_power = -x_rotated < -PurePursuitConfig.max_power ? -PurePursuitConfig.max_power :
                Math.min(-x_rotated, PurePursuitConfig.max_power);
        double y_power = -y_rotated < -PurePursuitConfig.max_power ? -PurePursuitConfig.max_power :
                Math.min(-y_rotated, PurePursuitConfig.max_power);
        double heading_power = powers.heading;
        return new Pose(x_power, y_power, heading_power);
    }

    public static Point lineCircleIntersection(Point pointA, Point pointB, Point center, double radius) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;
        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;
        double pBy2 = bBy2 / a;
        double q = c / a;
        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return pointA;
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;
        Point p1 = new Point(pointA.x - baX * abScalingFactor1, pointA.y - baY * abScalingFactor1);
        if (disc == 0) {
            return p1;
        }
        Point p2 = new Point(pointA.x - baX * abScalingFactor2, pointA.y - baY * abScalingFactor2);
        return Math.hypot(pointB.x - p1.x, pointB.y - p1.y) > Math.hypot(pointB.x - p2.x, pointB.y - p2.y) ? p2 : p1;
    }
}
