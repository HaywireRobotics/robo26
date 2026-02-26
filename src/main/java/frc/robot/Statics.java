package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Statics {
    public static double applyDeadband(double value, double deadbandSize){
        if (value < deadbandSize && value > -deadbandSize){
            return 0;
        }
        return value;
    }
    public static double trueMod(double value, double mod) {
        return (((value % mod) + mod) % mod);
    }
    public static double interpolate(double start, double end, double t) {
        return start + (end - start) * t;
    }
    public static Translation2d angleToSquarePoint(double theta, double radius){
        double rad = ((theta % (Math.PI *2)) + (Math.PI *2)) % (Math.PI *2);
        double angle = (rad / (Math.PI * 2)) * 360;
        double x = 0, y = 0;

        if (angle >= 45 && angle < 135) { // Top
            x = radius * Math.tan(Math.PI/2 - rad);
            y = radius;
        } else if (angle >= 135 && angle < 225) { // Left
            x = -radius;
            y = radius * Math.tan(rad - Math.PI);
        } else if (angle >= 225 && angle < 315) { // Bottom
            x = radius * Math.tan(3*Math.PI/2 - rad);
            y = -radius;
        } else { // Right (315-360 or 0-45)
            x = radius;
            y = radius * Math.tan(rad);
        }
        return new Translation2d(x, y);
    }
    public static int angleToPointOnLEDStrip(double theta, int start_index, int end_index) {
        double rotations = trueMod(theta, Math.PI * 2) / (Math.PI * 2);
        int length = end_index - start_index;

        int index = (int)Math.floor(rotations * length);

        return start_index + index;
    }
}
