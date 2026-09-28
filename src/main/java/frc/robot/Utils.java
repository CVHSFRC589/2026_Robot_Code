package frc.robot;

public class Utils {
    public static double Lerp(double start, double end, double percent) {
        return percent * (end - start) + start;
    }
}
