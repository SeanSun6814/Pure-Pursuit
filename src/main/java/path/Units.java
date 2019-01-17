package path;

public class Units {
    public static double in2m(double in) {
        return in / 39.37;
    }

    public static double m2in(double m) {
        return m * 39.27;
    }

    public static double ft2in(double ft) {
        return ft * 12.0;
    }

    public static double in2ft(double in) {
        return in / 12.0;
    }

    public static double ft2m(double ft) {
        return ft / 3.281;
    }

    public static double m2ft(double m) {
        return m * 3.281;
    }
}