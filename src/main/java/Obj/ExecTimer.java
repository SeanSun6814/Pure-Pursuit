package Obj;

/**
 * The purpose of this class is to get the elapsed time for some code, NICELY.
 * Instead of creating a variable and then subtracting and dividing, it is
 * simpler to just use this class which does exactly that.
 */
public class ExecTimer {
    private long startTime;

    /** This constructor creates and starts the timer. */
    public ExecTimer() {
        startTime = System.currentTimeMillis();
    }

    /**
     * Get the elapsed time since the timer was created in seconds.
     * 
     * @return elapsed time in seconds
     */
    public double time() {
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }
}