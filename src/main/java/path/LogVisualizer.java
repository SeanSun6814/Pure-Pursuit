package path;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import Obj.ExecTimer;

/**
 * This class is a runnable GUI that displays what the robot did in
 * path-following mode by reading its log file.
 */

class LogVisualizer extends JFrame implements KeyListener {
    private static final long serialVersionUID = -1045161208869488722L;
    static LogVisualizer instance;
    Draw draw;

    // final String fileName = "C:\\Users\\Sean\\Desktop\\LogJ.csv";
    final String fileName = "C:\\Users\\Sean\\Desktop\\LogBetter1.csv";
    // final String fileName = "C:\\Users\\Sean\\Desktop\\Thursday
    // logs\\LogLeftLast.csv";
    // final String fileName = "C:\\Users\\Sean\\Desktop\\LogLeftTooFast.csv";
    // final String fileName = "C:\\Users\\Sean\\Desktop\\LogLeftLast.csv";
    // final String fileName = "C:\\Users\\Sean\\Desktop\\LogLeft1.csv";

    List<String[]> file = new ArrayList<>();

    List<List<Double>> path = new ArrayList<List<Double>>();
    List<List<Double>> data = new ArrayList<List<Double>>();

    /**
     * This is the entry point of the program. This is not the program, it only
     * initiates the program by creating an instance of it.
     */
    public static void main(String[] args) {
        instance = new LogVisualizer();
    }

    /**
     * This is the program init function (inside the constructor of this class). On
     * init, we need to 1. load the log file. 2. parse the log file into the "data"
     * arrayList. 3. print out PID table so we can copy it into excel 4. setup the
     * GUI
     */
    public LogVisualizer() {
        ExecTimer timer = new ExecTimer();
        loadFile();
        System.out.println("Done loading file in " + timer.time() + " seconds.");
        timer = new ExecTimer();
        parseData();
        printPID();
        System.out.println("Done parsing data in " + timer.time() + " seconds.");
        initWindow();

    }

    private void printPID() {
        // using right wheel velocity
        System.out.println("=============== PID setpoint, actual, error ===============");
        for (int i = 0; i < data.get(17).size(); i++) {
            System.out.println(getData(7, i) + "; " + getData(17, i) + "; " + (getData(7, i) - getData(17, i)));
        }
    }

    private void initWindow() {
        draw = new Draw(data, path);
        setSize(1000, 800);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setContentPane(draw);
        addKeyListener(this);
        setFocusable(true);
        setFocusTraversalKeysEnabled(false);
        this.setTitle("Pure Pursuit Log Visualizer");
        setVisible(true);
    }

    public void parseData() {

        data.add(new ArrayList<>()); // idx 0: robot x pos
        data.add(new ArrayList<>()); // idx 1: robot y pos
        data.add(new ArrayList<>()); // idx 2: heading
        data.add(new ArrayList<>()); // idx3: curvature
        data.add(new ArrayList<>()); // idx 4: lookadheadPoint x pos
        data.add(new ArrayList<>()); // idx 5: lookadheadPoint y pos
        data.add(new ArrayList<>()); // idx 6: left motor
        data.add(new ArrayList<>()); // idx 7: right motor
        data.add(new ArrayList<>()); // idx 8: closest point x
        data.add(new ArrayList<>()); // idx 9: closest point y
        data.add(new ArrayList<>()); // idx 10: target veloctiy at closest point
        data.add(new ArrayList<>()); // idx 11: elapsed exec time
        data.add(new ArrayList<>()); // idx 12: actual vel left
        data.add(new ArrayList<>()); // idx 13: actual vel right
        data.add(new ArrayList<>()); // idx 14: dt: the actual refresh rate
        data.add(new ArrayList<>()); // idx 15: finished path: 1 if true, 0 if false
        data.add(new ArrayList<>()); // idx 16: on path: 1 if true, 0 if false
        data.add(new ArrayList<>()); // idx 17: encoder velocity

        path.add(new ArrayList<>()); // idx 0: x pos
        path.add(new ArrayList<>()); // idx 1: y pos

        for (int i = 0; i < file.size(); i++) {
            if (file.get(i).length < 4)
                continue;

            String level = file.get(i)[0].trim().toLowerCase();
            String title = file.get(i)[1].trim().toLowerCase();
            String timestamp = file.get(i)[2].trim().toLowerCase();
            String message = file.get(i)[3].trim().toLowerCase();

            if (title.equals("robotpos")) {
                String[] msg = message.split(";");
                double x = Double.parseDouble(msg[0]);
                double y = Double.parseDouble(msg[1]);
                data.get(0).add(x);
                data.get(1).add(y);
            } else if (title.equals("lookaheadpoint")) {
                String[] msg = message.split(";");
                double x = Double.parseDouble(msg[0]);
                double y = Double.parseDouble(msg[1]);
                data.get(4).add(x);
                data.get(5).add(y);
            } else if (title.equals("motoroutput")) {
                String[] msg = message.split(";");
                double l = Double.parseDouble(msg[0]);
                double r = Double.parseDouble(msg[1]);
                data.get(6).add(l);
                data.get(7).add(r);
            } else if (title.equals("actualmotoroutput")) {
                String[] msg = message.split(";");
                double l = Double.parseDouble(msg[0]);
                double r = Double.parseDouble(msg[1]);
                data.get(12).add(l);
                data.get(13).add(r);
            } else if (title.equals("gyro")) {
                double heading = Double.parseDouble(message);
                heading = -heading + 90; // gyro comes in degrees and have up as 0 deg
                data.get(2).add(heading);
            } else if (title.equals("curvature")) {
                double curvature = Double.parseDouble(message);
                data.get(3).add(curvature);
            } else if (title.equals("path")) {
                String[] croppedMsg = message.split("\\|");
                String[] msg = croppedMsg[0].split(";");
                double x = Double.parseDouble(msg[0]);
                double y = Double.parseDouble(msg[1]);
                path.get(0).add(x);
                path.get(1).add(y);
            } else if (title.equals("closestwaypoint")) {
                String[] croppedMsg = message.split("\\|");
                String[] msg = croppedMsg[0].split(";");
                double x = Double.parseDouble(msg[0]);
                double y = Double.parseDouble(msg[1]);
                double v = Double.parseDouble(croppedMsg[1]);
                data.get(8).add(x);
                data.get(9).add(y);
                data.get(10).add(v);
            } else if (title.equals("pathfollowcalctime")) {
                double elapsedTime = Double.parseDouble(message);
                data.get(11).add(elapsedTime);
            } else if (title.equals("dt")) {
                double dt = Double.parseDouble(message);
                data.get(14).add(dt);
            } else if (title.equals("finishedpath")) {
                double bool = Double.parseDouble(message);
                data.get(15).add(bool);
            } else if (title.equals("onpath")) {
                double bool = Double.parseDouble(message);
                data.get(16).add(bool);
            } else if (title.equals("encodervelocityr")) {
                double v = Double.parseDouble(message);
                data.get(17).add(v);
            }
        }

    }

    private void loadFile() {
        try {
            BufferedReader br;
            br = new BufferedReader(new FileReader(fileName));
            String line = "";
            while ((line = br.readLine()) != null) {
                file.add(line.split(","));
            }
            br.close();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
        }
    }

    public void keyPressed(KeyEvent e) {
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            draw.update(1);
        else if (e.getKeyCode() == KeyEvent.VK_LEFT)
            draw.update(-1);
        else if (e.getKeyCode() == KeyEvent.VK_DOWN)
            draw.update(-10);
        else if (e.getKeyCode() == KeyEvent.VK_UP)
            draw.update(10);
        else if (e.getKeyCode() == KeyEvent.VK_SHIFT)
            draw.update(-3);
        else if (e.getKeyCode() == KeyEvent.VK_SPACE)
            draw.update(3);
    }

    public void keyReleased(KeyEvent e) {

    }

    public void keyTyped(KeyEvent e) {

    }

    private double getData(int type, int index) {
        // System.out.println(index + ", " + type);
        if (type >= data.size()) {
            return 0;
        } else if (data.get(type) == null) {
            return 0;
        } else if (index >= data.get(type).size()) {
            return 0;
        }
        return data.get(type).get(index);
    }
}

class Draw extends JPanel {

    private static final long serialVersionUID = -7360661585380905446L;
    List<List<Double>> path;
    List<List<Double>> data;

    public int index = 0;

    public Draw(List<List<Double>> data, List<List<Double>> path) {
        this.data = data;
        this.path = path;
    }

    public void update(int deltaIndex) { // draw a index in the path
        index += deltaIndex;
        if (index < 0)
            index = 0;
        else if (index > data.get(0).size() - 1)
            index = data.get(0).size() - 1;
        // System.out.println(deltaIndex);
        this.repaint();
    }

    @Override
    protected void paintComponent(Graphics g) {
        setBackground(Color.white);
        super.paintComponent(g);

        drawPath(g);
        drawClosestPoint(g);
        drawLookaheadPoint(g);
        drawRobot(g);
        drawStats(g);

        // idx 0: robot x pos
        // idx 1: robot y pos
        // idx 2: heading
        // idx3: curvature
        // idx 4: lookadheadPoint x pos
        // idx 5: lookadheadPoint y pos
        // idx 6: left motor
        // idx 7: right motor
        // idx 8: closest point x
        // idx 9: closest point y
        // idx 10: target veloctiy at closest point
        // idx 11: elapsed exec time
        // idx 12: actual vel left
        // idx 13: actual vel right
        // idx 14: dt: the actual refresh rate
        // idx 15: finished path: 1 if true, 0 if false
        // idx 16: on path: 1 if true, 0 if false

        // path array
        // idx 0: x pos
        // idx 1: y pos
    }

    private void drawRobot(Graphics g) {
        ((Graphics2D) g).setStroke(new BasicStroke(4));
        // robot
        g.setColor(Color.blue);
        g.drawArc(x(getData(0)) - 25, y(getData(1)) - 25, 50, 50, 0, 360);

        // direction
        g.drawLine(x(getData(0)), y(getData(1)), x(getData(0) + 1 * Math.cos(Math.toRadians(getData(2)))),
                y(getData(1) + 1 * Math.sin(Math.toRadians(getData(2)))));
    }

    private void drawLookaheadPoint(Graphics g) {
        ((Graphics2D) g).setStroke(new BasicStroke(3));
        // lookahead point
        g.setColor(Color.green);
        g.drawArc(x(getData(4)) - 5, y(getData(5)) - 5, 10, 10, 0, 360);

        // connect lookaheadpoint and the robot
        g.drawLine(x(getData(0)), y(getData(1)), x(getData(4)), y(getData(5)));
    }

    private void drawClosestPoint(Graphics g) {
        ((Graphics2D) g).setStroke(new BasicStroke(3));
        // lookahead point
        g.setColor(Color.orange);
        g.drawArc(x(getData(8)) - 5, y(getData(9)) - 5, 10, 10, 0, 360);

        // connect lookaheadpoint and the robot
        g.drawLine(x(getData(0)), y(getData(1)), x(getData(8)), y(getData(9)));
    }

    private void drawCurvature(Graphics g) {
        double curvature = getData(3);
        double radius = 1 / curvature;
        double robotX = getData(0);
        double robotY = getData(1);
        double robotHeading = getData(2);
        double radiusTranslateAngle = 90 - robotHeading;
        double centerX = robotX + radius * Math.sin(Math.toRadians(radiusTranslateAngle));
        double centerY = robotY + radius * Math.cos(Math.toRadians(radiusTranslateAngle));

        System.out.println(centerX + "," + centerY);
        ((Graphics2D) g).setStroke(new BasicStroke(3));
        // lookahead point
        g.setColor(Color.black);
        g.drawArc(x(centerX - radius), y(centerY - radius), x(radius), y(radius), 0, 360);
    }

    private void drawStats(Graphics g) {
        g.setFont(new Font("Calibri", Font.PLAIN, 20));
        g.setColor(Color.black);

        int i = 1;
        int spacing = 25;
        int xPos = 50;

        g.drawString("Index: [" + String.valueOf(index) + "]; dt: " + getString(14) + "s;   elapsed time: "
                + getString(11) + "s", xPos, spacing * i++);
        g.drawString("Position: (" + getString(0) + ", " + getString(1) + ");  Direction: "
                + String.valueOf(-(getData(2) - 90)), xPos, spacing * i++);
        g.drawString("Target Vel: " + getString(10) + ";  Curvature: " + getString(3), xPos, spacing * i++);
        g.drawString("LookAhead Point: (" + getString(4) + ", " + getString(5) + ")", xPos, spacing * i++);
        g.drawString("Closest Point: (" + getString(8) + ", " + getString(9) + ")", xPos, spacing * i++);
        g.drawString("Motors Left Right: (" + getString(6) + ", " + getString(7) + ")", xPos, spacing * i++);
        g.drawString("Actual Motors Left Right: (" + getString(12) + ", " + getString(13) + ")", xPos, spacing * i++);
        g.drawString("On Path: " + (delta(getData(16), 1) < 0.1 ? "TRUE" : "FALSE") + ";  Finished Path: "
                + (delta(getData(15), 1) < 0.1 ? "TRUE" : "FALSE"), xPos, spacing * i++);

    }

    private double getData(int type) {
        // System.out.println(index + ", " + type);
        if (type >= data.size()) {
            return 0;
        } else if (data.get(type) == null) {
            return 0;
        } else if (index >= data.get(type).size()) {
            return 0;
        }
        return data.get(type).get(index);
    }

    private String getString(int type) {
        return String.valueOf(getData(type));
    }

    private int x(double x) {
        return scale(x) + scale(5);
    }

    private int y(double y) {
        return -scale(y) + scale(10);
    }

    protected void drawPath(Graphics g) {

        ((Graphics2D) g).setStroke(new BasicStroke(4));
        g.setColor(Color.red);

        // connects the path in red
        for (int i = 0; i < path.get(0).size() - 1; i++) {
            g.drawLine(x(path.get(0).get(i)), y(path.get(1).get(i)), x(path.get(0).get(i + 1)),
                    y(path.get(1).get(i + 1)));
            // System.out.println(path.get(0).get(0));
        }
        // log out the points to debug:
        // for (int i = 0; i < path.get(0).size() - 1; i++) {
        // System.out.println(x(path.get(0).get(i)) + "," + y(path.get(1).get(i)) + ", "
        // + x(path.get(0).get(i + 1))
        // + "," + y(path.get(1).get(i + 1)));
        // // System.out.println(path.get(0).get(0));
        // }

        ((Graphics2D) g).setStroke(new BasicStroke(2));

        // draw the single waypoints in black to debug path generation.
        g.setColor(Color.black);
        for (int i = 0; i < path.get(0).size() - 1; i++) {
            g.drawArc(x(path.get(0).get(i)) - 2, y(path.get(1).get(i + 1)) - 2, 5, 5, 0, 360);
        }
    }

    public int scale(double x) {
        return (int) (x * 60);
    }

    public static double round(double value, int places) {
        if (places < 0)
            throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

    private double delta(double x, double y) {
        return Math.abs(x - y);
    }

}
