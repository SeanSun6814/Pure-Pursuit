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

class LogVisualizer extends JFrame implements KeyListener {
    static LogVisualizer instance;
    Draw draw;

    final String fileName = "C:\\Users\\Sean\\Desktop\\LogOverlap.csv";

    List<String[]> file = new ArrayList<>();

    List<List<Double>> path = new ArrayList<List<Double>>();
    List<List<Double>> data = new ArrayList<List<Double>>();

    public static void main(String[] args) {
        instance = new LogVisualizer();
    }

    public LogVisualizer() {
        loadFile();
        System.out.println("Done loading file.");
        parseData();
        System.out.println("Done parsing data.");

        initWindow();

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
        setVisible(true);
    }

    public void parseData() {

        data.add(new ArrayList<>()); // idx 0: robot x pos
        data.add(new ArrayList<>());// idx 1: robot y pos
        data.add(new ArrayList<>());// idx 2: heading
        data.add(new ArrayList<>()); // idx3: curvature
        data.add(new ArrayList<>()); // idx 4: lookadheadPoint x pos
        data.add(new ArrayList<>());// idx 5: lookadheadPoint y pos
        data.add(new ArrayList<>());// idx 6: left motor
        data.add(new ArrayList<>()); // idx 7: right motor
        data.add(new ArrayList<>()); // idx 8: closest point x
        data.add(new ArrayList<>()); // idx 9: closest point y
        data.add(new ArrayList<>()); // idx 10: target veloctiy at closest point

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
    }

    public void keyReleased(KeyEvent e) {

    }

    public void keyTyped(KeyEvent e) {

    }
}

class Draw extends JPanel {

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
        System.out.println(deltaIndex);
        this.repaint();
    }

    @Override
    protected void paintComponent(Graphics g) {

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

        // path
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

        g.drawString("Index: " + String.valueOf(index), xPos, spacing * i++);
        g.drawString("Position: (" + getString(0) + ", " + getString(1) + ")", xPos, spacing * i++);
        g.drawString("Direction: " + String.valueOf(-(getData(2) - 90)), xPos, spacing * i++);
        g.drawString("Curvature: " + getString(3), xPos, spacing * i++);
        g.drawString("Target Vel: " + getString(10), xPos, spacing * i++);
        g.drawString("LookAhead Point: (" + getString(4) + ", " + getString(5) + ")", xPos, spacing * i++);
        g.drawString("Closest Point: (" + getString(8) + ", " + getString(9) + ")", xPos, spacing * i++);
        g.drawString("Motors Left Right: (" + getString(6) + ", " + getString(7) + ")", xPos, spacing * i++);
    }

    private double getData(int type) {
        System.out.println(index + ", " + type);
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

}
