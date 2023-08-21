package com.vlad.pathplanner;

import com.vlad.pathplanner.core.Vector2D;
import com.vlad.pathplanner.core.KinematicState;
import com.vlad.pathplanner.core.MotionProfile;
import com.vlad.pathplanner.core.Path;
import com.vlad.pathplanner.core.RDP;

import javafx.fxml.FXML;
import javafx.scene.canvas.Canvas;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.XYChart;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.Slider;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.text.Text;
import javafx.scene.control.TextArea;

import java.util.ArrayList;

public class Controller {

    public static ArrayList<Vector2D> drawnCurve = new ArrayList<>();
    public static ArrayList<Vector2D> RDPCurve = new ArrayList<>();
    public static Path path = null;
    public static boolean curveIsAvailable = false;

    @FXML
    private Canvas canvas;

    @FXML
    private CheckBox original;
    @FXML
    private CheckBox simplified;
    @FXML
    private CheckBox spline;

    @FXML
    private CheckBox showField;

    @FXML
    private Slider tangent_slider;

    @FXML
    private Text tangent_text;

    @FXML
    private LineChart<Number, Number> chart;

    @FXML
    ChoiceBox<String> profileChoiceBox;

    private boolean fieldIsShown = true;
    @FXML
    private ImageView fieldImage;

    @FXML
    public TextArea console;

    public CanvasRenderer canvasRenderer;

    public void changeCheckBoxes() {
        CanvasRenderer.drawOriginalCurve = original.isSelected();
        CanvasRenderer.drawSimplifiedCurve = simplified.isSelected();
        CanvasRenderer.drawSpline = spline.isSelected();
    }

    public void reset() {
        curveIsAvailable = false;
        path = null;
        drawnCurve.clear();
        RDPCurve.clear();
        CanvasRenderer.drawOriginalCurve = true;
        CanvasRenderer.drawSimplifiedCurve = true;
        original.setSelected(true);
        simplified.setSelected(true);
        chart.getData().clear();
        Console.getInstance().clearConsole();
    }

    public void simplify() {
        if (drawnCurve.size() <= 1)
            return;
        RDPCurve.clear();

        // Create the RDP Curve
        int total = drawnCurve.size();
        Vector2D start = drawnCurve.get(0);
        Vector2D end = drawnCurve.get(total - 1);
        RDPCurve.add(start);
        RDP.generateSimplifiedCurve(0, total - 1, drawnCurve, RDPCurve);
        RDPCurve.add(end);
        curveIsAvailable = true;
    }

    public void spline() {
        if (!curveIsAvailable)
            return;

        // Create a new path based on the simplified curve
        path = new Path(RDPCurve);

        // Also disable the other curves
        CanvasRenderer.drawSimplifiedCurve = false;
        simplified.setSelected(false);
        CanvasRenderer.drawOriginalCurve = false;
        original.setSelected(false);
    }

    public void toggleField() {
        showField.setSelected(!fieldIsShown);
        fieldIsShown = !fieldIsShown;
        fieldImage.setVisible(fieldIsShown);
    }

    public void generateProfile() {
        if (path == null)
            return;

        int numPoints = 200;
        MotionProfile profile = new MotionProfile(path, 30, 40, 3.14, 3.14);
        profile.makeProfile(0, 0, 0, 0, numPoints);

        chart.getData().clear();
        chart.setLegendVisible(true);
        chart.setLegendSide(javafx.geometry.Side.TOP);
        XYChart.Series<Number, Number> series1 = new XYChart.Series<>();
        series1.setName("\u1E8B");
        XYChart.Series<Number, Number> series2 = new XYChart.Series<>();
        series2.setName("\u1E8F");
        for (int i = 0; i < numPoints; i++) {
            KinematicState stateX = profile.timeProfileX.get(i);
            KinematicState stateY = profile.timeProfileY.get(i);
            Vector2D displState = profile.displacement_profile.get(i);
            series1.getData().add(new XYChart.Data<>(stateX.t, stateX.v));
            series2.getData().add(new XYChart.Data<>(stateY.t, stateY.v));
            //series1.getData().add(new XYChart.Data<>(displState.x, displState.y));
        }
        chart.getData().add(series1);
        chart.getData().add(series2);
    }

    public void addProfilesToChart(ArrayList<Vector2D> position, ArrayList<Vector2D> velocity,
            ArrayList<Vector2D> acceleration) {
        chart.getData().clear();
        if (position != null) {
            XYChart.Series<Number, Number> series = new XYChart.Series<>();

            for (Vector2D p : position)
                series.getData().add(new XYChart.Data<>(p.x, p.y));

            chart.getData().add(series);
        }
        if (velocity != null) {
            XYChart.Series<Number, Number> series = new XYChart.Series<>();

            for (Vector2D v : velocity)
                series.getData().add(new XYChart.Data<>(v.x, v.y));

            chart.getData().add(series);
        }

        if (acceleration != null) {
            XYChart.Series<Number, Number> series = new XYChart.Series<>();

            for (Vector2D a : acceleration)
                series.getData().add(new XYChart.Data<>(a.x, a.y));

            chart.getData().add(series);
        }
    }

    @FXML
    void initialize() {
        fieldImage.setVisible(fieldIsShown);
        showField.setSelected(fieldIsShown);

        profileChoiceBox.getItems().add("Path Curvature");
        profileChoiceBox.getItems().add("Displacement Profile");
        profileChoiceBox.getItems().add("Position Profile - WIP");
        profileChoiceBox.getItems().add("Velocity Profile");
        profileChoiceBox.getItems().add("Acceleration Profile");
        profileChoiceBox.setValue("Velocity Profile");

        drawEvents(canvas);

        tangent_text.setText("0.5");
        tangent_slider.adjustValue(0.5);

        tangent_slider.valueProperty().addListener((observableValue, number, t1) -> {
            Path.TANGENT_COEFFICIENT = tangent_slider.getValue();
            tangent_text.setText(String.format("%.2f", tangent_slider.getValue()));
        });

        original.setSelected(true);
        simplified.setSelected(true);
        spline.setSelected(true);

        chart.setAnimated(false);
        chart.setCreateSymbols(false);
        chart.setAxisSortingPolicy(LineChart.SortingPolicy.NONE);

        Console.getInstance().setController(this);

        Console.getInstance().addLine("> Path planning Demo GUI by Vlad Chira.\n");
        Console.getInstance().addLine("> Usage: Draw a path using the mouse, then press RDP followed by Spline.\n");
        Console.getInstance().addLine("> Usage: Press the Generate Profile button to create a motion profile based on the path.\n");

        canvasRenderer = new CanvasRenderer(canvas);
        canvasRenderer.start();
    }

    private void drawEvents(Canvas canvas) {
        canvas.addEventHandler(MouseEvent.MOUSE_PRESSED,
                event -> {
                    if (event.getX() <= 700 && CanvasRenderer.drawOriginalCurve) {
                        drawnCurve.add(new Vector2D(FieldUtils.canvasUnitsToCM(event.getX()),
                                FieldUtils.canvasUnitsToCM(event.getY())));
                        drawnCurve.add(new Vector2D(FieldUtils.canvasUnitsToCM(event.getX() + 0.01), FieldUtils.canvasUnitsToCM(event.getY() + 0.01))); // terrible hack
                    }
                });

        canvas.addEventHandler(MouseEvent.MOUSE_DRAGGED,
                event -> {
                    if (event.getX() <= 700 && CanvasRenderer.drawOriginalCurve) {
                        drawnCurve.add(new Vector2D(FieldUtils.canvasUnitsToCM(event.getX()),
                                FieldUtils.canvasUnitsToCM(event.getY())));
                    }
                });
    }
}