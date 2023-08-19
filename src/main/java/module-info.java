module com.vlad.pathplanner {
    requires javafx.controls;
    requires javafx.fxml;
    requires commons.math3;
    requires org.apache.commons.lang3;

    opens com.vlad.pathplanner to javafx.fxml;
    exports com.vlad.pathplanner;
}
