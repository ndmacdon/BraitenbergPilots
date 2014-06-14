import java.awt.*;

public class FlotsamApp {
  public static void main(String[] args) {
    Asteroids applet = new Asteroids(); // Applet we will run...
    Frame f = new Frame("Flotsam"); // Title of our Frame

    // Initialize our Applet:
    applet.init();

    // Add the Applet to this frame:
    f.setSize(600,600);
    f.add(applet, BorderLayout.CENTER);
    f.setVisible(true); // usual step to make frame visible
    applet.start();

  }
}