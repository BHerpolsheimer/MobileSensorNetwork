// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Class to hold information regarding a round obstacle that will be avoided
// by the sensors.

import java.io.*;

public class RoundObstacle {
  private double xPosition;
  private double yPosition;
  private double radius;
 
  // Parameterized RoundObstacle constructor.
  public RoundObstacle(double x, double y, double r) {
    setXPosition(x);
    setYPosition(y);
    setRadius(r);
  }

  // Methods to modify the private class values.
  public void setXPosition(double x) {
    xPosition = x;
  }
  public void setYPosition(double y) {
    yPosition = y;
  }
  public void setRadius(double r) {
  	radius = r;
  }

  // Methods to return the private class values.
  public double getXPosition() {
    return xPosition;
  }
  public double getYPosition() {
    return yPosition;
  }
  public double getRadius() {
  	return radius;
  }
}