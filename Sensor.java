// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Class to hold and update the information pertaining to a sensor within the network.

import java.io.*;

public class Sensor {
  private double xPosition;
  private double yPosition;
  private double xVelocity;
  private double yVelocity;

  // Default constructor for Sensor object.
  public Sensor() {
  	setXPosition(-10000);
  	setYPosition(-10000);
  	setXVelocity(0);
  	setYVelocity(0);
  }

  // Parameterized constructor for Sensor object.
  public Sensor(double xP, double yP, double xV, double yV) {
    setXPosition(xP);
    setYPosition(yP);
    setXVelocity(xV);
    setYVelocity(yV);
  }

  // Moves the sensor and updates the position and valocity based on
  // the given acceleration.
  public void move(double[] acceleration, double[] gridMinMax) {
  	setXPosition(getXPosition() + getXVelocity() * Parameters.delta_t + 
  		           acceleration[0] * (Parameters.delta_t * Parameters.delta_t / 2));
  	setXVelocity(getXVelocity() + acceleration[0] * Parameters.delta_t);
  	setYPosition(getYPosition() + getYVelocity() * Parameters.delta_t + 
  		           acceleration[1] * (Parameters.delta_t * Parameters.delta_t / 2));
  	setYVelocity(getYVelocity() + acceleration[1] * Parameters.delta_t);
  	if(getXPosition() < gridMinMax[0])
  		gridMinMax[0] = getXPosition();
  	else if(getXPosition() > gridMinMax[1])
  		gridMinMax[1] = getXPosition();
  	if(getYPosition() < gridMinMax[2])
  		gridMinMax[2] = getYPosition();
  	else if(getYPosition() > gridMinMax[3])
  		gridMinMax[3] = getYPosition();
  }

  // Methods to modify the private class values.
  public void setXPosition(double x) {
    xPosition = x;
  }
  public void setYPosition(double y) {
    yPosition = y;
  }
  public void setXVelocity(double x) {
    xVelocity = x;
  }
  public void setYVelocity(double y) {
    yVelocity = y;
  }

  // Methods to return the private class values.
  public double getXPosition() {
    return xPosition;
  }
  public double getYPosition() {
    return yPosition;
  }
  public double getXVelocity() {
    return xVelocity;
  }
  public double getYVelocity() {
    return yVelocity;
  }
}