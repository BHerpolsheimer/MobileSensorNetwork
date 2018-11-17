// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Class to hold information regarding sector objects that will
// allow for the display to be split up into parts.

import java.io.*;

public class Sector {
  private double x;
  private double y;
  private double height;
  private double width;

  // Parameterized constructor for Sector class.
  public Sector(double x, double y, double height, double width) {
    setX(x);
    setY(y);
    setHeight(height);
    setWidth(width);
  }

  // Methods to modify the private class values.
  public void setX(double x) {
  	this.x = x;
  }
  public void setY(double y) {
  	this.y = y;
  }
  public void setHeight(double height) {
  	this.height = height;
  }
  public void setWidth(double width) {
  	this.width = width;
  }

  // Methods to return the private class values.
  public double getX() {
  	return x;
  }
  public double getY() {
  	return y;
  }
  public double getHeight() {
  	return height;
  }
  public double getWidth() {
  	return width;
  }
}