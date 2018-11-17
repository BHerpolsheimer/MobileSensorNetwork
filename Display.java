// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Functions to create the objects that will be displayed on screen.

import java.io.*;
import java.util.*;
import java.awt.Color;

public class Display {
  private int xScaleMax;
  private int xscakeMin = 0;
  private int yScaleMax;
  private int yScaleMin = 0;
  private double widthRatio = 0.8;
  private double heightRatio = 0.8;


  // Default constructor for the display object.
  public Display() {
    StdDraw.setCanvasSize(1920, 1000);
    StdDraw.setXscale(0, 1920);
    StdDraw.setYscale(0, 1000);
    xScaleMax = 1920;
    yScaleMax = 1000;
  }

  // Parameterized constructor for the display object.
  public Display(int height, int width) {
    StdDraw.setCanvasSize(width, height);
    StdDraw.setXscale(0, width);
    StdDraw.setYscale(0, height);
    xScaleMax = width;
    yScaleMax = height;
  }

  // Creates a grid using the given parameters for the sector, min max values, intervals, etc.
  public void createGrid(Sector targetSector, double xInterval, double yInterval, 
  	                double[] gridMinMax,
  	                String title, String xLabel, String yLabel) {
    double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	double originX = targetSector.getX() + 0.5 * targetSector.getWidth();
  	double originY = targetSector.getY() + 0.5 * targetSector.getHeight();

    StdDraw.setPenColor(StdDraw.BLACK);
    StdDraw.rectangle(originX, originY, widthRatio/2 * targetSector.getWidth(), 
    	                heightRatio/2 * targetSector.getHeight());

    double intervalCount = ((xMax - xMax % xInterval)  - 
                            (xMin - xMin % xInterval)) / xInterval;
    for (int i = 0; i <= intervalCount; ++i) {
    	double markX = ((widthRatio * targetSector.getWidth()) * 
    		              (1 - (xMax % xInterval - xMin % xInterval) / (xMax - xMin))) * 
    	                ((i / intervalCount) - 0.5) + originX - (widthRatio * targetSector.getWidth()) * 
    	                (xMax % xInterval + xMin % xInterval) / (2 * (xMax - xMin));

      StdDraw.line(markX, originY - (heightRatio * targetSector.getHeight()) * (.50),
       	           markX, originY - (heightRatio * targetSector.getHeight()) * (.48));
      StdDraw.line(markX, originY + (heightRatio * targetSector.getHeight()) * (.50),
       	           markX, originY + (heightRatio * targetSector.getHeight()) * (.48));
      StdDraw.text(markX, originY - (heightRatio * targetSector.getHeight()) * (.50) - 25, 
      	           Double.toString(Math.round((i * xInterval + xMin - xMin % xInterval) * 10) / 10.0), 45);
    }

    intervalCount = ((yMax - yMax % yInterval)  - 
                     (yMin - yMin % yInterval)) / yInterval;
    for (int i = 0; i <= intervalCount; ++i) {
    	double markY = ((heightRatio * targetSector.getHeight()) * 
    		              (1 - (yMax % yInterval - yMin % yInterval) / (yMax - yMin))) * 
    	                ((i / intervalCount) - 0.5) + originY - (heightRatio * targetSector.getHeight()) * 
    	                (yMax % yInterval + yMin % yInterval) / (2 * (yMax - yMin));

      StdDraw.line(originX - (widthRatio * targetSector.getWidth()) * (.50), markY, 
       	           originX - (widthRatio * targetSector.getWidth()) * (.48), markY);
      StdDraw.line(originX + (widthRatio * targetSector.getWidth()) * (.50), markY,
       	           originX + (widthRatio * targetSector.getWidth()) * (.48), markY);
      StdDraw.text(originX - (widthRatio * targetSector.getWidth()) * (.50) - 30, markY, 
      	           Double.toString(Math.round((i * yInterval + yMin - yMin % yInterval) * 10) / 10.0));
    }
  }

  // Creates the given sensor network within the given sector.
  public void createSensorNetwork(Sector targetSector, Sensor[] sensorNetwork,
  	                              double[] gridMinMax) {
  	StdDraw.setPenColor(StdDraw.MAGENTA);
  	for (int i = 0; i < sensorNetwork.length; i++)
      createSensor(targetSector, sensorNetwork[i], gridMinMax);
  }

  // Creates the given beta network within the given sector.
  public void createBetaNetwork(Sector targetSector, Sensor[][] betaNetwork,
  	                              double[] gridMinMax) {
  	StdDraw.setPenColor(StdDraw.BLUE);
  	for(int i = 0; i < betaNetwork.length; i++)
  		for(int j = 0; j < betaNetwork[i].length; j++)
        createSensor(targetSector, betaNetwork[i][j], gridMinMax);
  }

  // Creates the specific sensor given within the given sector.
  public void createSensor(Sector targetSector, Sensor s1, double[] gridMinMax) {
  	double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	double velocityMagnitude = Math.sqrt(s1.getXVelocity() * s1.getXVelocity() + 
  		                                   s1.getYVelocity() * s1.getYVelocity());
  	double xUnit = (velocityMagnitude == 0) ? 1 : s1.getXVelocity() / velocityMagnitude;
  	double yUnit = (velocityMagnitude == 0) ? 0 : s1.getYVelocity() / velocityMagnitude;
    double area = (xMax - xMin) * (yMax - yMin);
    double triLength = 0.01 * Math.sqrt(targetSector.getHeight() * targetSector.getHeight() + targetSector.getWidth() * targetSector.getWidth());
  	double[] xPoints = {findXinSector(s1.getXPosition(), targetSector, xMax, xMin) + Math.sqrt(Math.cbrt(2500.0 / area)) * ((2 * triLength) * xUnit),
  	                    findXinSector(s1.getXPosition(), targetSector, xMax, xMin) + Math.sqrt(Math.cbrt(2500.0 / area)) * ((triLength) * yUnit - (triLength) * xUnit),
  	                    findXinSector(s1.getXPosition(), targetSector, xMax, xMin) + Math.sqrt(Math.cbrt(2500.0 / area)) * ((-triLength) * yUnit - (triLength) * xUnit)};
  	double[] yPoints = {findYinSector(s1.getYPosition(), targetSector, yMax, yMin) + Math.sqrt(Math.cbrt(2500.0 / area)) * ((2 * triLength) * yUnit),
  	                    findYinSector(s1.getYPosition(), targetSector, yMax, yMin) + Math.sqrt(Math.cbrt(2500.0 / area)) * ((-triLength) * xUnit - (triLength) * yUnit),
  	                    findYinSector(s1.getYPosition(), targetSector, yMax, yMin) + Math.sqrt(Math.cbrt(2500.0 / area)) * ((triLength) * xUnit - (triLength) * yUnit)};
  	StdDraw.filledPolygon(xPoints, yPoints);
  }

  // Creates the connections among the sensors within the network.
  public void createConnections(Sector targetSector, Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix,
  	                            double[] gridMinMax) {
  	double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	StdDraw.setPenColor(StdDraw.BLUE);
  	for(int i = 0; i < Parameters.sensorNumber; i++)
  		for(int j = 0; j < i; j++)
  			if(adjMatrix.getValue(i,j) > 0)
  				StdDraw.line(findXinSector(sensorNetwork[i].getXPosition(), targetSector, xMax, xMin), 
  					           findYinSector(sensorNetwork[i].getYPosition(), targetSector, yMax, yMin),
  			             	 findXinSector(sensorNetwork[j].getXPosition(), targetSector, xMax, xMin), 
  			             	 findYinSector(sensorNetwork[j].getYPosition(), targetSector, yMax, yMin));
  }

  // Creates trails behind the sensors as they move.
  public void createTrails(Sector targetSector, List<Double> xPoints, List<Double> yPoints, double[] gridMinMax, Color color) {
  	double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	StdDraw.setPenColor(color);
  	for(int i = 0; i < xPoints.size(); i++)
      StdDraw.point(findXinSector(xPoints.get(i), targetSector, xMax, xMin), findYinSector(yPoints.get(i), targetSector, yMax, yMin));
  }

  // Creates only the most recently updates portions of the trails.
  public void createTrailEnds(Sector targetSector, List<Double> xPoints, List<Double> yPoints, double[] gridMinMax) {
  	double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	StdDraw.setPenColor(StdDraw.DARK_GRAY);
  	for(int i = xPoints.size() - Parameters.sensorNumber; i < xPoints.size(); i++)
      StdDraw.point(findXinSector(xPoints.get(i), targetSector, xMax, xMin), findYinSector(yPoints.get(i), targetSector, yMax, yMin));
  }

  // Plots the valocities of the sensors in a spectrum of colors, one for each sensor node.
  public void plotVelocity(Sector targetSector, Sensor[] sensorNetwork, int iteration, double[] prevVelocity, double[] gridMinMax) {
    double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
    double red, green, blue, frequency = 0.05;
    for(int i = 0; i < sensorNetwork.length; i++) {
    	red = Math.sin(frequency * i + 0) * 127 + 128;
      green = Math.sin(frequency * i + 2 * Math.PI / 3) * 127 + 128;
      blue = Math.sin(frequency * i + 4 * Math.PI / 3) * 127 + 128;
      StdDraw.setPenColor((int) red, (int) green, (int) blue);
      double velocityMagnitude = Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + 
  		                                     sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity());
      StdDraw.line(findXinSector(iteration - 1, targetSector, xMax, xMin), 
      	           findYinSector(prevVelocity[i], targetSector, yMax, yMin),
      	           findXinSector(iteration, targetSector, xMax, xMin), 
      	           findYinSector(velocityMagnitude, targetSector, yMax, yMin));
      prevVelocity[i] = velocityMagnitude;
    }
  }

  // Plots the connectivity of the mobile sensor network.
  public void plotConnectivity(Sector targetSector, AdjacencyMatrix adjMatrix, double[] prevConnectivity, int iteration, double[] gridMinMax) {
    double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
    double newConnectivity = Algorithms.computeConnectivity(adjMatrix);
    StdDraw.setPenColor(StdDraw.BLUE);
    StdDraw.line(findXinSector(iteration - 1, targetSector, xMax, xMin), 
    	            findYinSector(prevConnectivity[0], targetSector, yMax, yMin),
    	            findXinSector(iteration, targetSector, xMax, xMin), 
    	            findYinSector(newConnectivity, targetSector, yMax, yMin));
    prevConnectivity[0] = newConnectivity;
  }

  // Creates the target that the sensors are tracking.
  public void createTarget(Sector targetSector, Sensor target, double[] gridMinMax) {
  	double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	StdDraw.setPenColor(StdDraw.RED);
  	double area = (xMax - xMin) * (yMax - yMin);
    double circleRadius = 0.01 * Math.sqrt(targetSector.getHeight() * targetSector.getHeight() + targetSector.getWidth() * targetSector.getWidth());
  	StdDraw.filledCircle(findXinSector(target.getXPosition(), targetSector, xMax, xMin), 
  		                   findYinSector(target.getYPosition(), targetSector, yMax, yMin),
  		                   Math.sqrt(Math.cbrt(2500.0 / area)) * circleRadius);
  	StdDraw.filledRectangle(findXinSector(target.getXPosition(), targetSector, xMax, xMin), 
	                          findYinSector(target.getYPosition(), targetSector, yMax, yMin),
	                          Math.sqrt(Math.cbrt(2500.0 / area)) * circleRadius * 0.25,
	                          Math.sqrt(Math.cbrt(2500.0 / area)) * circleRadius * 1.5);
  	StdDraw.filledRectangle(findXinSector(target.getXPosition(), targetSector, xMax, xMin), 
	                          findYinSector(target.getYPosition(), targetSector, yMax, yMin),
	                          Math.sqrt(Math.cbrt(2500.0 / area)) * circleRadius * 1.5,
	                          Math.sqrt(Math.cbrt(2500.0 / area)) * circleRadius * 0.25);
  	StdDraw.setPenColor(StdDraw.WHITE);
  	StdDraw.filledCircle(findXinSector(target.getXPosition(), targetSector, xMax, xMin), 
  		                   findYinSector(target.getYPosition(), targetSector, yMax, yMin),
  		                   Math.sqrt(Math.cbrt(2500.0 / area)) * circleRadius * 0.5);
  }

  // Creates a round obstacle for the sensors to avoid.
  public void createRoundObstacle(Sector targetSector, RoundObstacle object, double[] gridMinMax) {
  	double xMin = gridMinMax[0], xMax = gridMinMax[1], yMin = gridMinMax[2], yMax = gridMinMax[3];
  	double area = (xMax - xMin) * (yMax - yMin);
  	StdDraw.setPenColor(StdDraw.GREEN);
  	StdDraw.filledEllipse(findXinSector(object.getXPosition(), targetSector, xMax, xMin), 
  		                    findYinSector(object.getYPosition(), targetSector, yMax, yMin),
  		                    object.getRadius() * targetSector.getWidth() * widthRatio / (xMax - xMin),
  		                    object.getRadius() * targetSector.getHeight() * heightRatio / (yMax - yMin));
  	StdDraw.setPenColor(StdDraw.BLACK);
  	StdDraw.ellipse(findXinSector(object.getXPosition(), targetSector, xMax, xMin), 
  		              findYinSector(object.getYPosition(), targetSector, yMax, yMin),
  		              object.getRadius() * targetSector.getWidth() * widthRatio / (xMax - xMin),
  		              object.getRadius() * targetSector.getHeight() * heightRatio / (yMax - yMin));
  }

  // Clears the given sector of all objects displayed.
  public void clearSector(Sector targetSector) {
  	StdDraw.setPenColor(StdDraw.WHITE);
  	StdDraw.filledRectangle(targetSector.getX() + 0.5 * targetSector.getWidth(), 
  		                      targetSector.getY() + (0.5) * targetSector.getHeight(), 
  		                      0.5 * targetSector.getWidth(), 0.5 * targetSector.getHeight());
  }

  // Finds the given x position relative to the sector.
  public double findXinSector(double desiredX, Sector targetSector, double xMax, double xMin) {
  	return (((desiredX - xMin) / (xMax - xMin)) * (widthRatio) + (1 - widthRatio) / 2) * targetSector.getWidth() + targetSector.getX();
  }

  // Finds the given y position relative to the sector.
  public double findYinSector(double desiredY, Sector targetSector, double yMax, double yMin) {
  	return (((desiredY - yMin) / (yMax - yMin)) * (heightRatio) + (1 - heightRatio) / 2) * targetSector.getHeight() + targetSector.getY();
  }
}