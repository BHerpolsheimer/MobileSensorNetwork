// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Implementation of each of the five cases of the simulation.

import java.io.*;
import java.util.*;
import java.awt.event.KeyEvent;

public class SensorNetwork {
  public static void main(String[] args) {
    Display myDisplay =  new Display();
    myWait(myDisplay);

    executeCaseOne(myDisplay);
    myWait(myDisplay);

    executeCaseTwo(myDisplay);
    myWait(myDisplay);

    executeCaseThree(myDisplay);
    myWait(myDisplay);

    execuleCaseFour(myDisplay);
    myWait(myDisplay);

    execuleCaseFive(myDisplay);
    myWait(myDisplay);

    System.exit(0);
  }

  // Displays a message to press enter to continue and waits for the input.
  public static void myWait(Display myDisplay) {
  	StdDraw.setPenColor(StdDraw.BLACK);
    StdDraw.text(410, 975, "Press enter to continue...");
    StdDraw.show(1);
    while(!StdDraw.isKeyPressed(KeyEvent.VK_ENTER));
  }

  // Runs a simulation using algorithm one to calculate the
  // accelerations of the sensor nodes.
  public static void executeCaseOne(Display myDisplay) {
    Sensor[] sensorNetwork = new Sensor[Parameters.sensorNumber];
    for (int i = 0; i < sensorNetwork.length; ++i)
      sensorNetwork[i] = new Sensor();

    Sector sector1 = new Sector(10, 50, 900, 900);
    Sector sector2 = new Sector(920, 500, 500, 500);
    Sector sector3 = new Sector(1420, 500, 500, 500);
    Sector sector4 = new Sector(920, 0, 500, 500);
    Sector sector5 = new Sector(1420, 0, 500, 500);

    double[] futureGridMinMax = {0, 50, 0, 50};
    double[] gridMinMax = {0, 50, 0, 50};
    double[] velGridMinMax = {0, Parameters.iterations, 0, 0};
    double[] connectivityGridMinMax = {0, Parameters.iterations, 0, 1.05};

    disperseSensors(sensorNetwork, 50, 50);

    AdjacencyMatrix adjMatrix = new AdjacencyMatrix(Parameters.sensorNumber);
    adjMatrix.update(sensorNetwork);

    StdDraw.clear();
    StdDraw.show(1);

    double[] prevVelocity = new double[Parameters.sensorNumber];
    for(int i = 0; i < prevVelocity.length; i++)
      prevVelocity[i] = 0;

    List<Double> xPoints = new ArrayList<Double> ();
    List<Double> yPoints = new ArrayList<Double> ();

    simAlgOne(sensorNetwork, adjMatrix, futureGridMinMax, velGridMinMax);

    double[] prevConnectivity = {Algorithms.computeConnectivity(adjMatrix)};
    adjMatrix.update(sensorNetwork);

    double xConnectivityInterval = ((connectivityGridMinMax[1] - connectivityGridMinMax[0]) - (connectivityGridMinMax[1] - connectivityGridMinMax[0]) % 50) / 10;
    double yConnectivityInterval = 0.1;
    myDisplay.createGrid(sector4, xConnectivityInterval, yConnectivityInterval, connectivityGridMinMax, "Hello", "hello", "Hi");

    double xVelocityInterval = ((velGridMinMax[1] - velGridMinMax[0]) - (velGridMinMax[1] - velGridMinMax[0]) % 50) / 10;
    double yVelocityInterval = ((velGridMinMax[3] - velGridMinMax[2]) - (velGridMinMax[3] - velGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector3, xVelocityInterval, yVelocityInterval, velGridMinMax, "Hello", "hello", "Hi");

    double xPositionInterval = ((futureGridMinMax[1] - futureGridMinMax[0]) - (futureGridMinMax[1] - futureGridMinMax[0]) % 50) / 10;
    double yPositionInterval = ((futureGridMinMax[3] - futureGridMinMax[2]) - (futureGridMinMax[3] - futureGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector2, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");

    double xInterval = 5;
    double yInterval = 5;

    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      myDisplay.clearSector(sector1);
      myDisplay.plotVelocity(sector3, sensorNetwork, iteration, prevVelocity, velGridMinMax);
      for (int i = 0; i < sensorNetwork.length; ++i) {
        xPoints.add(sensorNetwork[i].getXPosition());
        yPoints.add(sensorNetwork[i].getYPosition());
        sensorNetwork[i].move(Algorithms.computeAlgOne(i, sensorNetwork, adjMatrix), gridMinMax);
      }
      xInterval = ((gridMinMax[1] - gridMinMax[0]) - (gridMinMax[1] - gridMinMax[0]) % (xInterval * 10)) / 10;
      yInterval = ((gridMinMax[3] - gridMinMax[2]) - (gridMinMax[3] - gridMinMax[2]) % (yInterval * 10)) / 10;
      myDisplay.createGrid(sector1, xInterval, yInterval, gridMinMax, "Hello", "hello", "Hi");
      myDisplay.plotConnectivity(sector4, adjMatrix, prevConnectivity, iteration, connectivityGridMinMax);
      adjMatrix.update(sensorNetwork);
      myDisplay.createConnections(sector1, sensorNetwork, adjMatrix, gridMinMax);
      myDisplay.createSensorNetwork(sector1, sensorNetwork, gridMinMax);
      myDisplay.createTrailEnds(sector2, xPoints, yPoints, futureGridMinMax);

      StdDraw.show(1);

      if(iteration % 150 == 0)
      	myWait(myDisplay);
    }
    myDisplay.createSensorNetwork(sector2, sensorNetwork, gridMinMax);
    StdDraw.show(1);
  }

  // Runs a simulation using algorithm two to calculate the
  // accelerations of the sensor nodes on a stationary target.
  public static void executeCaseTwo(Display myDisplay) {
    Sensor[] sensorNetwork = new Sensor[Parameters.sensorNumber];
    for (int i = 0; i < sensorNetwork.length; ++i)
      sensorNetwork[i] = new Sensor();

    Sensor target = new Sensor(150, 150, 0, 0);

    Sector sector1 = new Sector(10, 50, 900, 900);
    Sector sector2 = new Sector(920, 500, 500, 500);
    Sector sector3 = new Sector(1420, 500, 500, 500);
    Sector sector4 = new Sector(920, 0, 500, 500);
    Sector sector5 = new Sector(1420, 0, 500, 500);

    double[] futureGridMinMax = {0, 160, 0, 160};
    double[] gridMinMax = {0, 160, 0, 160};
    double[] velGridMinMax = {0, Parameters.iterations, 0, 0};
    double[] connectivityGridMinMax = {0, Parameters.iterations, 0, 1.05};

    disperseSensors(sensorNetwork, 50, 50);

    AdjacencyMatrix adjMatrix = new AdjacencyMatrix(Parameters.sensorNumber);
    adjMatrix.update(sensorNetwork);

    StdDraw.clear();
    StdDraw.show(1);

    double[] prevVelocity = new double[Parameters.sensorNumber];
    for(int i = 0; i < prevVelocity.length; i++)
      prevVelocity[i] = 0;

    List<Double> xPoints = new ArrayList<Double> ();
    List<Double> yPoints = new ArrayList<Double> ();

    simAlgTwo(sensorNetwork, adjMatrix, futureGridMinMax, velGridMinMax, target);

    double[] prevConnectivity = {Algorithms.computeConnectivity(adjMatrix)};
    adjMatrix.update(sensorNetwork);


    StdDraw.clear();
    double xConnectivityInterval = ((connectivityGridMinMax[1] - connectivityGridMinMax[0]) - (connectivityGridMinMax[1] - connectivityGridMinMax[0]) % 50) / 10;
    double yConnectivityInterval = 0.1;
    myDisplay.createGrid(sector4, xConnectivityInterval, yConnectivityInterval, connectivityGridMinMax, "Hello", "hello", "Hi");

    double xVelocityInterval = ((velGridMinMax[1] - velGridMinMax[0]) - (velGridMinMax[1] - velGridMinMax[0]) % 50) / 10;
    double yVelocityInterval = ((velGridMinMax[3] - velGridMinMax[2]) - (velGridMinMax[3] - velGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector3, xVelocityInterval, yVelocityInterval, velGridMinMax, "Hello", "hello", "Hi");

    double xPositionInterval = ((futureGridMinMax[1] - futureGridMinMax[0]) - (futureGridMinMax[1] - futureGridMinMax[0]) % 50) / 10;
    double yPositionInterval = ((futureGridMinMax[3] - futureGridMinMax[2]) - (futureGridMinMax[3] - futureGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector2, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");

    double xInterval = 5;
    double yInterval = 5;

    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      myDisplay.clearSector(sector1);
      myDisplay.plotVelocity(sector3, sensorNetwork, iteration, prevVelocity, velGridMinMax);
      for (int i = 0; i < sensorNetwork.length; ++i) {
        xPoints.add(sensorNetwork[i].getXPosition());
        yPoints.add(sensorNetwork[i].getYPosition());
        sensorNetwork[i].move(Algorithms.computeAlgTwo(i, sensorNetwork, adjMatrix, target), gridMinMax);
      }
      xInterval = ((gridMinMax[1] - gridMinMax[0]) - (gridMinMax[1] - gridMinMax[0]) % (xInterval * 10)) / 10;
      yInterval = ((gridMinMax[3] - gridMinMax[2]) - (gridMinMax[3] - gridMinMax[2]) % (yInterval * 10)) / 10;
      myDisplay.createGrid(sector1, xInterval, yInterval, gridMinMax, "Hello", "hello", "Hi");
      myDisplay.plotConnectivity(sector4, adjMatrix, prevConnectivity, iteration, connectivityGridMinMax);
      adjMatrix.update(sensorNetwork);
      myDisplay.createTarget(sector1, target, gridMinMax);
      myDisplay.createConnections(sector1, sensorNetwork, adjMatrix, gridMinMax);
      myDisplay.createSensorNetwork(sector1, sensorNetwork, gridMinMax);
      myDisplay.createTrailEnds(sector2, xPoints, yPoints, futureGridMinMax);

      StdDraw.show(1);

      if(iteration % 150 == 0)
      	myWait(myDisplay);
    }
    myDisplay.createSensorNetwork(sector2, sensorNetwork, gridMinMax);
    StdDraw.show(1);
  }

  // Runs a simulation using algorithm two to calculate the
  // accelerations of the sensor nodes with a target moving in
  // the path of a sine wave.
  public static void executeCaseThree(Display myDisplay) {
    Sensor[] sensorNetwork = new Sensor[Parameters.sensorNumber];
    for (int i = 0; i < sensorNetwork.length; ++i)
      sensorNetwork[i] = new Sensor();

    Sensor target = new Sensor(150, 150, 25, 50 * Parameters.iterations * Parameters.delta_t / (2 * Math.PI));

    Sector sector1 = new Sector(10, 50, 900, 900);
    Sector sector2 = new Sector(920, 500, 500, 500);
    Sector sector3 = new Sector(1420, 500, 500, 500);
    Sector sector4 = new Sector(920, 0, 500, 500);
    Sector sector5 = new Sector(1420, 0, 500, 500);

    double[] futureGridMinMax = {0, 160, 0, 160};
    double[] gridMinMax = {0, 160, 0, 160};
    double[] velGridMinMax = {0, Parameters.iterations, 0, 0};
    double[] connectivityGridMinMax = {0, Parameters.iterations, 0, 1.05};

    disperseSensors(sensorNetwork, 150, 150);

    AdjacencyMatrix adjMatrix = new AdjacencyMatrix(Parameters.sensorNumber);
    adjMatrix.update(sensorNetwork);

    StdDraw.clear();
    StdDraw.show(1);

    double[] prevVelocity = new double[Parameters.sensorNumber];
    for(int i = 0; i < prevVelocity.length; i++)
      prevVelocity[i] = 0;

    List<Double> xPoints = new ArrayList<Double> ();
    List<Double> yPoints = new ArrayList<Double> ();
    List<Double> xTargetPoints = new ArrayList<Double> ();
    List<Double> yTargetPoints = new ArrayList<Double> ();
    List<Double> xCoMPoints = new ArrayList<Double> ();
    List<Double> yCoMPoints = new ArrayList<Double> ();

    simCaseThree(sensorNetwork, adjMatrix, futureGridMinMax, velGridMinMax, target);

    double[] prevConnectivity = {Algorithms.computeConnectivity(adjMatrix)};
    adjMatrix.update(sensorNetwork);


    StdDraw.clear();
    double xConnectivityInterval = ((connectivityGridMinMax[1] - connectivityGridMinMax[0]) - (connectivityGridMinMax[1] - connectivityGridMinMax[0]) % 50) / 10;
    double yConnectivityInterval = 0.1;
    myDisplay.createGrid(sector4, xConnectivityInterval, yConnectivityInterval, connectivityGridMinMax, "Hello", "hello", "Hi");

    double xVelocityInterval = ((velGridMinMax[1] - velGridMinMax[0]) - (velGridMinMax[1] - velGridMinMax[0]) % 50) / 10;
    double yVelocityInterval = ((velGridMinMax[3] - velGridMinMax[2]) - (velGridMinMax[3] - velGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector3, xVelocityInterval, yVelocityInterval, velGridMinMax, "Hello", "hello", "Hi");

    double xPositionInterval = ((futureGridMinMax[1] - futureGridMinMax[0]) - (futureGridMinMax[1] - futureGridMinMax[0]) % 50) / 10;
    double yPositionInterval = ((futureGridMinMax[3] - futureGridMinMax[2]) - (futureGridMinMax[3] - futureGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector2, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");

    double xInterval = 5;
    double yInterval = 5;

    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      myDisplay.clearSector(sector1);
      myDisplay.clearSector(sector5);
      myDisplay.plotVelocity(sector3, sensorNetwork, iteration, prevVelocity, velGridMinMax);
      for (int i = 0; i < sensorNetwork.length; ++i) {
        xPoints.add(sensorNetwork[i].getXPosition());
        yPoints.add(sensorNetwork[i].getYPosition());
        sensorNetwork[i].move(Algorithms.computeAlgTwo(i, sensorNetwork, adjMatrix, target), gridMinMax);
      }

      xCoMPoints.add(Algorithms.computeXCoM(sensorNetwork));
      yCoMPoints.add(Algorithms.computeYCoM(sensorNetwork));

      double[] targetAcceleration = {0, -50 * Math.sin(2 * Math.PI * iteration / Parameters.iterations)};
      xTargetPoints.add(target.getXPosition());
      yTargetPoints.add(target.getYPosition());
      target.move(targetAcceleration, gridMinMax);

      xInterval = ((gridMinMax[1] - gridMinMax[0]) - (gridMinMax[1] - gridMinMax[0]) % (xInterval * 10)) / 10;
      yInterval = ((gridMinMax[3] - gridMinMax[2]) - (gridMinMax[3] - gridMinMax[2]) % (yInterval * 10)) / 10;
      myDisplay.createGrid(sector1, xInterval, yInterval, gridMinMax, "Hello", "hello", "Hi");
      myDisplay.createGrid(sector5, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");
      myDisplay.plotConnectivity(sector4, adjMatrix, prevConnectivity, iteration, connectivityGridMinMax);
      adjMatrix.update(sensorNetwork);

      myDisplay.createTrails(sector1, xTargetPoints, yTargetPoints, gridMinMax, StdDraw.RED);
      myDisplay.createTrails(sector1, xCoMPoints, yCoMPoints, gridMinMax, StdDraw.BLACK);
      myDisplay.createTarget(sector1, target, gridMinMax);
      myDisplay.createConnections(sector1, sensorNetwork, adjMatrix, gridMinMax);
      myDisplay.createSensorNetwork(sector1, sensorNetwork, gridMinMax);

      myDisplay.createTrails(sector5, xTargetPoints, yTargetPoints, futureGridMinMax, StdDraw.RED);
      myDisplay.createTrails(sector5, xCoMPoints, yCoMPoints, futureGridMinMax, StdDraw.BLACK);
      myDisplay.createTarget(sector5, target, futureGridMinMax);

      myDisplay.createTrailEnds(sector2, xPoints, yPoints, futureGridMinMax);

      StdDraw.show(1);

      if(iteration % 150 == 0)
      	myWait(myDisplay);
    }
    myDisplay.createSensorNetwork(sector2, sensorNetwork, futureGridMinMax);
    StdDraw.show(1); 
  }

  // Runs a simulation using algorithm two to calculate the
  // accelerations of the sensor nodes with a target moving in
  // the path of a circle.
  public static void execuleCaseFour(Display myDisplay) {
    Sensor[] sensorNetwork = new Sensor[Parameters.sensorNumber];
    for (int i = 0; i < sensorNetwork.length; ++i)
      sensorNetwork[i] = new Sensor();

    Sensor target = new Sensor(150, 150, 0, 50 * Parameters.iterations * Parameters.delta_t / (2 * Math.PI));

    Sector sector1 = new Sector(10, 50, 900, 900);
    Sector sector2 = new Sector(920, 500, 500, 500);
    Sector sector3 = new Sector(1420, 500, 500, 500);
    Sector sector4 = new Sector(920, 0, 500, 500);
    Sector sector5 = new Sector(1420, 0, 500, 500);

    double[] futureGridMinMax = {0, 160, 0, 160};
    double[] gridMinMax = {0, 160, 0, 160};
    double[] velGridMinMax = {0, Parameters.iterations, 0, 0};
    double[] connectivityGridMinMax = {0, Parameters.iterations, 0, 1.05};

    disperseSensors(sensorNetwork, 150, 150);

    AdjacencyMatrix adjMatrix = new AdjacencyMatrix(Parameters.sensorNumber);
    adjMatrix.update(sensorNetwork);

    StdDraw.clear();
    StdDraw.show(1);

    double[] prevVelocity = new double[Parameters.sensorNumber];
    for(int i = 0; i < prevVelocity.length; i++)
      prevVelocity[i] = 0;

    List<Double> xPoints = new ArrayList<Double> ();
    List<Double> yPoints = new ArrayList<Double> ();
    List<Double> xTargetPoints = new ArrayList<Double> ();
    List<Double> yTargetPoints = new ArrayList<Double> ();
    List<Double> xCoMPoints = new ArrayList<Double> ();
    List<Double> yCoMPoints = new ArrayList<Double> ();

    simCaseFour(sensorNetwork, adjMatrix, futureGridMinMax, velGridMinMax, target);

    double[] prevConnectivity = {Algorithms.computeConnectivity(adjMatrix)};
    adjMatrix.update(sensorNetwork);


    StdDraw.clear();
    double xConnectivityInterval = ((connectivityGridMinMax[1] - connectivityGridMinMax[0]) - (connectivityGridMinMax[1] - connectivityGridMinMax[0]) % 50) / 10;
    double yConnectivityInterval = 0.1;
    myDisplay.createGrid(sector4, xConnectivityInterval, yConnectivityInterval, connectivityGridMinMax, "Hello", "hello", "Hi");

    double xVelocityInterval = ((velGridMinMax[1] - velGridMinMax[0]) - (velGridMinMax[1] - velGridMinMax[0]) % 50) / 10;
    double yVelocityInterval = ((velGridMinMax[3] - velGridMinMax[2]) - (velGridMinMax[3] - velGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector3, xVelocityInterval, yVelocityInterval, velGridMinMax, "Hello", "hello", "Hi");

    double xPositionInterval = ((futureGridMinMax[1] - futureGridMinMax[0]) - (futureGridMinMax[1] - futureGridMinMax[0]) % 50) / 10;
    double yPositionInterval = ((futureGridMinMax[3] - futureGridMinMax[2]) - (futureGridMinMax[3] - futureGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector2, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");

    double xInterval = 5;
    double yInterval = 5;

    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      myDisplay.clearSector(sector1);
      myDisplay.clearSector(sector5);
      myDisplay.plotVelocity(sector3, sensorNetwork, iteration, prevVelocity, velGridMinMax);
      for (int i = 0; i < sensorNetwork.length; ++i) {
        xPoints.add(sensorNetwork[i].getXPosition());
        yPoints.add(sensorNetwork[i].getYPosition());
        sensorNetwork[i].move(Algorithms.computeAlgTwo(i, sensorNetwork, adjMatrix, target), gridMinMax);
      }

      xCoMPoints.add(Algorithms.computeXCoM(sensorNetwork));
      yCoMPoints.add(Algorithms.computeYCoM(sensorNetwork));

      double[] targetAcceleration = {50 * Math.cos(2 * Math.PI * iteration / Parameters.iterations), -50 * Math.sin(2 * Math.PI * iteration / Parameters.iterations)};
      xTargetPoints.add(target.getXPosition());
      yTargetPoints.add(target.getYPosition());
      target.move(targetAcceleration, gridMinMax);

      xInterval = ((gridMinMax[1] - gridMinMax[0]) - (gridMinMax[1] - gridMinMax[0]) % (xInterval * 10)) / 10;
      yInterval = ((gridMinMax[3] - gridMinMax[2]) - (gridMinMax[3] - gridMinMax[2]) % (yInterval * 10)) / 10;
      myDisplay.createGrid(sector1, xInterval, yInterval, gridMinMax, "Hello", "hello", "Hi");
      myDisplay.createGrid(sector5, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");
      myDisplay.plotConnectivity(sector4, adjMatrix, prevConnectivity, iteration, connectivityGridMinMax);
      adjMatrix.update(sensorNetwork);

      myDisplay.createTrails(sector1, xTargetPoints, yTargetPoints, gridMinMax, StdDraw.RED);
      myDisplay.createTrails(sector1, xCoMPoints, yCoMPoints, gridMinMax, StdDraw.BLACK);
      myDisplay.createTarget(sector1, target, gridMinMax);
      myDisplay.createConnections(sector1, sensorNetwork, adjMatrix, gridMinMax);
      myDisplay.createSensorNetwork(sector1, sensorNetwork, gridMinMax);

      myDisplay.createTrails(sector5, xTargetPoints, yTargetPoints, futureGridMinMax, StdDraw.RED);
      myDisplay.createTrails(sector5, xCoMPoints, yCoMPoints, futureGridMinMax, StdDraw.BLACK);
      myDisplay.createTarget(sector5, target, futureGridMinMax);

      myDisplay.createTrailEnds(sector2, xPoints, yPoints, futureGridMinMax);

      StdDraw.show(1);

      if(iteration % 150 == 0)
      	myWait(myDisplay);
    }
    myDisplay.createSensorNetwork(sector2, sensorNetwork, futureGridMinMax);
    StdDraw.show(1);
  }

  // Runs a simulation using algorithm three to calculate the
  // accelerations of the sensor nodes with a target moving in
  // the path of a circle with 3 stationary obstacles.
  public static void execuleCaseFive(Display myDisplay) {
    Sensor[] sensorNetwork = new Sensor[Parameters.sensorNumber];
    for (int i = 0; i < sensorNetwork.length; i++)
      sensorNetwork[i] = new Sensor();

    Sensor target = new Sensor(150, 150, 0, 50 * Parameters.iterations * Parameters.delta_t / (2 * Math.PI));

    Sector sector1 = new Sector(10, 50, 900, 900);
    Sector sector2 = new Sector(920, 500, 500, 500);
    Sector sector3 = new Sector(1420, 500, 500, 500);
    Sector sector4 = new Sector(920, 0, 500, 500);
    Sector sector5 = new Sector(1420, 0, 500, 500);

    RoundObstacle[] obstacles = {new RoundObstacle(140, 190, 25),
                                 new RoundObstacle(300, 200, 25),
                                 new RoundObstacle(100, 50, 25)};

    Sensor[][] betaNetwork = new Sensor[Parameters.sensorNumber][obstacles.length];
    for (int i = 0; i < sensorNetwork.length; i++)
      for(int j = 0; j < obstacles.length; j++)
        betaNetwork[i][j] = new Sensor();

    double[] futureGridMinMax = {0, 350, 0, 350};
    double[] gridMinMax = {0, 350, 0, 350};
    double[] velGridMinMax = {0, Parameters.iterations, 0, 0};
    double[] connectivityGridMinMax = {0, Parameters.iterations, 0, 1.05};

    disperseSensors(sensorNetwork, 50, 50);

    AdjacencyMatrix adjMatrix = new AdjacencyMatrix(Parameters.sensorNumber);
    adjMatrix.update(sensorNetwork);

    StdDraw.clear();
    StdDraw.show(1);

    double[] prevVelocity = new double[Parameters.sensorNumber];
    for(int i = 0; i < prevVelocity.length; i++)
      prevVelocity[i] = 0;

    List<Double> xPoints = new ArrayList<Double> ();
    List<Double> yPoints = new ArrayList<Double> ();
    List<Double> xTargetPoints = new ArrayList<Double> ();
    List<Double> yTargetPoints = new ArrayList<Double> ();
    List<Double> xCoMPoints = new ArrayList<Double> ();
    List<Double> yCoMPoints = new ArrayList<Double> ();

    simCaseFour(sensorNetwork, adjMatrix, futureGridMinMax, velGridMinMax, target);

    double[] prevConnectivity = {Algorithms.computeConnectivity(adjMatrix)};
    adjMatrix.update(sensorNetwork);

    AdjacencyMatrix betaMatrix = new AdjacencyMatrix(Parameters.sensorNumber, obstacles.length);

    StdDraw.clear();
    double xConnectivityInterval = ((connectivityGridMinMax[1] - connectivityGridMinMax[0]) - (connectivityGridMinMax[1] - connectivityGridMinMax[0]) % 50) / 10;
    double yConnectivityInterval = 0.1;
    myDisplay.createGrid(sector4, xConnectivityInterval, yConnectivityInterval, connectivityGridMinMax, "Hello", "hello", "Hi");

    double xVelocityInterval = ((velGridMinMax[1] - velGridMinMax[0]) - (velGridMinMax[1] - velGridMinMax[0]) % 50) / 10;
    double yVelocityInterval = ((velGridMinMax[3] - velGridMinMax[2]) - (velGridMinMax[3] - velGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector3, xVelocityInterval, yVelocityInterval, velGridMinMax, "Hello", "hello", "Hi");

    double xPositionInterval = ((futureGridMinMax[1] - futureGridMinMax[0]) - (futureGridMinMax[1] - futureGridMinMax[0]) % 50) / 10;
    double yPositionInterval = ((futureGridMinMax[3] - futureGridMinMax[2]) - (futureGridMinMax[3] - futureGridMinMax[2]) % 50) / 10;
    myDisplay.createGrid(sector2, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");

    double xInterval = 5;
    double yInterval = 5;

    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      myDisplay.clearSector(sector1);
      myDisplay.clearSector(sector5);
      myDisplay.plotVelocity(sector3, sensorNetwork, iteration, prevVelocity, velGridMinMax);
      for (int i = 0; i < sensorNetwork.length; ++i) {
        xPoints.add(sensorNetwork[i].getXPosition());
        yPoints.add(sensorNetwork[i].getYPosition());
        sensorNetwork[i].move(Algorithms.computeAlgThree(i, sensorNetwork, betaNetwork, adjMatrix, betaMatrix, target), gridMinMax);
        for(int j = 0; j < obstacles.length; j++) {
          betaNetwork[i][j].setXPosition(Algorithms.computeMu(sensorNetwork[i], obstacles[j]) * sensorNetwork[i].getXPosition() + 
                                         (1 - Algorithms.computeMu(sensorNetwork[i], obstacles[j])) * obstacles[j].getXPosition());
          betaNetwork[i][j].setYPosition(Algorithms.computeMu(sensorNetwork[i], obstacles[j]) * sensorNetwork[i].getYPosition() + 
                                         (1 - Algorithms.computeMu(sensorNetwork[i], obstacles[j])) * obstacles[j].getYPosition());
          betaNetwork[i][j].setXVelocity(Algorithms.computeMu(sensorNetwork[i], obstacles[j]) * Algorithms.computeRho(sensorNetwork[i], obstacles[j]) * sensorNetwork[i].getXVelocity());
          betaNetwork[i][j].setYVelocity(Algorithms.computeMu(sensorNetwork[i], obstacles[j]) * Algorithms.computeRho(sensorNetwork[i], obstacles[j]) * sensorNetwork[i].getYVelocity());
        }
      }

      xCoMPoints.add(Algorithms.computeXCoM(sensorNetwork));
      yCoMPoints.add(Algorithms.computeYCoM(sensorNetwork));

      double[] targetAcceleration = {50 * Math.cos(2 * Math.PI * iteration / Parameters.iterations), -50 * Math.sin(2 * Math.PI * iteration / Parameters.iterations)};
      xTargetPoints.add(target.getXPosition());
      yTargetPoints.add(target.getYPosition());
      target.move(targetAcceleration, gridMinMax);

      xInterval = ((gridMinMax[1] - gridMinMax[0]) - (gridMinMax[1] - gridMinMax[0]) % (xInterval * 10)) / 10;
      yInterval = ((gridMinMax[3] - gridMinMax[2]) - (gridMinMax[3] - gridMinMax[2]) % (yInterval * 10)) / 10;
      myDisplay.createGrid(sector1, xInterval, yInterval, gridMinMax, "Hello", "hello", "Hi");
      myDisplay.createGrid(sector5, xPositionInterval, yPositionInterval, futureGridMinMax, "Hello", "hello", "Hi");
      myDisplay.plotConnectivity(sector4, adjMatrix, prevConnectivity, iteration, connectivityGridMinMax);
      adjMatrix.update(sensorNetwork);
      betaMatrix.update(sensorNetwork, betaNetwork);

      for(int i = 0; i < obstacles.length; i++)
        myDisplay.createRoundObstacle(sector1, obstacles[i], gridMinMax);

      myDisplay.createTrails(sector1, xTargetPoints, yTargetPoints, gridMinMax, StdDraw.RED);
      myDisplay.createTrails(sector1, xCoMPoints, yCoMPoints, gridMinMax, StdDraw.BLACK);
      myDisplay.createTarget(sector1, target, gridMinMax);
      myDisplay.createConnections(sector1, sensorNetwork, adjMatrix, gridMinMax);
      myDisplay.createSensorNetwork(sector1, sensorNetwork, gridMinMax);

      myDisplay.createTrails(sector5, xTargetPoints, yTargetPoints, futureGridMinMax, StdDraw.RED);
      myDisplay.createTrails(sector5, xCoMPoints, yCoMPoints, futureGridMinMax, StdDraw.BLACK);
      myDisplay.createTarget(sector5, target, futureGridMinMax);

      myDisplay.createTrailEnds(sector2, xPoints, yPoints, futureGridMinMax);

      StdDraw.show(1);

      if(iteration % 150 == 0)
      	myWait(myDisplay);
    }
    myDisplay.createSensorNetwork(sector2, sensorNetwork, futureGridMinMax);
    StdDraw.show(1);
  }

  // Randomly dispurses the sensors with x and y values within
  // the given range, starting at zero.
  public static void disperseSensors(Sensor[] sensorNetwork, int xRange, int yRange) {
    for (int i = 0; i < sensorNetwork.length; ++i) {
      while (true) {
        boolean test = true;
        sensorNetwork[i].setXPosition(Math.random() * xRange);
        sensorNetwork[i].setYPosition(Math.random() * yRange);
        for (int l = 0; l < sensorNetwork.length; ++l) {
          if (l == i)
            continue;
            if (Algorithms.computeEuclidean(sensorNetwork[l], sensorNetwork[i]) < 1)
              test = false;
        }
        if (test == true)
          break;
      }
    }
  }

  // Runs a simulation of algorithm one with the given parameters.
  public static void simAlgOne(Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix, double[] gridMinMax, double[] velGridMinMax) {
    double[][] tempCoordinates = new double[sensorNetwork.length][2];
    for (int i = 0; i < sensorNetwork.length; ++i) {
      tempCoordinates[i][0] = sensorNetwork[i].getXPosition();
      tempCoordinates[i][1] = sensorNetwork[i].getYPosition();
    }
    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      for (int i = 0; i < sensorNetwork.length; i++) {
        sensorNetwork[i].move(Algorithms.computeAlgOne(i, sensorNetwork, adjMatrix), gridMinMax);
        if(velGridMinMax[3] < Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity()))
          velGridMinMax[3] = Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity());
      }
      adjMatrix.update(sensorNetwork);
    }
    for (int i = 0; i < sensorNetwork.length; i++) {
      sensorNetwork[i].setXPosition(tempCoordinates[i][0]);
      sensorNetwork[i].setYPosition(tempCoordinates[i][1]);
      sensorNetwork[i].setXVelocity(0);
      sensorNetwork[i].setYVelocity(0);
      adjMatrix.update(sensorNetwork);
    }
  }

  // Runs a simulation of algorithm two with the given parameters.
  public static void simAlgTwo(Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix, double[] gridMinMax, double[] velGridMinMax, Sensor target) {
    double[][] tempCoordinates = new double[sensorNetwork.length ][2];
    int i;
    for (i = 0; i < sensorNetwork.length; i++) {
      tempCoordinates[i][0] = sensorNetwork[i].getXPosition();
      tempCoordinates[i][1] = sensorNetwork[i].getYPosition();
    }
    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      for (i = 0; i < sensorNetwork.length; ++i) {
        sensorNetwork[i].move(Algorithms.computeAlgTwo(i, sensorNetwork, adjMatrix, target), gridMinMax);
        if(velGridMinMax[3] < Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity()))
          velGridMinMax[3] = Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity());
      }
      adjMatrix.update(sensorNetwork);
    }
    for (i = 0; i < sensorNetwork.length; i++) {
      sensorNetwork[i].setXPosition(tempCoordinates[i][0]);
      sensorNetwork[i].setYPosition(tempCoordinates[i][1]);
      sensorNetwork[i].setXVelocity(0);
      sensorNetwork[i].setYVelocity(0);
      adjMatrix.update(sensorNetwork);
    }
  }

  // Runs a simulation of case three with the given parameters.
  public static void simCaseThree(Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix, double[] gridMinMax, double[] velGridMinMax, Sensor target) {
    double[][] tempCoordinates = new double[sensorNetwork.length + 2][2];
    int i;
    for (i = 0; i < sensorNetwork.length; i++) {
      tempCoordinates[i][0] = sensorNetwork[i].getXPosition();
      tempCoordinates[i][1] = sensorNetwork[i].getYPosition();
    }
    tempCoordinates[i][0] = target.getXPosition();
    tempCoordinates[i][1] = target.getYPosition();
    tempCoordinates[i + 1][0] = target.getXVelocity();
    tempCoordinates[i + 1][1] = target.getYVelocity();
    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      for (i = 0; i < sensorNetwork.length; ++i) {
        sensorNetwork[i].move(Algorithms.computeAlgTwo(i, sensorNetwork, adjMatrix, target), gridMinMax);
        if(velGridMinMax[3] < Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity()))
          velGridMinMax[3] = Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity());
      }
      double[] targetAcceleration = {0, -50 * Math.sin(2 * Math.PI * iteration / Parameters.iterations)};
      target.move(targetAcceleration, gridMinMax);
      adjMatrix.update(sensorNetwork);
    }
    for (i = 0; i < sensorNetwork.length; i++) {
      sensorNetwork[i].setXPosition(tempCoordinates[i][0]);
      sensorNetwork[i].setYPosition(tempCoordinates[i][1]);
      sensorNetwork[i].setXVelocity(0);
      sensorNetwork[i].setYVelocity(0);
      adjMatrix.update(sensorNetwork);
    }
    target.setXPosition(tempCoordinates[i][0]);
    target.setYPosition(tempCoordinates[i][1]);
    target.setXVelocity(tempCoordinates[i + 1][0]);
    target.setYVelocity(tempCoordinates[i + 1][1]);
  }

  // Runs a simulation of case four with the given parameters.
  public static void simCaseFour(Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix, double[] gridMinMax, double[] velGridMinMax, Sensor target) {
    double[][] tempCoordinates = new double[sensorNetwork.length + 2][2];
    int i;
    for (i = 0; i < sensorNetwork.length; i++) {
      tempCoordinates[i][0] = sensorNetwork[i].getXPosition();
      tempCoordinates[i][1] = sensorNetwork[i].getYPosition();
    }
    tempCoordinates[i][0] = target.getXPosition();
    tempCoordinates[i][1] = target.getYPosition();
    tempCoordinates[i + 1][0] = target.getXVelocity();
    tempCoordinates[i + 1][1] = target.getYVelocity();
    for(int iteration = 0; iteration < Parameters.iterations; iteration++) {
      for (i = 0; i < sensorNetwork.length; ++i) {
        sensorNetwork[i].move(Algorithms.computeAlgTwo(i, sensorNetwork, adjMatrix, target), gridMinMax);
        if(velGridMinMax[3] < Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity()))
          velGridMinMax[3] = Math.sqrt(sensorNetwork[i].getXVelocity() * sensorNetwork[i].getXVelocity() + sensorNetwork[i].getYVelocity() * sensorNetwork[i].getYVelocity());
      }
      double[] targetAcceleration = {50 * Math.cos(2 * Math.PI * iteration / Parameters.iterations), -50 * Math.sin(2 * Math.PI * iteration / Parameters.iterations)};
      target.move(targetAcceleration, gridMinMax);
      adjMatrix.update(sensorNetwork);
    }
    for (i = 0; i < sensorNetwork.length; i++) {
      sensorNetwork[i].setXPosition(tempCoordinates[i][0]);
      sensorNetwork[i].setYPosition(tempCoordinates[i][1]);
      sensorNetwork[i].setXVelocity(0);
      sensorNetwork[i].setYVelocity(0);
      adjMatrix.update(sensorNetwork);
    }
    target.setXPosition(tempCoordinates[i][0]);
    target.setYPosition(tempCoordinates[i][1]);
    target.setXVelocity(tempCoordinates[i + 1][0]);
    target.setYVelocity(tempCoordinates[i + 1][1]);
  }
}
