// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Class to store the parameters for the simulations.

public class Parameters { 
  public static int sensorNumber = 100;
  public static int dimensions = 2;
  public static double desiredDistance = 15;
  public static double scalingFactor = 1.2;
  public static double interactionRange = desiredDistance * scalingFactor;
  public static double epsilon = 0.1;
  public static double delta_t = 0.009;

  public static double c1a = 30;
  public static double c2a = 2 * Math.sqrt(c1a);
  public static double a = 5;
  public static double b = 5;
  public static double c = Math.abs(a - b) / Math.sqrt(4 * a * b);

  public static double c1mt = 1.1;
  public static double c2mt  = 2 * Math.sqrt(c1mt);

  public static int iterations = 900;
}