// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Algorithm implemetations for the movement of the mobile sensor network.

import java.io.*;

public class Algorithms {
  // Computes the euclidean distance between two sensors.
  public static double computeEuclidean(Sensor s1, Sensor s2) {
    return Math.sqrt((s1.getXPosition() - s2.getXPosition()) * 
                     (s1.getXPosition() - s2.getXPosition()) + 
                     (s1.getYPosition() - s2.getYPosition()) * 
                     (s1.getYPosition() - s2.getYPosition())); 
  }

  // Computes the sigma norm of the position of two sensors.
  public static double computeSigmaNorm(Sensor s1, Sensor s2) {
    return (1/Parameters.epsilon) * (Math.sqrt(1 + (Parameters.epsilon * computeEuclidean(s1, s2) * computeEuclidean(s1, s2))) - 1);
  }

  // Computes the sigma norm of a given distance.
  public static double computeSigmaNorm(double input) {
    return (1/Parameters.epsilon) * (Math.sqrt(1 + (Parameters.epsilon * input * input)) - 1);
  }

  // Computes the bump function with respect to a given number.
  public static double computeBump(double input) {
    double h = 0.2;
    if(input >= 0 && input < h)
      return 1;
    else if(input >= h && input <= 1)
      return (0.5) * (1 + Math.cos(Math.PI * (input - h) / (1 - h)));
    else
      return 0;
  }

  // Computes the acceleration of a given sensor using algorithm three.
  public static double[] computeAlgThree(int i, Sensor[] sensorNetwork, Sensor[][] betaNetwork, AdjacencyMatrix adjMatrix, AdjacencyMatrix betaMatrix, Sensor target) {
  	double[] result = computeAlgTwo(i, sensorNetwork, adjMatrix, target);
  	double sum1x = 0, sum1y = 0;
  	double sum2x = 0, sum2y = 0;
    for(int j = 0; j < betaNetwork[i].length; j++)
      if(betaMatrix.getValue(i, j) > 0) {
        sum1x += computePhiAlpha(computeSigmaNorm(sensorNetwork[i], betaNetwork[i][j])) * 
                 computeNijX(sensorNetwork[i], betaNetwork[i][j]);
        sum1y += computePhiAlpha(computeSigmaNorm(sensorNetwork[i], betaNetwork[i][j])) * 
                 computeNijY(sensorNetwork[i], betaNetwork[i][j]);
        sum2x += betaMatrix.getValue(i, j) * (betaNetwork[i][j].getXVelocity() - sensorNetwork[i].getXVelocity());
        sum2y += betaMatrix.getValue(i, j) * (betaNetwork[i][j].getYVelocity() - sensorNetwork[i].getYVelocity());
      }
    result[0] += 120 * sum1x +  2 * Math.sqrt(75) * sum2x;
    result[1] += 120 * sum1y +  2 * Math.sqrt(75) * sum2y;
  	return result;
  }

  // Computes the acceleration of a given sensor using algorithm two.
  public static double[] computeAlgTwo(int i, Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix, Sensor target) {
  	double[] result = computeAlgOne(i, sensorNetwork, adjMatrix);
  	result[0] -= (Parameters.c1mt * (sensorNetwork[i].getXPosition() - target.getXPosition()) +
  		            Parameters.c2mt * (sensorNetwork[i].getXVelocity() - target.getXVelocity()));
    result[1] -= (Parameters.c1mt * (sensorNetwork[i].getYPosition() - target.getYPosition()) +
  		            Parameters.c2mt * (sensorNetwork[i].getYVelocity() - target.getYVelocity()));
  	return result;
  }

  // Computes the acceleration of a given sensor using algorithm one.
  public static double[] computeAlgOne(int i, Sensor[] sensorNetwork, AdjacencyMatrix adjMatrix) {
  	double sum1x = 0, sum1y = 0;
  	double sum2x = 0, sum2y = 0;
    for(int j = 0; j < sensorNetwork.length; j++)
      if(adjMatrix.getValue(i, j) > 0) {
        sum1x += computePhiAlpha(computeSigmaNorm(sensorNetwork[i], sensorNetwork[j])) * 
                 computeNijX(sensorNetwork[i], sensorNetwork[j]);
        sum1y += computePhiAlpha(computeSigmaNorm(sensorNetwork[i], sensorNetwork[j])) * 
                 computeNijY(sensorNetwork[i], sensorNetwork[j]);
        sum2x += adjMatrix.getValue(i, j) * (sensorNetwork[j].getXVelocity() - sensorNetwork[i].getXVelocity());
        sum2y += adjMatrix.getValue(i, j) * (sensorNetwork[j].getYVelocity() - sensorNetwork[i].getYVelocity());
      }

    double[] result = {Parameters.c1a * sum1x + Parameters.c2a * sum2x,
                       Parameters.c1a * sum1y + Parameters.c2a * sum2y};
    return result;
  }

  // Computes nijx for use in the algorithms above.
  public static double computeNijX(Sensor s1, Sensor s2) {
    return (s2.getXPosition() - s1.getXPosition()) / 
           Math.sqrt(1 + Parameters.epsilon * computeEuclidean(s1, s2) * computeEuclidean(s1, s2));
  }

  //Computes nijy for use in the algorithms above.
  public static double computeNijY(Sensor s1, Sensor s2) {
    return (s2.getYPosition() - s1.getYPosition()) / 
           Math.sqrt(1 + Parameters.epsilon * computeEuclidean(s1, s2) * computeEuclidean(s1, s2));
  }

  // Computees PhiAlpha for use in the algorithms above.
  public static double computePhiAlpha(double input) {
    return computeBump(input / computeSigmaNorm(Parameters.interactionRange)) *
    	     computePhi(input - computeSigmaNorm(Parameters.desiredDistance));
  }

  // Computes Phi for use in the algorithms above.
  public static double computePhi(double input) {
    return (0.5) * ((Parameters.a + Parameters.b) * 
    	     computeSigmaOne(input + Parameters.c) + (Parameters.a - Parameters.b));
  }

  // Computes SigmaOne for use in the algorithms above.
  public static double computeSigmaOne(double input) {
  	return input / Math.sqrt(1 + input * input);
  }

  // Computes connectivity.
  public static double computeConnectivity(AdjacencyMatrix adjMatrix) {
  	return (1.0 / Parameters.sensorNumber) * computeRank(adjMatrix);
  }

  // Computes the rank of the given adjacency matrix.
  public static int computeRank(AdjacencyMatrix adjMatrix) {
    int rank = Parameters.sensorNumber;
     
    for (int i = 0; i < rank; i++) {
      if (adjMatrix.getValue(i, i) != 0) {
        for (int j = 0; j < Parameters.sensorNumber; j++) {
          if (j != i) {
            double mult = adjMatrix.getValue(j, i) / adjMatrix.getValue(i, i);
                           
            for (int k = 0; k < rank; k++)
              adjMatrix.setValue(j, k, adjMatrix.getValue(j, k) - mult * adjMatrix.getValue(i, k));
          }
        }
      }
      else
      {
        boolean reduce = true;

        for (int j = i + 1; j < Parameters.sensorNumber; j++) {
          if (adjMatrix.getValue(j, i) != 0) {
            swap(adjMatrix, i, j, rank);
            reduce = false;
            break ;
          }
        }

        if (reduce)
        {
          rank--;

          for (int j = 0; j < Parameters.sensorNumber; j++)
            adjMatrix.setValue(j, i, adjMatrix.getValue(j, rank));
        }
        i--;
      }
    }
         
    return rank;
  }

  // Swaps two values in a given matrix.
  public static void swap(AdjacencyMatrix adjMatrix, int row1, int row2, int col) {
    for (int i = 0; i < col; i++)
    {
      double temp = adjMatrix.getValue(row1, i);
      adjMatrix.setValue(row1, i, adjMatrix.getValue(row2, i));
      adjMatrix.setValue(row2, i, temp);
    }
  }

  // Computes the x value of the center of mass.
  public static double computeXCoM(Sensor[] sensorNetwork) {
    double sum = 0;
    for(int i = 0; i < sensorNetwork.length; i++)
    	sum += sensorNetwork[i].getXPosition();
    return sum / sensorNetwork.length;
  }

  // Computes the y values of the center of mass.
  public static double computeYCoM(Sensor[] sensorNetwork) {
    double sum = 0;
    for(int i = 0; i < sensorNetwork.length; i++)
    	sum += sensorNetwork[i].getYPosition();
    return sum / sensorNetwork.length;
  }

  // Computes Mu for use in obstacle avoidance.
  public static double computeMu(Sensor s1, RoundObstacle o1) {
  	return o1.getRadius() / computeEuclidean(s1, new Sensor(o1.getXPosition(), o1.getYPosition(), 0, 0));
  }

  // Computer Rho for use in obstacle avoidance.
  public static double computeRho(Sensor s1, RoundObstacle o1) {
  	double akx = (s1.getXPosition() - o1.getXPosition()) / computeEuclidean(s1, new Sensor(o1.getXPosition(), o1.getYPosition(), 0, 0));
  	double aky = (s1.getXPosition() - o1.getXPosition()) / computeEuclidean(s1, new Sensor(o1.getXPosition(), o1.getYPosition(), 0, 0));
  	return 1 - (akx * akx + aky * aky);
  }
}