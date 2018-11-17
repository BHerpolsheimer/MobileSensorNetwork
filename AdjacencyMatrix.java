// Brandon Herpolsheimer
// February 2, 2018
// CS 455
// Adjacency matrix class to store the adjacency values of the mobile sensor network.

import java.io.*;

public class AdjacencyMatrix {
	private double[][] adjMatrix;

    // Parameterized constructor for the adjacency matrix.
	public AdjacencyMatrix(int sensorNumber) {
		adjMatrix = new double[sensorNumber][sensorNumber];

		for(int i = 0; i < sensorNumber; i++)
			for(int j = 0; j < sensorNumber; j++)
        setValue(i, j, 0);
	}

    // Parameterized constructor for an adjacency matrix of beta adgents.
	public AdjacencyMatrix(int sensorNumber, int obstacleNumber) {
		adjMatrix = new double[sensorNumber][obstacleNumber];

		for(int i = 0; i < sensorNumber; i++)
			for(int j = 0; j < obstacleNumber; j++)
        setValue(i, j, 0);
	}

    // Updates the adjacency matrix based off of the current positions of the sensors.
	public void update(Sensor[] sensorNetwork) {
		for(int i = 0; i < sensorNetwork.length; i++)
			for(int j = 0; j < sensorNetwork.length; j++) {
				if(j == i) {
					setValue(i, j, 0);
					continue;
				}
				setValue(i, j, Algorithms.computeBump(Algorithms.computeSigmaNorm(sensorNetwork[i], sensorNetwork[j]) / 
					                                    Algorithms.computeSigmaNorm(Parameters.interactionRange)));
			}
	}

    // Updates the beta matrix based off of the current positions of the sensors.
	public void update(Sensor[] sensorNetwork, Sensor[][] betaNetwork) {
		for(int i = 0; i < sensorNetwork.length; i++)
			for(int j = 0; j < betaNetwork[0].length; j++) {
				setValue(i, j, Algorithms.computeBump(Algorithms.computeSigmaNorm(sensorNetwork[i], betaNetwork[i][j]) / 
					                                    Algorithms.computeSigmaNorm(Parameters.interactionRange)));
			}
	}

    // Returns a value within the adjacency matrix.
	public double getValue(int i, int j) {
		return adjMatrix[i][j];
	}

    // Sets a value within the adjacency matrix.
	public void setValue(int i, int j, double adjValue) {
     adjMatrix[i][j] = adjValue;
	}

	// For debugging purposes.
	public void print() {
		for(int i = 0; i < adjMatrix[0].length; i++) {
			for(int j = 0; j < adjMatrix[0].length; j++)
				System.out.print(getValue(i, j) +  " ");
			System.out.print("\n");
		}
	}
}