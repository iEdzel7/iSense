#include <Ultrasonic.h>

#define MidtrigPin 13 // MidTrigpin of Ultrasonic Sensor
#define MidechoPin 12 // MidEchopin of Ultrasonic Sensor
#define LefttrigPin 11 // LeftTrigpin of Ultrasonic Sensor
#define LeftechoPin 10 // LeftEchopin of Ultrasonic Sensor
#define RighttrigPin 9 // RightTrigpin of Ultrasonic Sensor
#define RightechoPin 8 // RightEchopin of Ultrasonic Sensor

int speakerPin = 7;
int midVibratorPin = 6; // Pin connected to the vibrator for MidSensor
int leftVibratorPin = 5; // Pin connected to the vibrator for LeftSensor
int rightVibratorPin = 4; // Pin connected to the vibrator for RightSensor
int piezoBuzzerPin = 3; // Pin connected to the piezo buzzer

// Create instances of Ultrasonic for each sensor
Ultrasonic MidSensor(MidtrigPin, MidechoPin);
Ultrasonic LeftSensor(LefttrigPin, LeftechoPin);
Ultrasonic RightSensor(RighttrigPin, RightechoPin);

int lastObstacleAvoided = 0; // Variable to store the last obstacle avoided

// Memoization table to store solutions to subproblems
int memo[3][3][3];

// Function prototypes
void deactivateVibrator(int vibratorPin);
void avoidLeftObstacle();
void avoidMidObstacle();
void avoidRightObstacle();
int getClosestObstacle(float MidDistance, float LeftDistance, float RightDistance);
void MidprintDistance(float MidDistance);
void LeftprintDistance(float LeftDistance);
void RightprintDistance(float RightDistance);
void activateVibrator(int vibratorPin);
void activateBuzzer(int frequency, int duration);

void setup() {
  Serial.begin(9600);

  pinMode(MidtrigPin, OUTPUT);   // Mid Ultrasonic Sensor
  pinMode(MidechoPin, INPUT);    // Mid Ultrasonic Sensor
  pinMode(LefttrigPin, OUTPUT);  // Left Ultrasonic Sensor
  pinMode(LeftechoPin, INPUT);   // Left Ultrasonic Sensor
  pinMode(RighttrigPin, OUTPUT); // Right Ultrasonic Sensor
  pinMode(RightechoPin, INPUT);  // Right Ultrasonic Sensor
  pinMode(speakerPin, OUTPUT);
  pinMode(midVibratorPin, OUTPUT);
  pinMode(leftVibratorPin, OUTPUT);
  pinMode(rightVibratorPin, OUTPUT);
  pinMode(piezoBuzzerPin, OUTPUT);

}

void loop() {
  float MidDistance, LeftDistance, RightDistance;

  // Measure distances from sensors
  MidDistance = getDistance(MidSensor);
  LeftDistance = getDistance(LeftSensor);
  RightDistance = getDistance(RightSensor);

  // Handle obstacle avoidance using greedy algorithm
  greedyPathPlanning(MidDistance, LeftDistance, RightDistance);

  // Print sensor distances
  MidprintDistance(MidDistance);
  LeftprintDistance(LeftDistance);
  RightprintDistance(RightDistance);

  // Delay for stability
  delay(250);
}

unsigned int getDistance(Ultrasonic sensor) {
  // Measure distance using the Ultrasonic sensor
  return sensor.read();
}

void greedyPathPlanning(float MidDistance, float LeftDistance, float RightDistance) {
  // Real-time obstacle avoidance using Greedy algorithm
  if (MidDistance < 40) {
    activateVibrator(midVibratorPin);
    activateBuzzer(2000, 1000); // Buzz at a rate of 2000 Hz for 1000 milliseconds
  } else {
    deactivateVibrator(midVibratorPin);
  }

  if (LeftDistance < 40) {
    activateVibrator(leftVibratorPin);
    activateBuzzer(2000, 1000); // Buzz at a rate of 2000 Hz for 1000 milliseconds
  } else {
    deactivateVibrator(leftVibratorPin);
  }

  if (RightDistance < 40) {
    activateVibrator(rightVibratorPin);
    activateBuzzer(2000, 1000); // Buzz at a rate of 2000 Hz for 1000 milliseconds
  } else {
    deactivateVibrator(rightVibratorPin);
  }

  // Identify the closest obstacle using dynamic programming (memoization)
  int closestObstacle = getClosestObstacle(MidDistance, LeftDistance, RightDistance);

  // Check if the closest obstacle has changed
  if (closestObstacle != lastObstacleAvoided) {
    // Avoid the obstacle based on its location
    if (closestObstacle == 1) {
      // Obstacle is on the left
      avoidLeftObstacle();
    } else if (closestObstacle == 2) {
      // Obstacle is in the middle
      avoidMidObstacle();
    } else if (closestObstacle == 3) {
      // Obstacle is on the right
      avoidRightObstacle();
    }

    // Update the last obstacle avoided
    lastObstacleAvoided = closestObstacle;
  }
}

void deactivateVibrator(int vibratorPin) {
  digitalWrite(vibratorPin, LOW);
}

int getClosestObstacle(float MidDistance, float LeftDistance, float RightDistance) {
  const float threshold = 40.0;

  // Convert distances to integer indices for memoization
  int midIndex = int(MidDistance);
  int leftIndex = int(LeftDistance);
  int rightIndex = int(RightDistance);

  // Check if the solution to the subproblem is already memoized
  if (memo[midIndex][leftIndex][rightIndex] == 0) {
    // Calculate and memoize the solution
    if (LeftDistance < threshold && LeftDistance <= MidDistance && LeftDistance <= RightDistance) {
      memo[midIndex][leftIndex][rightIndex] = 1; // Memoize solution
      return 1; // Left sensor detects the closest obstacle
    } else if (MidDistance < threshold && MidDistance <= LeftDistance && MidDistance <= RightDistance) {
      memo[midIndex][leftIndex][rightIndex] = 2; // Memoize solution
      return 2; // Mid sensor detects the closest obstacle
    } else if (RightDistance < threshold && RightDistance <= LeftDistance && RightDistance <= MidDistance) {
      memo[midIndex][leftIndex][rightIndex] = 3; // Memoize solution
      return 3; // Right sensor detects the closest obstacle
    } else {
      memo[midIndex][leftIndex][rightIndex] = 0; // No obstacle within the threshold
      return 0;
    }
  } else {
    return memo[midIndex][leftIndex][rightIndex]; // Return memoized solution
  }
}

void avoidLeftObstacle() {
  // Avoidance logic for obstacles on the left...
  activateVibrator(leftVibratorPin);
  activateBuzzer(2000, 1000);
}

void avoidMidObstacle() {
  // Avoidance logic for obstacles in the middle...
  activateVibrator(midVibratorPin);
  activateBuzzer(2000, 1000);
}

void avoidRightObstacle() {
  // Avoidance logic for obstacles on the right...
  activateVibrator(rightVibratorPin);
  activateBuzzer(2000, 1000);
}

void MidprintDistance(float MidDistance) {
  Serial.print("Distance of Mid sensor: ");
  Serial.print(MidDistance);
  Serial.println(" cm");
  delay(250);
}

void LeftprintDistance(float LeftDistance) {
  Serial.print("Distance of Left sensor: ");
  Serial.print(LeftDistance);
  Serial.println(" cm");
  delay(250);
}

void RightprintDistance(float RightDistance) {
  Serial.print("Distance of Right sensor: ");
  Serial.print(RightDistance);
  Serial.println(" cm");
  delay(250);
}

void activateVibrator(int vibratorPin) {
  for (int i = 0; i < 10; i++) {
    digitalWrite(vibratorPin, HIGH);
    delay(50); // Vibrate for 50 milliseconds
    digitalWrite(vibratorPin, LOW);
    delay(25); // Pause for 25 milliseconds
  }
}

void activateBuzzer(int frequency, int duration) {
  int buzzRate = 100; // Buzz at a rate of 100 milliseconds per cycle

  for (int i = 0; i < duration / buzzRate; i++) {
    tone(piezoBuzzerPin, frequency);
    delay(buzzRate / 2); // Buzz for half of the specified duration
    noTone(piezoBuzzerPin); // Turn off the buzzer
    delay(buzzRate / 2); // Pause for the remaining duration
  }
}
