
// This file includes utility functions for the IR localization

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "opencv2/opencv.hpp"

// Function to find the term-wise average of a Point2f vector
cv::Point2f term_wise_mean_point2f(std::vector<cv::Point2f> im_points){

    float x_bar = 0.0;
    float y_bar = 0.0;
    cv::Point2f vec_bar;

    for(int i = 0; i < (int)im_points.size(); i++)
    {           
        x_bar += im_points[i].x;
        y_bar += im_points[i].y;

    }
    
    x_bar = x_bar/im_points.size();
    y_bar = y_bar/im_points.size();

    vec_bar.x = x_bar;
    vec_bar.y = y_bar;

    return vec_bar;

} // term_wise_mean_point2f()

// Function to get indices of value in vector matching input
std::vector<int> getIndicesOfValue(const std::vector<int>& vec, 
        int targetValue) {

    std::vector<int> indices;

    // Iterate through the vector to find matching values
    for (size_t i = 0; i < vec.size(); ++i) {
        if (vec[i] == targetValue) {
            indices.push_back(static_cast<int>(i));
        }
    }

    return indices;

} // getIndicesOfValue()

// Function to find coordinate distances for matching id values
std::vector<float> computeDistances(int id, const cv::Point2f& inputCoordinates,
        const std::vector<int>& idValues, 
        const std::vector<cv::Point2f>& vectorCoordinates) {
    
    std::vector<float> distances;

    // Iterate through the idValues vector to find matching ids
    for (size_t i = 0; i < idValues.size(); ++i) {

        if (idValues[i] == id) {

            // Distance between inputCoordinates and vectorCoordinates[i]
            float distance = cv::norm(inputCoordinates - vectorCoordinates[i]);
            distances.push_back(distance);

        }

    }

    return distances;

} // Coordinate distance function

// Function to find the minimum element in a vector of floats
int findMinIndex(const std::vector<float>& vec) {

    // Find the iterator pointing to the minimum element
    auto minIterator = std::min_element(vec.begin(), vec.end());

    // Calculate the index of the minimum element using std::distance
    int minIndex = std::distance(vec.begin(), minIterator);

    return minIndex;

} // Minimum element in vector

// Remove elements from a vector at given indices
template <typename T>
void removeElementsAtIndices(std::vector<T>& vec, 
        const std::vector<int>& indicesToRemove) {

    // Sort the indicesToRemove vector in descending order to 
    // prevent invalidating indices
    std::vector<int> sortedIndices(indicesToRemove);
    std::sort(sortedIndices.rbegin(), sortedIndices.rend());

    // Erase elements at specified indices
    for (int index : sortedIndices) {
        if (index >= 0 && static_cast<size_t>(index) < vec.size()) {
            vec.erase(vec.begin() + index);
        }
    }

} // removeElementsAtIndices()

// Function to find repeated values 
std::vector<int> findRepeatedValues(const std::vector<int>& inputVector) {

    std::unordered_map<int, int> frequencyMap;
    std::vector<int> repeatedValues;

    // Count the frequency of each element in the input vector
    for (int num : inputVector) {
        frequencyMap[num]++;
    }

    // Check for repeated values and add them to the result vector
    for (const auto& pair : frequencyMap) {
        if (pair.second > 1) {
            repeatedValues.push_back(pair.first);
        }
    }

    return repeatedValues;

} // findRepeatedValues()

// Function to check if all elements in int vec have same sign
bool haveSameSign(const std::vector<double>& vec) {

    if (vec.size() <= 1) {

        // If the vector has 0 or 1 element, they trivially have the same sign.
        return true;

    }

    double firstElement = vec[0];

    // Iterate through the vector starting from the second element.
    for (size_t i = 1; i < vec.size(); ++i) {

        // Check if the signs are different.
        if ((firstElement >= 0 && vec[i] < 0) || 
            (firstElement < 0 && vec[i] >= 0)) {

            return false; // Signs are different, return false.

        }

    }

    // If the loop completes, all elements have the same sign.
    return true;

} // haveSameSign()

// Function to return element of double vector with maximum magnitude
double getMaxMagnitudeElement(const std::vector<double>& vec) {

    if (vec.empty()) {

        // Handle the case where the vector is empty, return 0
        return 0.0;

    }

    // Initialize with the magnitude of the first element.
    double maxMagnitude = std::abs(vec[0]); 
    double maxElement = vec[0]; // Initialize with the first element.

    // Iterate through the vector starting from the second element.
    for (size_t i = 1; i < vec.size(); ++i) {

        double magnitude = std::abs(vec[i]);

        if (magnitude > maxMagnitude) {

            maxMagnitude = magnitude; // Update the maximum magnitude.
            maxElement = vec[i];      // Update the corresponding element.

        }

    }

    return maxElement;
} // getMaxMagnitudeElement()

// Function to check if two values have matching signs
template <typename T>
bool signsMatch(T value1, T value2) {

    return ((value1 >= 0 && value2 >= 0) || (value1 < 0 && value2 < 0));

} // signsMatch()

// Function to return the max positive and min negative values of a vector
std::pair<double, double> getMinMaxValues(const std::vector<double>& vec) {

    if (vec.empty()) {
        // Handle the case where the vector is empty, return a pair with both 
        // elements set to 0.0.
        return std::make_pair(0.0, 0.0);
    }

    // Initialize max positive with negative infinity
    double maxPositive = -std::numeric_limits<double>::infinity(); 
    // Initialize max negative with positive infinity
    double minNegative = std::numeric_limits<double>::infinity();  

    // Iterate through the vector.
    for (const double& value : vec) {

        if (value > 0 && value > maxPositive) {
            maxPositive = value; // Update the maximum positive value.
        } else if (value < 0 && value < minNegative) {
            minNegative = value; // Update the minimum negative value.
        }

    }

    return std::make_pair(maxPositive, minNegative);
} // getMinMaxValues()

// Function to bound a value +- by another value
template<typename T>
void boundByMagnitude(T& value, T bound) {

    // Ensure the bound is positive
    T positiveBound = (bound < 0) ? -bound : bound;

    // Bound the value in positive direction
    if (value > positiveBound) {
        value = positiveBound;
    }

    // Bound the value in negative direction
    if (value < -positiveBound) {
        value = -positiveBound;
    }

} // boundByMagnitude()








