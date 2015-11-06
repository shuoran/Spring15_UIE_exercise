/*
*     Gesture.hpp
*
*     This class implements a gesture
*     to be used with the 1$ recognizer
*     Wobbrock, J.O., Wilson, A.D. and Li, Y. -- UIST 2007
*     (http://faculty.washington.edu/wobbrock/pubs/uist-07.1.pdf)
*
*     Author: Fabrizio Pece, ETH Zurich
*     fabrizio.pece@inf.ethz.ch
*
*/

#pragma once

#include <vector>
#include <cmath>
#include <cfloat>
#include <sstream>
#include <limits>
#include <algorithm>
using namespace std;

#define METHOD 0 // 0 for $1, 1 for protractor

//! A 2D point
/*!
*/
struct Point2D
{
  double x;
  double y;

  Point2D()
  {
    x = 0.0f;
    y = 0.0f;
  }

  Point2D(double _x, double _y)
  {
    x = _x;
    y = _y;
  }

  //! Computes the Euclidean distance from a point.
  /*!
  \param pt Point to which the distance is computed
  \return The Euclidean distance
  */
  double distance(Point2D pt)
  {
    double dx = pow(x - pt.x,2);
    double dy = pow(y - pt.y,2);
    double s = dx + dy;
    double ret = s < std::numeric_limits<double>::epsilon() ? (std::numeric_limits<double>::epsilon()) : sqrt(s);
    return ret;
  }

  Point2D operator -(const Point2D& rOther) {
    Point2D tmp;
    tmp.x = x - rOther.x;
    tmp.y = y - rOther.y;
    return tmp;
  }

  Point2D operator +(const Point2D& rOther) {
    Point2D tmp;
    tmp.x = x + rOther.x;
    tmp.y = y + rOther.y;
    return tmp;
  }
  
  Point2D operator =(const Point2D& rOther) {
    x = rOther.x;
    y = rOther.y;
  }

  Point2D operator *(const double& d) {
    return Point2D(x * d, y * d);
  }
  
  friend std::ostream& operator <<(std::ostream& os, const Point2D& pt) {
    os << pt.x << ';' << pt.y << ';';
    return os;
  }

};

//! A 2D rectangle
/*!
*/
struct Rect {
  double x, y, w, h;
  Rect(){}
  Rect(double _x, double _y, double _w, double _h)
  {
    x = _x;
    y = _y;
    w = _w;
    h = _h;
  }
};

//! Gesture class.
//! Implements a Gesture to be used with the $1 recognizer.
/*!
*/
class Gesture
{
public:

  //! Constructor
  /*!
  \param _name The Gesture name. can be left empty, and set later with setName.
  */
  Gesture(std::string _name = std::string("dummy")) :
    name(_name)
  {
    golden_ratio = 0.5 * (-1.0 + sqrt(5.0));
    angle_precision = 1.0;
  }

  //! Destructor
  /*!
  */
  ~Gesture(){}

  //! Normalizes a gesture given the Step 1--3 in the paper.
  /*!
  \param numResampled Number of points to use for sequence resampling. Should be
         The same as the one used for the templates.
  \param squareSize Size of the square to use for normalization. Should be the
         same as the one used for the templates.
  */
  void normaliseGesture(int numResampled, int squareSize);

  //! Clears the current gesture.
  //! It empties the point vectors, and resets the name.
  /*!
  */
  void clear();

  //! Coputes the distance at best angle, as in Step 4 of the paper
  /*!
  \param gest Gesture to recognize
  \return The distance between gest and the gesture
  */
  double distanceAtBestAngle(Gesture gest);

  //! Returns the vector of points for the gesture
  /*!
  \return A vector of Point2D that form the gesture
  */
  inline std::vector<Point2D> getPointVector(){return current_points;}

  //! Returns the name of the gesture
  /*!
  \return The gesture name
  */
  inline std::string getName(){ return name; }

  //! Adds points to the current list of points
  /*!
  \param _pt the Point2D to add
  */
  inline void addPoint(Point2D _pt) { current_points.push_back(_pt); }

  //! Sets the name of the gesture
  /*!
  \param _name the name to use for the gesture
  */
  inline void setName(std::string _name){ name = std::string(_name); }

  friend std::ostream& operator <<(std::ostream& os, const Gesture& rGesture) {
    os << rGesture.name.c_str() << ' ';
    std::vector<Point2D>::const_iterator it = rGesture.current_points.begin();
    while(it != rGesture.current_points.end()) {
      os << (*it).x << ';' << (*it).y << ';';
      ++it;
    };
    return os;
  }

  friend std::istream& operator >>(std::istream& is, Gesture& rGesture) {
    double x,y;
    std::istringstream ss;
    is >> rGesture.name;
    while(is) {
      is >> x; is.ignore(1); is >> y; is.ignore(1);
      if(is) {
        rGesture.current_points.push_back(Point2D(x,y));
      }
    }
    return is;
  }


private:

  //! Resamples the point vector as per Step 1 of the paper
  /*!
  \param path The path to resmaple
  \param n The number of points to keep
  \return The resampled vector of Point2D
  */
  std::vector<Point2D> resample(std::vector<Point2D> path, int n);

  //! Rotates (in place) the gesture points to zero angle as per Step 2 of the paper
  /*!
  \param path The path to rotate (in place)
  */
  void rotateToZero(std::vector<Point2D>& path);

  //! Rotates (in place) a gesture path of a given angle
  /*!
  \param path The path to rotate (in place)
  \param C The centroid of the path
  \param theta The angle of rotation
  */
  void rotateBy(std::vector<Point2D>& path, Point2D C, double theta);

  //! Translates (in place) to zero a gesture, as per Step 3 of the paper
  /*!
  \param path The paths to translate to zero (in place)
  */
  void translateToOrigin(std::vector<Point2D>& path);

  //! Scales (in place) a gesture path to a given square size, as per Step 3 of the paper
  /*!
  \param path Gesture path to scale (in place)
  \param nSize Sqaure size
  */
  void scaleToSquare(std::vector<Point2D>& path, double nSize);

  //! Computes the distance at a given angle from a Gesture, as per Step 4 of the paper
  /*!
  \param nAngle The angle to test
  \param gesture The gesture to which compute the distance
  \return The distance at a given angle of the gesture
  */
  double distanceAtAngle(double nAngle, Gesture gesture);

  //! Computes the centroid of a collection of Point2D
  /*!
  \param path The collection of points for which compute the centroid
  \return The centroid
  */
  Point2D centroid(std::vector<Point2D> path);

  //! Compares a bounding box of a gesture path
  /*!
  \param path The gesture path
  \return The bounding box enclosing the path
  */
  Rect boundingBox(const std::vector<Point2D>& path);

  //! Computes a path lenght
  /*!
  \param path The path of which the length needs to be computed
  \return The length
  */
  double pathLength(std::vector<Point2D> path);

  //! Computes the distance of a gesture from a path, as per step 4 of the paper
  /*!
  \param path The target path
  \param gesture The gesture to from which the distance is computed
  \return The distance of the gesture to the path
  */
  double pathDistance(std::vector<Point2D> path, Gesture gesture);

  //! The points forming the gesture
  std::vector<Point2D> current_points;
  //! The gesgture name
  std::string name;
  //! The golden ration
  double golden_ratio;
  //! The angle precision to use to compute distances
  double angle_precision;
};

void Gesture::normaliseGesture(int numResampled, int squareSize)
{

  /**
  YOUR CODE HERE
  **/
  current_points = resample(current_points, numResampled);
  rotateToZero(current_points);
  scaleToSquare(current_points, squareSize);
  translateToOrigin(current_points);
}

void Gesture::clear()
{
  current_points.erase(current_points.begin(), current_points.end());
  name = std::string("dummy");
}

double Gesture::pathLength(std::vector<Point2D> path)
{
  double dist = 0;

  /**
  YOUR CODE HERE
  **/

  for (int i = 1; i < path.size(); i ++) {
      dist += path[i].distance(path[i - 1]);
  }
  return dist;
}

std::vector<Point2D> Gesture::resample(std::vector<Point2D> path, int n) {

  std::vector<Point2D> resampled_points;

  /**
  YOUR CODE HERE
  **/

  double I = pathLength(path);
  double D = 0;
  resampled_points.push_back(path[0]);
  for (int i = 1; i < path.size(); i ++) {
      double d = path[i].distance(path[i - 1]);
      if (D + d >= I) {
	  Point2D q = path[i - 1] + (path[i] - path[i - 1]) * ((I - D) / d);
	  resampled_points.push_back(q);
	  path.insert(resampled_points.begin() + i, q);
	  D = 0;
      }
      else D = D + d;
  }
  return resampled_points;
}

Point2D Gesture::centroid(std::vector<Point2D> path) {

  double x = 0;
  double y = 0;

  /**
  YOUR CODE HERE
  **/
  for (int i = 0; i < path.size(); i ++) {
    x += path[i].x;
    y += path[i].y;
  }
  x /= path.size();
  y /= path.size();
  return Point2D(x,y);
}

void Gesture::rotateToZero(std::vector<Point2D>& path)
{

  /**
  YOUR CODE HERE
  **/
  Point2D c = centroid(path);
  double theta = atan((path[0].y - c.y) / (path[0].x - c.x));
  rotateBy(path, c, theta);
}

void Gesture::rotateBy(std::vector<Point2D>& path, Point2D C, double theta)
{

  /**
  YOUR CODE HERE
  **/
  for (int i = 0; i < path.size(); i ++) {
      double x = (path[i].x - C.x) * cos(theta) - (path[i].y - C.y) * sin(theta) + C.x;
      double y = (path[i].x - C.x) * sin(theta) + (path[i].y - C.y) * cos(theta) + C.y;
      path[i] = Point2D(x, y);
  }
}

void Gesture::translateToOrigin(std::vector<Point2D>& path) {

  /**
  YOUR CODE HERE
  **/
  Point2D c = centroid(path);
  for (int i = 0; i < path.size(); i ++) {
      path[i] = path[i] - c;
  }
}

Rect Gesture::boundingBox(const std::vector<Point2D>& path) {

  double min_x = (double)FLT_MAX;
  double min_y = (double)FLT_MAX;
  double max_x = (double)FLT_MIN;
  double max_y = (double)FLT_MIN;

  /**
  YOUR CODE HERE
  **/

  for (int i = 0; i < path.size(); i ++) {
      min_x = min(min_x, path[i].x);
      min_y = min(min_y, path[i].y);
      max_x = max(max_x, path[i].x);
      max_y = max(max_y, path[i].y);
  }
  return Rect(min_x, min_y, (max_x - min_x), (max_y - min_y));
}

void Gesture::scaleToSquare(std::vector<Point2D>& path, double nSize) {

  /**
  YOUR CODE HERE
  **/
  Rect rt = boundingBox(path);
  for (int i = 0; i < path.size(); i ++) {
	path[i].x *= nSize / rt.w;
	path[i].y *= nSize / rt.h;
  }
}

double Gesture::distanceAtBestAngle(Gesture gest)
{

  /**
  YOUR CODE HERE
  **/
     if (METHOD == 0) { // Original $1 implementation
	  double theta_a = -45;
	  double theta_b = 45;
	  double x1 = golden_ratio * theta_a + (1 - golden_ratio) * theta_b; 
	  double f1 = distanceAtAngle(x1, gest);
	  double x2 = (1 - golden_ratio) * theta_a + golden_ratio * theta_b;
	  double f2 = distanceAtAngle(x2, gest);
	  while (abs(theta_a - theta_b) > angle_precision) {
	      if (f1 < f2) {
		  theta_b = x2;
		  x2 = x1;
		  f2 = f1;
		  x1 = golden_ratio * theta_a + (1 - golden_ratio) * theta_b;
		  f1 = distanceAtAngle(x1, gest);
	      }
	      else {
		  theta_a = x1;
		  x1 = x2;
		  f1 = f2;
		  x2 = (1 - golden_ratio) * theta_a + golden_ratio * theta_b;
		  f2 = distanceAtAngle(x2, gest);
	      }
	  }
	  return std::min(f1, f2);
      }
      else { // protractor
	  double a = 0;
	  double b = 0;
	  double tx, ty, gx, gy;
	  for (int i = 0; i < current_points.size(); i ++) {
	      tx = current_points[i].x;
	      ty = current_points[i].y;
	      gx = (gest.current_points)[i].x;
	      gy = (gest.current_points)[i].y;
	      a += tx * gx + ty * gy;
	      b += tx * gy - ty * gx;
	  }
	  return distanceAtAngle(atan(b / a), gest);
      }
}

double Gesture::distanceAtAngle(double nAngle, Gesture gesture) {

  /**
  YOUR CODE HERE
  **/
  rotateBy(current_points, centroid(current_points), nAngle / 180 * 3.14);
  return pathDistance(current_points, gesture);
}

double Gesture::pathDistance(std::vector<Point2D> path, Gesture gesture)
{

  /**
  YOUR CODE HERE
  **/
  double d = 0;
  for (int i = 0; i < path.size(); i ++) {
      d += path[i].distance(gesture.current_points[i]);
  }
  return d / path.size();

}
