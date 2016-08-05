#include "QuadVector.h"

QuadVector::QuadVector(){
	x = 0;
	y = 0;
	z = 0;
}

QuadVector::QuadVector(geometry_msgs::Point a)
{
	x = a.x;
	y = a.y;
	z = a.z;
}

QuadVector::QuadVector(geometry_msgs::Quaternion q)
{
	x = q.x;
	y = q.y;
	z = q.z;
}

QuadVector QuadVector::add(QuadVector b){
	QuadVector c;
	c.x = x + b.x;
	c.y = y + b.y;
	c.z = z + b.z;
	return c;
}

QuadVector QuadVector::cross(QuadVector b){
	QuadVector c;
	c.x = (y * b.z) - (z * b.y);
	c.y = (z * b.x) - (x * b.z);
	c.z = (x * b.y) - (y * b.x);
	return c;
}

QuadVector QuadVector::mult(float scalar){
	geometry_msgs::Point c;
	c.x = x * scalar;
	c.y = y * scalar;
	c.z = z * scalar;
	return c;
}

float QuadVector::dot(QuadVector b){
	return (x * b.x) + (y * b.y) + (z * b.z);
}

QuadVector QuadVector::rotate(geometry_msgs::Quaternion q){
	QuadVector quatVector(q);
	float scalar = q.w;

	QuadVector a = quatVector.mult(2 * this->dot(quatVector));
	QuadVector b = this->mult((scalar * scalar) - quatVector.dot(quatVector));
	QuadVector c = this->cross(quatVector).mult(2*scalar);

	return a.add(b).add(c);
	
	// geometry_msgs::Point a = vectorMult(quatVector, 2 * vectorDot(quatVector, p));
	// geometry_msgs::Point b = vectorMult(p, scalar*scalar - vectorDot(quatVector, quatVector));
	// geometry_msgs::Point c = vectorMult(vectorCross(quatVector, p), 2*scalar);

	// return vectorAdd(vectorAdd(a, b), c);
}