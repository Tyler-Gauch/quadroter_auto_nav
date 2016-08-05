#include "Radio.h"

Radio::Radio(){
	max = 0;
	min = 0;
	trim = 0;
}

int Radio::getMin(){
	return min;
}
int Radio::getMax(){
	return max;
}
int Radio::getTrim(){
	return trim;
}
int Radio::getOutput(){
	return output;
}
void Radio::setMin(int min){
	this->min = min;
}
void Radio::setMax(int max){
	this->max = max;
}
void Radio::setTrim(int trim){
	this->trim = trim;
}
void Radio::setOutput(int output){
	if(output > this->max)
	{
		output == max;
	}else if(output < this->min)
	{
		output == min;
	}
	
	this->output = output;
}
