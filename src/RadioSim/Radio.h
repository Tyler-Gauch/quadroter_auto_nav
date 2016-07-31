#ifndef _RADIO_
#define _RADIO_

class Radio{
public:
	Radio();
	int getMin();
	int getMax();
	int getTrim();
	int getOutput();
	void setMin(int min);
	void setMax(int max);
	void setTrim(int trim);
	void setOutput(int output);
private:
	int max;
	int min;
	int trim;
	int output;
};

#endif