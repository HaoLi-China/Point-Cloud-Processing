#ifndef COLOR_OP_H
#define COLOR_OP_H

#include <iostream>
#include <vector>

class ColorType{
public:
    ColorType();
    ColorType(int mRed,int mGreen,int mBlue);
    ~ColorType();

public:
    int mRed;
    int mGreen;
    int mBlue;
};

int getUniqueColors(unsigned int count, std::vector<ColorType>& colors,const std::vector<ColorType>& excludeColors);
int getDiffColors(unsigned int count, std::vector<ColorType>& colors);

#endif // COLOR_OP_H

