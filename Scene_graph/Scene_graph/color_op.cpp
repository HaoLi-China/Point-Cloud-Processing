#include "color_op.h"

ColorType::ColorType(){
}

ColorType::ColorType(int mRed,int mGreen,int mBlue){
  this->mRed=mRed;
  this->mGreen=mGreen;
  this->mBlue=mBlue;
}

ColorType::~ColorType(){
}


int getDiffColors(unsigned int count, std::vector<ColorType>& colors){

  std::vector<ColorType> baseColors;
  baseColors.push_back(ColorType(0,0,255));
  baseColors.push_back(ColorType(255,0,0));
  baseColors.push_back(ColorType(0,255,0));
  baseColors.push_back(ColorType(0,255,255));
  baseColors.push_back(ColorType(255,255,0));
  baseColors.push_back(ColorType(255,0,255));
  baseColors.push_back(ColorType(255,255,255));
  baseColors.push_back(ColorType(0,0,255));

  if(count<=7){
    for (int k = 0; k < count; k++){
      ColorType colorToInsert;
      colorToInsert.mRed = baseColors[k].mRed;
      colorToInsert.mGreen = baseColors[k].mGreen;
      colorToInsert.mBlue = baseColors[k].mBlue;

      std::cout<<"colorToInsert.mRed:"<<colorToInsert.mRed<<" "<<"colorToInsert.mGreen:"<<colorToInsert.mGreen<<" "<<"colorToInsert.mBlue:"<<colorToInsert.mBlue<<std::endl;


      colors.push_back(colorToInsert);
    }
  }

  return 0;
}

/**
* 产生一个或多个唯一的颜色
* @param count 要产生的颜色的个数
* @param colors 用于保存生成颜色的向量
* @param excludeColors 要排除的颜色
* @return 产生的颜色的个数
*/
int getUniqueColors(unsigned int count, std::vector<ColorType>& colors, const std::vector<ColorType>& excludeColors)
{
  unsigned int i, j, k, l;
  unsigned int numUnique = 0;
  double slValues[] = {0.0, 1.0, 0.5, 0.8, 0.3, 0.6, 0.9, 0.2, 0.7, 0.4, 0.1};
  ColorType baseColors[] =
  {
    ColorType(0,0,255),
    ColorType(0,255,0),
    ColorType(255,0,0),
    ColorType(0,255,255),
    ColorType(255,255,0),
    ColorType(255,0,255),
    ColorType(255,255,255)
  };

  for (i = 0; i < sizeof(slValues) / sizeof(slValues[0]); i++)
  {
    for (j = 0; j < sizeof(slValues) / sizeof(slValues[0]); j++)
    {
      for (k = 0; k < sizeof(baseColors) / sizeof(baseColors[0]); k++)
      {
        int newColor[3];
        int maxValue;

        newColor[0] = (int) (baseColors[k].mRed * slValues[j] + 0.5);
        newColor[1] = (int) (baseColors[k].mGreen * slValues[j] + 0.5);
        newColor[2] = (int) (baseColors[k].mBlue * slValues[j] + 0.5);

        maxValue = 0;
        for (l = 0; l < 3; l++)
        {
          if (newColor[l] > maxValue)
          {
            maxValue = newColor[l];
          }
        }

        maxValue = (int) (maxValue * slValues[i] + 0.5);
        for (l = 0; l < 3; l++)
        {
          if (newColor[l] < maxValue)
          {
            newColor[l] = maxValue;
          }
        }

        ColorType colorToInsert;
        colorToInsert.mRed = newColor[0];
        colorToInsert.mGreen = newColor[1];
        colorToInsert.mBlue = newColor[2];

        for (l=0; l<excludeColors.size(); l++)
        {
          if (excludeColors[l].mRed == colorToInsert.mRed &&
            excludeColors[l].mGreen == colorToInsert.mGreen &&
            excludeColors[l].mBlue == colorToInsert.mBlue)
          {
            break;
          }
        }
        if (l == excludeColors.size())
        {
          for (l = 0; l < colors.size(); l++)
          {
            if (colors[l].mRed == colorToInsert.mRed &&
              colors[l].mGreen == colorToInsert.mGreen &&
              colors[l].mBlue == colorToInsert.mBlue)
            {
              break;
            }
          }
          if (l == colors.size())
          {
            colors.push_back (colorToInsert);
            ++numUnique;
            if (colors.size() == count)
            {
              return numUnique;
            }
          }
        }
      }
    }
  }
  return numUnique;
}
