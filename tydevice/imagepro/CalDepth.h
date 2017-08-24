#ifndef __CAL_DEPTH__H
#define __CAL_DEPTH__H

#include<vector>

namespace neolix{

#define  SPLIT_HIST 0.02


class caldepth
{
public:
	caldepth(std::vector<short> *depths);
	void calDepthHist();
	unsigned short getdepth(double &confidence);
	void refreshDepthsData(std::vector<short> *depths);
	//~caldepth();
protected:
private:
	typedef struct __HIST_INDEX
	{
		unsigned int count;
		int index;
	} hist_index;
	std::vector<std::vector<float>> depths;//深度集合
	std::vector<int> countDepths;//深度
	std::vector<short> orDepths;//原始的深度集合
	float lenth;
	unsigned short NumLenth;
	unsigned short maxValue;//最大深度
	unsigned short minValue;//最下深度
	bool hasDepthInfo;//判断是否有深度信息，也就是判断像素中是否存在不为0的数

	void calMaxMinValue();
	void computerlength();

	//sortIndex为返回值，返回值中是按照直方图数值降序
	static void sort_hist(std::vector<int> hist, std::vector<hist_index> &sortIndex);
	static int calIndex(unsigned short value, unsigned short minValue, float lenth);
	
	static bool lessThan(const hist_index &hi1, const hist_index &hi2);
	static bool grateThan(const hist_index &hi1, const hist_index &hi2);
	static void refreshDepths(std::vector<std::vector<float>> &depths);
};
}

#endif