#include "CalDepth.h"
#include <algorithm>

namespace neolix{
caldepth::caldepth(std::vector<short> *depths)
{
	orDepths.clear();
	for (size_t i = 0; i< depths->size(); i++) this->orDepths.push_back((*depths)[i]);
	hasDepthInfo = false;
}
void  caldepth::refreshDepthsData(std::vector<short> *depths)
{
	orDepths.clear();
	for (size_t i = 0; i< depths->size(); i++) this->orDepths.push_back((*depths)[i]);
	hasDepthInfo = false;
}
void caldepth::calMaxMinValue()
{
	//找到第=一个不为0的数
	//在此修正一下直方图的区域范围NumLenth
	size_t i;
	size_t count = 0;
	for(i = 0; i < orDepths.size(); i++)
	{
		if (orDepths[i] > 0  && hasDepthInfo == false)
		{
			minValue = orDepths[i];
			maxValue = orDepths[i];
			hasDepthInfo =  true;
		}else if (orDepths[i] > 0 &&hasDepthInfo == true)
		{
			count++;
			if(orDepths[i] < minValue) minValue = orDepths[i];
			if(orDepths[i] > maxValue) maxValue = orDepths[i];
		}
	}
	count++;
	this->NumLenth = static_cast<unsigned short>(count/(count*SPLIT_HIST));
}
void caldepth::computerlength()
{
	 lenth =(maxValue - minValue+1)/(float)NumLenth;
	 countDepths.resize(NumLenth);
	 for (size_t i= 0; i < countDepths.size(); i++)
	 {
		 countDepths[i] = 0;
	 }
	 depths.resize(NumLenth);
}

int caldepth::calIndex(unsigned short value, unsigned short minValue, float lenth)
{
	unsigned short relationPost = value - minValue;
	float index_t = relationPost/lenth;
	return static_cast<int>(index_t);
}

void caldepth::refreshDepths(std::vector<std::vector<float>> &depths)
{
	for (size_t i = 0; i < depths.size(); i++)
	{
		depths[i].clear();
	}
	depths.clear();
}

void caldepth::calDepthHist()
{

	calMaxMinValue();
	computerlength();
	refreshDepths(this->depths);
	depths.resize(NumLenth);
	countDepths.clear();
	countDepths.resize(NumLenth);
	for (size_t i = 0; i < orDepths.size(); i++)
	{
		if (0 < orDepths[i] )
		{
			short depth = orDepths[i];
			int index = caldepth::calIndex(depth,this->minValue,this->lenth);
			countDepths[index]++;
			depths[index].push_back(depth);
		}
	}

}

bool caldepth::lessThan(const hist_index &hi1, const hist_index &hi2)
{
	return hi1.count < hi2.count;
}

bool caldepth::grateThan(const hist_index &hi1, const hist_index &hi2)
{
	return hi1.count > hi2.count;
}
void caldepth::sort_hist(std::vector<int> hist, std::vector<hist_index> &sortIndex)
{
	sortIndex.clear();
	for (size_t i = 0 ; i < hist.size(); i++)
	{
		hist_index hi;
		hi.count = hist[i];
		hi.index = i;
		sortIndex.push_back(hi);
	}
	std::sort(sortIndex.begin(),sortIndex.end(),caldepth::grateThan);
}

unsigned short caldepth::getdepth(double &confidence)
{
	std::vector<hist_index> sortIndex;
	caldepth::sort_hist(countDepths,sortIndex);
	unsigned long totalPiex = 0;
	double totalDepth = 0.0;
	float depth;
	unsigned long usedPiex = 0;
	for (size_t i = 0; i < sortIndex.size(); i++)
	{
		totalPiex += sortIndex[i].count;
	}
	for (size_t i = 0; i < sortIndex.size(); i++)
	{
		usedPiex += sortIndex[i].count;
		for (size_t k = 0; k < sortIndex[i].count; k++)
		{
			totalDepth +=this->depths[sortIndex[i].index][k];
		}
		//用的区域中45%的深度,当检测带大于45%的深度点就计算深度
		confidence = usedPiex/(double)totalPiex ;
		if(confidence > 0.1500)break;
	}

	depth =static_cast<float>(totalDepth/usedPiex);

	return static_cast<unsigned short>(depth);
}

}