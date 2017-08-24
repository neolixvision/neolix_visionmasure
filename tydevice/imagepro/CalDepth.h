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
	std::vector<std::vector<float>> depths;//��ȼ���
	std::vector<int> countDepths;//���
	std::vector<short> orDepths;//ԭʼ����ȼ���
	float lenth;
	unsigned short NumLenth;
	unsigned short maxValue;//������
	unsigned short minValue;//�������
	bool hasDepthInfo;//�ж��Ƿ��������Ϣ��Ҳ�����ж��������Ƿ���ڲ�Ϊ0����

	void calMaxMinValue();
	void computerlength();

	//sortIndexΪ����ֵ������ֵ���ǰ���ֱ��ͼ��ֵ����
	static void sort_hist(std::vector<int> hist, std::vector<hist_index> &sortIndex);
	static int calIndex(unsigned short value, unsigned short minValue, float lenth);
	
	static bool lessThan(const hist_index &hi1, const hist_index &hi2);
	static bool grateThan(const hist_index &hi1, const hist_index &hi2);
	static void refreshDepths(std::vector<std::vector<float>> &depths);
};
}

#endif