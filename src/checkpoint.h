#pragma once

#include "math.h"
#include "quadblock.h"

#include <string>
#include <vector>

class Checkpoint
{
public:
	Checkpoint(int index);
	void UpdateIndex(int index);
	bool GetDelete() const;
	void RemoveInvalidCheckpoints(const std::vector<int>& invalidIndexes);
	void UpdateInvalidCheckpoints(const std::vector<int>& invalidIndexes);
	std::vector<uint8_t> Serialize() const;
	void RenderUI(size_t numCheckpoints, const std::vector<Quadblock>& quadblocks);

private:
	int m_index;
	Vec3 m_pos;
	float m_distToFinish;
	int m_up;
	int m_down;
	int m_left;
	int m_right;
	bool m_delete;
	std::string m_uiPosQuad;
	std::string m_uiLinkUp;
	std::string m_uiLinkDown;
	std::string m_uiLinkLeft;
	std::string m_uiLinkRight;
};