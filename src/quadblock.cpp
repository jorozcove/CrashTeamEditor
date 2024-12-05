#include "quadblock.h"
#include "psx_types.h"

#include <imgui.h>

#include <unordered_map>
#include <unordered_set>

Quadblock::Quadblock(const std::string& name, Quad& q0, Quad& q1, Quad& q2, Quad& q3, const Vec3& normal)
{
	std::unordered_map<Vec3, unsigned> vRefCount;
	for (size_t i = 0; i < 4; i++)
	{
		vRefCount[q0.p[i]]++; vRefCount[q1.p[i]]++; vRefCount[q2.p[i]]++; vRefCount[q3.p[i]]++;
	}

	Vec3 centerVertex;
	std::unordered_set<Vec3> uniqueVertices;
	for (const auto& [v, count] : vRefCount)
	{
		if (count == 1) { uniqueVertices.insert(v); }
		else if (count == 4) { centerVertex = v; }
	}

	for (size_t i = 0; i < 4; i++)
	{
		if (uniqueVertices.contains(q0.p[i])) { m_p[0] = Vertex(q0.p[i]); }
		if (uniqueVertices.contains(q1.p[i])) { m_p[2] = Vertex(q1.p[i]); }
		if (uniqueVertices.contains(q2.p[i])) { m_p[6] = Vertex(q2.p[i]); }
		if (uniqueVertices.contains(q3.p[i])) { m_p[8] = Vertex(q3.p[i]); }
	}

	auto FindAdjacentVec = [](const Quad& from, const Quad& to, const Vec3& ignore) -> const Vec3*
		{
			for (size_t i = 0; i < 4; i++)
			{
				if (from.p[i] == ignore) { continue; }
				for (size_t j = 0; j < 4; j++)
				{
					if (from.p[i] == to.p[j]) { return &from.p[i]; }
				}
			}
			return nullptr;
		};

	const Vec3* p1 = FindAdjacentVec(q0, q1, centerVertex);
	const Vec3* p3 = FindAdjacentVec(q0, q2, centerVertex);
	if (!p1)
	{
		Swap(m_p[2], m_p[8]);
		Swap(q1, q3);
	}
	else if (!p3)
	{
		Swap(m_p[6], m_p[8]);
		Swap(q2, q3);
	}

	m_p[1] = p1 ? Vertex(*p1) : Vertex(*FindAdjacentVec(q0, q1, centerVertex));
	m_p[3] = p3 ? Vertex(*p3) : Vertex(*FindAdjacentVec(q0, q2, centerVertex));
	m_p[5] = Vertex(*FindAdjacentVec(q1, q3, centerVertex));
	m_p[7] = Vertex(*FindAdjacentVec(q2, q3, centerVertex));
	m_p[4] = Vertex(centerVertex);

	Vec3 quadNormal = ComputeNormalVector(0, 2, 6);
	quadNormal = quadNormal / quadNormal.Length();
	Vec3 invertedQuadNormal = quadNormal * -1;
	if ((normal - invertedQuadNormal).Length() < (normal - quadNormal).Length())
	{
		Swap(m_p[0], m_p[2]);
		Swap(m_p[3], m_p[5]);
		Swap(m_p[6], m_p[8]);
	}

	ComputeBoundingBox();
	m_name = name;
	m_checkpointIndex = -1;
	m_flags = QuadFlags::GROUND | QuadFlags::TRIGGER_COLLISION;
	m_terrain = TerrainType::ASPHALT;
}

const std::string& Quadblock::Name() const
{
	return m_name;
}

const Vec3& Quadblock::Center() const
{
	return m_p[4].m_pos;
}

const BoundingBox& Quadblock::GetBoundingBox() const
{
	return m_bbox;
}

std::vector<Vertex> Quadblock::GetVertices() const
{
	                              /*   0       1       2       3       4       5       6       7       8    */
	std::vector<Vertex> vertices = { m_p[0], m_p[2], m_p[6], m_p[8], m_p[1], m_p[3], m_p[4], m_p[5], m_p[7] };
	return vertices;
}

std::vector<uint8_t> Quadblock::Serialize(size_t id, size_t offTextures, size_t offVisibleSet, const std::vector<size_t>& vertexIndexes) const
{
	PSX::Quadblock quadblock = {};
	std::vector<uint8_t> buffer(sizeof(quadblock));
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		quadblock.index[i] = static_cast<uint16_t>(vertexIndexes[i]);
	}
	quadblock.flags = m_flags;
	quadblock.drawOrderLow = 0;
	quadblock.drawOrderHigh = 0;
	quadblock.offMidTextures[0] = static_cast<uint32_t>(offTextures);
	quadblock.offMidTextures[1] = static_cast<uint32_t>(offTextures);
	quadblock.offMidTextures[2] = static_cast<uint32_t>(offTextures);
	quadblock.offMidTextures[3] = static_cast<uint32_t>(offTextures);
	quadblock.bbox.min = ConvertVec3(m_bbox.min, FP_ONE_GEO);
	quadblock.bbox.max = ConvertVec3(m_bbox.max, FP_ONE_GEO);
	quadblock.terrain = m_terrain;
	quadblock.weatherIntensity = 0;
	quadblock.weatherVanishRate = 0;
	quadblock.speedImpact = 0;
	quadblock.id = static_cast<uint16_t>((id & 0xffe0) + 0x1f - (id & 0x1f));
	quadblock.checkpointIndex = static_cast<uint8_t>(m_checkpointIndex);
	quadblock.triNormalVecBitshift = static_cast<uint8_t>(std::round(std::log2(std::max(ComputeNormalVector(0, 2, 6).Length(), ComputeNormalVector(2, 8, 6).Length()) * 512.0f)));
	quadblock.offLowTexture = static_cast<uint32_t>(offTextures);
	quadblock.offVisibleSet = static_cast<uint32_t>(offVisibleSet);

	auto CalculateNormalDividend = [this](size_t id0, size_t id1, size_t id2, float scaler) -> uint16_t
		{
			return static_cast<uint16_t>(std::round(scaler / ComputeNormalVector(id0, id1, id2).Length()));
		};

	float scaler = static_cast<float>(1 << quadblock.triNormalVecBitshift);
	quadblock.triNormalVecDividend[0] = CalculateNormalDividend(0, 1, 3, scaler);
	quadblock.triNormalVecDividend[1] = CalculateNormalDividend(1, 4, 3, scaler);
	quadblock.triNormalVecDividend[2] = CalculateNormalDividend(4, 1, 2, scaler);
	quadblock.triNormalVecDividend[3] = CalculateNormalDividend(3, 4, 6, scaler);
	quadblock.triNormalVecDividend[4] = CalculateNormalDividend(7, 4, 5, scaler);
	quadblock.triNormalVecDividend[5] = CalculateNormalDividend(5, 8, 7, scaler);
	quadblock.triNormalVecDividend[6] = CalculateNormalDividend(2, 5, 4, scaler);
	quadblock.triNormalVecDividend[7] = CalculateNormalDividend(6, 4, 7, scaler);
	quadblock.triNormalVecDividend[8] = CalculateNormalDividend(0, 2, 6, scaler);
	quadblock.triNormalVecDividend[9] = CalculateNormalDividend(2, 8, 6, scaler);
	std::memcpy(buffer.data(), &quadblock, sizeof(quadblock));
	return buffer;
}

void Quadblock::RenderUI(size_t checkpointCount)
{
	if (ImGui::TreeNode(m_name.c_str()))
	{
		if (ImGui::TreeNode("Vertices"))
		{
			for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
			{
				m_p[i].RenderUI(i);
				if (m_p[i].IsEdited()) { ComputeBoundingBox(); }
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Bounding Box"))
		{
			m_bbox.RenderUI();
			ImGui::TreePop();
		}
		ImGui::Text("Checkpoint Index: ");
		ImGui::SameLine();
		if (ImGui::InputInt("##cp", &m_checkpointIndex)) { m_checkpointIndex = Clamp(m_checkpointIndex, -1, static_cast<int>(checkpointCount)); }
		ImGui::TreePop();
	}
}

Vec3 Quadblock::ComputeNormalVector(size_t id0, size_t id1, size_t id2) const
{
	Vec3 a = m_p[id0].m_pos - m_p[id1].m_pos;
	Vec3 b = m_p[id2].m_pos - m_p[id0].m_pos;
	return a.Cross(b);
}

void Quadblock::ComputeBoundingBox()
{
	Vec3 min = Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec3 max = Vec3(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
	for (size_t i = 0; i < NUM_VERTICES_QUADBLOCK; i++)
	{
		if (m_p[i].m_pos.x < min.x) { min.x = m_p[i].m_pos.x; } if (m_p[i].m_pos.x > max.x) { max.x = m_p[i].m_pos.x; }
		if (m_p[i].m_pos.y < min.y) { min.y = m_p[i].m_pos.y; } if (m_p[i].m_pos.y > max.y) { max.y = m_p[i].m_pos.y; }
		if (m_p[i].m_pos.z < min.z) { min.z = m_p[i].m_pos.z; } if (m_p[i].m_pos.z > max.z) { max.z = m_p[i].m_pos.z; }
	}
	m_bbox.min = min;
	m_bbox.max = max;
}