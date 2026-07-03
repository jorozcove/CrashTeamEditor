#include "instance.h"

Instance::Instance(std::string model)
{
	m_name = "NewInstance";
	m_scale = Vec3(1.0f, 1.0f, 1.0f);
	m_pos = Vec3(0.0f, 0.0f, 0.0f);
	m_rot = Vec3(0.0f, 0.0f, 0.0f);
	m_modelID = 0xFFFFFFFF;
	m_color = Color(0.0f, 0.0f, 0.0f);
	m_modelName = model;
	m_flags = 0xB;
	m_unk24 = 0;
	m_unk28 = 0;
	m_hitbox = InstanceHitbox();
}


std::vector<uint8_t> Instance::Serialize() const
{
	PSX::InstDef inst = {};
	std::memset(inst.name, 0, sizeof(inst.name));
	std::memcpy(inst.name, m_name.data(), std::min(m_name.size(), sizeof(inst.name)));
	inst.offModel = 0; // Set later during SaveLEV
	inst.scale = ConvertVec3(m_scale, FP_ONE); // 0x1000 is 1.0 scaling
	inst.maybeScaleMaybePadding = 0; 
	inst.colorRGBA = ConvertColor(m_color);
	inst.flags = m_flags;
	inst.unk24 = m_unk24;
	inst.unk28 = m_unk28;
	inst.offInstance = 0; // Probably unused data
	inst.pos = ConvertVec3(m_pos, FP_ONE_GEO);
	inst.rot = ConvertAngle(m_rot);
	inst.modelID = m_modelID;

	std::vector<uint8_t> result(sizeof(PSX::InstDef));
	std::memcpy(result.data(), &inst, sizeof(PSX::InstDef));
	return result;
}