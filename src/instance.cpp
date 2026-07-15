#include "instance.h"

InstanceModel::InstanceModel(std::string name, std::vector<uint8_t> rawData)
	: m_name(std::move(name))
	, m_rawData(std::move(rawData))
{
}

Instance::Instance(std::string model)
{
	m_name = "NewInstance";
	m_scale = Vec3(1.0f, 1.0f, 1.0f);
	m_pos = Vec3(0.0f, 0.0f, 0.0f);
	m_rot = Vec3(0.0f, 0.0f, 0.0f);
	m_modelID = ModelId::NONE;
	m_color = Color(0.0f, 0.0f, 0.0f);
	m_modelName = model;
	m_flags = 0xB;
	m_unk24 = 0;
	m_unk28 = 0;
	m_hitbox = InstanceHitbox();
}

Instance::Instance(PSX::InstDef inst)
{
	m_name = std::string(inst.name, strnlen(inst.name, sizeof(inst.name)));
	m_scale = ConvertPSXVec3(inst.scale, FP_ONE);
	m_pos = ConvertPSXVec3(inst.pos, FP_ONE_GEO);
	m_rot = ConvertPSXAngle(inst.rot);
	m_rot.x = -m_rot.x;
	m_rot.y += 180.0f;
	m_rot.z = -m_rot.z;
	m_modelID = static_cast<ModelId>(inst.modelID);
	m_color = ConvertColor(inst.colorRGBA);
	m_flags = inst.flags;
	m_unk24 = inst.unk24;
	m_unk28 = inst.unk28;

	m_modelName = "";
	m_hitbox = InstanceHitbox();
}

void Instance::SetHitbox(const PSX::InstHitbox& hitbox)
{
	m_hitbox.enabled = true;
	m_hitbox.flags = hitbox.flags;
	m_hitbox.halfExtent = ConvertFP(hitbox.halfExtent, FP_ONE_GEO);
	m_hitbox.yOffset = ConvertFP(hitbox.center.y, FP_ONE_GEO) - m_pos.y;
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
	inst.modelID = static_cast<int32_t>(m_modelID);

	std::vector<uint8_t> buffer(sizeof(PSX::InstDef));
	std::memcpy(buffer.data(), &inst, sizeof(PSX::InstDef));
	return buffer;
}

BoundingBox Instance::ComputeBBox()
{
	Vec3 center = m_pos + Vec3(0.0f, m_hitbox.yOffset, 0.0f);
	Vec3 half_ext = Vec3(m_hitbox.halfExtent, m_hitbox.halfExtent, m_hitbox.halfExtent);
	BoundingBox bbox{};
	bbox.min = center - half_ext;
	bbox.max = center + half_ext;
	return bbox;
}


PSX::InstHitbox Instance::SerializeHitbox(uint32_t insatnceOffset) const
{	// Don't call on Instances that have hitbox disabled. 
	// Serialization will still work, but shouldn't be called.

	Vec3 center = m_pos + Vec3(0.0f, m_hitbox.yOffset, 0.0f);
	Vec3 half_ext = Vec3(m_hitbox.halfExtent, m_hitbox.halfExtent, m_hitbox.halfExtent);

	PSX::InstHitbox hitbox = {};
	hitbox.flags = m_hitbox.flags;
	hitbox.bbox.min = ConvertVec3(center - half_ext, FP_ONE_GEO);
	hitbox.bbox.max = ConvertVec3(center + half_ext, FP_ONE_GEO);
	hitbox.center = ConvertVec3(center, FP_ONE_GEO);
	hitbox.halfExtent = ConvertFloat(m_hitbox.halfExtent, FP_ONE_GEO);
	hitbox.halfExtentSq = hitbox.halfExtent * hitbox.halfExtent;
	hitbox.padding = 0;
	hitbox.offInstDef = insatnceOffset;

	return hitbox;
}