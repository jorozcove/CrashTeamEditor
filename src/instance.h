#pragma once

#include "geo.h"
#include "psx_types.h"
#include "quadblock.h"
#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>


// Per-InstDef settings for emitting a BSP-leaf collision hitbox at save time.
// Presets mirror flag/extent combos observed in vanilla levels (proto8).
struct InstanceHitbox
{
	enum Preset : int { PICKUP = 0, SOLID_WALL = 1, STATIC_DECORATION = 2, CUSTOM = 3 };

	bool enabled = false;
	int preset = Preset::PICKUP;
	uint32_t flags = 0x000004C0; // vanilla pickup flags (bit 0x80 set = trigger-only)
	int16_t halfExtent = 76; // vanilla pickup trigger radius
	int16_t yOffset = 0; // hitbox center offset above InstDef position
};



class InstanceModel
{
public:

private:
	std::string m_name;
	std::vector<uint8_t> m_rawData; //.ctrmodel file
};


class Instance
{
public:
	Instance(std::string model);

	// Name
	const std::string& GetName() const { return m_name; }
	void SetName(const std::string& name) { m_name = name; }

	// Scale
	const Vec3& GetScale() const { return m_scale; }
	void SetScale(const Vec3& scale) { m_scale = scale; }

	// Position
	const Vec3& GetPos() const { return m_pos; }
	void SetPos(const Vec3& pos) { m_pos = pos; }

	// Rotation
	const Vec3& GetRot() const { return m_rot; }
	void SetRot(const Vec3& rot) { m_rot = rot; }

	// Model ID
	int32_t GetModelID() const { return m_modelID; }
	void SetModelID(int32_t modelID) { m_modelID = modelID; }

	// Color
	const Color& GetColor() const { return m_color; }
	void SetColor(const Color& color) { m_color = color; }

	// Model name
	std::string GetModelName() const { return m_modelName; }
	void SetModelName(std::string model) { m_modelName = model; }

	// Flags
	uint32_t GetFlags() const { return m_flags; }
	void SetFlags(uint32_t flags) { m_flags = flags; }

	// Unknown fields
	uint32_t GetUnk24() const { return m_unk24; }
	void SetUnk24(uint32_t unk24) { m_unk24 = unk24; }

	uint32_t GetUnk28() const { return m_unk28; }
	void SetUnk28(uint32_t unk28) { m_unk28 = unk28; }

	// Hitbox
	const InstanceHitbox& GetHitbox() const { return m_hitbox; }
	InstanceHitbox& GetHitbox() { return m_hitbox; }
	void SetHitbox(const InstanceHitbox& hitbox) { m_hitbox = hitbox; }


	void RenderUI(bool& shouldDelete, int index, std::unordered_map<std::string, std::vector<uint8_t>>& importedModels, Vec3& queryPoint);
	std::vector<uint8_t> Serialize() const;
private:
	std::string m_name;
	Vec3 m_scale;
	Vec3 m_pos;
	Vec3 m_rot;
	int32_t m_modelID;
	Color m_color;
	std::string m_modelName;
	uint32_t m_flags;
	uint32_t m_unk24;
	uint32_t m_unk28;
	InstanceHitbox m_hitbox;
};
